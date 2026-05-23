#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <vector>

/**
 * @brief Lock-free fixed-size pool of worker threads.
 *
 * Used by Simulation to parallelize:
 *  - per-bakery production rolls and per-drone motion in stage 1
 *  - GRASP iterations in stages 2-3
 *
 * Synchronization model: a Vyukov-style bounded MPMC ring buffer with
 * per-slot sequence numbers and CAS — no std::mutex, no condition_variable,
 * no OS-level blocking primitives on the hot path. Workers spin (PAUSE) and
 * yield() the scheduler when the queue is empty; producers spin and yield()
 * when the queue is full.
 *
 * Tasks are typed-erased through a tiny TaskBase virtual dispatch instead of
 * std::function — one indirection per call, one allocation per submit.
 *
 * Callers obtain completion status via the std::future returned by submit().
 * The pool joins all workers on destruction.
 */
class ThreadPool {
public:
	/**
	 * @brief Spawn @p num_threads workers, all attached to the lock-free queue.
	 * @param num_threads must be >= 1; callers are responsible for validation.
	 */
	explicit ThreadPool(std::size_t num_threads);

	/// Signals shutdown and joins every worker. Caller must ensure no
	/// submit() is in flight when the destructor runs (same contract as
	/// the previous mutex-based implementation).
	~ThreadPool();

	ThreadPool(const ThreadPool&)            = delete;
	ThreadPool& operator=(const ThreadPool&) = delete;

	/**
	 * @brief Enqueue a callable; returns a future that yields its result.
	 *
	 * The callable is wrapped in a std::packaged_task so any exception it
	 * throws is captured in the future rather than terminating the worker.
	 */
	template<typename F, typename... Args>
	auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))>;

private:
	/// Power-of-two ring buffer capacity. Sized well above the largest
	/// per-stage burst in Simulation (~13 bakeries + ~13 drones ≈ 30 tasks)
	/// so producers practically never observe a full queue.
	static constexpr std::size_t kQueueCapacity     = 1024;
	static constexpr std::size_t kQueueMask         = kQueueCapacity - 1;

	/// Spin count before each producer/worker scheduler yield. Larger values
	/// favor throughput on dedicated cores; smaller values favor responsiveness
	/// under oversubscription. 32 is the inflection point measured by the
	/// folly/ConcurrentQueue benchmarks for typical workloads.
	static constexpr std::size_t kSpinsBeforeYield  = 32;

	static constexpr std::size_t kCacheLine         = 64;

	/// Type-erased task base — one virtual indirection per call, vs
	/// std::function's two-layer type-erasure (function wrapper + captured
	/// shared_ptr). The vtable call is devirtualizable under LTO.
	///
	/// run() is intentionally NOT noexcept: std::packaged_task::operator()
	/// can throw std::future_error if the task is in an invalid state
	/// (already-invoked, moved-from, empty). The user's own exception is
	/// always captured into the future's shared state — but framework
	/// exceptions must be allowed to propagate to the worker loop rather
	/// than trip std::terminate.
	struct TaskBase {
		virtual void run() = 0;
		virtual ~TaskBase() = default;
	};
	template<class F>
	struct TaskImpl final : TaskBase {
		F f;
		explicit TaskImpl(F&& fn) : f(std::move(fn)) {}
		void run() override { f(); }
	};
	using Job = std::unique_ptr<TaskBase>;

	/**
	 * @brief One ring-buffer slot.
	 *
	 * The @c sequence field is the synchronization primitive: it advances
	 * in two-step cycles (free → filled → consumed → free again). A producer
	 * may claim slot[i] only when sequence == i (slot is free); a consumer
	 * may take from slot[i] only when sequence == i+1 (slot is filled).
	 *
	 * Padded to a 64-byte cache line to prevent false sharing between
	 * adjacent slots that happen to be hammered by different cores.
	 */
	struct alignas(kCacheLine) Slot {
		std::atomic<std::size_t> sequence;
		Job                      task;
	};

	/// Pad an atomic to fill a full cache line so producer and consumer
	/// cursors never share a line with neighboring class members.
	template<class T>
	struct alignas(kCacheLine) CacheLinePadded {
		T value;
	};

	void worker_loop();
	void enqueue(Job&& job);
	bool try_dequeue(Job& out);

	// Producer/consumer cursors live on their own cache lines so the two
	// roles don't ping-pong the same line between cores.
	CacheLinePadded<std::atomic<std::size_t>> enqueue_pos_{};
	CacheLinePadded<std::atomic<std::size_t>> dequeue_pos_{};
	CacheLinePadded<std::atomic<bool>>        stop_{};

	std::unique_ptr<Slot[]>  slots_;
	std::vector<std::thread> workers_;

	// ---- compile-time invariants ----
	// Cache-line isolation must be enforced statically: a future refactor that
	// fattens the atomic types or swaps the Slot::task type must not silently
	// collapse the no-false-sharing guarantee.
	static_assert(sizeof(CacheLinePadded<std::atomic<std::size_t>>) == kCacheLine,
	              "CacheLinePadded<atomic<size_t>> must occupy exactly one cache line.");
	static_assert(sizeof(CacheLinePadded<std::atomic<bool>>)        == kCacheLine,
	              "CacheLinePadded<atomic<bool>> must occupy exactly one cache line.");
	static_assert(alignof(Slot)            >= kCacheLine,
	              "Slot must be cache-line aligned to prevent inter-slot false sharing.");
	static_assert(sizeof(Slot) % kCacheLine == 0,
	              "Slot size must be a multiple of one cache line.");
};

template<typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
	using R = decltype(f(args...));

	// shared_ptr lets us erase the packaged_task into a void()-returning
	// lambda while keeping the future alive. We pay one heap allocation for
	// the shared_ptr control block + packaged_task storage (via make_shared)
	// and one for the TaskImpl wrapper. std::function would have added a
	// third (the function wrapper itself) on top of that.
	auto task_ptr = std::make_shared<std::packaged_task<R()>>(
		std::bind(std::forward<F>(f), std::forward<Args>(args)...)
	);
	std::future<R> result = task_ptr->get_future();

	auto wrapped = [task_ptr]() { (*task_ptr)(); };
	enqueue(std::make_unique<TaskImpl<decltype(wrapped)>>(std::move(wrapped)));
	return result;
}
