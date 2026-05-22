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
 * no OS-level blocking primitives on the hot path. Workers spin and yield()
 * the scheduler when the queue is empty; producers spin and yield() when the
 * queue is full.
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

	using Job = std::function<void()>;

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
	struct alignas(64) Slot {
		std::atomic<std::size_t> sequence;
		Job                      task;
	};

	void worker_loop();
	void enqueue(Job&& job);
	bool try_dequeue(Job& out);

	// Producer/consumer cursors live on their own cache lines so the two
	// roles don't ping-pong the same line between cores.
	alignas(64) std::atomic<std::size_t> enqueue_pos_{0};
	alignas(64) std::atomic<std::size_t> dequeue_pos_{0};
	alignas(64) std::atomic<bool>        stop_{false};

	std::unique_ptr<Slot[]>  slots_;
	std::vector<std::thread> workers_;
};

template<typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
	using return_type = decltype(f(args...));

	// Same shared_ptr<packaged_task> idiom as before: lets us erase the
	// return type into a std::function<void()> while keeping the future alive.
	auto task = std::make_shared<std::packaged_task<return_type()>>(
		std::bind(std::forward<F>(f), std::forward<Args>(args)...)
	);

	std::future<return_type> result = task->get_future();
	enqueue([task]() { (*task)(); });
	return result;
}
