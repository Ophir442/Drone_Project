#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

/**
 * @brief Fixed-size pool of worker threads consuming a shared task queue.
 *
 * Used by Simulation to parallelize:
 *  - per-bakery production rolls and per-drone motion in stage 1
 *  - GRASP iterations in stages 2–3
 *
 * Synchronization model: a single mutex protects the task queue; one
 * condition variable wakes idle workers. Callers obtain completion
 * status via the std::future returned by submit() — there is no
 * separate wait_all path. The pool joins all workers on destruction.
 */
class ThreadPool {
public:
	/**
	 * @brief Spawn @p num_threads workers, all waiting on the queue.
	 * @param num_threads must be >= 1; callers are responsible for validation.
	 */
	explicit ThreadPool(std::size_t num_threads);

	/// Signals shutdown, wakes all workers, and joins every thread.
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
	std::vector<std::thread>          workers;
	std::queue<std::function<void()>> tasks;
	std::mutex                        queue_mutex;
	std::condition_variable           condition;
	bool                              stop;
};

template<typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
	using return_type = decltype(f(args...));

	auto task = std::make_shared<std::packaged_task<return_type()>>(
		std::bind(std::forward<F>(f), std::forward<Args>(args)...)
	);

	std::future<return_type> result = task->get_future();
	{
		std::unique_lock<std::mutex> lock(queue_mutex);
		tasks.emplace([task]() { (*task)(); });
	}
	condition.notify_one();
	return result;
}
