/**
 * @file thread_pool.cpp
 * @brief Worker-thread loop and shutdown for ThreadPool.
 *
 * The worker loop drains @c tasks until @c stop is set AND the queue is empty.
 * Tasks queued before shutdown are completed before the worker exits, so a
 * caller still holding a future can safely call get().
 */

#include "thread_pool.h"

ThreadPool::ThreadPool(std::size_t num_threads) : stop(false) {
	for (std::size_t i = 0; i < num_threads; ++i) {
		workers.emplace_back([this] {
			while (true) {
				std::function<void()> task;
				{
					std::unique_lock<std::mutex> lock(queue_mutex);
					condition.wait(lock, [this] { return stop || !tasks.empty(); });
					if (stop && tasks.empty()) return;
					task = std::move(tasks.front());
					tasks.pop();
				}
				task();
			}
		});
	}
}

ThreadPool::~ThreadPool() {
	{
		std::unique_lock<std::mutex> lock(queue_mutex);
		stop = true;
	}
	condition.notify_all();
	for (auto& worker : workers) worker.join();
}
