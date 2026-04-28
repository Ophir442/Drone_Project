#include "thread_pool.hpp"

ThreadPool::ThreadPool(size_t num_threads) : stop(false), active_tasks(0) {
	for (size_t i = 0; i < num_threads; ++i) {
		workers.emplace_back([this] {
			while (true) {
				std::function<void()> task;
				{
					std::unique_lock<std::mutex> lock(queue_mutex);
					condition.wait(lock, [this] {
						return stop || !tasks.empty();
					});
					if (stop && tasks.empty()) {
						return;
					}
					task = std::move(tasks.front());
					tasks.pop();
				}
				task();
				{
					std::unique_lock<std::mutex> lock(queue_mutex);
					active_tasks--;
					if (active_tasks == 0 && tasks.empty()) {
						done_condition.notify_all();
					}
				}
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
	for (auto& worker : workers) {
		worker.join();
	}
}

void ThreadPool::wait_all() {
	std::unique_lock<std::mutex> lock(queue_mutex);
	done_condition.wait(lock, [this] {
		return active_tasks == 0 && tasks.empty();
	});
}
