#include "thread_pool.h"

using namespace std;

ThreadPool::ThreadPool(size_t num_threads) : stop(false), active_tasks(0) {
	for (size_t i = 0; i < num_threads; ++i) {
		workers.emplace_back([this] {
			while (true) {
				function<void()> task;
				{
					unique_lock<mutex> lock(queue_mutex);
					condition.wait(lock, [this] { return stop || !tasks.empty(); });
					if (stop && tasks.empty()) return;
					task = move(tasks.front());
					tasks.pop();
				}
				task();
				{
					unique_lock<mutex> lock(queue_mutex);
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
		unique_lock<mutex> lock(queue_mutex);
		stop = true;
	}
	condition.notify_all();
	for (auto& worker : workers) worker.join();
}

void ThreadPool::wait_all() {
	unique_lock<mutex> lock(queue_mutex);
	done_condition.wait(lock, [this] {
		return active_tasks == 0 && tasks.empty();
	});
}
