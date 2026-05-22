/**
 * @file thread_pool.cpp
 * @brief Lock-free MPMC task queue (Vyukov-style) and spin/yield worker loop.
 *
 * Memory model
 * ------------
 * The synchronization point is per-slot @c sequence (NOT the producer/consumer
 * cursors). The cursors only coordinate which producer/consumer claims which
 * slot; the actual happens-before between producers' task writes and
 * consumers' task reads is established through release/acquire on @c sequence:
 *
 *   producer:  (CAS enqueue_pos)        -> store task     -> store sequence (release)
 *   consumer:  (load sequence acquire)  -> move task      -> CAS dequeue_pos
 *                                                          -> store sequence (release)
 *
 * The producer's release-store of @c sequence synchronizes-with the consumer's
 * acquire-load, so the consumer is guaranteed to see the @c task write. The
 * consumer's release-store of @c sequence (advancing it by kQueueCapacity)
 * synchronizes-with the NEXT producer's acquire-load on the same slot, so the
 * next producer is guaranteed to see the previous task's destructor side
 * effects before overwriting the slot. This also prevents memory leaks: the
 * shared_ptr<packaged_task> captured inside the std::function is destroyed
 * exactly once when the slot is overwritten or when the queue is destroyed.
 *
 * The producer/consumer cursors themselves can be memory_order_relaxed: the
 * CAS just decides "who got this slot first." Cross-thread visibility of the
 * payload is the slot sequence's job.
 *
 * Shutdown
 * --------
 * The destructor sets @c stop_ (release) and joins. Workers acquire-load
 * @c stop_ after every empty-queue probe, drain any remaining tasks (handles
 * the race where a producer enqueued just before stop_ flipped), then exit.
 * Same caller-side contract as the previous mutex-based implementation: do
 * not submit() once the destructor has begun.
 */

#include "thread_pool.h"

#include <cassert>

namespace {

/// Signed difference of two unsigned slot positions. Using signed arithmetic
/// makes the comparison wrap-safe: as long as the gap between producer and
/// consumer never exceeds 2^63 (it never will at our task rates), seq - pos
/// gives the correct relative ordering regardless of overflow on either side.
inline std::intptr_t signed_diff(std::size_t a, std::size_t b) {
	return static_cast<std::intptr_t>(a - b);
}

}  // namespace


ThreadPool::ThreadPool(std::size_t num_threads)
	: slots_(std::make_unique<Slot[]>(kQueueCapacity))
{
	static_assert((kQueueCapacity & kQueueMask) == 0,
	              "ThreadPool queue capacity must be a power of two so the "
	              "ring index can be computed with a mask.");

	// Initial state: slot[i].sequence = i means "slot i is free for the
	// producer at logical position i to fill." memory_order_relaxed is
	// sufficient because no other thread can observe slots_ until workers
	// are spawned below.
	for (std::size_t i = 0; i < kQueueCapacity; ++i) {
		slots_[i].sequence.store(i, std::memory_order_relaxed);
	}

	workers_.reserve(num_threads);
	for (std::size_t i = 0; i < num_threads; ++i) {
		workers_.emplace_back([this] { worker_loop(); });
	}
}

ThreadPool::~ThreadPool() {
	// Release-store so any task ENQUEUED before this point happens-before
	// the corresponding worker's acquire-load of stop_, even if that worker
	// only observes stop_ after a few extra spin iterations.
	stop_.store(true, std::memory_order_release);
	for (auto& w : workers_) w.join();
}

void ThreadPool::enqueue(Job&& job) {
	std::size_t pos   = enqueue_pos_.load(std::memory_order_relaxed);
	std::size_t spins = 0;

	for (;;) {
		Slot& slot      = slots_[pos & kQueueMask];
		const std::size_t seq = slot.sequence.load(std::memory_order_acquire);
		const std::intptr_t diff = signed_diff(seq, pos);

		if (diff == 0) {
			// Slot is free at our logical position — try to claim it.
			if (enqueue_pos_.compare_exchange_weak(
			        pos, pos + 1,
			        std::memory_order_relaxed,
			        std::memory_order_relaxed))
			{
				// We own slot[pos]. Write the task, then publish via the
				// release-store on sequence. The consumer that later sees
				// sequence == pos + 1 via acquire-load is guaranteed to
				// see this write.
				slot.task = std::move(job);
				slot.sequence.store(pos + 1, std::memory_order_release);
				return;
			}
			// CAS failed: another producer beat us; pos was updated.
		} else if (diff < 0) {
			// Queue full: a consumer hasn't drained slot[pos] yet. Spin a
			// bounded number of iterations to absorb short stalls, then
			// yield the scheduler. yield() is NOT an OS blocking primitive
			// — it's a scheduling hint that lets ready peers run.
			if (++spins >= kSpinsBeforeYield) {
				std::this_thread::yield();
				spins = 0;
			}
			pos = enqueue_pos_.load(std::memory_order_relaxed);
		} else {
			// Another producer has already moved enqueue_pos_ past us
			// (seq > pos means slot[pos] is the consumer's job now). Reread.
			pos = enqueue_pos_.load(std::memory_order_relaxed);
		}
	}
}

bool ThreadPool::try_dequeue(Job& out) {
	std::size_t pos = dequeue_pos_.load(std::memory_order_relaxed);

	for (;;) {
		Slot& slot      = slots_[pos & kQueueMask];
		const std::size_t seq = slot.sequence.load(std::memory_order_acquire);
		const std::intptr_t diff = signed_diff(seq, pos + 1);

		if (diff == 0) {
			// Slot is filled and ours to take — try to claim it.
			if (dequeue_pos_.compare_exchange_weak(
			        pos, pos + 1,
			        std::memory_order_relaxed,
			        std::memory_order_relaxed))
			{
				// Acquire above synchronizes-with the producer's release
				// on the same slot, so this move sees the producer's
				// fully-constructed std::function.
				out = std::move(slot.task);
				// Eagerly drop the captured shared_ptr<packaged_task> by
				// resetting the slot. Belt-and-suspenders: std::function's
				// move leaves the source empty in libstdc++/libc++, but
				// the standard says only "valid but unspecified."
				slot.task = nullptr;
				// Release-store advances sequence by kQueueCapacity so the
				// NEXT producer at logical position (pos + kQueueCapacity)
				// can claim the same physical slot, and so sees the destruction
				// of the old task that just happened above.
				slot.sequence.store(pos + kQueueCapacity, std::memory_order_release);
				return true;
			}
			// CAS failed: another consumer beat us; pos was updated.
		} else if (diff < 0) {
			// Queue empty at our logical position.
			return false;
		} else {
			// Another consumer already moved dequeue_pos_ past us. Reread.
			pos = dequeue_pos_.load(std::memory_order_relaxed);
		}
	}
}

void ThreadPool::worker_loop() {
	std::size_t spins = 0;

	for (;;) {
		Job job;
		if (try_dequeue(job)) {
			job();
			spins = 0;
			continue;
		}

		// Empty probe — check for shutdown.
		// Acquire pairs with the destructor's release-store so any task
		// enqueued before ~ThreadPool() began is guaranteed visible here.
		if (stop_.load(std::memory_order_acquire)) {
			// Drain race-window: a producer may have published a task
			// after our first try_dequeue but before we sampled stop_.
			// A second probe closes that hole.
			if (try_dequeue(job)) {
				job();
				continue;
			}
			return;
		}

		// No task, not stopping — spin briefly, then yield. The bounded
		// spin keeps us responsive when a task arrives in microseconds;
		// the yield prevents complete starvation of co-located threads
		// when the queue stays empty for longer.
		if (++spins >= kSpinsBeforeYield) {
			std::this_thread::yield();
			spins = 0;
		}
	}
}
