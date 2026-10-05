#ifndef XGC2_WORLD_LIDAR_SCAN_POOL_H
#define XGC2_WORLD_LIDAR_SCAN_POOL_H

// A fixed set of threads that runs one batch of independent jobs at a time
// (one robot's heavy scan per job; packing/publication stays with its caller), without ROS. Job
// indices are handed out one at a time, so a robot with a large cloud does
// not hold up a static share of the others; the calling thread works too.
// The threads live as long as the pool: a batch costs one wakeup per thread,
// not a thread start per robot.

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace xgc2_world_lidar {

// Threads for `scans` robot sensors: one per sensor, up to half the hardware
// threads (at least one), which leaves the rest of the host to the simulation
// and the robots' software. `hardware` 0 means unknown and counts as 2.
std::size_t defaultScanThreads(std::size_t scans, unsigned hardware);

class ScanPool {
public:
    // `threads` is the parallelism including the calling thread; threads - 1
    // workers are started (none for 0 or 1).
    explicit ScanPool(std::size_t threads);
    ~ScanPool();
    ScanPool(const ScanPool&) = delete;
    ScanPool& operator=(const ScanPool&) = delete;

    std::size_t threads() const { return workers_.size() + 1; }

    // Runs job(i) exactly once for every i in [0, count) and returns when all
    // have finished. Jobs must not depend on each other. If jobs throw, the
    // others still run and the first exception is rethrown here. Not
    // reentrant: one batch at a time.
    // thread_limit includes the caller and preserves this batch's original
    // effective width even when the shared pool has a larger cold capacity.
    void run(std::size_t count, std::size_t thread_limit,
             const std::function<void(std::size_t)>& job);

private:
    void work(std::size_t ordinal);
    void drain();

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable start_;
    std::condition_variable done_;
    const std::function<void(std::size_t)>* job_ = nullptr;
    std::size_t count_ = 0;
    std::atomic<std::size_t> next_{0};
    std::size_t busy_ = 0; // workers that have not finished the current batch
    std::size_t selected_workers_ = 0;
    uint64_t batch_ = 0;
    bool stop_ = false;
    std::exception_ptr error_;
};

} // namespace xgc2_world_lidar

#endif // XGC2_WORLD_LIDAR_SCAN_POOL_H
