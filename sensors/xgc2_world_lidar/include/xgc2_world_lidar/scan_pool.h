#ifndef XGC2_WORLD_LIDAR_SCAN_POOL_H
#define XGC2_WORLD_LIDAR_SCAN_POOL_H

// A fixed set of threads that runs one batch of independent jobs at a time
// (the fleet node: one robot's scan and publish per job), without ROS. Job
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
    void run(std::size_t count, const std::function<void(std::size_t)>& job);

private:
    void work();
    void drain();

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable start_;
    std::condition_variable done_;
    const std::function<void(std::size_t)>* job_ = nullptr;
    std::size_t count_ = 0;
    std::atomic<std::size_t> next_{0};
    std::size_t busy_ = 0; // workers that have not finished the current batch
    uint64_t batch_ = 0;
    bool stop_ = false;
    std::exception_ptr error_;
};

} // namespace xgc2_world_lidar

#endif // XGC2_WORLD_LIDAR_SCAN_POOL_H
