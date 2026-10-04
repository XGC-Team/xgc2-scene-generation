#include "xgc2_world_lidar/scan_pool.h"

#include <algorithm>

namespace xgc2_world_lidar {

std::size_t defaultScanThreads(std::size_t scans, unsigned hardware) {
    const std::size_t half = std::max<std::size_t>(1, (hardware == 0 ? 2 : hardware) / 2);
    return std::max<std::size_t>(1, std::min(scans, half));
}

ScanPool::ScanPool(std::size_t threads) {
    try {
        for (std::size_t i = 1; i < threads; ++i)
            workers_.emplace_back([this, i] { work(i); });
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        start_.notify_all();
        for (auto& worker : workers_)
            worker.join();
        throw;
    }
}

ScanPool::~ScanPool() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    start_.notify_all();
    for (auto& worker : workers_)
        worker.join();
}

void ScanPool::drain(std::size_t worker) {
    for (;;) {
        const std::size_t i = next_.fetch_add(1, std::memory_order_relaxed);
        if (i >= count_)
            return;
        try {
            (*job_)(i, worker);
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!error_)
                error_ = std::current_exception();
        }
    }
}

void ScanPool::work(std::size_t worker) {
    uint64_t seen = 0;
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
        start_.wait(lock, [&] { return stop_ || batch_ != seen; });
        if (stop_)
            return;
        seen = batch_;
        lock.unlock();
        drain(worker);
        lock.lock();
        if (--busy_ == 0)
            done_.notify_one();
    }
}

void ScanPool::run(std::size_t count, const std::function<void(std::size_t)>& job) {
    runWithWorker(count, [&job](std::size_t i, std::size_t) { job(i); });
}

void ScanPool::runWithWorker(std::size_t count,
                             const std::function<void(std::size_t, std::size_t)>& job) {
    if (count == 0)
        return;
    if (workers_.empty() || count == 1) {
        std::exception_ptr error;
        for (std::size_t i = 0; i < count; ++i) {
            try {
                job(i, 0);
            } catch (...) {
                if (!error)
                    error = std::current_exception();
            }
        }
        if (error)
            std::rethrow_exception(error);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job_ = &job;
        count_ = count;
        next_.store(0, std::memory_order_relaxed);
        error_ = nullptr;
        busy_ = workers_.size();
        ++batch_;
    }
    start_.notify_all();
    drain(0);
    std::exception_ptr error;
    {
        // Every worker leaves the batch before the job goes out of scope.
        std::unique_lock<std::mutex> lock(mutex_);
        done_.wait(lock, [&] { return busy_ == 0; });
        job_ = nullptr;
        std::swap(error, error_);
    }
    if (error)
        std::rethrow_exception(error);
}

} // namespace xgc2_world_lidar
