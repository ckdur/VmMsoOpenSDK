// Periodic timer running a callback on its own thread until the callback
// returns false or Stop() is called.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace vmopen {

class Timer {
public:
    ~Timer() { Stop(); }

    void Start(unsigned period_ms, std::function<bool()> fn)
    {
        Stop();
        std::lock_guard<std::mutex> lk(m_);
        stop_ = false;
        running_ = true;
        th_ = std::thread([this, period_ms, fn] {
            std::unique_lock<std::mutex> lk(m_);
            while (!stop_) {
                if (cv_.wait_for(lk, std::chrono::milliseconds(period_ms), [this] { return stop_; }))
                    break;
                lk.unlock();
                bool again = fn();
                lk.lock();
                if (!again)
                    break;
            }
            running_ = false;
        });
    }

    // Safe to call from the callback itself (the thread is then detached).
    void Stop()
    {
        std::thread t;
        {
            std::lock_guard<std::mutex> lk(m_);
            stop_ = true;
            cv_.notify_all();
            t = std::move(th_);
        }
        if (t.joinable()) {
            if (t.get_id() == std::this_thread::get_id())
                t.detach();
            else
                t.join();
        }
    }

    bool IsRunning() const { return running_; }

private:
    std::mutex m_;
    std::condition_variable cv_;
    std::thread th_;
    bool stop_ = true;
    std::atomic<bool> running_{false};
};

inline void SleepMs(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

} // namespace vmopen
