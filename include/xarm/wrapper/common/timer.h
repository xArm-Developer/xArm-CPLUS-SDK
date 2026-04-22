/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef WRAPPER_COMMON_TIMER_H_
#define WRAPPER_COMMON_TIMER_H_

#include <iostream>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>

class Timer final {
public:
  using Task = std::function<void()>;

  enum class Mode {
    kNone,
    kOneShot,
    kPeriodic,
  };

  Timer() = default;

  ~Timer() {
    stop();
    if (worker_.joinable()) {
      if (worker_.get_id() == std::this_thread::get_id()) {
        // worker_.detach();
        std::terminate();
      }
      else {
        worker_.join();
      }
    }
  }

  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;
  Timer(Timer&&) = delete;
  Timer& operator=(Timer&&) = delete;

  bool start_once(std::chrono::milliseconds delay, Task task) {
    return _start(delay, std::move(task), Mode::kOneShot);
  }

  bool start_periodic(std::chrono::milliseconds interval, Task task) {
    return _start(interval, std::move(task), Mode::kPeriodic);
  }

  void stop() {
    std::thread worker;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested_ = true;
      cv_.notify_all();
      if (!worker_.joinable()) {
        running_ = false;
        mode_ = Mode::kNone;
        task_ = nullptr;
        return;
      }
      if (worker_.get_id() == std::this_thread::get_id()) {
        return;
      }
      worker = std::move(worker_);
    }
    if (worker.joinable()) {
      worker.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      mode_ = Mode::kNone;
      task_ = nullptr;
    }
  }

  bool running() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
  }

  bool stop_requested() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_requested_;
  }

  static std::shared_ptr<Timer> AsyncWait(int after, Task task) {
    auto timer = std::make_shared<Timer>();
    if (!timer->start_once(std::chrono::milliseconds(after), std::move(task))) {
      return nullptr;
    }
    return timer;
  }

private:
  bool _start(std::chrono::milliseconds interval, Task task, Mode mode) {
    if (!task) {
      return false;
    }
    if (interval.count() < 0) {
      return false;
    }
    if (mode == Mode::kPeriodic && interval.count() == 0) {
      return false;
    }

    std::thread finished_worker;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (running_) {
        return false;
      }
      if (worker_.joinable()) {
        if (worker_.get_id() == std::this_thread::get_id()) {
          return false;
        }
        finished_worker = std::move(worker_);
      }
    }
    if (finished_worker.joinable()) {
      finished_worker.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ || worker_.joinable()) {
      return false;
    }
    interval_ = interval;
    task_ = std::move(task);
    mode_ = mode;
    stop_requested_ = false;
    running_ = true;
    worker_ = std::thread(&Timer::_worker_loop, this);
    return true;
  }

  void _worker_loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_requested_) {
      std::chrono::milliseconds interval = interval_;
      Mode mode = mode_;
      if (cv_.wait_for(lock, interval, [this] { return stop_requested_; })) {
        break;
      }

      Task task = task_;
      lock.unlock();
      try {
        task();
      }
      catch (...) {
      }
      lock.lock();

      if (mode == Mode::kOneShot) {
        break;
      }
    }

    running_ = false;
    mode_ = Mode::kNone;
    task_ = nullptr;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;

  Task task_;
  std::chrono::milliseconds interval_{0};
  Mode mode_{Mode::kNone};

  bool running_{false};
  bool stop_requested_{false};
};

// class Timer {
// public:
//   Timer() :expired_(true), try_to_expire_(false) {
//   }

//   Timer(const Timer& t) {
//     expired_ = t.expired_.load();
//     try_to_expire_ = t.try_to_expire_.load();
//   }
//   ~Timer() {
//     Expire();
//     // std::cout << "timer destructed!" << std::endl;
//   }

//   void StartTimer(int interval, std::function<void()> task) {
//     if (expired_ == false) {
//       // std::cout << "timer is currently running, please expire it first..." << std::endl;
//       return;
//     }
//     expired_ = false;
//     std::thread([this, interval, task]() {
//       while (!try_to_expire_) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(interval));
//         task();
//       }
//       // std::cout << "stop task..." << std::endl;
//       {
//         std::lock_guard<std::mutex> locker(mutex_);
//         expired_ = true;
//         expired_cond_.notify_one();
//       }
//     }).detach();
//   }

//   void Expire() {
//     if (expired_) {
//       return;
//     }

//     if (try_to_expire_) {
//       // std::cout << "timer is trying to expire, please wait..." << std::endl;
//       return;
//     }
//     try_to_expire_ = true;
//     {
//       std::unique_lock<std::mutex> locker(mutex_);
//       expired_cond_.wait(locker, [this] {return expired_ == true; });
//       if (expired_ == true) {
//         // std::cout << "timer expired!" << std::endl;
//         try_to_expire_ = false;
//       }
//     }
//   }

//   template<typename callable, class... arguments>
//   void SyncWait(int after, callable&& f, arguments&&... args) {

//     std::function<typename std::result_of<callable(arguments...)>::type()> task
//     (std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
//     std::this_thread::sleep_for(std::chrono::milliseconds(after));
//     task();
//   }
//   template<typename callable, class... arguments>
//   void AsyncWait(int after, callable&& f, arguments&&... args) {
//     std::function<typename std::result_of<callable(arguments...)>::type()> task
//     (std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));

//     std::thread([after, task]() {
//       std::this_thread::sleep_for(std::chrono::milliseconds(after));
//       task();
//     }).detach();
//   }

// private:
//   std::atomic<bool> expired_;
//   std::atomic<bool> try_to_expire_;
//   std::mutex mutex_;
//   std::condition_variable expired_cond_;
// };

class ThreadPool {
public:
  ThreadPool(int max_thread_count = 10) : max_thread_count_(max_thread_count), total_thread_count_(0), free_thread_count_(0), stoped_(false) {};
  ~ThreadPool() {
    stop();
  };

  static void thread_handle(void *arg) {
    ThreadPool *pool = (ThreadPool *)arg;
    pool->_thread_process();
  };

  void _thread_process(void) {
    std::unique_lock<std::mutex> locker(mutex_);
    total_thread_count_ += 1;
    free_thread_count_ += 1;
    // int thread_inx = total_thread_count_;
    locker.unlock();

    // std::cout << "callback thread start, thread_index=" << thread_inx << ", thread_id=" << std::this_thread::get_id() << std::endl;
    while (!stoped_) {
      std::function<void()> task;
      locker.lock();
      task_cond_.wait(locker, [this] {
        return stoped_ || !task_que_.empty();
      });
      if (stoped_ && task_que_.empty()) {
        locker.unlock();
        break;
      }
      free_thread_count_ -= 1;
      task = std::move(task_que_.front());
      task_que_.pop();
      locker.unlock();
      task();
      locker.lock();
      free_thread_count_ += 1;
      locker.unlock();
    }
    locker.lock();
    total_thread_count_ -= 1;
    free_thread_count_ -= 1;
    locker.unlock();
    // std::cout << "callback thread finished, thread_index=" << thread_inx << ", thread_id=" << std::this_thread::get_id() << std::endl;
  };

  void reset() {
    stop();
    std::lock_guard<std::mutex> lock{ mutex_ };
    task_que_ = std::queue<Task>();
    total_thread_count_ = 0;
    free_thread_count_ = 0;
    stoped_.store(false);
  }

  void stop() {
    stoped_.store(true);
    task_cond_.notify_all();
    for (std::thread& thread : thread_vector_) {
      try {
        if (thread.joinable()) thread.join();
      }
      catch (...) {}
    }
    thread_vector_.clear();
  };

  void set_max_thread_count(int max_thread_count) {
    max_thread_count_ = max_thread_count;
  };

  template<typename callable, class... arguments>
  bool dispatch(callable&& f, arguments&&... args) {
    std::function<typename std::result_of<callable(arguments...)>::type()> task(
      std::bind(std::forward<callable>(f),
      std::forward<arguments>(args)...));
    std::lock_guard<std::mutex> lock{ mutex_ };
    if (stoped_.load()) {
      return false;
    }
    task_que_.emplace([task]() {
      task();
    });
    _thread_check();
    task_cond_.notify_one();
    return true;
  }

  template<typename callable, class... arguments>
  void commit(callable&& f, arguments&&... args) {
    std::function<typename std::result_of<callable(arguments...)>::type()> task
    (std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
    task();
  }

private:
  void _thread_check(void) {
    if (stoped_.load()) return;
    if (free_thread_count_ == 0 && (max_thread_count_ < 0 || total_thread_count_ < max_thread_count_)) {
      thread_vector_.emplace_back(std::thread(thread_handle, this));
    }
  }

private:
  using Task = std::function<void()>;
  std::queue<Task> task_que_;
  std::vector<std::thread> thread_vector_;
  std::mutex mutex_;
  std::condition_variable task_cond_;

  int max_thread_count_;
  int total_thread_count_;
  int free_thread_count_;
  std::atomic<bool> stoped_;
};

#endif // WRAPPER_COMMON_TIMER_H_
