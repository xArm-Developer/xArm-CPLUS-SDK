/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef CORE_UTILS_THREAD_POOL_H_
#define CORE_UTILS_THREAD_POOL_H_

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
#include "xarm/core/utils/log.h"

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
      catch (const std::exception& e) {
        XARM_LOG_WARN("thread join exception: %s\n", e.what());
      }
      catch (...) {
        XARM_LOG_WARN("thread join exception: unknown\n");
      }
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

#endif // CORE_UTILS_THREAD_POOL_H_
