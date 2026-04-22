/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include <string.h>
#include "xarm/core/common/queue_memcpy.h"

QueueMemcpy::QueueMemcpy(size_t n, size_t n_size) 
  : total_(n), annode_size_(n_size), buf_(total_ * annode_size_, 0) {
  flush();
}

QueueMemcpy::~QueueMemcpy(void) {}

int QueueMemcpy::flush(void) {
  std::lock_guard<std::mutex> locker(mutex_);
  cnt_ = 0;
  head_ = 0;
  tail_ = 0;
  std::fill(buf_.begin(), buf_.end(), 0); 
  return 0;
}

size_t QueueMemcpy::size() {
  std::lock_guard<std::mutex> locker(mutex_);
  return cnt_;
}

bool QueueMemcpy::is_full() {
  std::lock_guard<std::mutex> locker(mutex_);
  if (total_ <= cnt_)
    return true;
  else
    return false;
}

size_t QueueMemcpy::node_size() { 
  std::lock_guard<std::mutex> locker(mutex_);
  return annode_size_; 
}

int QueueMemcpy::pop(void *data) {
  std::lock_guard<std::mutex> locker(mutex_);
  if (0 >= cnt_) {
    return -1;
  }
  if (total_ <= tail_) tail_ = 0;

  memcpy(data, buf_.data() + tail_ * annode_size_, annode_size_);
  tail_++;
  cnt_--;
  return 0;
}

int QueueMemcpy::get(void *data) {
  std::lock_guard<std::mutex> locker(mutex_);
  if (0 >= cnt_) {
    return -1;
  }
  if (total_ <= tail_) tail_ = 0;

  memcpy(data, buf_.data() + tail_ * annode_size_, annode_size_);

  return 0;
}

int QueueMemcpy::push(const void *data, bool full_auto_pop) {
  std::lock_guard<std::mutex> locker(mutex_);
  if (total_ <= cnt_) {
    if (full_auto_pop) {
      if (total_ <= tail_) tail_ = 0;
      // memcpy(data, buf_.data() + tail_ * annode_size_, annode_size_);
      tail_++;
      cnt_--;
    }
    else
      return -1;
  }
  if (total_ <= head_) head_ = 0;

  memcpy(buf_.data() + head_ * annode_size_, data, annode_size_);
  head_++;
  cnt_++;
  return 0;
}
