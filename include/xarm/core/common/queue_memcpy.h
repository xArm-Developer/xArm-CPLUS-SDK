/**
 * Software License Agreement (MIT License)
 * 
 * @copyright Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef CORE_COMMON_QUEUE_MEMCPY_H_
#define CORE_COMMON_QUEUE_MEMCPY_H_

#include <mutex>
#include <vector>

class QueueMemcpy {
public:
  QueueMemcpy(size_t n, size_t n_size);
  ~QueueMemcpy(void);
  int flush(void);
  int push(const void *data, bool full_auto_pop=false);
  int pop(void *data);
  int get(void *data);
  size_t size();
  size_t node_size();
  bool is_full();

protected:
private:
  size_t total_;
  size_t annode_size_;

  size_t cnt_;
  size_t head_;
  size_t tail_;
  std::vector<char> buf_;
  
  std::mutex mutex_;
};
#endif
