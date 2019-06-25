/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_LINUX_THREAD_H_
#define CORE_LINUX_THREAD_H_

#include <pthread.h>

typedef void *(*fun_point_t)(void *);

void thread_delete(pthread_t id);
pthread_t thread_init(fun_point_t fun_point, void *arg);

#endif
