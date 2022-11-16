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

 /*#ifndef CORE_LINUX_THREAD_H_
 #define CORE_LINUX_THREAD_H_

 #include <pthread.h>

 typedef void *(*fun_point_t)(void *);

 void thread_delete(pthread_t id);
 pthread_t thread_init(fun_point_t fun_point, void *arg);

 #endif
 */

#ifndef THREAD_H_
#define THREAD_H_

#ifdef _WIN32
#include <windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif


#ifdef WIN32
//typedef unsigned __stdcall *(*fun_point_t)(void *);
typedef unsigned __stdcall fun_point_t(void*);
void thread_delete(HANDLE m_handle);
HANDLE thread_init(fun_point_t fun_point, void *arg);
#else
typedef void *(*fun_point_t)(void *);
void thread_delete(pthread_t id);
pthread_t thread_init(fun_point_t fun_point, void *arg);
#endif

#endif // THREAD_H_