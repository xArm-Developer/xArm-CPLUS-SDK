/*
* \file xarm/wrapper/utils.h
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#ifndef WRAPPER_UTILS_H_
#define WRAPPER_UTILS_H_

#include <sys/timeb.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

inline void sleep_milliseconds(unsigned long milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
    usleep(milliseconds * 1000); // 100 ms
#endif
}

inline long long get_system_time()
{
    struct timeb t;
    ftime(&t);
    return 1000 * t.time + t.millitm;
}
#endif
