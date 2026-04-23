/**
 * Software License Agreement (MIT License)
 * 
 * @copyright Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef CORE_UTILS_LOG_H_
#define CORE_UTILS_LOG_H_

#include "xarm/core/common/data_type.h"

enum class LogLevel { Debug, Pure, Info, Warn, Error };

void xarm_log(LogLevel level, const char *fmt, ...);

#define XARM_LOG_DEBUG(...)  xarm_log(LogLevel::Debug,  __VA_ARGS__)
#define XARM_LOG_PURE(...)  xarm_log(LogLevel::Pure,  __VA_ARGS__)
#define XARM_LOG_INFO(...)  xarm_log(LogLevel::Info,  __VA_ARGS__)
#define XARM_LOG_WARN(...)  xarm_log(LogLevel::Warn,  __VA_ARGS__)
#define XARM_LOG_ERROR(...) xarm_log(LogLevel::Error, __VA_ARGS__)

#define print_log(...) xarm_log(LogLevel::Pure, __VA_ARGS__)

void print_nvect(const char *str, const double vect[], int n);
void print_nvect(const char *str, const float *vect, int n);
void print_nvect(const char *str, const unsigned char vect[], int n);
void print_nvect(const char *str, const int vect[], int n);
void print_hex(const char *str, const unsigned char *hex, int len);

#endif // CORE_UTILS_LOG_H_
