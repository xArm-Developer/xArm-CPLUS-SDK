/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif

#include <stdarg.h>
#include <vector>
#include <string>
#include "xarm/core/utils/log.h"
#include "xarm/core/common/data_type.h"

static std::string vformat_string(const char *fmt, va_list args) {
  va_list args_copy;
  va_copy(args_copy, args);
  int size = vsnprintf(nullptr, 0, fmt, args_copy);
  va_end(args_copy);
  if (size <= 0) return {};

  std::string out(size, '\0');
  vsnprintf(&out[0], out.size() + 1, fmt, args);
  return out;
}

void xarm_log(LogLevel level, const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  std::string msg = vformat_string(fmt, args);
  va_end(args);

  const char *prefix = "";
  FILE *stream = stdout;
  switch (level) {
    case LogLevel::Debug: prefix = "[D] "; break;
    case LogLevel::Info:  prefix = "[I] "; break;
    case LogLevel::Warn:  prefix = "[W] "; stream = stderr; break;
    case LogLevel::Error: prefix = "[E] "; stream = stderr; break;
    case LogLevel::Pure:  break;
  }
  fprintf(stream, "%s%s", prefix, msg.c_str());
}

void print_nvect(const char *str, const double vect[], int n) {
  std::string line = str ? str : "";
  for (int i = 0; i < n; ++i) {
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%0.3f ", vect[i]);
    line += buf;
  }
  line += "\n";
  XARM_LOG_PURE("%s", line.c_str());
}

void print_nvect(const char *str, const float *vect, int n) {
  std::string line = str ? str : "";
  for (int i = 0; i < n; ++i) {
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%0.3f ", vect[i]);
    line += buf;
  }
  line += "\n";
  XARM_LOG_PURE("%s", line.c_str());
}

void print_nvect(const char *str, const unsigned char vect[], int n) {
  std::string line = str ? str : "";
  for (int i = 0; i < n; ++i) {
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%d ", vect[i]);
    line += buf;
  }
  line += "\n";
  XARM_LOG_PURE("%s", line.c_str());
}

void print_nvect(const char *str, const int vect[], int n) {
  std::string line = str ? str : "";
  for (int i = 0; i < n; ++i) {
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%d ", vect[i]);
    line += buf;
  }
  line += "\n";
  XARM_LOG_PURE("%s", line.c_str());
}

void print_hex(const char *str, const unsigned char *hex, int len) {
  std::string line = str ? str : "";
  line += " ";
  char buf[4] = {0};
  for (int i = 0; i < len; ++i) {
    snprintf(buf, sizeof(buf), "%02x ", hex[i]);
    line += buf;
  }
  line += "\n";
  XARM_LOG_PURE("%s", line.c_str());
}
