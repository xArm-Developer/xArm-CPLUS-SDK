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

#ifndef CORE_DEBUG_DEBUG_PRINT_H_
#define CORE_DEBUG_DEBUG_PRINT_H_

#include "xarm/core/common/data_type.h"

void print_log(const char *format, ...);
void print_nvect(const char *str, double vect[], int n);
void print_nvect(const char *str, float *vect, int n);
void print_nvect(const char *str, unsigned char vect[], int n);
void print_nvect(const char *str, int vect[], int n);
void print_hex(const char *str, unsigned char *hex, int len);

#endif
