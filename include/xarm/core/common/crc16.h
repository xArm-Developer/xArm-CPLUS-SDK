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

#ifndef CORE_COMMON_CRC16_H_
#define CORE_COMMON_CRC16_H_

#include "xarm/core/common/data_type.h"

int modbus_crc(unsigned char *data, int len);

#endif
