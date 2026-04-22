/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include "xarm/wrapper/xarm_api.h"


int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  std::string port(argv[1]);

  auto arm = std::make_shared<XArmAPI>(port);
  sleep_milliseconds(500);
  
  int ret;

  ret = arm->clean_linear_motor_error();
  printf("clean_linear_motor_error, ret=%d\n", ret);

  ret = arm->set_linear_motor_back_origin();
  printf("set_linear_motor_back_origin, ret=%d\n", ret);
  
  ret = arm->set_linear_motor_enable(true);
  printf("set_linear_motor_enable, ret=%d\n", ret);

  ret = arm->set_linear_motor_speed(100);
  printf("set_linear_motor_speed, ret=%d\n", ret);

  ret = arm->set_linear_motor_pos(500);
  printf("set_linear_motor_pos, ret=%d\n", ret);

  ret = arm->set_linear_motor_pos(200);
  printf("set_linear_motor_pos, ret=%d\n", ret);

  ret = arm->set_linear_motor_stop();
  printf("set_linear_motor_stop, ret=%d\n", ret);

  return 0;
}