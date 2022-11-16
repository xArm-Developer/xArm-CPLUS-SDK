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

  XArmAPI *arm = new XArmAPI(port);
  sleep_milliseconds(500);
  if (arm->error_code != 0) arm->clean_error();
  if (arm->warn_code != 0) arm->clean_warn();
  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);
  sleep_milliseconds(500);

  printf("=========================================\n");
  int ret;

  ret = arm->set_gripper_enable(true);
  printf("set_gripper_enable, ret=%d\n", ret);
  ret = arm->set_gripper_speed(5000);
  printf("set_gripper_speed, ret=%d\n", ret);
  while (arm->is_connected() && arm->error_code == 0) {
    ret = arm->set_gripper_position(600, true);
    printf("set_gripper_position, pos=600, ret=%d\n", ret);
    ret = arm->set_gripper_position(200, true);
    printf("set_gripper_position, pos=200, ret=%d\n", ret);
  }

  return 0;
}