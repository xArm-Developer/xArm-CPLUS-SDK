/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/

#include "xarm/wrapper/xarm_api.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  std::string port(argv[1]);

  XArmAPI *arm = new XArmAPI(port);
  sleep_milliseconds(1000);

  if (arm->warn_code != 0) arm->clean_warn();
  if (arm->error_code != 0) arm->clean_error();

  int ret;
  ret = arm->motion_enable(true);
  printf("motion_enable, ret=%d\n", ret);
  ret = arm->set_mode(0);
  printf("set_mode, ret=%d\n", ret);
  ret = arm->set_state(0);
  printf("set_state, ret=%d\n", ret);
  sleep_milliseconds(1000);

  int err;
  ret = arm->get_gripper_err_code(&err);
  printf("get_gripper_err_code, ret=%d, err=%d\n", ret, err);

  ret = arm->set_gripper_mode(0);
  printf("set_gripper_mode, ret=%d\n", ret);
  ret = arm->set_gripper_enable(true);
  printf("set_gripper_enable, ret=%d\n", ret);
  ret = arm->set_gripper_speed(5000);
  printf("set_gripper_speed, ret=%d\n", ret);
  float pos;
  ret = arm->get_gripper_position(&pos);
  printf("get_gripper_position, ret=%d, pos=%f\n", ret, pos);
  ret = arm->set_gripper_position(300, true);
  printf("set_gripper_position, ret=%d\n", ret);
  ret = arm->set_gripper_position(600, true);
  printf("set_gripper_position, ret=%d\n", ret);

  arm->disconnect();

  return 0;
}
