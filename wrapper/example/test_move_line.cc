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

  int ret;
  ret = arm->clean_warn();
  printf("clean_warn, ret=%d\n", ret);
  ret = arm->clean_error();
  printf("clean_error, ret=%d\n", ret);
  ret = arm->motion_enable(true);
  printf("motion_enable, ret=%d\n", ret);
  ret = arm->set_mode(0);
  printf("set_mode, ret=%d\n", ret);
  ret = arm->set_state(0);
  printf("set_state, ret=%d\n", ret);
  sleep_milliseconds(1000);

  arm->reset(true);
  fp32 poses[6][6] = {
    {300, 0, 200, 180, 0, 0},
    {300, 200, 200, 180, 0, 0},
    {500, 200, 200, 180, 0, 0},
    {500, -200, 200, 180, 0, 0},
    {300, -200, 200, 180, 0, 0},
    {300, 0, 200, 180, 0, 0},
  };
  for (int i = 0; i < 6; i++) {
    ret = arm->set_position(poses[i], true);
    printf("set_position, ret=%d\n", ret);
  }
  return 0;
}
