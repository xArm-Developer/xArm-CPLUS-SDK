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
  arm->move_gohome(true);

  fp32 firstPose[6] = { 300, 0, 300, 180, 0, 0 };
  arm->set_position(firstPose, true);
  arm->set_mode(1);
  arm->set_state(0);
  sleep_milliseconds(100);

  fp32 pose[6] = { 0, 0, 0, 0, (fp32)0.2, 0 };
  for (int i = 1; i < 450; i++) {
    ret = arm->set_servo_cartesian_aa(pose, false, true);
    printf("set_servo_cartesian_aa, ret=%d\n", ret);
    sleep_milliseconds(5);
  }
  return 0;
}