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

  arm->set_mode(1);
  arm->set_state(0);
  sleep_milliseconds(100);

  while (arm->is_connected() && arm->state != 4)
  {
    for (int i = 1; i <= 100; i++) {
      fp32 angles[7] = { (fp32)i, 0, 0, 0, 0, 0, 0 };
      ret = arm->set_servo_angle_j(angles);
      printf("set_servo_angle_j, ret=%d\n", ret);
      sleep_milliseconds(10);
    }
    for (int i = 1; i <= 100; i++) {
      fp32 angles[7] = { (fp32)(100-i), 0, 0, 0, 0, 0, 0 };
      ret = arm->set_servo_angle_j(angles);
      printf("set_servo_angle_j, ret=%d\n", ret);
      sleep_milliseconds(10);
    }
  }
  return 0;
}