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

  arm->move_gohome(true);

  arm->set_mode(4);
  arm->set_state(0);
  sleep_milliseconds(1000);

  int ret;
  int cnt = 6;
  fp32 stop_velocity[7] = {0, 0, 0, 0, 0, 0, 0};
  fp32 velocity_1[7] = {-50, 0, 0, 0, 0, 0, 0};
  fp32 velocity_2[7] = {50, 0, 0, 0, 0, 0, 0};
  while(cnt--) {
    ret = arm->vc_set_joint_velocity(velocity_1);
    printf("vc_set_joint_velocity, ret=%d\n", ret);
    sleep_milliseconds(2000);
    ret = arm->vc_set_joint_velocity(velocity_2);
    printf("vc_set_joint_velocity, ret=%d\n", ret);
    sleep_milliseconds(2000);
  }
  ret = arm->vc_set_joint_velocity(stop_velocity);
  printf("vc_set_joint_velocity, ret=%d\n", ret);
  return 0;
}