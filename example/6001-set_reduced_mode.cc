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

  ret = arm->set_reduced_max_tcp_speed(500);
  printf("set_reduced_max_tcp_speed, ret=%d\n", ret);
  ret = arm->set_reduced_max_joint_speed(100);
  printf("set_reduced_max_tcp_speed, ret=%d\n", ret);
  ret = arm->set_reduced_mode(true);
  printf("set_reduced_mode, ret=%d\n", ret);

  // int reduced_is_on, fense_is_on, rebound_is_on;
  // int xyz_limit[6];
  // float tcp_speed, joint_speed;
  // float jrange[14];
  // ret = arm->get_reduced_states(&reduced_is_on, xyz_limit, &tcp_speed, &joint_speed, jrange, &fense_is_on, &rebound_is_on);
  // printf("get_reduced_states, ret=%d\n", ret);
  // printf("* reduced_is_on: %d\n", reduced_is_on);
  // printf("* max_tcp_speed: %f\n", tcp_speed);
  // printf("* max_joint_speed: %f\n", joint_speed);
  // printf("* jrange:\n");
  // for (int i = 0; i < 7; ++i) {
  //   printf("    Joint-%d: [%f, %f]\n", i, jrange[i*2], jrange[i*2+1]);
  // }
  // printf("* rebound_is_on: %d\n", rebound_is_on);

    return 0;
}