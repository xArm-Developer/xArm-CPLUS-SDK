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

  int boundary[6] = {500, -500, 500, -500, 600, -400};
  ret = arm->set_reduced_tcp_boundary(boundary);
  printf("set_reduced_tcp_boundary, ret=%d\n", ret);
  ret = arm->set_fense_mode(true);
  printf("set_fense_mode, ret=%d\n", ret);

  // int reduced_is_on, fense_is_on, rebound_is_on;
  // int xyz_limit[6];
  // float tcp_speed, joint_speed;
  // float jrange[14];
  // ret = arm->get_reduced_states(&reduced_is_on, xyz_limit, &tcp_speed, &joint_speed, jrange, &fense_is_on, &rebound_is_on);
  // printf("get_reduced_states, ret=%d\n", ret);
  // printf("* fense_is_on: %d\n", fense_is_on);
  // printf("* boundary:\n");
  // printf("     x=[%d, %d]\n", xyz_limit[0], xyz_limit[1]);
  // printf("     y=[%d, %d]\n", xyz_limit[2], xyz_limit[3]);
  // printf("     z=[%d, %d]\n", xyz_limit[4], xyz_limit[5]);
  // printf("* rebound_is_on: %d\n", rebound_is_on);

  return 0;
}