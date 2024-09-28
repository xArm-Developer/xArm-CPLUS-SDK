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
  fp32 poses[5][6] = {
    {300, 0, 100, -180, 0, 0},
    {300, 100, 100, -180, 0, 0},
    {400, 100, 100, -180, 0, 0},
    {400, -100, 100, -180, 0, 0},
    {300, 0, 300, -180, 0, 0},
  };
  ret = arm->set_position(poses[0], true);
  printf("set_position, ret=%d\n", ret);
  ret = arm->move_circle(poses[1], poses[2], 50, 200, 1000, 0, true);
  printf("move_circle, ret=%d\n", ret);
  ret = arm->move_circle(poses[3], poses[4], 200, 200, 1000, 0, true);
  printf("move_circle, ret=%d\n", ret);

  arm->move_gohome(true);
  return 0;
}