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
  fp32 poses[6][6] = {
    {100, 0, 0, 0, 0, 0},
    {0, 200, 0, 0, 0, 0},
    {200, 0, 0, 0, 0, 0},
    {0, -400, 0, 0, 0, 0},
    {-200, 0, 0, 0, 0, 0},
    {0, 200, 0, 0, 0, 0},
  };
  for (int i = 0; i < 6; i++) {
    ret = arm->set_tool_position(poses[i], true);
    printf("set_tool_position, ret=%d\n", ret);
  }

  arm->move_gohome(true);
  return 0;
}