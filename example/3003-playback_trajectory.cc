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

  auto arm = std::make_shared<XArmAPI>(port);
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
  std::string filename("test.traj");
  ret = arm->load_trajectory(filename.c_str());
  printf("load_trajectory, ret=%d\n", ret);
  ret = arm->playback_trajectory(1, nullptr, true);
  printf("playback_trajectory, ret=%d\n", ret);

  return 0;
}