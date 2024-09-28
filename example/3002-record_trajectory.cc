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
  arm->set_mode(2);
  arm->set_state(0);
  ret = arm->start_record_trajectory();
  printf("start_record_trajectory, ret=%d\n", ret);
  sleep_milliseconds(1000 * 20);
  arm->stop_record_trajectory();
  printf("stop_record_trajectory, ret=%d\n", ret);
  std::string filename("test.traj");
  arm->save_record_trajectory((char *)filename.data());
  printf("save_record_trajectory, ret=%d\n", ret);
  sleep_milliseconds(1000);

  arm->set_mode(0);
  arm->set_state(0);

  return 0;
}