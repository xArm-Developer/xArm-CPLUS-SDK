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


void count_changed_callback(int cmdnum) {
  printf("counter val: %d\n", cmdnum);
}


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

  arm->register_count_changed_callback(count_changed_callback);

  int ret;
  arm->move_gohome(true);
  fp32 poses[6][6] = {
    {300, 0, 200, 180, 0, 0},
    {300, 200, 200, 180, 0, 0},
    {500, 200, 200, 180, 0, 0},
    {500, -200, 200, 180, 0, 0},
    {300, -200, 200, 180, 0, 0},
    {300, 0, 200, 180, 0, 0},
  };
  fp32 radius = 0;
  arm->set_counter_reset();
  arm->set_pause_time((fp32)0.2);
  for (int j = 0; j < 10; j++) {
    for (int i = 0; i < 6; i++) {
      ret = arm->set_position(poses[i], radius, false);
      printf("set_position, ret=%d\n", ret);
    }
    arm->set_counter_increase();
  }

  arm->move_gohome(true);
  arm->release_count_changed_callback(count_changed_callback);
  return 0;
}