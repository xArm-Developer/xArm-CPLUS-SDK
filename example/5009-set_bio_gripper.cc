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

  printf("=========================================\n");
  int ret;

  ret = arm->set_bio_gripper_enable(true);
  printf("set_bio_gripper_enable, ret=%d\n", ret);

  sleep_milliseconds(3000);

  ret = arm->set_bio_gripper_speed(500);
  printf("set_bio_gripper_speed, ret=%d\n", ret);

  while (arm->is_connected()) {
    ret = arm->open_bio_gripper(true);
    printf("open_bio_gripper, ret=%d\n", ret);

    ret = arm->close_bio_gripper(true);
    printf("close_bio_gripper, ret=%d\n", ret);
  }

  return 0;
}