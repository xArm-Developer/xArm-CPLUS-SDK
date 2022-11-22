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

  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);
  sleep_milliseconds(1000);

  printf("=========================================\n");

  int ret = 0;
  unsigned char version[40] = {0};
  ret = arm->get_version(version);
  printf("ret=%d, version: %s\n", ret, version);

  int state = 0;
  ret = arm->get_state(&state);
  printf("ret=%d, state: %d, mode: %d\n", ret, state, arm->mode);

  int cmdnum = 0;
  ret = arm->get_cmdnum(&cmdnum);
  printf("ret=%d, cmdnum: %d\n", ret, cmdnum);

  int err_warn[2] = {0};
  ret = arm->get_err_warn_code(err_warn);
  printf("ret=%d, err: %d, warn: %d\n", ret, err_warn[0], err_warn[1]);

  fp32 pose[6] = {0};
  ret = arm->get_position(pose);
  printf("ret=%d, ", ret);
  print_nvect("pose: ", pose, 6);

  fp32 angles[7] = {0};
  ret = arm->get_servo_angle(angles);
  printf("ret=%d, ", ret);
  print_nvect("angles: ", angles, 7);

  return 0;
}