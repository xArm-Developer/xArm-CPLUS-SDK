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

  for (int i = 0; i < 8; ++i) {
    ret = arm->set_cgpio_digital(i, 0);
    printf("set_cgpio_digital, ret=%d, io%d=0\n", ret, i);
  }
  sleep_milliseconds(2000);
  
  ret = arm->set_cgpio_analog(0, (fp32)2.6);
  printf("set_cgpio_analog, ret=%d, io=0, val=2.6\n", ret);
  ret = arm->set_cgpio_analog(1, (fp32)3.6);
  printf("set_cgpio_analog, ret=%d, io=1, val=3.6\n", ret);

  sleep_milliseconds(2000);
  for (int i = 0; i < 8; ++i) {
    ret = arm->set_cgpio_digital(i, 1);
    printf("set_cgpio_digital, ret=%d, io%d=1\n", ret, i);
  }

  return 0;
}