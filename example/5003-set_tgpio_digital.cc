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
  ret = arm->set_tgpio_digital(0, 1);
  printf("set_tgpio_digital, io=0, val=1, ret=%d\n", ret);
  sleep_milliseconds(2000);
  ret = arm->set_tgpio_digital(1, 1);
  printf("set_tgpio_digital, io=1, val=1, ret=%d\n", ret);
  sleep_milliseconds(2000);
  ret = arm->set_tgpio_digital(0, 0);
  printf("set_tgpio_digital, io=0, val=0, ret=%d\n", ret);
  sleep_milliseconds(2000);
  ret = arm->set_tgpio_digital(1, 0);
  printf("set_tgpio_digital, io=1, val=0, ret=%d\n", ret);

  return 0;
}