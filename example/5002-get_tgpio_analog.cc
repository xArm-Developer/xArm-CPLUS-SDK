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
  int ret1, ret2;
  float io0, io1;
  while (arm->is_connected() && arm->error_code == 0) {
    ret1 = arm->get_tgpio_analog(0, &io0);
    ret2 = arm->get_tgpio_analog(1, &io1);
    printf("get_tgpio_analog, ret1=%d, io0=%f, ret2=%d, io1=%f\n", ret1, io0, ret2, io1);
    sleep_milliseconds(1000);
  }

  return 0;
}