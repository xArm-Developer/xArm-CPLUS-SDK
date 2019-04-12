/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/

#include "xarm/wrapper/xarm_api.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  std::string port(argv[1]);

  XArmAPI *arm = new XArmAPI(port);
  sleep_milliseconds(1000);

  if (arm->warn_code != 0) arm->clean_warn();
  if (arm->error_code != 0) arm->clean_error();

  int ret;

  float io1, io2;
  float last_analogs[2] = {-1, -1};
  while (arm->is_connected() && arm->error_code != 28) {
    ret = arm->get_tgpio_analog(1, &io1);
    printf("ret: %d, analog(io1): %f\n", ret, io1);
    sleep_milliseconds(200);
    ret = arm->get_tgpio_analog(2, &io2);
    printf("ret: %d, analog(io2): %f\n", ret, io2);
    sleep_milliseconds(500);
  }

  return 0;
}
