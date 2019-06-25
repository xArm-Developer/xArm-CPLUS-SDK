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
  int io1, io2;
  int last_digitals[2] = {-1, -1};
  while (arm->is_connected() && arm->error_code != 28 && arm->error_code != 19) {
    ret = arm->get_tgpio_digital(&io1, &io2);
    if (last_digitals[0] == 1 && io1 != last_digitals[0]) {
      printf("IO1 output high level\n");
    }
    if (last_digitals[1] == 1 && io2 != last_digitals[1]) {
      printf("IO2 output high level\n");
    }
    last_digitals[0] = io1;
    last_digitals[1] = io2;
    sleep_milliseconds(100);
  }

  return 0;
}
