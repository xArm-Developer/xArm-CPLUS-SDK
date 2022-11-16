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
  int io0 = -1, io1 = -1, prev_io0 = -1, prev_io1 = -1;
  while (arm->is_connected() && arm->error_code == 0) {
    ret = arm->get_tgpio_digital(&io0, &io1);
    if (ret == 0) {
      if (io0 == 1 && io0 != prev_io0) {
          printf("IO0 input high level\n");
      }
      if (io1 == 1 && io1 != prev_io1) {
          printf("IO1 input high level\n");
      }
      prev_io0 = io0;
      prev_io1 = io1;
    }
    sleep_milliseconds(100);
  }

  return 0;
}