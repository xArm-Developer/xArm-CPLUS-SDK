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

  int digitals[8] = {0};
  int digitals2[8] = {0};
  float io0 = 0, io1 = 0;
  ret = arm->get_cgpio_digital(digitals, digitals2);
  printf("get_cgpio_digital, ret=%d", ret);
  for (int i = 0; i < 8; ++i) { printf(", io%d=%d", i, digitals[i]); }
  for (int i = 0; i < 8; ++i) { printf(", io%d=%d", i+8, digitals2[i]); }
  printf("\n");
  
  ret = arm->get_cgpio_analog(0, &io0);
  printf("get_cgpio_analog, ret=%d, io=0, val=%f\n", ret, io0);
  ret = arm->get_cgpio_analog(1, &io1);
  printf("get_cgpio_analog, ret=%d, io=1, val=%f\n", ret, io1);

  return 0;
}