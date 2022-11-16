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
  unsigned char ret_data[6] = { 0 };

  ret = arm->robotiq_reset(ret_data);
  printf("robotiq_reset, ret=%d, ret_data=", ret);
  for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
  printf("\n");

  ret = arm->robotiq_set_activate(true, ret_data);
  printf("robotiq_set_activate, ret=%d, ret_data=", ret);
  for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
  printf("\n");

  unsigned char robotiq_status[9];
  ret = arm->robotiq_get_status(robotiq_status);
  printf("robotiq_get_status, ret=%d, ret_data=", ret);
  for (int i = 0; i < 9; ++i) { printf("%d ", robotiq_status[i]); }
  printf("\n");

  while (arm->is_connected()) {
    ret = arm->robotiq_close(true, ret_data);
    printf("robotiq_close, ret=%d, ret_data=", ret);
    for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
    printf("\n");

    ret = arm->robotiq_open(true, ret_data);
    printf("robotiq_open, ret=%d, ret_data=", ret);
    for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
    printf("\n");
  }

  return 0;
}