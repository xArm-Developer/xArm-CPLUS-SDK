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

void iden_progress_changed_callback(int progress) {
  printf("progress: %d\n", progress);
}

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

  arm->register_iden_progress_changed_callback(iden_progress_changed_callback);

  printf("start iden tcp load\n");
  int ret;
  float result[4] = {0};
  ret = arm->iden_tcp_load(result);
  printf("ret=%d, result=[%f, %f, %f, %f]\n", ret, result[0], result[1], result[2], result[3]);

  arm->release_iden_progress_changed_callback(true);

  return 0;
}