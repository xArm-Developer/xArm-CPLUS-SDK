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

  printf("start ft_sensor iden load\n");
  int ret;
  float result[10] = {0};
  ret = arm->ft_sensor_enable(1);
  printf("ft_sensor_enable, ret=%d\n", ret);
  ret = arm->ft_sensor_iden_load(result);
  printf("ret=%d, result=[", ret);
  for (unsigned int i = 0; i < 10; i++) {
    printf("%f%s", result[i], i == 9 ? "" : ", ");
  }
  printf("]\n");

  arm->release_iden_progress_changed_callback(true);

  ret = arm->ft_sensor_app_set(0);
  printf("ft_sensor_app_set, ret=%d\n", ret);
  ret = arm->ft_sensor_enable(0);
  printf("ft_sensor_enable, ret=%d\n", ret);

  arm->disconnect();

  return 0;
}