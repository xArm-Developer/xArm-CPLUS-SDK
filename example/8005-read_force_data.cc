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
  ret = arm->ft_sensor_enable(1);
  printf("ft_sensor_enable, ret=%d\n", ret);
  int err;

  while (arm->is_connected() && arm->error_code == 0) {
    print_nvect("raw_force: ", arm->ft_raw_force, 6);
    print_nvect("ext_force: ", arm->ft_ext_force, 6);
    sleep_milliseconds(200);
  }


  ret = arm->ft_sensor_enable(0);
  printf("ft_sensor_enable, ret=%d\n", ret);

  arm->disconnect();

  return 0;
}