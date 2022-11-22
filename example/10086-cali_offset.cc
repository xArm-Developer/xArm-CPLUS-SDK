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
  float zero_offset[6] = {0};
  arm->set_tcp_offset(zero_offset);
  arm->set_world_offset(zero_offset);

  int code, code1, code2;
  float xyz[3] = {0};
  float rpy[3] = {0};
  float pose[6] = {0};
  float rpy_bt[3] = {0, 0, 0};

  float four_points[4][6] = {
    {-26.7, -265, 296.4, 162.2, -9.4, -29.6},
    {106.9, -278, 234.3, -130, 14.7, 92.9},
    {-27.1, -476.5, 215.6, -122.1, -2.9, -2.7},
    {-148.4, -212.2, 201.5, 174.1, -68.7, -39.1}
  };
  code1 = arm->calibrate_tcp_coordinate_offset(four_points, xyz);
  code = arm->get_position(pose);
  code2 = arm->calibrate_tcp_orientation_offset(pose + 3, rpy_bt, rpy);
  printf("code1=%d, code2=%d, code=%d\n", code1, code2, code);
  printf("cali_tcp_offset: [%f, %f, %f, %f, %f, %f]\n", xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);

  float three_points[3][6] = {
    {290, -88, 333, 180, 0, 0},
    {320, -88, 303, 180, 0, 0},
    {320, -158, 303, 180, 0, 0}
  };
  code1 = arm->calibrate_user_orientation_offset(three_points, rpy);
  code = arm->get_position(pose);
  code2 = arm->calibrate_user_coordinate_offset(rpy, pose, xyz);
  printf("code1=%d, code2=%d, code=%d\n", code1, code2, code);
  printf("cali_user_offset: [%f, %f, %f, %f, %f, %f]\n", xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);

  return 0;
}