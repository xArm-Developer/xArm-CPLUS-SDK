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

  auto arm = std::make_shared<XArmAPI>(port);
  sleep_milliseconds(500);
  if (arm->error_code != 0) arm->clean_error();
  if (arm->warn_code != 0) arm->clean_warn();
  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);
  sleep_milliseconds(500);

  printf("=========================================\n");

  float zero_offset[6] = {0.0f};
  arm->set_tcp_offset(zero_offset);
  arm->set_world_offset(zero_offset);

  int code, code1, code2;
  float xyz[3] = {0.0f};
  float rpy[3] = {0.0f};
  float pose[6] = {0.0f};
  float rpy_bt[3] = {0.0f, 0.0f, 0.0f};

  float four_points[4][6] = {
    {-26.7f, -265.0f, 296.4f, 162.2f, -9.4f, -29.6f},
    {106.9f, -278.0f, 234.3f, -130.0f, 14.7f, 92.9f},
    {-27.1f, -476.5f, 215.6f, -122.1f, -2.9f, -2.7f},
    {-148.4f, -212.2f, 201.5f, 174.1f, -68.7f, -39.1f}
  };
  code1 = arm->calibrate_tcp_coordinate_offset(four_points, xyz);
  code = arm->get_position(pose);
  code2 = arm->calibrate_tcp_orientation_offset(pose + 3, rpy_bt, rpy);
  printf("code1=%d, code2=%d, code=%d\n", code1, code2, code);
  printf("cali_tcp_offset: [%f, %f, %f, %f, %f, %f]\n", xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);

  float three_points[3][6] = {
    {290.0f, -88.0f, 333.0f, 180.0f, 0.0f, 0.0f},
    {320.0f, -88.0f, 303.0f, 180.0f, 0.0f, 0.0f},
    {320.0f, -158.0f, 303.0f, 180.0f, 0.0f, 0.0f}
  };
  code1 = arm->calibrate_user_orientation_offset(three_points, rpy);
  code = arm->get_position(pose);
  code2 = arm->calibrate_user_coordinate_offset(rpy, pose, xyz);
  printf("code1=%d, code2=%d, code=%d\n", code1, code2, code);
  printf("cali_user_offset: [%f, %f, %f, %f, %f, %f]\n", xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);

  return 0;
}
