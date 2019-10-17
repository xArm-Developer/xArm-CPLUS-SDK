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

  int ret;
  unsigned char version[40];
  ret = arm->get_version(version);
  printf("ret=%d, version: %s\n", ret, version);

  int state;
  ret = arm->get_state(&state);
  printf("ret=%d, state: %d, mode: %d\n", ret, state, arm->mode);

  int cmdnum;
  ret = arm->get_cmdnum(&cmdnum);
  printf("ret=%d, cmdnum: %d\n", ret, cmdnum);

  int err_warn[2];
  ret = arm->get_err_warn_code(err_warn);
  printf("ret=%d, err: %d, warn: %d\n", ret, err_warn[0], err_warn[1]);

  fp32 pose[6];
  ret = arm->get_position(pose);
  printf("ret=%d, ", ret);
  print_nvect("pose: ", pose, 6);

  fp32 angles[7];
  ret = arm->get_servo_angle(angles);
  printf("ret=%d, ", ret);
  print_nvect("angles: ", angles, 7);

  printf("=======================================\n");

  printf("version: %s\n", arm->version);
  printf("state: %d\n", arm->state);
  printf("mode: %d\n", arm->mode);
  printf("cmdnum: %d\n", arm->cmd_num);
  printf("err: %d, warn: %d\n", arm->error_code, arm->warn_code);
  print_nvect("pose: ", arm->position, 6);
  print_nvect("angles: ", arm->angles, 7);

  unsigned char v[3];

  ret = arm->get_servo_version(v);
  printf("ret=%d, servo_version=%d.%d.%d\n", ret, v[0], v[1], v[2]);

  ret = arm->get_tgpio_version(v);
  printf("ret=%d, tgpio_version=%d.%d.%d\n", ret, v[0], v[1], v[2]);

  ret = arm->get_gripper_version(v);
  printf("ret=%d, gripper_version=%d.%d.%d\n", ret, v[0], v[1], v[2]);

  int suctioncup_state;
  ret = arm->get_suction_cup_state(&suctioncup_state);
  printf("ret=%d, suction_cup_state=%d\n", ret, suctioncup_state);

  ret = arm->set_suction_cup(true);
  printf("ret=%d, set_suction_cup\n", ret);

  ret = arm->get_suction_cup_state(&suctioncup_state);
  printf("ret=%d, suction_cup_state=%d\n", ret, suctioncup_state);

  int on;
  float tcp_speed;
  float joint_speed;
  int xyz_list[6];
  ret = arm->get_reduced_states(&on, xyz_list, &tcp_speed, &joint_speed);
  printf("state: %d, tcp_speed: %f, joint_speed: %f\n", on, tcp_speed, joint_speed);
  print_nvect("xyz: ", xyz_list, 6);

  return 0;
}
