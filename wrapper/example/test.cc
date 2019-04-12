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
  XArmAPI *arm = new XArmAPI("192.168.1.145");
  usleep(1000000);

  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);
  // // float pos[6] = {500, 0, 200, 180, 0, 0};
  // // printf("before move: %lld\n", get_system_time());
  // // arm->set_position(pos, true);
  // // printf("after move: %lld\n", get_system_time());
  // // print_nvect("pose    = ", arm->last_used_position, 6);

  // arm->move_gohome(true);

  // float angles[7] = {90, 0, 0, 0, 0, 0, 0};
  // printf("before move: %lld\n", get_system_time());
  // arm->set_servo_angle(angles, true);
  // printf("after move: %lld\n", get_system_time());
  // print_nvect("angles    = ", arm->last_used_angles, 7);
  // angles[1] = -50;
  // arm->set_servo_angle(angles, true);
  // printf("after move: %lld\n", get_system_time());
  // print_nvect("angles    = ", arm->last_used_angles, 7);
  // arm->move_gohome(true);
  // printf("move gohome: %lld\n", get_system_time());

  // float pos[6];
  // float angles[7];
  // unsigned char version[40];
  // int state;
  // int cmdnum;
  // int err_warn[2];

  while (1) {
    usleep(100000);
    // arm->get_position(pos);
    // arm->get_servo_angle(angles);
    // arm->get_version(version);
    // arm->get_state(&state);
    // arm->get_cmdnum(&cmdnum);
    // arm->get_err_warn_code(err_warn);
    // printf("version: %s\n", version);
    // printf("state: %d, cmdnum: %d, err: %d, warn: %d\n", state, cmdnum, err_warn[0], err_warn[1]);
    // print_nvect("pose    = ", pos, 6);
    // print_nvect("angles  = ", angles, 7);
  }
}
