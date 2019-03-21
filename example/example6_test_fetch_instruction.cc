/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include <unistd.h>

#include "xarm/connect.h"
#include "xarm/debug/debug_print.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  char *server_ip = argv[1];
  // UxbusCmd *arm_cmd = connect_rs485_control(server_ip);
  UxbusCmd *arm_cmd = connect_tcp_control(server_ip);
  if (arm_cmd == NULL) return 0;

  u8 rx_data[40];
  int ret;
  ret = arm_cmd->get_version(rx_data);
  printf("get_version = [%s], ret = %d\n", rx_data, ret);

  ret = arm_cmd->get_state(rx_data);
  printf("get_state = [%d], ret = %d\n", rx_data[0], ret);

  u16 rx_data_u16;
  ret = arm_cmd->get_cmdnum(&rx_data_u16);
  printf("get_cmdnum = [%d], ret = %d\n", rx_data_u16, ret);

  ret = arm_cmd->get_errcode(rx_data);
  printf("get_errcode = [%d %d], ret = %d\n", rx_data[0], rx_data[1], ret);

  fp32 angles[7], pose[6];
  ret = arm_cmd->get_tcp_pose(pose);
  printf("get_tcp_pose, ret = %d\n", ret);
  print_nvect("get_tcp_pose = ", pose, 6);

  ret = arm_cmd->get_joint_pose(angles);
  printf("get_joint_pose, ret = %d\n", ret);
  print_nvect("get_joint_pose = ", angles, 7);

  ret = arm_cmd->get_ik(pose, angles);
  printf("get_ik, ret = %d\n", ret);
  print_nvect("get_ik = ", angles, 7);

  ret = arm_cmd->get_fk(angles, pose);
  printf("get_fk, ret = %d\n", ret);
  print_nvect("get_fk = ", pose, 6);

  int value;
  ret = arm_cmd->is_joint_limit(angles, &value);
  printf("is_joint_limit = %d, ret = %d\n", value, ret);

  ret = arm_cmd->is_tcp_limit(pose, &value);
  printf("is_tcp_limit = %d, ret = %d\n", value, ret);

  arm_cmd->close();
  while (1) sleep(1);
}
