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
  if (arm_cmd == NULL) { return 0; }

  int ret;
  int temp_value[2];
  float value_fp32;

  ret = arm_cmd->clean_err();
  printf("clean_err, ret = %d\n", ret);
  ret = arm_cmd->clean_war();
  printf("clean_war, ret = %d\n", ret);

  ret = arm_cmd->gripper_get_pos(&value_fp32);
  printf("gripper_get_pos = [%f], ret = %d\n", value_fp32, ret);

  ret = arm_cmd->gripper_get_errcode(temp_value);
  printf("gripper_get_errcode = [%d %d], ret = %d\n", temp_value[0], temp_value[1],
         ret);

  ret = arm_cmd->gripper_clean_err();
  printf("gripper_clean_err, ret = %d\n", ret);

  ret = arm_cmd->gripper_get_errcode(temp_value);
  printf("gripper_get_errcode = [%d %d], ret = %d\n", temp_value[0], temp_value[1],
         ret);

  ret = arm_cmd->gripper_set_en(1);
  printf("gripper_set_en, ret = %d\n", ret);

  // ret = arm_cmd->gripper_set_mode(u16 value);
  // ret = arm_cmd->gripper_set_zero(void);

  ret = arm_cmd->gripper_set_posspd(8000);
  printf("gripper_set_posspd, ret = %d\n", ret);

  ret = arm_cmd->gripper_set_pos(100);
  printf("gripper_set_pos, ret = %d\n", ret);

  usleep(2000000);
  ret = arm_cmd->gripper_get_pos(&value_fp32);
  printf("gripper_get_pos = [%f], ret = %d\n", value_fp32, ret);

  arm_cmd->close();
  while (1) { sleep(1); }
}
