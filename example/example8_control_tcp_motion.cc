/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include <unistd.h>

#include "xarm/connect.h"
#include "xarm/instruction/uxbus_cmd_config.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  char *server_ip = argv[1];
  UxbusCmd *arm_cmd = connect_tcp_control(server_ip);
  if (arm_cmd == NULL) return 0;

  int ret = arm_cmd->motion_en(8, 1);
  printf("motion_en, ret = %d\n", ret);

  ret = arm_cmd->set_mode(XARM_MODE::SERVO);
  printf("set_state, ret = %d\n", ret);

  ret = arm_cmd->set_state(XARM_STATE::START);
  printf("set_state, ret = %d\n", ret);

  float joint[2][7] = {{0, 0, 0, 0, 0, 0, 0}, {0, -1.7, 0, 0, 0, 0, 0}};

  ret = arm_cmd->move_servoj(joint[0], 50, 100, 0);
  printf("move_servoj, ret = %d\n", ret);

  sleep(2);
  ret = arm_cmd->move_servoj(joint[1], 50, 100, 0);
  printf("move_servoj, ret = %d\n", ret);

  arm_cmd->close();
}
