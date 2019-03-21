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
    printf("Please enter com port\n");
    return 0;
  }
  char *server_com = argv[1];
  UxbusCmd *arm_cmd = connect_rs485_control(server_com);
  if (arm_cmd == NULL) return 0;

  int ret = arm_cmd->motion_en(8, 1);
  printf("motion_en, ret = %d\n", ret);

  ret = arm_cmd->set_mode(XARM_MODE::POSE);
  printf("set_state, ret = %d\n", ret);

  ret = arm_cmd->set_state(XARM_STATE::START);
  printf("set_state, ret = %d\n", ret);

  ret = arm_cmd->move_gohome(10.0 / 57.0, 100, 0);
  printf("move_gohome, ret = %d\n", ret);

  ret = arm_cmd->sleep_instruction(0.1);
  printf("sleep_instruction, ret = %d\n", ret);

  float pose[5][6] = {{300, 0, 100, -3.14, 0, 0},
                      {300, 100, 100, -3.14, 0, 0},
                      {400, 100, 100, -3.14, 0, 0},
                      {400, -100, 100, -3.14, 0, 0},
                      {300, 0, 100, -3.14, 0, 0}};

  for (int i = 0; i < 5; i++) {
    ret = arm_cmd->move_lineb(pose[i], 50, 100, 0, 0);
    printf("move_lineb, ret = %d\n", ret);
  }

  arm_cmd->close();
  while (1) {
    sleep(1);
  }
}
