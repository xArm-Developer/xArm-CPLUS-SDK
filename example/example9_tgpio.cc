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
  if (arm_cmd == NULL) { return 0; }

  int io1, io2;
  int ret = arm_cmd->tgpio_get_digital(&io1, &io2);
  printf("gpio_get_digital, ret = %d, io1 = %d, io2 = %d\n", ret, io1, io2);
  sleep(0.5);

  ret = arm_cmd->tgpio_set_digital(1, 1);
  printf("gpio_set_digital, ret = %d\n", ret);
  sleep(0.5);

  float value;
  ret = arm_cmd->tgpio_get_analog1(&value);
  printf("gpio_get_analog1, ret = %d, value = %f\n", ret, value);
  sleep(0.5);

  ret = arm_cmd->tgpio_get_analog2(&value);
  printf("gpio_get_analog2, ret = %d, value = %f\n", ret, value);

  arm_cmd->close();
}
