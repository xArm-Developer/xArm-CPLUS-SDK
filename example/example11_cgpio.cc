/* Copyright 201９ UFACTORY Inc. All Rights Reserved.
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


  int temp;
  int ret = arm_cmd->cgpio_get_auxdigit(&temp);
  printf("cgpio_get_auxdigit, ret = %d, auxdigit = 0x%x\n", ret, temp);

  float temp_f;
  ret = arm_cmd->cgpio_get_analog1(&temp_f);
  printf("cgpio_get_analog1, ret = %d, analog1 = %f\n", ret, temp_f);
  ret = arm_cmd->cgpio_get_analog2(&temp_f);
  printf("cgpio_get_analog2, ret = %d, analog2 = %f\n", ret, temp_f);

  for (int i = 0; i < 8; i++) {
    ret = arm_cmd->cgpio_set_auxdigit(i, 1);
  }
  printf("cgpio_set_auxdigit, ret = %d\n", ret);

  ret = arm_cmd->cgpio_set_analog1(2.6);
  printf("cgpio_set_analog1, ret = %d\n", ret);
  ret = arm_cmd->cgpio_set_analog2(3.6);
  printf("cgpio_set_analog2, ret = %d\n", ret);


  ret = arm_cmd->cgpio_set_infun(1, 2);
  printf("cgpio_set_infun, ret = %d\n", ret);
  ret = arm_cmd->cgpio_set_outfun(2, 0);
  printf("cgpio_set_outfun, ret = %d\n", ret);

  int state[2], digit_io[4], input_conf[8], output_conf[8];
  float analog[4];
  ret = arm_cmd->cgpio_get_state(state, digit_io, analog, input_conf, output_conf);
  printf("cgpio_get_state, ret = %d\n", ret);
  printf("    state = %d %d\n", state[0], state[1]);
  printf("    digit_io = %x %x %x %x\n", digit_io[0], digit_io[1], digit_io[2], digit_io[3]);
  printf("    analog = %f %f %f %f\n", analog[0], analog[1], analog[2], analog[3]);
  printf("    input_conf = %d %d %d %d %d %d %d %d\n",
         input_conf[0], input_conf[1], input_conf[2], input_conf[3],
         input_conf[4], input_conf[5], input_conf[6], input_conf[7]);
  printf("    output_conf = %d %d %d %d %d %d %d %d\n",
         output_conf[0], output_conf[1], output_conf[2], output_conf[3],
         output_conf[4], output_conf[5], output_conf[6], output_conf[7]);
  arm_cmd->close();
}
