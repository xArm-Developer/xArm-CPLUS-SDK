/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2023, UFACTORY, Inc.
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
  
  unsigned char bits[16];

  // get cgpio digital input
  ret = arm->read_input_bits(0x00, 16, bits);
  printf("ret=%d, ", ret);
  print_nvect("cgpio_digital_input: ", bits, 16);

  unsigned char w_bits[16] = {0};
  ret = arm->write_multiple_coil_bits(0x00, 16, w_bits);
  printf("write_multiple_coil_bits, ret=%d\n", ret);

  // get cgpio digital output
  ret = arm->read_coil_bits(0x00, 16, bits);
  printf("ret=%d, ", ret);
  print_nvect("cgpio_digital_output: ", bits, 16);

  unsigned char w_bits2[8] = {1, 0, 1, 0, 1, 1, 0, 1};
  ret = arm->write_multiple_coil_bits(0x00, 8, w_bits2);
  printf("write_multiple_coil_bits, ret=%d\n", ret);

  // get cgpio digital output
  ret = arm->read_coil_bits(0x00, 16, bits);
  printf("ret=%d, ", ret);
  print_nvect("cgpio_digital_output: ", bits, 16);

  int regs[9];
  fp32 tcp_pose[9];
  ret = arm->read_input_registers(0x40, 9, regs, true);
  printf("ret=%d, ", ret);
  for (int i = 0; i < 9; i++) {
    tcp_pose[i] = regs[i] / 10.0;
  }
  print_nvect("x/y/z/roll/pitch/yaw/rx/ry/rz: ", tcp_pose, 9);

  return 0;
}