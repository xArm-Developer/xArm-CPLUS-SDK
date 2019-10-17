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

  // if (arm->warn_code != 0) arm->clean_warn();
  // if (arm->error_code != 0) arm->clean_error();

  int ret;
  // ret = arm->motion_enable(true);
  // printf("motion_enable, ret=%d\n", ret);
  // ret = arm->set_mode(0);
  // printf("set_mode, ret=%d\n", ret);
  // ret = arm->set_state(0);
  // printf("set_state, ret=%d\n", ret);
  // sleep_milliseconds(1000);

  int digitals[8];
  ret = arm->get_cgpio_digital(digitals);
  printf("get_cgpio_digital, ret=%d, digitals= ", ret);
  for (int i = 0; i < 8; i++) {
    printf("%d ", digitals[i]);
  }
  printf("\n");
  float analog1, analog2;
  ret = arm->get_cgpio_analog(0, &analog1);
  printf("get_cgpio_analog(0), ret=%d, value=%f\n", ret, analog1);
  ret = arm->get_cgpio_analog(1, &analog2);
  printf("get_cgpio_analog(1), ret=%d, value=%f\n", ret, analog2);

  // for (int i = 0; i < 8; i++) {
  //   ret = arm->set_cgpio_digital(i, 1);
  //   printf("set_cgpio_digital(%d), ret=%d\n", i, ret);
  //   sleep_milliseconds(1000);
  // }
  // for (int i = 0; i < 8; i++) {
  //   ret = arm->set_cgpio_digital(i, 0);
  //   printf("set_cgpio_digital(%d), ret=%d\n", i, ret);
  //   sleep_milliseconds(1000);
  // }

  int states[2];
  int digit_io[4];
  float analog[4];
  int input_conf[8];
  int output_conf[8];
  ret = arm->get_cgpio_state(states, digit_io, analog, input_conf, output_conf);
  printf("get_cgpio_state, ret=%d\n", ret);
  printf("GPIO state: %d\n", states[0]);
  printf("GPIO error code: %d\n", states[1]);
  printf("Digital->Input->FunctionalIO:");
  for (int i = 0; i < 8; i++) {
    printf(" %d", digit_io[0] >> i & 0x0001);
  }
  printf("\n");
  printf("Digital->Input->ConfiguringIO:");
  for (int i = 0; i < 8; i++) {
    printf(" %d", digit_io[1] >> i & 0x0001);
  }
  printf("\n");
  printf("Digital->Output->FunctionalIO:");
  for (int i = 0; i < 8; i++) {
    printf(" %d", digit_io[2] >> i & 0x0001);
  }
  printf("\n");
  printf("Digital->Output->ConfiguringIO:");
  for (int i = 0; i < 8; i++) {
    printf(" %d", digit_io[3] >> i & 0x0001);
  }
  printf("\n");
  printf("Analog->Input: %f %f\n", analog[0], analog[1]);
  printf("Analog->Output: %f %f\n", analog[2], analog[3]);
  printf("Digital->Input->Conf:");
  for (int i = 0; i < 8; i++) {
    printf(" %d", input_conf[i]);
  }
  printf("\n");
  printf("Digital->Output->Conf:");
  for (int i = 0; i < 8; i++) {
    printf(" %d", output_conf[i]);
  }
  printf("\n");

  // for (int i = 0; i < 4; i++) {
  //   ret = arm->set_cgpio_digital_input_function(i, 255);
  //   printf("set_cgpio_digital_input_function, ret=%d\n", ret);
  // }
  // for (int i = 4; i < 8; i++) {
  //   ret = arm->set_cgpio_digital_output_function(i, 255);
  //   printf("set_cgpio_digital_output_function, ret=%d\n", ret);
  // }

  // sleep_milliseconds(3000);
  // arm->disconnect();

  return 0;
}
