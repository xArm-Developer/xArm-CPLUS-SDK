/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
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

  printf("=========================================\n");
  int ret;
  unsigned char ret_data[6] = { 0 };

  ret = arm->set_tgpio_modbus_baudrate(2000000);

  unsigned char send_enable_data[6] = { 0x08, 0x06, 0x01, 0x00, 0x00, 0x01 };
  ret = arm->getset_tgpio_modbus_data(send_enable_data, 6, ret_data, 6);
  printf("set_bio_gripper_enable, ret=%d, ret_data=", ret);
  for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
  printf("\n");

  int speed = 1000;
  unsigned char send_speed_data[6] = { 0x08, 0x06, 0x03, 0x03, (unsigned char)(speed / 256 % 256), (unsigned char)(speed % 256) };
  ret = arm->getset_tgpio_modbus_data(send_speed_data, 6, ret_data, 6);
  printf("set_bio_gripper_speed, ret=%d, ret_data=", ret);
  for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
  printf("\n");

  unsigned char send_open_data[11] = { 0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x0, 0x0, 0x0, 130 };
  unsigned char send_close_data[11] = { 0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x0, 0x0, 0x0, 50 };

  while (arm->is_connected() && arm->error_code == 0) {
    ret = arm->getset_tgpio_modbus_data(send_open_data, 11, ret_data, 6);
    printf("open_bio_gripper, ret=%d, ret_data=", ret);
    for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
    printf("\n");
    sleep_milliseconds(2000);

    ret = arm->getset_tgpio_modbus_data(send_close_data, 11, ret_data, 6);
    printf("close_bio_gripper, ret=%d, ret_data=", ret);
    for (int i = 0; i < 6; ++i) { printf("%d ", ret_data[i]); }
    printf("\n");
    sleep_milliseconds(2000);
  }

  return 0;
}