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

  auto arm = std::make_shared<XArmAPI>(port);
  sleep_milliseconds(500);
  if (arm->error_code != 0) arm->clean_error();
  if (arm->warn_code != 0) arm->clean_warn();

  printf("=========================================\n");
  int ret;

  ret = arm->set_tgpio_modbus_timeout(20);
  printf("set_tgpio_modbus_timeout, ret=%d\n", ret);
  ret = arm->set_tgpio_modbus_baudrate(115200);
  printf("set_tgpio_modbus_baudrate, ret=%d\n", ret);
  sleep_milliseconds(2000);

  unsigned char recv_data[100] = {0};
  unsigned char modbus_data_1[6] = { 0x01, 0x06, 0x00, 0x0A, 0x00, 0x03 };
  unsigned char modbus_data_2[6] = { 0x01, 0x06, 0x00, 0x0A, 0x03, 0x60 };

  while (arm->is_connected()) {
    ret = arm->getset_tgpio_modbus_data(modbus_data_1, 6, recv_data, 100);
    printf("getset_tgpio_modbus_data, ret=%d\n", ret);
    printf("recv_data:");
    for (int i = 0; i < 100; ++i) { printf("%c ", recv_data[i]); }
    printf("\n");

    sleep_milliseconds(2000);

    ret = arm->getset_tgpio_modbus_data(modbus_data_2, 6, recv_data, 100);
    printf("getset_tgpio_modbus_data, ret=%d\n", ret);
    printf("recv_data:");
    for (int i = 0; i < 100; ++i) { printf("%c ", recv_data[i]); }
    printf("\n");

    sleep_milliseconds(2000);
  }

  return 0;
}