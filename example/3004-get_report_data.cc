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
  if (argc < 3) {
    printf("Usage: %s robot_ip report_port(30001/30002/30003)\n", argv[0]);
    return 0;
  }
  std::string robot_ip(argv[1]);
  int report_port = atoi(argv[2]);

  SocketPort *sock = new SocketPort((char*)robot_ip.c_str(), report_port, 10, 320, 1);
  if (sock->is_ok() != 0) {
    fprintf(stderr, "Error: Tcp Report connection failed\n");
    return -1;
  }

  int code = 0;
  int total = 0;
  float report_pose[6];
  float report_angles[7];
  unsigned char buf[256];
  unsigned char *data_fp;
  while (sock->is_ok() == 0)
  {
    if (sock->read_frame(buf) == 0)
    {
      data_fp = &buf[4];
      total = bin8_to_32(data_fp);
      hex_to_nfp32(&data_fp[7], report_angles, 7);
      hex_to_nfp32(&data_fp[35], report_pose, 6);
      printf("total: %d, ms: %lld\n", total, get_system_time());
      print_nvect("pose: ", report_pose, 6);
      print_nvect("angles: ", report_angles, 7);
    }
    else {
      sleep_milliseconds(1);
    }
  }
  printf("sock is disconnect\n");

  return 0;
}