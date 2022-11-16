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

void report_location_callback(const fp32* pose, const fp32* angles) {
  print_nvect("pose    = ", (fp32*)pose, 6);
  print_nvect("angles  = ", (fp32*)angles, 7);
}

void connect_changed_callback(bool connected, bool reported) {
  printf("connected: %d, reported: %d\n", connected, reported);
}

void state_changed_callback(int state) {
  printf("state: %d\n", state);
}

void mode_changed_callback(int mode) {
  printf("mode: %d\n", mode);
}

void mtable_mtbrake_changed_callback(int mtable, int mtbrake) {
  printf("mtable: %d, mtbrake: %d\n", mtable, mtbrake);
}

void error_warn_changed_callback(int err, int warn) {
  printf("err: %d, warn: %d\n", err, warn);
}

void cmdnum_changed_callback(int cmdnum) {
  printf("cmdnum: %d\n", cmdnum);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  std::string port(argv[1]);
  XArmAPI *arm = new XArmAPI(port, false, true, true, true, true, false, true, true, 0);

  printf("register events\n");
  arm->register_report_location_callback(report_location_callback);
  arm->register_connect_changed_callback(connect_changed_callback);
  arm->register_state_changed_callback(state_changed_callback);
  arm->register_mode_changed_callback(mode_changed_callback);
  arm->register_mtable_mtbrake_changed_callback(mtable_mtbrake_changed_callback);
  arm->register_error_warn_changed_callback(error_warn_changed_callback);
  arm->register_cmdnum_changed_callback(cmdnum_changed_callback);

  arm->connect();
  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);
  sleep_milliseconds(100000);

  printf("release events\n");
  arm->release_report_location_callback(report_location_callback);
  arm->release_connect_changed_callback(connect_changed_callback);
  arm->release_state_changed_callback(state_changed_callback);
  arm->release_mode_changed_callback(mode_changed_callback);
  arm->release_mtable_mtbrake_changed_callback(mtable_mtbrake_changed_callback);
  arm->release_error_warn_changed_callback(error_warn_changed_callback);
  arm->release_cmdnum_changed_callback(cmdnum_changed_callback);

  // sleep_milliseconds(30000);
  return 0;
}
