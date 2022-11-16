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
  sleep_milliseconds(500);

  printf("=========================================\n");

  float kp[6] = { 0.005, 0.005, 0.005, 0.005, 0.005, 0.005 }; // range: 0 ~ 0.05
  float ki[6] = { 0.00006, 0.00006, 0.00006, 0.00006, 0.00006, 0.00006 }; // range: 0 ~ 0.0005
  float kd[6] = { 0 }; // range: 0 ~ 0.05
  float xe_limits[6] = { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 }; // max adjust velocity(mm/s), range: 0 ~ 200

  int coord = 1; // 0 : base , 1 : tool
  int c_axis[6] = { 0, 0, 1, 0, 0, 0 }; // only control force along z axis
  // MAKE SURE reference frame and force taget sign are correct !!
  float f_ref[6] = { 0, 0, 5.0, 0, 0, 0 }; // the force(N) that xArm will apply to the environment
  float limits[6] = { 0 }; // limits are reserved, just give zeros
  int ret;

  // set pid parmeters for force control
  ret = arm->set_force_control_pid(kp, ki, kd, xe_limits);
  printf("set_force_control_pid, ret=%d\n", ret);

  ret = arm->config_force_control(coord, c_axis, f_ref, limits);
  printf("config_force_control, ret=%d\n", ret);

  // enable ft sensor communication
  ret = arm->ft_sensor_enable(1);
  printf("ft_sensor_enable, ret=%d\n", ret);
  // will overwrite previous sensor zero and payload configuration
  ret = arm->ft_sensor_set_zero(); // remove this if zero_offset and payload already identified & compensated!
  printf("ft_sensor_set_zero, ret=%d\n", ret);
  sleep_milliseconds(200); // wait for writting zero operation to take effect, do not remove

  // move robot in force control
  ret = arm->ft_sensor_app_set(2);
  printf("ft_sensor_app_set, ret=%d\n", ret);
  // will start after set_state(0)
  ret = arm->set_state(0);

  // keep for 5 secs
  sleep_milliseconds(1000 * 5);

  // remember to reset ft_sensor_app when finished
  ret = arm->ft_sensor_app_set(0);
  printf("ft_sensor_app_set, ret=%d\n", ret);
  ret = arm->ft_sensor_enable(0);
  printf("ft_sensor_enable, ret=%d\n", ret);

  arm->disconnect();

  return 0;
}