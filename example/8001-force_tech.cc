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

  // Hand guided Cartesian jogging can be achieved by setting zero stiffness to target axis
  // set teach parmeters
  // Attention: for M, smaller value means less effort to drive the arm, but may also be less stable, please be careful. 
  // x/y/z equivalent mass; range: 0.02 ~ 1 kg
  // Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
  float M[6] = { 0.05, 0.05, 0.05, 0.0005, 0.0005, 0.0005 }; // M => {x, y, z, Rx, Ry, Rz} 
  // x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
  // Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)
  float K[6] = { 0 }; // K => {x, y, z, Rx, Ry, Rz}
  float B[6] = { 0 };
  int coord = 0; // 0 : base , 1 : tool
  int c_axis[6] = { 0, 0, 1, 0, 0, 0 }; // compliant axis: z
  int ret;

  ret = arm->set_impedance_mbk(M, K, B);
  printf("set_impedance_mbk, ret=%d\n", ret);
  ret = arm->set_impedance_config(coord, c_axis);
  printf("set_impedance_config, ret=%d\n", ret);

  // enable ft sensor communication
  ret = arm->ft_sensor_enable(1);
  printf("ft_sensor_enable, ret=%d\n", ret);
  // will overwrite previous sensor zero and payload configuration
  // ret = arm->ft_sensor_set_zero(); // remove this if zero_offset and payload already identified & compensated!
  // printf("ft_sensor_set_zero, ret=%d\n", ret);
  sleep_milliseconds(200); // wait for writting zero operation to take effect, do not remove

  // 1: impendance control mode
  ret = arm->ft_sensor_app_set(1);
  printf("ft_sensor_app_set, ret=%d\n", ret);
  // will start after set_state(0)
  ret = arm->set_state(0);

  // You can drag and teach robot along compliant axis now
  sleep_milliseconds(1000 * 10);

  // remember to reset ft_sensor_app when finished
  ret = arm->ft_sensor_app_set(0);
  printf("ft_sensor_app_set, ret=%d\n", ret);
  ret = arm->ft_sensor_enable(0);
  printf("ft_sensor_enable, ret=%d\n", ret);

  arm->disconnect();

  return 0;
}