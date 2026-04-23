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
  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);
  sleep_milliseconds(500);

  printf("=========================================\n");

  // set tool admittance parmeters
  // Attention: for M, smaller value means less effort to drive the arm, but may also be less stable, please be careful. 
  // x/y/z equivalent mass; range: 0.02 ~ 1 kg
  // Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
  float M[6] = { 0.06f, 0.06f, 0.06f, 0.0006f, 0.0006f, 0.0006f }; // M => {x, y, z, Rx, Ry, Rz} 
  // x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
  // Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)
  float K[6] = { 300, 300, 300, 4, 4, 4 }; // K => {x, y, z, Rx, Ry, Rz}
  float B[6] = { 0 };
  int coord = 0; // 0 : base , 1 : tool
  int c_axis[6] = { 0, 0, 1, 0, 0, 0 }; // set z axis as compliant axis
  int ret;

  ret = arm->set_ft_sensor_admittance_parameters(M, K, B);
  printf("set_ft_sensor_admittance_parameters(mbk), ret=%d\n", ret);
  ret = arm->set_ft_sensor_admittance_parameters(coord, c_axis);
  printf("set_ft_sensor_admittance_parameters, ret=%d\n", ret);

  // enable ft sensor communication
  ret = arm->set_ft_sensor_enable(1);
  printf("set_ft_sensor_enable, ret=%d\n", ret);
  // will overwrite previous sensor zero and payload configuration
  // ret = arm->set_ft_sensor_zero(); // remove this if zero_offset and payload already identified & compensated!
  // printf("set_ft_sensor_zero, ret=%d\n", ret);
  sleep_milliseconds(200); // wait for writting zero operation to take effect, do not remove

  // move robot in admittance control application
  ret = arm->set_ft_sensor_mode(1);
  printf("set_ft_sensor_mode, ret=%d\n", ret);
  // will start after set_state(0)
  ret = arm->set_state(0);

  // compliance effective for 10 secs, you may send position command to robot, or just keep it still
  sleep_milliseconds(1000 * 10);

  // remember to reset ft_sensor_app when finished
  ret = arm->set_ft_sensor_mode(0);
  printf("set_ft_sensor_mode, ret=%d\n", ret);
  ret = arm->set_ft_sensor_enable(0);
  printf("set_ft_sensor_enable, ret=%d\n", ret);

  arm->disconnect();

  return 0;
}