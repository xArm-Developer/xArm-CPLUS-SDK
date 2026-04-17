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

int XArmAPI::set_ft_sensor_admittance_parameters(int coord, int c_axis[6], float M[6], float K[6], float B[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_admittance(coord, c_axis, M, K, B);
  return _check_code(ret);
}

int XArmAPI::set_ft_sensor_admittance_parameters(int coord, int c_axis[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_admittance_config(coord, c_axis);
  return _check_code(ret);
}
int XArmAPI::set_ft_sensor_admittance_parameters(float M[6], float K[6], float B[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_admittance_mbk(M, K, B);
  return _check_code(ret);
}

int XArmAPI::set_ft_sensor_force_parameters(int coord, int c_axis[6], float f_ref[6], float limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret1 = core->config_force_control(coord, c_axis, f_ref, limits);
  ret1 =  _check_code(ret1);
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret2 = core->set_force_control_pid(kp, ki, kd, xe_limit);
  ret2 = _check_code(ret2);
  return ret2 ? ret2 : ret1;
}
int XArmAPI::set_ft_sensor_force_parameters(int coord, int c_axis[6], float f_ref[6], float limits[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->config_force_control(coord, c_axis, f_ref, limits);
  return _check_code(ret);
}
int XArmAPI::set_ft_sensor_force_parameters(float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_force_control_pid(kp, ki, kd, xe_limit);
  return _check_code(ret);
}

// int XArmAPI::set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6])
// {
//   return set_ft_sensor_admittance_parameters(coord, c_axis, M, K, B);
//   // if (!is_connected()) return API_CODE::NOT_CONNECTED;
//   // int ret = core->set_admittance(coord, c_axis, M, K, B);
//   // return _check_code(ret);
// }

// int XArmAPI::set_impedance_mbk(float M[6], float K[6], float B[6])
// {
//   return set_ft_sensor_admittance_parameters(M, K, B);
//   // if (!is_connected()) return API_CODE::NOT_CONNECTED;
//   // int ret = core->set_admittance_mbk(M, K, B);
//   // return _check_code(ret);
// }

// int XArmAPI::set_impedance_config(int coord, int c_axis[6])
// {
//   return set_ft_sensor_admittance_parameters(coord, c_axis);
//   // if (!is_connected()) return API_CODE::NOT_CONNECTED;
//   // int ret = core->set_admittance_config(coord, c_axis);
//   // return _check_code(ret);
// }

// int XArmAPI::config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6])
// {
//   return set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits);
//   // if (!is_connected()) return API_CODE::NOT_CONNECTED;
//   // int ret = core->config_force_control(coord, c_axis, f_ref, limits);
//   // return _check_code(ret);
// }

// int XArmAPI::set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6])
// {
//   return set_ft_sensor_force_parameters(kp, ki, kd, xe_limit);
//   // if (!is_connected()) return API_CODE::NOT_CONNECTED;
//   // int ret = core->set_force_control_pid(kp, ki, kd, xe_limit);
//   // return _check_code(ret);
// }

int XArmAPI::set_ft_sensor_zero(void)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->ft_sensor_set_zero();
  return _check_code(ret);
}

int XArmAPI::iden_ft_sensor_load_offset(float result[10])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int protocol_identifier = core->get_protocol_identifier();
  core->set_protocol_identifier(2);
  keep_heart_ = false;
  int ret = core->ft_sensor_iden_load(result);
  if (ret == 0) {
    result[1] = result[1] * (float)1000.0; // x_centroid, m to mm
    result[2] = result[2] * (float)1000.0; // y_centroid, m to mm
    result[3] = result[3] * (float)1000.0; // z_centroid, m to mm
  }
  core->set_protocol_identifier(protocol_identifier);
  keep_heart_ = true;
  return _check_code(ret);
}

int XArmAPI::set_ft_sensor_load_offset(float load2[10], bool association_setting_tcp_load, float m, float x, float y, float z)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  float load[10];
  memcpy(load, load2, sizeof(float) * 10);
  load[1] = load[1] / (float)1000.0; // x_centroid, mm to m
  load[2] = load[2] / (float)1000.0; // y_centroid, mm to m
  load[3] = load[3] / (float)1000.0; // z_centroid, mm to m
  int ret = core->ft_sensor_cali_load(load);
  ret = _check_code(ret);
  if (ret == 0 && association_setting_tcp_load) {
    float mass = load[0] + m;
    float center_of_gravity[3] = {
      (m * x + load[0] * load[1]) / mass,
      (m * y + load[0] * load[2]) / mass,
      (m * z + load[0] * (32 + load[3])) / mass
    };
    set_state(0);
    return set_tcp_load(mass, center_of_gravity);
  }
  return ret;
}

int XArmAPI::set_ft_sensor_enable(int on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->ft_sensor_enable(on_off);
  return _check_code(ret);
}

int XArmAPI::set_ft_sensor_mode(int mode)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->ft_sensor_app_set(mode);
  return _check_code(ret);
}

int XArmAPI::get_ft_sensor_mode(int *mode)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->ft_sensor_app_get(mode);
  return _check_code(ret);
}

int XArmAPI::get_ft_sensor_data(float ft_data[6], bool is_raw)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  is_raw = _version_is_ge(2, 6, 109) ? is_raw : false;
  int ret = core->ft_sensor_get_data(ft_data, _version_is_ge(1, 8, 3), is_raw);
  return _check_code(ret);
}

int XArmAPI::get_ft_sensor_config(int *ft_mode, int *ft_is_started, int *ft_type, int *ft_id, int *ft_freq, 
  float *ft_mass, float *ft_dir_bias, float ft_centroid[3], float ft_zero[6], int *imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6],
  int *f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->ft_sensor_get_config(ft_mode, ft_is_started, ft_type, ft_id, ft_freq,
    ft_mass, ft_dir_bias, ft_centroid, ft_zero, imp_coord, imp_c_axis, M, K, B,
    f_coord, f_c_axis, f_ref, f_limits, kp, ki, kd, xe_limit
  );
  return _check_code(ret);
}

int XArmAPI::get_ft_sensor_error(int *err)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->ft_sensor_get_error(err, _version_is_ge(2, 7, 100));
  return _check_code(ret);
}
