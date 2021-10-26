/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"

int XArmAPI::set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_impedance(coord, c_axis, M, K, B);
	return _check_code(ret);
}

int XArmAPI::set_impedance_mbk(float M[6], float K[6], float B[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_impedance_mbk(M, K, B);
	return _check_code(ret);
}

int XArmAPI::set_impedance_config(int coord, int c_axis[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_impedance_config(coord, c_axis);
	return _check_code(ret);
}

int XArmAPI::config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->config_force_control(coord, c_axis, f_ref, limits);
	return _check_code(ret);
}

int XArmAPI::set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_force_control_pid(kp, ki, kd, xe_limit);
	return _check_code(ret);
}

int XArmAPI::ft_sensor_set_zero(void)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_set_zero();
	return _check_code(ret);
}

int XArmAPI::ft_sensor_iden_load(float result[10])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_iden_load(result);
	return _check_code(ret);
}

int XArmAPI::ft_sensor_cali_load(float load[10], bool association_setting_tcp_load, float m, float x, float y, float z)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_cali_load(load);
	ret = _check_code(ret);
	if (ret == 0 && association_setting_tcp_load) {
		float mass = load[0] + m;
		float center_of_gravity[3] = {
			(m * x + load[0] * load[1]) / mass,
			(m * y + load[0] * load[2]) / mass,
			(m * z + load[0] * (32 + load[3])) / mass
		};
		return set_tcp_load(mass, center_of_gravity);
	}
	return ret;
}

int XArmAPI::ft_sensor_enable(int on_off)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_enable(on_off);
	return _check_code(ret);
}

int XArmAPI::ft_sensor_app_set(int app_code)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_app_set(app_code);
	return _check_code(ret);
}

int XArmAPI::ft_sensor_app_get(int *app_code)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_app_get(app_code);
	return _check_code(ret);
}

int XArmAPI::get_ft_sensor_data(float ft_data[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_get_data(ft_data, _version_is_ge(1, 8, 3));
	return _check_code(ret);
}

int XArmAPI::get_ft_sensor_config(int *ft_app_status, int *ft_is_started, int *ft_type, int *ft_id, int *ft_freq, 
	float *ft_mass, float *ft_dir_bias, float ft_centroid[3], float ft_zero[6], int *imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6],
	int *f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_get_config(ft_app_status, ft_is_started, ft_type, ft_id, ft_freq,
		ft_mass, ft_dir_bias, ft_centroid, ft_zero, imp_coord, imp_c_axis, M, K, B,
		f_coord, f_c_axis, f_ref, f_limits, kp, ki, kd, xe_limit
	);
	return _check_code(ret);
}

int XArmAPI::get_ft_sensor_error(int *err)
{
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_get_error(err);
	return _check_code(ret);
}

int XArmAPI::iden_tcp_load(float result[4])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->iden_tcp_load(result);
	return _check_code(ret);
}