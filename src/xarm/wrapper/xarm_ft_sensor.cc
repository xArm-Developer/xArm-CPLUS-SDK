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

int XArmAPI::ft_sensor_cali_load(float load[10])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->ft_sensor_cali_load(load);
	return _check_code(ret);
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

int XArmAPI::get_exe_ft(float exe_ft[6])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_exe_ft(exe_ft);
	return _check_code(ret);
}

int XArmAPI::iden_tcp_load(float result[4])
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->iden_tcp_load(result);
	return _check_code(ret);
}