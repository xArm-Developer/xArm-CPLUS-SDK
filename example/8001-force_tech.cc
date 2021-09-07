/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
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

	float M[6] = { 0.5, 0.5, 0.5, 0.005, 0.005, 0.005 };
	float K[6] = { 0, 0, 0, 4, 4, 4 };
	float B[6] = { 0 };
	int coord = 1;
	int c_axis[6] = { 0, 0, 1, 0, 0, 0 };
	int ret;

	ret = arm->set_impedance_mbk(M, K, B);
	printf("set_impedance_mbk, ret=%d\n", ret);
	ret = arm->set_impedance_config(coord, c_axis);
	printf("set_impedance_config, ret=%d\n", ret);

	ret = arm->ft_sensor_enable(1);
	printf("ft_sensor_enable, ret=%d\n", ret);
	ret = arm->ft_sensor_set_zero();
	printf("ft_sensor_set_zero, ret=%d\n", ret);
	sleep_milliseconds(200);

	ret = arm->ft_sensor_app_set(1);
	printf("ft_sensor_app_set, ret=%d\n", ret);
	ret = arm->set_state(0);

	// you can teach robot now
	sleep_milliseconds(1000 * 10);

	ret = arm->ft_sensor_app_set(0);
	printf("ft_sensor_app_set, ret=%d\n", ret);
	ret = arm->ft_sensor_enable(0);
	printf("ft_sensor_enable, ret=%d\n", ret);

	arm->disconnect();

	return 0;
}