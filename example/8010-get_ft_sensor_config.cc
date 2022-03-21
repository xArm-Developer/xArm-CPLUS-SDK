/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
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

    // arm->motion_enable(true);
    // arm->set_mode(0);
    // arm->set_state(0);
    // sleep_milliseconds(1000);

    printf("=========================================\n");

    int ret;
    int ft_app_status, ft_is_started, ft_type, ft_id, ft_freq; 
	float ft_mass, ft_dir_bias, ft_centroid[3], ft_zero[6];
    int imp_coord, imp_c_axis[6];
    float M[6], K[6], B[6];
	int f_coord, f_c_axis[6];
    float f_ref[6], f_limits[6], kp[6], ki[6], kd[6], xe_limit[6];
    ret = arm->get_ft_sensor_config(&ft_app_status, &ft_is_started, &ft_type, &ft_id, &ft_freq,
		&ft_mass, &ft_dir_bias, ft_centroid, ft_zero, &imp_coord, imp_c_axis, M, K, B,
		&f_coord, f_c_axis, f_ref, f_limits, kp, ki, kd, xe_limit);

    printf("ft_app_status=%d, ft_is_started=%d, ft_type=%d, ft_id=%d, ft_freq=%d\n", ft_app_status, ft_is_started, ft_type, ft_id, ft_freq);
    printf("ft_mass=%f, ft_dir_bias=%f\n", ft_mass, ft_dir_bias);
    print_nvect("ft_centroid: ", ft_centroid, 3);
    print_nvect("ft_zero: ", ft_zero, 6);
    printf("imp_coord=%d\n", imp_coord);
    print_nvect("imp_c_axis: ", imp_c_axis, 6);
    print_nvect("M: ", M, 6);
    print_nvect("K: ", K, 6);
    print_nvect("B: ", B, 6);
    printf("f_coord=%d\n", f_coord);
    print_nvect("f_c_axis: ", f_c_axis, 6);
    print_nvect("f_ref: ", f_ref, 6);
    print_nvect("f_limits: ", f_limits, 6);
    print_nvect("kp: ", kp, 6);
    print_nvect("ki: ", ki, 6);
    print_nvect("kd: ", kd, 6);
    print_nvect("xe_limit: ", xe_limit, 6);

    return 0;
}