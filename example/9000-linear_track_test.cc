/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2020, UFACTORY, Inc.
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
	
    int ret;

    int int_val;
    float fp_val;
    ret = arm->get_linear_track_error(&int_val);
    printf("get_linear_track_error, ret=%d, err=%d\n", ret, int_val);
    ret = arm->get_linear_track_status(&int_val);
    printf("get_linear_track_status, ret=%d, status=%d\n", ret, int_val);
    ret = arm->get_linear_track_pos(&fp_val);
    printf("get_linear_track_pos, ret=%d, pos=%f\n", ret, fp_val);
    ret = arm->check_linear_track_on_zero(&int_val);
    printf("check_linear_track_on_zero, ret=%d, status=%d\n", ret, int_val);
    
    ret = arm->clean_linear_track_error();
    printf("clean_linear_track_error, ret=%d\n", ret);

    ret = arm->set_linear_track_back_origin();
    printf("set_linear_track_back_origin, ret=%d\n", ret);
    
    ret = arm->set_linear_track_enable(true);
    printf("set_linear_track_enable, ret=%d\n", ret);

    ret = arm->set_linear_track_speed(500);
    printf("set_linear_track_speed, ret=%d\n", ret);

    ret = arm->set_linear_track_pos(500);
    printf("set_linear_track_pos, ret=%d\n", ret);

    ret = arm->set_linear_track_pos(200);
    printf("set_linear_track_pos, ret=%d\n", ret);

    ret = arm->stop_linear_track();
    printf("stop_linear_track, ret=%d\n", ret);

	return 0;
}