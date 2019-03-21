/*
* \file xarm/wrapper/xarm_api.cc
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/

#include "xarm/wrapper/xarm_api.h"


int XArmAPI::set_collision_sensitivity(u8 sensitivity) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_collis_sens(sensitivity);
    } else {
        ret = cmd_ser_->set_collis_sens(sensitivity);
    }
    return ret;
}

int XArmAPI::set_teach_sensitivity(u8 sensitivity) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_teach_sens(sensitivity);
    } else {
        ret = cmd_ser_->set_teach_sens(sensitivity);
    }
    return ret;
}

int XArmAPI::clean_conf(void) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->clean_conf();
    } else {
        ret = cmd_ser_->clean_conf();
    }
    return ret;
}

int XArmAPI::save_conf(void) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->save_conf();
    } else {
        ret = cmd_ser_->save_conf();
    }
    return ret;
}

int XArmAPI::set_tcp_offset(fp32 pose_offset[6]) {
    if (!is_connected()) return -1;
    int ret = 0;
    fp32 offset[6];
    for (u32 i = 0; i < 6; i++) {
        offset[i] = default_is_radian || i < 3  ? pose_offset[i] : pose_offset[i] / RAD_DEGREE;
    }
    if (is_tcp_) {
        ret = cmd_tcp_->set_tcp_offset(offset);
    } else {
        ret = cmd_ser_->set_tcp_offset(offset);
    }
    return ret;
}

int XArmAPI::set_tcp_load(fp32 weight, fp32 center_of_gravity[3]) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_tcp_load(weight, center_of_gravity);
    } else {
        ret = cmd_ser_->set_tcp_load(weight, center_of_gravity);
    }
    return ret;
}

int XArmAPI::set_tcp_jerk(fp32 jerk) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_tcp_jerk(jerk);
    } else {
        ret = cmd_ser_->set_tcp_jerk(jerk);
    }
    return ret;
}

int XArmAPI::set_tcp_maxacc(fp32 acc) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_tcp_maxacc(acc);
    } else {
        ret = cmd_ser_->set_tcp_maxacc(acc);
    }
    return ret;
}

int XArmAPI::set_joint_jerk(fp32 jerk) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_joint_jerk(default_is_radian ? jerk : jerk / RAD_DEGREE);
    } else {
        ret = cmd_ser_->set_joint_jerk(default_is_radian ? jerk : jerk / RAD_DEGREE);
    }
    return ret;
}

int XArmAPI::set_joint_maxacc(fp32 acc) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->set_joint_maxacc(default_is_radian ? acc : acc / RAD_DEGREE);
    } else {
        ret = cmd_ser_->set_joint_maxacc(default_is_radian ? acc : acc / RAD_DEGREE);
    }
    return ret;
}
