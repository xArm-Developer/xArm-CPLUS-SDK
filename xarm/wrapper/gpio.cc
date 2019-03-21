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

int XArmAPI::set_gripper_enable(bool enable) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_set_en(u16(enable));
    } else {
        ret = cmd_ser_->gripper_set_en(u16(enable));
    }
    return ret;
}

int XArmAPI::set_gripper_mode(u16 mode) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_set_mode(mode);
    } else {
        ret = cmd_ser_->gripper_set_mode(mode);
    }
    return ret;
}

int XArmAPI::set_gripper_speed(fp32 speed) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_set_posspd(speed);
    } else {
        ret = cmd_ser_->gripper_set_posspd(speed);
    }
    return ret;
}

int XArmAPI::get_gripper_position(fp32 *pos) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_get_pos(pos);
    } else {
        ret = cmd_ser_->gripper_get_pos(pos);
    }
    return ret;
}

int XArmAPI::get_gripper_err_code(u8 err_warn[2]) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_get_errcode(err_warn);
    } else {
        ret = cmd_ser_->gripper_get_errcode(err_warn);
    }
    return ret;
}

int XArmAPI::set_gripper_position(fp32 pos, bool wait, fp32 timeout) {
    if (!is_connected()) return -1;
    int ret = 0;
    float last_pos = 0, pos_tmp, cur_pos;;
    u8 err_warn[2];
    bool is_add = true;
    ret = get_gripper_position(&pos_tmp);
    if (ret != 0) {
        get_err_warn_code(err_warn);
    }
    if (error_code != 28) {
        last_pos = pos_tmp;
        if (int(last_pos) == int(pos))
            return 0;
        is_add = pos > last_pos ? true : false;
    }
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_set_pos(pos);
    } else {
        ret = cmd_ser_->gripper_set_pos(pos);
    }
    if (wait) {
        long long start_time = get_system_time();
        int ret2 = 0;
        int count = 0;

        while (error_code != 28 && get_system_time() - start_time < timeout * 1000) {
            ret2 = get_gripper_position(&pos_tmp);
            if (ret2 != 0) {
                get_err_warn_code(err_warn);
            }
            if (error_code != 28) {
                cur_pos = pos_tmp;
                if (fabs(pos - cur_pos) < 1) {
                    break;
                }
                if (is_add) {
                    if (cur_pos <= last_pos) {
                        count += 1;
                    } else {
                        last_pos = cur_pos;
                        count = 0;
                    }
                } else {
                    if (cur_pos >= last_pos) {
                        count += 1;
                    } else {
                        last_pos = cur_pos;
                        count = 0;
                    }

                }
                if (count >= 25) {
                    printf("gripper target: %f, current: %f\n", pos, cur_pos);
                    break;
                }
            }
            sleep_milliseconds(20);
        }
    }
    return ret;
}

int XArmAPI::clean_gripper_error(void) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gripper_clean_err();
    } else {
        ret = cmd_ser_->gripper_clean_err();
    }
    return ret;
}

int XArmAPI::get_gpio_digital(int *io1, int *io2) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gpio_get_digital(io1, io2);
    } else {
        ret = cmd_ser_->gpio_get_digital(io1, io2);
    }
    return ret;
}

int XArmAPI::set_gpio_digital(int ionum, int value) {
    if (!is_connected()) return -1;
    int ret = 0;
    if (is_tcp_) {
        ret = cmd_tcp_->gpio_set_digital(ionum, value);
    } else {
        ret = cmd_ser_->gpio_set_digital(ionum, value);
    }
    return ret;
}

int XArmAPI::get_gpio_analog(int ionum, float *value) {
    if (!is_connected()) return -1;
    assert(ionum == 1 || ionum == 2);
    int ret = 0;
    if (is_tcp_) {
        if (ionum == 1)
            ret = cmd_tcp_->gpio_get_analog1(value);
        else
            ret = cmd_tcp_->gpio_get_analog2(value);
    } else {
        if (ionum == 1)
            ret = cmd_ser_->gpio_get_analog1(value);
        else
            ret = cmd_ser_->gpio_get_analog2(value);
    }
    return ret;
}
