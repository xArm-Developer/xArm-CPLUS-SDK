/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"

#define TRACK_BAUD 2000000

int XArmAPI::get_linear_track_error(int *err)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

    unsigned char rx_data[6] = {0};
    int ret = core->track_modbus_r16s(SERVO3_RG::ERR_CODE, rx_data, 1);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
	if (ret == 0) linear_track_error_code_ = bin8_to_16(&rx_data[4]);
    if (err != NULL) *err = linear_track_error_code_;
    return ret;
}

int XArmAPI::get_linear_track_status(int *status)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    
    unsigned char rx_data[6] = {0};
    int ret = core->track_modbus_r16s(SERVO3_RG::GET_STATUS, rx_data, 1);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    *status = bin8_to_16(&rx_data[4]);
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::get_linear_track_pos(fp32 *pos)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    
    unsigned char rx_data[8] = {0};
    int ret = core->track_modbus_r16s(SERVO3_RG::CURR_POS, rx_data, 2);
	get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    *pos = (float)(bin8_to_32(&rx_data[4]) / 2000);
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::check_linear_track_on_zero(int *status)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

    unsigned char rx_data[6] = {0};
    int ret = core->track_modbus_r16s(SERVO3_RG::CHECK_ON_ORIGIN, rx_data, 1);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    *status = rx_data[5];
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::clean_linear_track_error(void)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    
    unsigned char send_data[2];
    bin16_to_8(1, send_data);
    unsigned char rx_data[7] = {0};
    linear_track_error_code_ = 0;
    int ret = core->track_modbus_w16s(SERVO3_RG::RESET_ERR, send_data, 1, rx_data);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_enable(bool enable)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

    unsigned char rx_data[7] = {0};
    unsigned char send_data[2];
    bin16_to_8((int)enable, send_data);
    int ret = core->track_modbus_w16s(SERVO3_RG::CON_EN, send_data, 1, rx_data);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_speed(int speed)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    
    unsigned char rx_data[7] = {0};
    unsigned char send_data[2];
    bin16_to_8(speed, send_data);
    int ret = core->track_modbus_w16s(SERVO3_RG::POS_SPD, send_data, 1, rx_data);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_back_origin(bool wait, bool auto_enable)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    
    unsigned char rx_data[7] = {0};
    int ret = core->track_modbus_r16s(SERVO3_RG::BACK_ORIGIN, rx_data, 1, 0x06);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);

    if (ret == 0 && wait) {
        ret = _wait_linear_track_back_origin();
    }
    if (auto_enable) {
        ret = set_linear_track_enable(true);
    }
    return ret;
}

int XArmAPI::set_linear_track_pos(fp32 pos, bool wait, fp32 timeout)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    unsigned char rx_data[7] = {0};
    unsigned char send_data[4];
    bin32_to_8((int)(pos * 2000), send_data);
    int ret = core->track_modbus_w16s(SERVO3_RG::TAGET_POS, send_data, 2, rx_data);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);

    if (ret == 0 && wait) {
        ret = _wait_linear_track_stop(timeout);
    }
    return ret;
}

int XArmAPI::stop_linear_track(void)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(TRACK_BAUD) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
    
    unsigned char rx_data[7] = {0};
    unsigned char send_data[2];
    bin16_to_8(1, send_data);
    int ret = core->track_modbus_w16s(SERVO3_RG::STOP_TRACK, send_data, 1, rx_data);
    get_linear_track_error(NULL);
    ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
    return linear_track_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::_wait_linear_track_back_origin(fp32 timeout)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int failed_cnt = 0;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	int status;
	while (timeout <= 0 || get_system_time() < expired) {
		int code2 = check_linear_track_on_zero(&status);
		failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
		if (code2 == 0 && status == 1) {
            return 0;
		}
		else {
            if (linear_track_error_code_ != 0) return API_CODE::END_EFFECTOR_HAS_FAULT;
			code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
		}
		if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
		sleep_milliseconds(100);
	}
	return code;
}

int XArmAPI::_wait_linear_track_stop(fp32 timeout)
{
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int failed_cnt = 0;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	int status;
	while (timeout <= 0 || get_system_time() < expired) {
		int code2 = get_linear_track_status(&status);
		failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
		if (code2 == 0 && (status & 0x01) == 0) {
            return 0;
		}
		else {
            if (linear_track_error_code_ != 0) return API_CODE::END_EFFECTOR_HAS_FAULT;
			code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
		}
		if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
		sleep_milliseconds(100);
	}
	return code;
}
