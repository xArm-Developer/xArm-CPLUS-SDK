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


int XArmAPI::_bio_gripper_send_modbus(unsigned char *send_data, int length, unsigned char *ret_data, int ret_length) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_bio_baud_) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
  int ret = getset_tgpio_modbus_data(send_data, length, ret_data, ret_length);
  return ret;
}

int XArmAPI::_get_bio_gripper_register(unsigned char *ret_data, int address, int number_of_registers) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char params[6] = { 0x08, 0x03, (unsigned char)(address >> 8), (unsigned char)address, (unsigned char)(number_of_registers >> 8), (unsigned char)number_of_registers };
  return _bio_gripper_send_modbus(params, 6, ret_data, 3 + 2 * number_of_registers);
}

int XArmAPI::_bio_gripper_wait_motion_completed(fp32 timeout) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int failed_cnt = 0;
  long long expired = get_system_time() + (long long)(timeout * 1000);
  int code = API_CODE::WAIT_FINISH_TIMEOUT;
  int status = BIO_STATE::IS_MOTION;
  while (timeout <= 0 || get_system_time() < expired) {
    int code2 = get_bio_gripper_status(&status);
    failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
    if (code2 == 0) {
      code = (status & 0x03) == BIO_STATE::IS_MOTION ? code : (status & 0x03) == BIO_STATE::IS_FAULT ? API_CODE::END_EFFECTOR_HAS_FAULT : 0;
    }
    else {
      code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
    }
    if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
    sleep_milliseconds(100);
  }
  if (code == 0 && !bio_gripper_is_enabled_) code = API_CODE::END_EFFECTOR_NOT_ENABLED;
  return code;
}

int XArmAPI::_bio_gripper_wait_enable_completed(fp32 timeout) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int failed_cnt = 0;
  long long expired = get_system_time() + (long long)(timeout * 1000);
  int code = API_CODE::WAIT_FINISH_TIMEOUT;
  int status = BIO_STATE::IS_NOT_ENABLED;
  while (timeout <= 0 || get_system_time() < expired) {
    int code2 = get_bio_gripper_status(&status);
    failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
    if (code2 == 0) {
      code = bio_gripper_is_enabled_ ? 0 : code;
    }
    else {
      code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
    }
    if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
    sleep_milliseconds(100);
  }
  return code;
}

int XArmAPI::set_bio_gripper_enable(bool enable, bool wait, fp32 timeout) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char params[6] = { 0x08, 0x06, 0x01, 0x00, 0x00, (unsigned char)enable };
  unsigned char rx_data[6] = { 0 };
  int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
  if (ret == 0 && enable && wait) { ret = _bio_gripper_wait_enable_completed(timeout); }
  return ret;
}

int XArmAPI::set_bio_gripper_speed(int speed) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned short tmp = speed;
  unsigned char params[6] = { 0x08, 0x06, 0x03, 0x03, (unsigned char)(tmp >> 8), (unsigned char)tmp };
  unsigned char rx_data[6] = { 0 };
  int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
  if (ret == 0) { bio_gripper_speed_ = speed; }
  return ret;
}

int XArmAPI::_get_bio_gripper_sn(unsigned char sn[32])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char rx_data[40] = { 0 };
  int ret = _get_bio_gripper_register(rx_data, 0x0B10, 16);
  if (ret == API_CODE::MODBUS_ERR_LENG)
  {
    bio_gripper_version_ = 1;
    return ret;
  }
  else if (ret != 0)
  {
    bio_gripper_version_ = 0;
    return ret;
  }    
  else{
    bio_gripper_version_ = 2;
    memcpy(sn, &rx_data[3], 32);
    return ret;
  }
}

int XArmAPI::_get_bio_gripper_control_mode(int *mode)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char rx_data[5] = { 0 };
  int ret = _get_bio_gripper_register(rx_data, 0x010A);
  *mode = (rx_data[3] << 8) + rx_data[4];
  return ret;
}

int XArmAPI::set_bio_gripper_control_mode(int mode)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char params[6] = { 0x08, 0x06, 0x11, 0x0A, 0x00, (unsigned char)mode };
  unsigned char rx_data[6] = { 0 };
  int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
  // reset mcu
  unsigned char params2[6] = { 0x08, 0x06, 0x06, 0x07, 0x00, 0x01 };
  _bio_gripper_send_modbus(params2, 6, rx_data, 6);
  sleep_milliseconds(600);
  bio_gripper_speed_ = 0;
  return ret;
}

int XArmAPI::set_bio_gripper_force(int force)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char val = force < 1 ? 1 : force > 100 ? 100 : (unsigned char)force;
  unsigned char params[6] = { 0x08, 0x06, 0x05, 0x06, 0x00, val };
  unsigned char rx_data[6] = { 0 };
  int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
  return ret;
}

int XArmAPI::set_bio_gripper_position(int pos, int speed, int force, bool wait, fp32 timeout, bool wait_motion) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (speed > 0 && speed != bio_gripper_speed_) { set_bio_gripper_speed(speed); }
  if (bio_gripper_version_ == 0)
  {
    unsigned char sn[32];
    _get_bio_gripper_sn(sn);
  }
  if (bio_gripper_version_ == 2)
  {
    int mode = 0;
    int code = _get_bio_gripper_control_mode(&mode);
    if (mode == 1) {
      pos = int(pos * 3.798 - 269.620);
    }
    set_bio_gripper_force(force);
  }

  unsigned char params[11] = { 0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04 };
  params[7] = (unsigned char)(pos >> 24);
  params[8] = (unsigned char)(pos >> 16);
  params[9] = (unsigned char)(pos >> 8);
  params[10] = (unsigned char)(pos);
  unsigned char rx_data[6] = { 0 };
  if (wait_motion) {
    bool has_error = error_code != 0;
    bool is_stop = state == 4 || state == 5;
    int code = _wait_move(NO_TIMEOUT);
    if (!(code == 0 || (is_stop && code == API_CODE::EMERGENCY_STOP) || (has_error && code == API_CODE::HAS_ERROR))) {
      return code;
    }
  }
  int ret = _bio_gripper_send_modbus(params, 11, rx_data, 6);
  if (ret == 0 && wait) { ret = _bio_gripper_wait_motion_completed(timeout); }
  return ret;
}

int XArmAPI::open_bio_gripper(int speed, bool wait, fp32 timeout, bool wait_motion) {
  return set_bio_gripper_position(130, speed, 100, wait, timeout, wait_motion);
}

int XArmAPI::open_bio_gripper(bool wait, fp32 timeout, bool wait_motion) {
  return open_bio_gripper(bio_gripper_speed_, wait, timeout, wait_motion);
}

int XArmAPI::close_bio_gripper(int speed, bool wait, fp32 timeout, bool wait_motion) {
  return set_bio_gripper_position(50, speed, 100, wait, timeout, wait_motion);
}

int XArmAPI::close_bio_gripper(bool wait, fp32 timeout, bool wait_motion) {
  return close_bio_gripper(bio_gripper_speed_, wait, timeout, wait_motion);
}

int XArmAPI::get_bio_gripper_status(int *status) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char rx_data[5] = { 0 };
  int ret = _get_bio_gripper_register(rx_data, 0x00);
  *status = (rx_data[3] << 8) + rx_data[4];
  if (ret == 0) {
    if ((*status & 0x03) == BIO_STATE::IS_FAULT) {
      int err;
      get_bio_gripper_error(&err);
    }
    bio_gripper_error_code_ = (*status & 0x03) == BIO_STATE::IS_FAULT ? bio_gripper_error_code_ : 0;
    bio_gripper_is_enabled_ = ((*status >> 2) & 0x03) == BIO_STATE::IS_ENABLED ? true : false;
  }
  return ret;
}

int XArmAPI::get_bio_gripper_error(int *err) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char rx_data[5] = { 0 };
  int ret = _get_bio_gripper_register(rx_data, 0x0F);
  *err = (rx_data[3] << 8) + rx_data[4];
  if (ret == 0) bio_gripper_error_code_ = *err;
  return ret;
}

int XArmAPI::clean_bio_gripper_error(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char params[6] = { 0x08, 0x06, 0x00, 0x0F, 0x00, 0x00 };
  unsigned char rx_data[6] = { 0 };
  int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
  int status;
  get_bio_gripper_status(&status);
  return ret;
}

int XArmAPI::get_bio_gripper_position(fp32 *pos)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char rx_data[7] = { 0 };
  int ret = _get_bio_gripper_register(rx_data, 0x0700, 2);
  *pos = (float)((rx_data[5] << 8) + rx_data[6]);
  return ret;
}