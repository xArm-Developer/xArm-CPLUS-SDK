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

#define TRACK_BAUD 2000000

int XArmAPI::_get_linear_track_registers(unsigned char *ret_data, int addr, int number_of_registers)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

  int ret = core->track_modbus_r16s(addr, ret_data, number_of_registers);
  ret = _check_modbus_code(ret, ret_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  return ret;
}

int XArmAPI::get_linear_track_registers(LinearTrackStatus *status, int addr, int number_of_registers)
{
  if (!((addr == 0x0A20 && number_of_registers >= 2) || (addr >= 0x0A22 && addr <= 0x0A27 && number_of_registers >= 1)))
    return API_CODE::PARAM_ERROR;
  unsigned char *rx_data = new unsigned char[4 + 2 * number_of_registers]();
  int ret = _get_linear_track_registers(rx_data, addr, number_of_registers);
  if (ret == 0) {
    if (addr == 0x0A20 && number_of_registers >= 2) {
      linear_track_status.pos = (int)(bin8_to_32(&rx_data[4]) / 2000);
    }
    int start_inx;
    if (addr > (0x0A22 - number_of_registers) && addr <= 0x0A22) {
      start_inx = (0x0A22 - addr) * 2 + 4;
      linear_track_status.status = bin8_to_16(&rx_data[start_inx]);
    }
    if (addr > (0x0A23 - number_of_registers) && addr <= 0x0A23) {
      start_inx = (0x0A23 - addr) * 2 + 4;
      linear_track_status.error = bin8_to_16(&rx_data[start_inx]);
    }
    if (addr > (0x0A24 - number_of_registers) && addr <= 0x0A24) {
      start_inx = (0x0A24 - addr) * 2 + 4;
      linear_track_status.is_enabled = bin8_to_16(&rx_data[start_inx]) & 0x01;
    }
    if (addr > (0x0A25 - number_of_registers) && addr <= 0x0A25) {
      start_inx = (0x0A25 - addr) * 2 + 4;
      linear_track_status.on_zero = bin8_to_16(&rx_data[start_inx]) & 0x01;
    }
    if (addr > (0x0A26 - number_of_registers) && addr <= 0x0A26) {
      start_inx = (0x0A26 - addr) * 2 + 4;
      linear_track_status.sci = (bin8_to_16(&rx_data[start_inx]) >> 1) & 0x01;
    }
    if (addr > (0x0A27 - number_of_registers) && addr <= 0x0A27) {
      start_inx = (0x0A27 - addr) * 2 + 4;
      int sco = bin8_to_16(&rx_data[start_inx]);
      linear_track_status.sco[0] = sco & 0x01;
      linear_track_status.sco[1] = (sco >> 1) & 0x01;
    }
    if (status != NULL) {
      memcpy(status, &linear_track_status, sizeof(linear_track_status));
    }
  }
  delete[] rx_data;
  return ret;
}

int XArmAPI::get_linear_track_error(int *err)
{
  int ret = get_linear_track_registers(NULL, 0x0A23, 1);
  if (ret == 0 && err != NULL) {
    *err = linear_track_status.error;
  }
  return ret;
}

int XArmAPI::get_linear_track_status(int *status)
{
  int ret = get_linear_track_registers(NULL, 0x0A22, 1);
  if (ret == 0 && status != NULL) {
    *status = linear_track_status.status;
  }
  return ret;
}

int XArmAPI::get_linear_track_pos(int *pos)
{
  int ret = get_linear_track_registers(NULL, 0x0A20, 2);
  if (ret == 0 && pos != NULL) {
    *pos = linear_track_status.pos;
  }
  return ret;
}

int XArmAPI::get_linear_track_is_enabled(int *status)
{
  int ret = get_linear_track_registers(NULL, 0x0A24, 1);
  if (ret == 0 && status != NULL) {
    *status = linear_track_status.is_enabled;
  }
  return ret;
}

int XArmAPI::get_linear_track_on_zero(int *status)
{
  int ret = get_linear_track_registers(NULL, 0x0A25, 1);
  if (ret == 0 && status != NULL) {
    *status = linear_track_status.on_zero;
  }
  return ret;
}

int XArmAPI::get_linear_track_sci(int *sci1)
{
  int ret = get_linear_track_registers(NULL, 0x0A26, 1);
  if (ret == 0 && sci1 != NULL) {
    *sci1 = linear_track_status.sci;
  }
  return ret;
}

int XArmAPI::get_linear_track_sco(int sco[2])
{
  int ret = get_linear_track_registers(NULL, 0x0A27, 1);
  if (ret == 0 && sco != NULL) {
    sco[0] = linear_track_status.sco[0];
    sco[1] = linear_track_status.sco[1];
  }
  return ret;
}

int XArmAPI::clean_linear_track_error(void)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
  
  unsigned char send_data[2];
  bin16_to_8(1, send_data);
  unsigned char rx_data[7] = {0};
  int ret = core->track_modbus_w16s(SERVO3_RG::RESET_ERR, send_data, 1, rx_data);
  get_linear_track_error(NULL);
  ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  return linear_track_status.error != 0 ? API_CODE::LINEAR_TRACK_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_enable(bool enable)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

  unsigned char rx_data[7] = {0};
  unsigned char send_data[2];
  bin16_to_8((int)enable, send_data);
  int ret = core->track_modbus_w16s(SERVO3_RG::CON_EN, send_data, 1, rx_data);
  ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  get_linear_track_registers(NULL, 0x0A23, 3);
  return linear_track_status.error != 0 ? API_CODE::LINEAR_TRACK_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_speed(int speed)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
  
  unsigned char rx_data[7] = {0};
  unsigned char send_data[2];
  bin16_to_8((int)(speed * 6.667), send_data);
  int ret = core->track_modbus_w16s(SERVO3_RG::POS_SPD, send_data, 1, rx_data);
  ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  if (ret == 0) {
    linear_track_speed_ = speed;
  }
  return ret;
}

int XArmAPI::set_linear_track_back_origin(bool wait, bool auto_enable)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

  unsigned char rx_data[7] = {0};
  int ret = core->track_modbus_r16s(SERVO3_RG::BACK_ORIGIN, rx_data, 1, 0x06);
  ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  get_linear_track_registers(NULL, 0x0A23, 3);
  if (ret == 0 && wait) {
    ret = _wait_linear_track_back_origin();
  }
  if (auto_enable) {
    ret = set_linear_track_enable(true);
  }
  return linear_track_status.error != 0 ? API_CODE::LINEAR_TRACK_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_pos(int pos, int speed, bool wait, fp32 timeout, bool auto_enable)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
  
  int ret = get_linear_track_registers(NULL, 0x0A23, 3);
  if (ret == 0 && linear_track_status.on_zero != 1) {
    fprintf(stderr, "[WARN] linear track is not on zero, please set linear track back to origin\n");
    return API_CODE::LINEAR_TRACK_NOT_INIT;
  }
  if (auto_enable && (ret != 0 || linear_track_status.is_enabled != 1)) {
    set_linear_track_enable(true);
  }
  if (speed > 0 && speed != linear_track_speed_) {
    set_linear_track_speed(speed);
  }
  unsigned char rx_data[7] = {0};
  unsigned char send_data[4];
  bin32_to_8(pos * 2000, send_data);
  ret = core->track_modbus_w16s(SERVO3_RG::TAGET_POS, send_data, 2, rx_data);
  ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  get_linear_track_registers(NULL, 0x0A23, 3);

  if (ret == 0 && wait) {
    ret = _wait_linear_track_stop(timeout);
  }
  return linear_track_status.error != 0 ? API_CODE::LINEAR_TRACK_HAS_FAULT : ret;
}

int XArmAPI::set_linear_track_stop(void)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (baud_checkset_flag_ && _checkset_modbus_baud(default_linear_track_baud_, true, UXBUS_CONF::LINEAR_TRACK_HOST_ID) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;

  unsigned char rx_data[7] = {0};
  unsigned char send_data[2];
  bin16_to_8(1, send_data);
  int ret = core->track_modbus_w16s(SERVO3_RG::STOP_TRACK, send_data, 1, rx_data);
  ret = _check_modbus_code(ret, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID);
  get_linear_track_registers(NULL, 0x0A22, 2);
  return linear_track_status.error != 0 ? API_CODE::LINEAR_TRACK_HAS_FAULT : ret;
}

int XArmAPI::_wait_linear_track_back_origin(fp32 timeout)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int failed_cnt = 0;
  long long expired = get_system_time() + (long long)(timeout * 1000);
  int code = API_CODE::WAIT_FINISH_TIMEOUT;
  while (timeout <= 0 || get_system_time() < expired) {
    int code2 = get_linear_track_registers(NULL, 0x0A22, 5);
    if (code2 == 0 && linear_track_status.sci == 0) {
      return API_CODE::LINEAR_TRACK_SCI_IS_LOW;
    }
    if (code2 == 0 && linear_track_status.error != 0) {
      return API_CODE::LINEAR_TRACK_HAS_FAULT;
    }
    failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
    if (code2 == 0 && linear_track_status.on_zero == 1) {
      return 0;
    }
    else {
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
  while (timeout <= 0 || get_system_time() < expired) {
    int code2 = get_linear_track_registers(NULL, 0x0A22, 5);
    if (code2 == 0 && linear_track_status.sci == 0) {
      return API_CODE::LINEAR_TRACK_SCI_IS_LOW;
    }
    if (code2 == 0 && linear_track_status.error != 0) {
      return API_CODE::LINEAR_TRACK_HAS_FAULT;
    }
    failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
    if (code2 == 0 && (linear_track_status.status & 0x01) == 0) {
      return 0;
    }
    else {
      code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
    }
    if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
    sleep_milliseconds(100);
  }
  return code;
}
