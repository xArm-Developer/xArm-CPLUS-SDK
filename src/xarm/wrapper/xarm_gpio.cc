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


static int BAUDRATES[13] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000 };

static int get_baud_inx(int baud) {
  for (int i = 0; i < 13; i++) { if (BAUDRATES[i] == baud) return i; }
  return -1;
}


int XArmAPI::get_tgpio_version(unsigned char versions[3]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  float val1 = 0, val2 = 0, val3 = 0;
  int code = 0;
  versions[0] = 0;
  versions[1] = 0;
  versions[2] = 0;
  int ret1 = core->tgpio_addr_r16(0x0801, &val1);
  int ret2 = core->tgpio_addr_r16(0x0802, &val2);
  int ret3 = core->tgpio_addr_r16(0x0803, &val3);
  if (ret1 == 0) { versions[0] = (unsigned char)val1; }
  else { code = ret1; }
  if (ret2 == 0) { versions[1] = (unsigned char)val2; }
  else { code = ret2; }
  if (ret3 == 0) { versions[2] = (unsigned char)val3; }
  else { code = ret3; }
  return code;
}


int XArmAPI::get_tgpio_digital(int *io0, int *io1, int *io2, int *io3, int *io4) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->tgpio_get_digital(io0, io1, io2, io3, io4);
}

int XArmAPI::set_tgpio_digital(int ionum, int value, float delay_sec, bool sync) {
  if (ionum != 0 && ionum != 1 && ionum != 2 && ionum != 3 && ionum != 4) return API_CODE::PARAM_ERROR;
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (delay_sec > 0) {
    return core->tgpio_delay_set_digital(ionum + 1, value, delay_sec);
  }
  else {
    return core->tgpio_set_digital(ionum + 1, value, _version_is_ge(2, 4, 101) ? sync : -1);
  }
}

int XArmAPI::get_tgpio_analog(int ionum, float *value) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (ionum != 0 && ionum != 1) return API_CODE::PARAM_ERROR;
  if (ionum == 0) {
    return core->tgpio_get_analog1(value);
  }
  else {
    return core->tgpio_get_analog2(value);
  }
}

int XArmAPI::get_cgpio_digital(int *digitals, int *digitals2) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int tmp = 0;
  int ret = core->cgpio_get_auxdigit(&tmp);
  for (int i = 0; i < 8; i++) {
    digitals[i] = tmp >> i & 0x0001;
  }
  if (digitals2 != NULL) {
    for (int i = 8; i < 16; i++) {
      digitals2[i-8] = tmp >> i & 0x0001;
    }
  }
  return ret;
}

int XArmAPI::get_cgpio_analog(int ionum, fp32 *value) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (ionum != 0 && ionum != 1) return API_CODE::PARAM_ERROR;
  if (ionum == 0) {
    return core->cgpio_get_analog1(value);
  }
  else {
    return core->cgpio_get_analog2(value);
  }
}

int XArmAPI::set_cgpio_digital(int ionum, int value, float delay_sec, bool sync) {
  if (ionum < 0 || ionum >= 16) return API_CODE::PARAM_ERROR;
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (delay_sec > 0) {
    return core->cgpio_delay_set_digital(ionum, value, delay_sec);
  }
  else {
    return core->cgpio_set_auxdigit(ionum, value, _version_is_ge(2, 4, 101) ? sync : -1);
  }
}

int XArmAPI::set_cgpio_analog(int ionum, fp32 value, bool sync) {
  if (ionum != 0 && ionum != 1) return API_CODE::PARAM_ERROR;
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (ionum == 0) {
    return core->cgpio_set_analog1(value, _version_is_ge(2, 4, 101) ? sync : -1);
  }
  else {
    return core->cgpio_set_analog2(value, _version_is_ge(2, 4, 101) ? sync : -1);
  }
}

int XArmAPI::set_cgpio_digital_input_function(int ionum, int fun) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (ionum < 0 || ionum >= 16) return API_CODE::PARAM_ERROR;
  return core->cgpio_set_infun(ionum, fun);
}

int XArmAPI::set_cgpio_digital_output_function(int ionum, int fun) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (ionum < 0 || ionum >= 16) return API_CODE::PARAM_ERROR;
  return core->cgpio_set_outfun(ionum, fun);
}

int XArmAPI::get_cgpio_state(int *state_, int *digit_io, fp32 *analog, int *input_conf, int *output_conf, int *input_conf2, int *output_conf2) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->cgpio_get_state(state_, digit_io, analog, input_conf, output_conf, input_conf2, output_conf2);
}


int XArmAPI::get_suction_cup(int *val, int hardware_version) {
  if (hardware_version == 1) {
    int io1;
    return get_tgpio_digital(val, &io1);
  }
  else {
    int io0, io1;
    return get_tgpio_digital(&io0, &io1, NULL, val);
  }
}

int XArmAPI::set_suction_cup(bool on, bool wait, float timeout, float delay_sec, bool sync, int hardware_version) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  int code1, code2;
  if (on) {
    code1 = set_tgpio_digital(hardware_version == 1 ? 0 : 3, 1, delay_sec, sync);
    code2 = set_tgpio_digital(hardware_version == 1 ? 1 : 4, 0, delay_sec, sync);
  }
  else {
    code1 = set_tgpio_digital(hardware_version == 1 ? 0 : 3, 0, delay_sec, sync);
    code2 = set_tgpio_digital(hardware_version == 1 ? 1 : 4, 1, delay_sec, sync);
  }
  code = code1 == 0 ? code2 : code1;
  if (code == 0 && wait) {
    long long start_time = get_system_time();
    int val = 0, ret = 0;
    code = API_CODE::SUCTION_CUP_TOUT;
    while (get_system_time() - start_time < timeout * 1000) {
      ret = get_suction_cup(&val, hardware_version);
      if (ret == UXBUS_STATE::ERR_CODE) {
        code = UXBUS_STATE::ERR_CODE;
        break;
      }
      if (ret == 0) {
        if (on && val == 1) {
          code = 0;
          break;
        }
        if (!on && val == 0) {
          code = 0;
          break;
        }
      }
      sleep_milliseconds(100);
    }
  }

  return code;
}

int XArmAPI::set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->tgpio_position_set_digital(ionum, value, xyz, tol_r);
}

int XArmAPI::set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->cgpio_position_set_digital(ionum, value, xyz, tol_r);
}

int XArmAPI::set_cgpio_analog_with_xyz(int ionum, float value, float xyz[3], float tol_r) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->cgpio_position_set_analog(ionum, value, xyz, tol_r);
}

int XArmAPI::_check_modbus_code(int ret, unsigned char *rx_data, unsigned char host_id) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (ret == 0 || ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
    if (rx_data != NULL && rx_data[0] != host_id)
      return API_CODE::HOST_ID_ERR;
    if (ret != 0) {
      if (host_id == UXBUS_CONF::TGPIO_HOST_ID) {
        if (error_code != 19 && error_code != 28) {
          int err_warn[2] = { 0 };
          get_err_warn_code(err_warn);
        }
        ret = (error_code != 19 && error_code != 28) ? 0 : ret;
      }
      else {
        if (error_code != 100 + host_id) {
          int err_warn[2] = { 0 };
          get_err_warn_code(err_warn);
        }
        ret = (error_code != 100 + host_id) ? 0 : ret;
      }
    }
  }
  return ret;
}

int XArmAPI::_get_modbus_baudrate(int *baud_inx, unsigned char host_id) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  float val = 0;
  int ret = core->tgpio_addr_r16(SERVO3_RG::MODBUS_BAUDRATE & 0x0FFF, &val, host_id);
  *baud_inx = (int)val;
  if (ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
    if (host_id == UXBUS_CONF::TGPIO_HOST_ID) {
      if (error_code != 19 && error_code != 28) {
        int err_warn[2] = { 0 };
        get_err_warn_code(err_warn);
      }
      ret = (error_code != 19 && error_code != 28) ? 0 : ret;
    }
    else {
      if (error_code != 100 + host_id) {
        int err_warn[2] = { 0 };
        get_err_warn_code(err_warn);
      }
      ret = (error_code != 100 + host_id) ? 0 : ret;
    }
    ret = (error_code != 19 && error_code != 28) ? 0 : ret;
  }
  if (ret == 0 && *baud_inx >= 0 && *baud_inx < 13) {
    if (host_id == UXBUS_CONF::TGPIO_HOST_ID) {
      modbus_baud_ = BAUDRATES[*baud_inx];
    }
    else if (host_id == UXBUS_CONF::LINEAR_TRACK_HOST_ID) {
      linear_track_baud_ = BAUDRATES[*baud_inx];
    }
  }
  return ret;
}

int XArmAPI::_checkset_modbus_baud(int baudrate, bool check, unsigned char host_id) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  // skip checkset if check is true and (baud_checkset_flag_ is false or baudrate == 0)
  if (check && (!baud_checkset_flag_ || baudrate <= 0)) return 0;
  if (check && ((host_id == UXBUS_CONF::TGPIO_HOST_ID && modbus_baud_ == baudrate) || (host_id == UXBUS_CONF::LINEAR_TRACK_HOST_ID && linear_track_baud_ == baudrate)))
    return 0;
  int baud_inx = get_baud_inx(baudrate);
  if (baud_inx == -1) return API_CODE::MODBUS_BAUD_NOT_SUPPORT;
  int cur_baud_inx;
  int ret = _get_modbus_baudrate(&cur_baud_inx, host_id);
  if (ret == 0) {
    if (cur_baud_inx != baud_inx) {
      try {
        ignore_error_ = true;
        ignore_state_ = (state != 4 && state != 5) ? true : false;
        int state_ = state;
        // core->tgpio_addr_w16(SERVO3_RG::MODBUS_BAUDRATE, (float)baud_inx);
        core->tgpio_addr_w16(0x1a0b, (float)baud_inx, host_id);
        sleep_milliseconds(300);
        core->tgpio_addr_w16(SERVO3_RG::SOFT_REBOOT, 1, host_id);
        int err_warn[2] = { 0 };
        get_err_warn_code(err_warn);
        if (host_id == UXBUS_CONF::TGPIO_HOST_ID) {
          if (error_code == 19 || error_code == 28) {
            clean_error();
            if (ignore_state_) set_state(state_ >= 3 ? state_ : 0);
          }
          sleep_milliseconds(1000);
        }
        else {
          if (error_code == 100 + host_id) {
            clean_error();
            if (ignore_state_) set_state(state_ >= 3 ? state_ : 0);
          }
          sleep_milliseconds(1000);
        }
      }
      catch (...) {
        ignore_error_ = false;
        ignore_state_ = false;
        return API_CODE::API_EXCEPTION;
      }
      ignore_error_ = false;
      ignore_state_ = false;
      ret = _get_modbus_baudrate(&cur_baud_inx, host_id);
    }
    if (ret == 0 && cur_baud_inx < 13) {
      if (host_id == UXBUS_CONF::TGPIO_HOST_ID) {
        modbus_baud_ = BAUDRATES[cur_baud_inx];
      }
      else if (host_id == UXBUS_CONF::LINEAR_TRACK_HOST_ID) {
        linear_track_baud_ = BAUDRATES[cur_baud_inx];
      }
    }
  }
  if (host_id == UXBUS_CONF::TGPIO_HOST_ID) {
    return modbus_baud_ == baudrate ? 0 : API_CODE::MODBUS_BAUD_NOT_CORRECT;
  }
  else if (host_id == UXBUS_CONF::LINEAR_TRACK_HOST_ID) {
    return linear_track_baud_ == baudrate ? 0 : API_CODE::MODBUS_BAUD_NOT_CORRECT;
  }
  else {
    if (ret == 0 && cur_baud_inx < 13) {
      return BAUDRATES[cur_baud_inx] == baudrate ? 0 : API_CODE::MODBUS_BAUD_NOT_CORRECT;
    }
    return API_CODE::MODBUS_BAUD_NOT_CORRECT;
  }
}

int XArmAPI::set_tgpio_modbus_timeout(int timeout, bool is_transparent_transmission) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_modbus_timeout(timeout, is_transparent_transmission);
}

int XArmAPI::set_tgpio_modbus_baudrate(int baud) {
  return _checkset_modbus_baud(baud, false);
}

int XArmAPI::get_tgpio_modbus_baudrate(int *baud) {
  int cur_baud_inx = 0;
  int ret = _get_modbus_baudrate(&cur_baud_inx);
  if (ret == 0 && cur_baud_inx < 13) 
    modbus_baud_ = BAUDRATES[cur_baud_inx];
  *baud = modbus_baud_;
  return ret;
}

int XArmAPI::getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, unsigned char host_id, bool is_transparent_transmission, bool use_503_port) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  unsigned char *rx_data = new unsigned char[ret_length + 1]();
  int ret = 0;
  if (is_transparent_transmission) {
    if (use_503_port) {
      if (!_is_connected_503() && _connect_503() != 0) {
        delete[] rx_data;
        return API_CODE::NOT_CONNECTED;
      }
      ret = core503_->tgpio_set_modbus(modbus_data, modbus_length, rx_data, host_id, 0.0, true);
      
    }
    else {
      ret = core->tgpio_set_modbus(modbus_data, modbus_length, rx_data, host_id, 0.0, true);
    }
  }
  else {
    ret = core->tgpio_set_modbus(modbus_data, modbus_length, rx_data, host_id);
  }
  ret = _check_modbus_code(ret, rx_data);
  memcpy(ret_data, rx_data + 1, ret_length);
  delete[] rx_data;
  return ret;
}
