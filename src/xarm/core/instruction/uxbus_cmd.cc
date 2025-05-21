/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include <string.h>
#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/instruction/servo3_config.h"
#include "xarm/core/instruction/uxbus_cmd_config.h"
#include "xarm/core/debug/debug_print.h"

static int BAUDRATES[13] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000 };

static int get_baud_inx(int baud) {
  for (int i = 0; i < 13; i++) { if (BAUDRATES[i] == baud) return i; }
  return -1;
}

UxbusCmd::UxbusCmd(void) {
  state_is_ready = false;
  last_modbus_comm_us_ = get_us();
  last_recv_ms = get_system_time();
  has_feedback_key_transid_func_ = false;
  feedback_type_ = 0;
}

UxbusCmd::UxbusCmd(std::function<void (std::string, int, unsigned char)> set_feedback_key_transid) {
  state_is_ready = false;
  last_modbus_comm_us_ = get_us();
  last_recv_ms = get_system_time();
  has_feedback_key_transid_func_ = true;
  set_feedback_key_transid_ = set_feedback_key_transid;
  feedback_type_ = 0;
}

UxbusCmd::~UxbusCmd(void) {}

void UxbusCmd::close(void) {}

int UxbusCmd::is_ok(void) {return -1;}

int UxbusCmd::set_timeout(float timeout) {
  G_TOUT_ = (int)(timeout * 1000);
  S_TOUT_ = (int)(timeout * 1000);
  return 0;
}

int UxbusCmd::_send_modbus_request(unsigned char unit_id, unsigned char *pdu_data, unsigned short pdu_len, int prot_id)
{
  return -11;
}

int UxbusCmd::_recv_modbus_response(unsigned char t_unit_id, unsigned short t_trans_id, unsigned char *ret_data, unsigned short ret_len, int timeout, int t_prot_id)
{
  return -11;
}

int UxbusCmd::_check_private_protocol(unsigned char *data)
{
  return -11;
}


/*******************************************************
 * Uxbus generic protocol function
 *******************************************************/

int UxbusCmd::_set_nu8(int funcode, unsigned char *datas, int num, std::string feedback_key, unsigned char feedback_type) {
  std::lock_guard<std::mutex> locker(mutex_);
  bool need_set_fb = feedback_type != 0 && (feedback_type_ & feedback_type) != feedback_type;
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_ | feedback_type);
  }
  unsigned short trans_id = _get_trans_id();
  if (has_feedback_key_transid_func_ && feedback_key != "") {
    set_feedback_key_transid_(feedback_key, trans_id, feedback_type_);
  }
  int ret = _send_modbus_request(funcode, datas, num);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  int timeout = (funcode != UXBUS_RG::MOTION_EN || (funcode == UXBUS_RG::MOTION_EN && S_TOUT_ >= 5000)) ? S_TOUT_ : 5000;
  ret = _recv_modbus_response(funcode, ret, NULL, 0, timeout);
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_);
  }
  return ret;
}

int UxbusCmd::_set_nu8(int funcode, int *datas, int num, std::string feedback_key, unsigned char feedback_type) {
  unsigned char *send_data = new unsigned char[num]();
  for (int i = 0; i < num; i++) { send_data[i] = (unsigned char)datas[i]; }
  int ret = _set_nu8(funcode, send_data, num, feedback_key, feedback_type);
  delete[] send_data;
  return ret;
}

int UxbusCmd::_get_nu8(int funcode, int *rx_data, int num) {
  unsigned char *datas = new unsigned char[num]();
  int ret = _get_nu8(funcode, datas, num);
  for (int i = 0; i < num; i++) { rx_data[i] = datas[i]; }
  delete[] datas;
  return ret;
}

int UxbusCmd::_get_nu8(int funcode, unsigned char *rx_data, int num) {
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, 0, 0);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(funcode, ret, rx_data, num, G_TOUT_);
}

int UxbusCmd::_getset_nu8(int funcode, unsigned char *tx_data, int tx_num, unsigned char *rx_data, int rx_num)
{
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, tx_data, tx_num);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(funcode, ret, rx_data, rx_num, G_TOUT_);
}

int UxbusCmd::_set_nu16(int funcode, int *datas, int num, char *add_data, int add_len) {
  unsigned char *send_data = new unsigned char[num * 2 + add_len]();
  for (int i = 0; i < num; i++) { bin16_to_8(datas[i], &send_data[i * 2]); }
  for (int i = 0; i < add_len; i++) { send_data[num * 2 + i] = add_data[i]; }

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, send_data, num * 2 + add_len);
  delete[] send_data;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(funcode, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::_get_nu16(int funcode, int *rx_data, int num) {
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, 0, 0);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *datas = new unsigned char[num * 2]();
  ret = _recv_modbus_response(funcode, ret, datas, num * 2, G_TOUT_);
  for (int i = 0; i < num; i++) { rx_data[i] = bin8_to_16(&datas[i * 2]); }
  delete[] datas;
  return ret;
}

int UxbusCmd::_set_nfp32(int funcode, float *datas, int num, std::string feedback_key, unsigned char feedback_type) {
  unsigned char *send_data = new unsigned char[num * 4]();
  nfp32_to_hex(datas, send_data, num);

  std::lock_guard<std::mutex> locker(mutex_);
  bool need_set_fb = feedback_type != 0 && (feedback_type_ & feedback_type) != feedback_type;
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_ | feedback_type);
  }
  unsigned short trans_id = _get_trans_id();
  if (has_feedback_key_transid_func_ && feedback_key != "") {
    set_feedback_key_transid_(feedback_key, trans_id, feedback_type_);
  }
  int ret = _send_modbus_request(funcode, send_data, num * 4);
  delete[] send_data;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  ret = _recv_modbus_response(funcode, ret, NULL, 0, S_TOUT_);
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_);
  }
  return ret;
}

int UxbusCmd::_set_nint32(int funcode, int *datas, int num, std::string feedback_key, unsigned char feedback_type) {
  unsigned char *send_data = new unsigned char[num * 4]();
  nint32_to_hex(datas, send_data, num);

  std::lock_guard<std::mutex> locker(mutex_);
  bool need_set_fb = feedback_type != 0 && (feedback_type_ & feedback_type) != feedback_type;
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_ | feedback_type);
  }
  unsigned short trans_id = _get_trans_id();
  if (has_feedback_key_transid_func_ && feedback_key != "") {
    set_feedback_key_transid_(feedback_key, trans_id, feedback_type_);
  }
  int ret = _send_modbus_request(funcode, send_data, num * 4);
  delete[] send_data;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  ret = _recv_modbus_response(funcode, ret, NULL, 0, S_TOUT_);
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_);
  }
  return ret;
}

int UxbusCmd::_get_nfp32(int funcode, float *rx_data, int num) {
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, 0, 0);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *datas = new unsigned char[num * 4]();
  ret = _recv_modbus_response(funcode, ret, datas, num * 4, G_TOUT_);
  hex_to_nfp32(datas, rx_data, num);
  delete[] datas;
  return ret;
}

int UxbusCmd::_swop_nfp32(int funcode, float tx_datas[], int txn, float *rx_data, int rxn) {
  unsigned char *send_data = new unsigned char[128]();
  nfp32_to_hex(tx_datas, send_data, txn);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, send_data, txn * 4);
  delete[] send_data;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *datas = new unsigned char[128]();
  ret = _recv_modbus_response(funcode, ret, datas, rxn * 4, G_TOUT_);
  hex_to_nfp32(datas, rx_data, rxn);
  delete[] datas;
  return ret;
}

int UxbusCmd::_is_nfp32(int funcode, float tx_datas[], int txn, int *value) {
  unsigned char *send_data = new unsigned char[txn * 4]();
  nfp32_to_hex(tx_datas, send_data, txn);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, send_data, txn * 4);
  delete[] send_data;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[txn * 4]();
  ret = _recv_modbus_response(funcode, ret, rx_data, 1, G_TOUT_);
  *value = rx_data[0];
  delete[] rx_data;
  return ret;
}

int UxbusCmd::_set_nfp32_with_bytes(int funcode, float *tx_data, int tx_num, char *add_data, int add_len, unsigned char *rx_data, int rx_len, int timeout, std::string feedback_key, unsigned char feedback_type) {
  unsigned char *send_data = new unsigned char[tx_num * 4 + add_len]();
  nfp32_to_hex(tx_data, send_data, tx_num);
  for (int i = 0; i < add_len; i++) { send_data[tx_num * 4 + i] = add_data[i]; }

  std::lock_guard<std::mutex> locker(mutex_);
  bool need_set_fb = feedback_type != 0 && (feedback_type_ & feedback_type) != feedback_type;
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_ | feedback_type);
  }
  unsigned short trans_id = _get_trans_id();
  if (has_feedback_key_transid_func_ && feedback_key != "") {
    set_feedback_key_transid_(feedback_key, trans_id, feedback_type_);
  }
  int ret = _send_modbus_request(funcode, send_data, tx_num * 4 + add_len);
  delete[] send_data;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  ret = _recv_modbus_response(funcode, ret, rx_data, rx_len, timeout);
  if (need_set_fb && feedback_key != "") {
    _set_feedback_type_no_lock(feedback_type_);
  }
  return ret;
}

int UxbusCmd::_get_nfp32_with_bytes(int funcode, unsigned char *tx_data, int tx_num, float *rx_data, int rxn, int timeout)
{
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(funcode, tx_data, tx_num);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }

  unsigned char *datas = new unsigned char[rxn * 4]();
  ret = _recv_modbus_response(funcode, ret, datas, rxn * 4, timeout);
  hex_to_nfp32(datas, rx_data, rxn);
  delete[] datas;
  return ret;
}

/*******************************************************
 * controler setting
 *******************************************************/
int UxbusCmd::get_version(unsigned char rx_data[40]) {
  return _get_nu8(UXBUS_RG::GET_VERSION, rx_data, 40);
}

int UxbusCmd::get_robot_sn(unsigned char rx_data[40]) {
  return _get_nu8(UXBUS_RG::GET_ROBOT_SN, rx_data, 40);
}

int UxbusCmd::check_verification(int *rx_data) {
  return _get_nu8(UXBUS_RG::CHECK_VERIFY, rx_data, 1);
}

int UxbusCmd::system_control(int value) {
  return _set_nu8(UXBUS_RG::SYSTEM_CONTROL, &value, 1);
}

int UxbusCmd::set_record_traj(int value) {
  int txdata[1] = { value };
  return _set_nu8(UXBUS_RG::SET_TRAJ_RECORD, txdata, 1);
}

int UxbusCmd::playback_traj(int times, int spdx, std::string feedback_key) {
  int txdata[2] = { times, spdx };
  return _set_nint32(UXBUS_RG::PLAY_TRAJ, txdata, 2, feedback_key, FeedbackType::OTHER_FINISH);
}

int UxbusCmd::playback_traj_old(int times) {
  int txdata[1] = { times };
  return _set_nint32(UXBUS_RG::PLAY_TRAJ, txdata, 1);
}

int UxbusCmd::save_traj(char filename[81], std::string feedback_key) {
  return _set_nu8(UXBUS_RG::SAVE_TRAJ, (unsigned char*)filename, 81, feedback_key, FeedbackType::OTHER_FINISH);
}

int UxbusCmd::load_traj(char filename[81], std::string feedback_key) {
  // std::lock_guard<std::mutex> locker(mutex_);
  // int ret = _send_modbus_request(UXBUS_RG::LOAD_TRAJ, (unsigned char*)filename, 81);
  // if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  // return _recv_modbus_response(UXBUS_RG::LOAD_TRAJ, ret, NULL, 0, S_TOUT_);
  return _set_nu8(UXBUS_RG::LOAD_TRAJ, (unsigned char*)filename, 81, feedback_key, FeedbackType::OTHER_FINISH);
}

int UxbusCmd::get_traj_rw_status(int *rx_data) {
  return _get_nu8(UXBUS_RG::GET_TRAJ_RW_STATUS, rx_data, 1);
}

int UxbusCmd::set_reduced_mode(int on_off) {
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::SET_REDUCED_MODE, txdata, 1);
}

int UxbusCmd::set_reduced_linespeed(float lspd_mm) {
  float txdata[1] = { lspd_mm };
  return _set_nfp32(UXBUS_RG::SET_REDUCED_TRSV, txdata, 1);
}

int UxbusCmd::set_reduced_jointspeed(float jspd_rad) {
  float txdata[1] = { jspd_rad };
  return _set_nfp32(UXBUS_RG::SET_REDUCED_P2PV, txdata, 1);
}

int UxbusCmd::get_reduced_mode(int *rx_data) {
  return _get_nu8(UXBUS_RG::GET_REDUCED_MODE, rx_data, 1);
}

int UxbusCmd::get_reduced_states(int *on, int xyz_list[6], float *tcp_speed, float *joint_speed, float jrange_rad[14], int *fense_is_on, int *collision_rebound_is_on, int length) {
  unsigned char *rx_data = new unsigned char[length]();
  int ret = _get_nu8(UXBUS_RG::GET_REDUCED_STATE, rx_data, length);
  *on = rx_data[0];
  bin8_to_ns16(&rx_data[1], xyz_list, 6);
  *tcp_speed = hex_to_fp32(&rx_data[13]);
  *joint_speed = hex_to_fp32(&rx_data[17]);
  if (length == 79) {
    if (jrange_rad != NULL) { hex_to_nfp32(&rx_data[21], jrange_rad, 14); }
    if (fense_is_on != NULL) { *fense_is_on = rx_data[77]; }
    if (collision_rebound_is_on != NULL) { *collision_rebound_is_on = rx_data[78]; }
  }
  delete[] rx_data;
  return ret;
}

int UxbusCmd::set_xyz_limits(int xyz_list[6]) {
  return _set_nint32(UXBUS_RG::SET_LIMIT_XYZ, xyz_list, 6);
}

int UxbusCmd::set_world_offset(float pose_offset[6]) {
  return _set_nfp32(UXBUS_RG::SET_WORLD_OFFSET, pose_offset, 6);
}

int UxbusCmd::cnter_reset(void) {
  return _set_nu8(UXBUS_RG::CNTER_RESET, (int*)0, 0);
}

int UxbusCmd::cnter_plus(void) {
  return _set_nu8(UXBUS_RG::CNTER_PLUS, (int*)0, 0);
}

int UxbusCmd::set_reduced_jrange(float jrange_rad[14]) {
  return _set_nfp32(UXBUS_RG::SET_REDUCED_JRANGE, jrange_rad, 14);
}

int UxbusCmd::set_fense_on(int on_off) {
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::SET_FENSE_ON, txdata, 1);
}

int UxbusCmd::set_collis_reb(int on_off) {
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::SET_COLLIS_REB, txdata, 1);
}

int UxbusCmd::motion_en(int id, int value) {
  int txdata[2] = { id, value };
  return _set_nu8(UXBUS_RG::MOTION_EN, txdata, 2);
}

int UxbusCmd::set_state(int value) {
  return _set_nu8(UXBUS_RG::SET_STATE, &value, 1);
}

int UxbusCmd::get_state(int *rx_data) {
  return _get_nu8(UXBUS_RG::GET_STATE, rx_data, 1);
}

int UxbusCmd::get_cmdnum(int *rx_data) {
  return _get_nu16(UXBUS_RG::GET_CMDNUM, rx_data, 1);
}

int UxbusCmd::get_err_code(int * rx_data) {
  return _get_nu8(UXBUS_RG::GET_ERROR, rx_data, 2);
}

int UxbusCmd::get_hd_types(int *rx_data) {
  return _get_nu8(UXBUS_RG::GET_HD_TYPES, rx_data, 2);
}

int UxbusCmd::reload_dynamics(void) {
  int txdata[1] = { 0 };
  return _set_nu8(UXBUS_RG::RELOAD_DYNAMICS, txdata, 0);
}

int UxbusCmd::clean_err(void) {
  int txdata[1] = { 0 };
  return _set_nu8(UXBUS_RG::CLEAN_ERR, txdata, 0);
}

int UxbusCmd::clean_war(void) {
  int txdata[1] = { 0 };
  return _set_nu8(UXBUS_RG::CLEAN_WAR, txdata, 0);
}

int UxbusCmd::set_brake(int axis, int en) {
  int txdata[2] = { axis, en };
  return _set_nu8(UXBUS_RG::SET_BRAKE, txdata, 2);
}

int UxbusCmd::set_mode(int value, int detection_param) {
  if (detection_param > 0) {
    int txdata[2] = { value, detection_param };
    return _set_nu8(UXBUS_RG::SET_MODE, txdata, 2);
  }
  else {
    int txdata[1] = { value };
    return _set_nu8(UXBUS_RG::SET_MODE, txdata, 1);
  }
}

/*******************************************************
 * controler motion
 *******************************************************/
int UxbusCmd::move_line(float mvpose[6], float mvvelo, float mvacc, float mvtime, unsigned char only_check_type, unsigned char *only_check_result, unsigned char motion_type) {
  float txdata[9] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  if (only_check_type <= 0 && motion_type == 0) {
    return _set_nfp32(UXBUS_RG::MOVE_LINE, txdata, 9);
  }
  else {
    unsigned char rx_data[3] = { 0 };
    int ret = 0;
    if (motion_type == 0) {
      char additional[1] = { (char)only_check_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE, txdata, 9, additional, 1, rx_data, 3, 10000);
    }
    else {
      char additional[2] = { (char)only_check_type, (char)motion_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE, txdata, 9, additional, 2, rx_data, 3, 10000);
    }
    if (only_check_type > 0 && ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_LINE, txdata, 9);
}

int UxbusCmd::move_lineb(float mvpose[6], float mvvelo, float mvacc, float mvtime,
  float mvradii, unsigned char only_check_type, unsigned char *only_check_result, unsigned char motion_type) {
  float txdata[10] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  txdata[9] = mvradii;
  if (only_check_type <= 0 && motion_type == 0) {
    return _set_nfp32(UXBUS_RG::MOVE_LINEB, txdata, 10);
  }
  else {
    unsigned char rx_data[3] = { 0 };
    int ret = 0;
    if (motion_type == 0) {
      char additional[1] = { (char)only_check_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINEB, txdata, 10, additional, 1, rx_data, 3, 10000);
    }
    else {
      char additional[2] = { (char)only_check_type, (char)motion_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE, txdata, 10, additional, 2, rx_data, 3, 10000);
    }
    if (ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_LINEB, txdata, 10);
}

int UxbusCmd::move_joint(float mvjoint[7], float mvvelo, float mvacc,
  float mvtime, unsigned char only_check_type, unsigned char *only_check_result, std::string feedback_key) {
  float txdata[10] = { 0 };
  for (int i = 0; i < 7; i++) { txdata[i] = mvjoint[i]; }
  txdata[7] = mvvelo;
  txdata[8] = mvacc;
  txdata[9] = mvtime;
  if (only_check_type <= 0) {
    return _set_nfp32(UXBUS_RG::MOVE_JOINT, txdata, 10, feedback_key);
  }
  else {
    char additional[1] = { (char)only_check_type };
    unsigned char rx_data[3] = { 0 };
    int ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_JOINT, txdata, 10, additional, 1, rx_data, 3, 10000, feedback_key);
    if (ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_JOINT, txdata, 10);
}

int UxbusCmd::move_jointb(float mvjoint[7], float mvvelo, float mvacc, float mvradii, unsigned char only_check_type, unsigned char *only_check_result, std::string feedback_key) {
  float txdata[10] = { 0 };
  for (int i = 0; i < 7; i++) { txdata[i] = mvjoint[i]; }
  txdata[7] = mvvelo;
  txdata[8] = mvacc;
  txdata[9] = mvradii;
  if (only_check_type <= 0) {
    return _set_nfp32(UXBUS_RG::MOVE_JOINTB, txdata, 10, feedback_key);
  }
  else {
    char additional[1] = { (char)only_check_type };
    unsigned char rx_data[3] = { 0 };
    int ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_JOINTB, txdata, 10, additional, 1, rx_data, 3, 10000, feedback_key);
    if (ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_JOINTB, txdata, 10);
}

int UxbusCmd::move_line_tool(float mvpose[6], float mvvelo, float mvacc, float mvtime, unsigned char only_check_type, unsigned char *only_check_result, unsigned char motion_type) {
  float txdata[9] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  if (only_check_type <= 0 && motion_type == 0) {
    return _set_nfp32(UXBUS_RG::MOVE_LINE_TOOL, txdata, 9);
  }
  else {
    unsigned char rx_data[3] = { 0 };
    int ret = 0;
    if (motion_type == 0) {
      char additional[1] = { (char)only_check_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_TOOL, txdata, 9, additional, 1, rx_data, 3, 10000);
    }
    else {
      char additional[2] = { (char)only_check_type, (char)motion_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_TOOL, txdata, 9, additional, 2, rx_data, 3, 10000);
    }
    if (only_check_type > 0 && ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_LINE_TOOL, txdata, 9);
}

int UxbusCmd::move_gohome(float mvvelo, float mvacc, float mvtime, unsigned char only_check_type, unsigned char *only_check_result, std::string feedback_key) {
  float txdata[3] = { 0 };
  txdata[0] = mvvelo;
  txdata[1] = mvacc;
  txdata[2] = mvtime;
  if (only_check_type <= 0) {
    return _set_nfp32(UXBUS_RG::MOVE_HOME, txdata, 3, feedback_key);
  }
  else {
    char additional[1] = { (char)only_check_type };
    unsigned char rx_data[3] = { 0 };
    int ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_HOME, txdata, 3, additional, 1, rx_data, 3, 10000, feedback_key);
    if (ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_HOME, txdata, 3);
}

int UxbusCmd::move_servoj(float mvjoint[7], float mvvelo, float mvacc, float mvtime) {
  float txdata[10] = { 0 };
  for (int i = 0; i < 7; i++) { txdata[i] = mvjoint[i]; }
  txdata[7] = mvvelo;
  txdata[8] = mvacc;
  txdata[9] = mvtime;
  return _set_nfp32(UXBUS_RG::MOVE_SERVOJ, txdata, 10);
}

int UxbusCmd::move_servo_cartesian(float mvpose[6], float mvvelo, float mvacc, float mvtime) {
  float txdata[9] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  return _set_nfp32(UXBUS_RG::MOVE_SERVO_CART, txdata, 9);
}

// // this interface is no longer supported
// int UxbusCmd::set_servot(float jnt_taus[7]) {
//   float txdata[7] = { 0 };
//   for (int i = 0; i < 7; i++) { txdata[i] = jnt_taus[i]; }
//   return _set_nfp32(UXBUS_RG::SET_SERVOT, txdata, 7);
// }

int UxbusCmd::get_joint_tau(float jnt_taus[7]) {
  return _get_nfp32(UXBUS_RG::GET_JOINT_TAU, jnt_taus, 7);
}

int UxbusCmd::set_safe_level(int level) {
  int txdata[1] = { level };
  return _set_nu8(UXBUS_RG::SET_SAFE_LEVEL, txdata, 1);
}

int UxbusCmd::get_safe_level(int *level) {
  return _get_nu8(UXBUS_RG::GET_SAFE_LEVEL, level, 1);
}

int UxbusCmd::sleep_instruction(float sltime) {
  float txdata[1] = { sltime };
  return _set_nfp32(UXBUS_RG::SLEEP_INSTT, txdata, 1);
}

int UxbusCmd::move_circle(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent, unsigned char only_check_type, unsigned char *only_check_result) {

  float txdata[16] = { 0 };
  for (int i = 0; i < 6; i++) {
    txdata[i] = pose1[i];
    txdata[6 + i] = pose2[i];
  }
  txdata[12] = mvvelo;
  txdata[13] = mvacc;
  txdata[14] = mvtime;
  txdata[15] = percent;
  if (only_check_type <= 0) {
    return _set_nfp32(UXBUS_RG::MOVE_CIRCLE, txdata, 16);
  }
  else {
    char additional[1] = { (char)only_check_type };
    unsigned char rx_data[3] = { 0 };
    int ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_CIRCLE, txdata, 16, additional, 1, rx_data, 3, 10000);
    if (ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // return _set_nfp32(UXBUS_RG::MOVE_CIRCLE, txdata, 16);
}

int UxbusCmd::set_tcp_jerk(float jerk) {
  float txdata[1] = { jerk };
  return _set_nfp32(UXBUS_RG::SET_TCP_JERK, txdata, 1);
}

int UxbusCmd::set_tcp_maxacc(float maxacc) {
  float txdata[1] = { maxacc };
  return _set_nfp32(UXBUS_RG::SET_TCP_MAXACC, txdata, 1);
}

int UxbusCmd::set_joint_jerk(float jerk) {
  float txdata[1] = { jerk };
  return _set_nfp32(UXBUS_RG::SET_JOINT_JERK, txdata, 1);
}

int UxbusCmd::set_joint_maxacc(float maxacc) {
  float txdata[1] = { maxacc };
  return _set_nfp32(UXBUS_RG::SET_JOINT_MAXACC, txdata, 1);
}

int UxbusCmd::set_tcp_offset(float pose_offset[6]) {
  return _set_nfp32(UXBUS_RG::SET_TCP_OFFSET, pose_offset, 6);
}

int UxbusCmd::set_tcp_load(float mass, float load_offset[3], std::string feedback_key) {
  float txdata[4] = { mass, load_offset[0], load_offset[1], load_offset[2] };
  return _set_nfp32(UXBUS_RG::SET_LOAD_PARAM, txdata, 4, feedback_key, FeedbackType::TRIGGER);
}

int UxbusCmd::set_collis_sens(int value) {
  return _set_nu8(UXBUS_RG::SET_COLLIS_SENS, &value, 1);
}

int UxbusCmd::set_teach_sens(int value) {
  return _set_nu8(UXBUS_RG::SET_TEACH_SENS, &value, 1);
}

int UxbusCmd::set_gravity_dir(float gravity_dir[3]) {
  return _set_nfp32(UXBUS_RG::SET_GRAVITY_DIR, gravity_dir, 3);
}

int UxbusCmd::clean_conf() {
  return _set_nu8(UXBUS_RG::CLEAN_CONF, (int*)0, 0);
}

int UxbusCmd::save_conf() {
  return _set_nu8(UXBUS_RG::SAVE_CONF, (int*)0, 0);
}

int UxbusCmd::get_tcp_pose(float pose[6]) {
  return _get_nfp32(UXBUS_RG::GET_TCP_POSE, pose, 6);
}

int UxbusCmd::get_joint_pose(float angles[7]) {
  return _get_nfp32(UXBUS_RG::GET_JOINT_POS, angles, 7);
}

int UxbusCmd::get_joint_states(float position[7], float velocity[7], float effort[7], int num) {
  float fp_tmp[21] = { 0 };
  unsigned char u8_tmp = num;
  int ret = _get_nfp32_with_bytes(UXBUS_RG::GET_JOINT_POS, &u8_tmp, 1, fp_tmp, num * 7);
  memcpy(position, fp_tmp, sizeof(float) * 7);
  if (num >= 2 && velocity != NULL)
    memcpy(velocity, fp_tmp + 7, sizeof(float) * 7);
  if (num >= 3 && effort != NULL)
    memcpy(effort, fp_tmp + 14, sizeof(float) * 7);
  return ret;
}

int UxbusCmd::get_ik(float pose[6], float angles[7]) {
  return _swop_nfp32(UXBUS_RG::GET_IK, pose, 6, angles, 7);
}

int UxbusCmd::get_fk(float angles[7], float pose[6]) {
  return _swop_nfp32(UXBUS_RG::GET_FK, angles, 7, pose, 6);
}

int UxbusCmd::is_joint_limit(float joint[7], int *value) {
  return _is_nfp32(UXBUS_RG::IS_JOINT_LIMIT, joint, 7, value);
}

int UxbusCmd::is_tcp_limit(float pose[6], int *value) {
  return _is_nfp32(UXBUS_RG::IS_TCP_LIMIT, pose, 6, value);
}

/*******************************************************
 * gripper
 *******************************************************/
int UxbusCmd::gripper_addr_w16(int addr, float value) {
  unsigned char *txdata = new unsigned char[7]();
  txdata[0] = UXBUS_CONF::GRIPPER_ID;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_W16B, txdata, 7);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::TGPIO_W16B, ret, NULL, 0, G_TOUT_);
}

int UxbusCmd::gripper_addr_r16(int addr, float *value) {
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = UXBUS_CONF::GRIPPER_ID;
  bin16_to_8(addr, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_R16B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::TGPIO_R16B, ret, rx_data, 4, G_TOUT_);
  *value = (float)bin8_to_32(rx_data);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::gripper_addr_w32(int addr, float value) {
  unsigned char *txdata = new unsigned char[7]();
  txdata[0] = UXBUS_CONF::GRIPPER_ID;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_W32B, txdata, 7);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::TGPIO_W32B, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::gripper_addr_r32(int addr, float *value) {
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = UXBUS_CONF::GRIPPER_ID;
  bin16_to_8(addr, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_R32B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::TGPIO_R32B, ret, rx_data, 4, G_TOUT_);
  *value = (float)bin8_to_32(rx_data);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::gripper_set_en(int value) {
  return gripper_addr_w16(SERVO3_RG::CON_EN, (float)value);
}

int UxbusCmd::gripper_set_mode(int value) {
  return gripper_addr_w16(SERVO3_RG::CON_MODE, (float)value);
}

int UxbusCmd::gripper_set_zero() {
  return gripper_addr_w16(SERVO3_RG::MT_ZERO, 1);
}

int UxbusCmd::gripper_get_pos(float *pulse) {
  return gripper_addr_r32(SERVO3_RG::CURR_POS, pulse);
}

int UxbusCmd::gripper_set_pos(float pulse) {
  return gripper_addr_w32(SERVO3_RG::TAGET_POS, pulse);
}

int UxbusCmd::gripper_set_posspd(float speed) {
  return gripper_addr_w16(SERVO3_RG::POS_SPD, speed);
}

int UxbusCmd::gripper_get_errcode(int rx_data[2]) {
  return _get_nu8(UXBUS_RG::TGPIO_ERR, rx_data, 2);
}

int UxbusCmd::gripper_clean_err() {
  return gripper_addr_w16(SERVO3_RG::RESET_ERR, 1);
}

/*******************************************************
 * tool gpio
 *******************************************************/
int UxbusCmd::tgpio_addr_w16(int addr, float value, unsigned char host_id, char *add_data, int add_len) {
  unsigned char *txdata = new unsigned char[7 + add_len]();
  txdata[0] = host_id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);
  for (int i = 0; i < add_len; i++) { txdata[7 + i] = add_data[i]; }

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_W16B, txdata, 7);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::TGPIO_W16B, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::tgpio_addr_r16(int addr, float *value, unsigned char host_id) {
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = host_id;
  bin16_to_8(addr, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_R16B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::TGPIO_R16B, ret, rx_data, 4, G_TOUT_);
  *value = (float)bin8_to_32(rx_data);
  delete[] rx_data;
  return ret;
}
int UxbusCmd::tgpio_addr_w32(int addr, float value, unsigned char host_id) {
  unsigned char *txdata = new unsigned char[7]();
  txdata[0] = host_id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_W32B, txdata, 7);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::TGPIO_W32B, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::tgpio_addr_r32(int addr, float *value, unsigned char host_id) {
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = host_id;
  bin16_to_8(addr, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::TGPIO_R32B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::TGPIO_R32B, ret, rx_data, 4, G_TOUT_);
  *value = (float)bin8_to_32(rx_data);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::tgpio_get_digital(int *io0, int *io1, int *io2, int *io3, int *io4) {
  float tmp;
  int ret = tgpio_addr_r16(SERVO3_RG::DIGITAL_IN, &tmp);

  *io0 = (int)tmp & 0x0001;
  *io1 = ((int)tmp & 0x0002) >> 1;
  if (io3 != NULL)
    *io3 = ((int)tmp & 0x0004) >> 2;
  if (io4 != NULL)
    *io4 = ((int)tmp & 0x0008) >> 3;
  if (io2 != NULL) {
    int ret2 = tgpio_addr_r16(0x0A12, &tmp);
    *io2 = (int)tmp & 0x0001;
    return ret2 != 0 ? ret2 : ret;
  }
  return ret;
}

int UxbusCmd::tgpio_set_digital(int ionum, int value, int sync) {
  int tmp = 0;
  switch (ionum) {
    case 1:
    {
      tmp = tmp | 0x0100;
      if (value) { tmp = tmp | 0x0001; }
      break;
    }
    case 2:
    {
      tmp = tmp | 0x0200;
      if (value) { tmp = tmp | 0x0002; }
      break;
    }
    case 3:
    {
      tmp = tmp | 0x1000;
      if (value) { tmp = tmp | 0x0010; }
      break;
    }
    case 4:
    {
      tmp = tmp | 0x0400;
      if (value) { tmp = tmp | 0x0004; }
      break;
    }
    case 5:
    {
      tmp = tmp | 0x0800;
      if (value) { tmp = tmp | 0x0008; }
      break;
    }
    default:
      return -1; 
  }
  if (sync >= 0) {
    char additional[1] = { (char)sync };
    return tgpio_addr_w16(SERVO3_RG::DIGITAL_OUT, (float)tmp, UXBUS_CONF::TGPIO_HOST_ID, additional, 1);
  }
  return tgpio_addr_w16(SERVO3_RG::DIGITAL_OUT, (float)tmp);
}

int UxbusCmd::tgpio_get_analog1(float * value) {
  float tmp;
  int ret = tgpio_addr_r16(SERVO3_RG::ANALOG_IO1, &tmp);
  *value = (float)(tmp * 3.3 / 4095.0);
  return ret;
}

int UxbusCmd::tgpio_get_analog2(float * value) {
  float tmp;
  int ret = tgpio_addr_r16(SERVO3_RG::ANALOG_IO2, &tmp);
  // printf("tmp = %f\n", tmp);
  *value = (float)(tmp * 3.3 / 4095.0);
  return ret;
}

/*******************************************************
 * tgpio modbus
 *******************************************************/

int UxbusCmd::set_modbus_timeout(int value, bool is_transparent_transmission) {
  return _set_nu16(is_transparent_transmission ? UXBUS_RG::TGPIO_COM_TIOUT : UXBUS_RG::TGPIO_MB_TIOUT, &value, 1);
}

int UxbusCmd::set_modbus_baudrate(int baud) {
  float val = 0;
  int baud_inx = get_baud_inx(baud);
  if (baud_inx == -1) return -1;
  int ret = tgpio_addr_r16(SERVO3_RG::MODBUS_BAUDRATE & 0x0FFF, &val);
  if (ret == 0) {
    int baud_i = (int)val;
    if (baud_i != baud_inx) {
      // tgpio_addr_w16(SERVO3_RG::MODBUS_BAUDRATE, (float)baud_inx);
      tgpio_addr_w16(0x1a0b, (float)baud_inx);
      sleep_milliseconds(300);
      return tgpio_addr_w16(SERVO3_RG::SOFT_REBOOT, 1);
    }
  }
  return ret;
}

int UxbusCmd::tgpio_set_modbus(unsigned char *modbus_t, int len_t, unsigned char *rx_data, unsigned char host_id, float limit_sec, bool is_transparent_transmission) {
  unsigned char *txdata = new unsigned char[len_t + 1]();
  txdata[0] = host_id;
  for (int i = 0; i < len_t; i++) { txdata[i + 1] = modbus_t[i]; }

  std::lock_guard<std::mutex> locker(mutex_);
  if (limit_sec > 0) {
    long long diff_us = get_us() - last_modbus_comm_us_;
    long long limit_us = (long long)(limit_sec * 1000000);
    if (diff_us < limit_us) sleep_us(limit_us - diff_us);
  }
  int ret = _send_modbus_request(is_transparent_transmission ? UXBUS_RG::TGPIO_COM_DATA : UXBUS_RG::TGPIO_MODBUS, txdata, len_t + 1);
  delete[] txdata;
  if (-1 == ret) { 
    last_modbus_comm_us_ = get_us();
    return UXBUS_STATE::ERR_NOTTCP;
  }

  ret = _recv_modbus_response(is_transparent_transmission ? UXBUS_RG::TGPIO_COM_DATA : UXBUS_RG::TGPIO_MODBUS, ret, rx_data, -1, S_TOUT_);
  last_modbus_comm_us_ = get_us();
  return ret;
}

int UxbusCmd::gripper_modbus_w16s(int addr, float value, int len) {
  unsigned char *txdata = new unsigned char[11]();
  unsigned char *rx_data = new unsigned char[254]();
  txdata[0] = UXBUS_CONF::GRIPPER_ID;
  txdata[1] = 0x10;
  bin16_to_8(addr, &txdata[2]);
  bin16_to_8(len, &txdata[4]);
  txdata[6] = len * 2;
  fp32_to_hex(value, &txdata[7]);
  int ret = tgpio_set_modbus(txdata, len * 2 + 7, rx_data);
  delete[] txdata;
  delete[] rx_data;
  return ret;
}

int UxbusCmd::gripper_modbus_r16s(int addr, int len, unsigned char *rx_data) {
  unsigned char *txdata = new unsigned char[6]();
  txdata[0] = UXBUS_CONF::GRIPPER_ID;
  txdata[1] = 0x03;
  bin16_to_8(addr, &txdata[2]);
  bin16_to_8(len, &txdata[4]);
  int ret = tgpio_set_modbus(txdata, 6, rx_data);
  delete[] txdata;
  return ret;
}

int UxbusCmd::gripper_modbus_set_en(int value) {
  unsigned char *txdata = new unsigned char[4]();
  bin16_to_8(value, &txdata[0]);
  float _value = hex_to_fp32(txdata);
  delete[] txdata;
  return gripper_modbus_w16s(SERVO3_RG::CON_EN, _value, 1);
}

int UxbusCmd::gripper_modbus_set_mode(int value) {
  unsigned char *txdata = new unsigned char[4]();
  bin16_to_8(value, &txdata[0]);
  float _value = hex_to_fp32(txdata);
  delete[] txdata;
  return gripper_modbus_w16s(SERVO3_RG::CON_MODE, _value, 1);
}

int UxbusCmd::gripper_modbus_set_zero(void) {
  return gripper_modbus_w16s(SERVO3_RG::MT_ZERO, 1, 1);
}

int UxbusCmd::gripper_modbus_get_pos(float *pulse) {
  unsigned char *rx_data = new unsigned char[254]();
  int ret = gripper_modbus_r16s(SERVO3_RG::CURR_POS, 2, rx_data);
  *pulse = (float)bin8_to_32(&rx_data[4]);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::gripper_modbus_set_pos(float pulse) {
  unsigned char *txdata = new unsigned char[4]();
  txdata[0] = ((int)pulse >> 24) & 0xFF;
  txdata[1] = ((int)pulse >> 16) & 0xFF;
  txdata[2] = ((int)pulse >> 8) & 0xFF;
  txdata[3] = (int)pulse & 0xFF;
  float value = hex_to_fp32(txdata);
  delete[] txdata;
  return gripper_modbus_w16s(SERVO3_RG::TAGET_POS, value, 2);
}

int UxbusCmd::gripper_modbus_set_posspd(float speed) {
  unsigned char *txdata = new unsigned char[4]();
  bin16_to_8((int)speed, &txdata[0]);
  float value = hex_to_fp32(txdata);
  delete[] txdata;
  return gripper_modbus_w16s(SERVO3_RG::POS_SPD, value, 1);
}

int UxbusCmd::gripper_modbus_get_errcode(int *err) {
  unsigned char *rx_data = new unsigned char[254]();
  int ret = gripper_modbus_r16s(SERVO3_RG::ERR_CODE, 1, rx_data);
  *err = bin8_to_16(&rx_data[4]);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::gripper_modbus_clean_err(void) {
  return gripper_modbus_w16s(SERVO3_RG::RESET_ERR, 1, 1);
}

/*******************************************************
 * uservo
 *******************************************************/
int UxbusCmd::servo_set_zero(int id) {
  return _set_nu8(UXBUS_RG::SERVO_ZERO, &id, 1);
}

int UxbusCmd::servo_get_dbmsg(int rx_data[16]) {
  return _get_nu8(UXBUS_RG::SERVO_DBMSG, rx_data, 16);
}

int UxbusCmd::servo_addr_w16(int id, int addr, float value) {
  unsigned char *txdata = new unsigned char[7]();
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::SERVO_W16B, txdata, 7);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::SERVO_W16B, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::servo_addr_r16(int id, int addr, float *value) {
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::SERVO_R16B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::SERVO_R16B, ret, rx_data, 4, G_TOUT_);
  *value = (float)bin8_to_32(rx_data);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::servo_addr_w32(int id, int addr, float value) {
  unsigned char *txdata = new unsigned char[7]();
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);
  fp32_to_hex(value, &txdata[3]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::SERVO_W32B, txdata, 7);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::SERVO_W32B, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::servo_addr_r32(int id, int addr, float *value) {
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = id;
  bin16_to_8(addr, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::SERVO_R32B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::SERVO_R32B, ret, rx_data, 4, G_TOUT_);
  *value = (float)bin8_to_32(rx_data);
  delete[] rx_data;
  return ret;
}



/*******************************************************
 * controler gpio
 *******************************************************/
int UxbusCmd::cgpio_get_auxdigit(int *value) {
  return _get_nu16(UXBUS_RG::CGPIO_GET_DIGIT, value, 1);
}
int UxbusCmd::cgpio_get_analog1(float *value) {
  int tmp = 0;
  int ret = _get_nu16(UXBUS_RG::CGPIO_GET_ANALOG1, &tmp, 1);
  *value = (float)(tmp * 10.0 / 4095.0);
  return ret;
}
int UxbusCmd::cgpio_get_analog2(float *value) {
  int tmp = 0;
  int ret = _get_nu16(UXBUS_RG::CGPIO_GET_ANALOG2, &tmp, 1);
  *value = (float)(tmp * 10.0 / 4095.0);
  return ret;
}
int UxbusCmd::cgpio_set_auxdigit(int ionum, int value, int sync) {
  if (ionum > 7) {
    int tmp[2] = {0, 0};
    tmp[1] = tmp[1] | (0x0100 << (ionum - 8));
    if (value)
    {
      tmp[1] = tmp[1] | (0x0001 << (ionum - 8));
    }
    if (sync >= 0) {
      char additional[1] = { (char)sync };
      return _set_nu16(UXBUS_RG::CGPIO_SET_DIGIT, tmp, 2, additional, 1);
    }
    return _set_nu16(UXBUS_RG::CGPIO_SET_DIGIT, tmp, 2);
  }
  else {
    int tmp = 0;
    tmp = tmp | (0x0100 << ionum);
    if (value)
    {
      tmp = tmp | (0x0001 << ionum);
    }
    if (sync >= 0) {
      char additional[1] = { (char)sync };
      return _set_nu16(UXBUS_RG::CGPIO_SET_DIGIT, &tmp, 1, additional, 1);
    }
    return _set_nu16(UXBUS_RG::CGPIO_SET_DIGIT, &tmp, 1);
  }
}

int UxbusCmd::cgpio_set_analog1(float value, int sync) {
  int val = (int)(value / 10.0 * 4095.0);
  if (sync >= 0) {
    char additional[1] = { (char)sync };
    return _set_nu16(UXBUS_RG::CGPIO_SET_ANALOG1, &val, 1, additional, 1);
  }
  return _set_nu16(UXBUS_RG::CGPIO_SET_ANALOG1, &val, 1);
}

int UxbusCmd::cgpio_set_analog2(float value, int sync) {
  int val = (int)(value / 10.0 * 4095.0);
  if (sync >= 0) {
    char additional[1] = { (char)sync };
    return _set_nu16(UXBUS_RG::CGPIO_SET_ANALOG2, &val, 1, additional, 1);
  }
  return _set_nu16(UXBUS_RG::CGPIO_SET_ANALOG2, &val, 1);
}

int UxbusCmd::cgpio_set_infun(int num, int fun) {
  int txdata[2] = { num, fun };
  return _set_nu8(UXBUS_RG::CGPIO_SET_IN_FUN, txdata, 2);
}
int UxbusCmd::cgpio_set_outfun(int num, int fun) {
  int txdata[2] = { num, fun };
  return _set_nu8(UXBUS_RG::CGPIO_SET_OUT_FUN, txdata, 2);
}

/**
 * get controler gpio all state infomation
 * 
 * @param state     [description]
 * @param digit_io: [digital input functional gpio state,
                     digital input configuring gpio state,
                     digital output functional gpio state,
                     digital output configuring gpio state]
 * @param analog: [analog-1 input value,
                   analog-2 input value,
                   analog-1 output value,
                   analog-2 output value]
 * @param input_conf: [digital(0-7) input functional info]
 * @param output_conf: [digital(0-7) output functional info]
 * @param input_conf2: [digital(8-15) input functional info]
 * @param output_conf2: [digital(8-15) output functional info]
 * @return: [description]
 */
int UxbusCmd::cgpio_get_state(int *state, int *digit_io, float *analog, int *input_conf, int *output_conf, int *input_conf2, int *output_conf2) {
  unsigned char *rx_data = new unsigned char[50]();
  int ret = _get_nu8(UXBUS_RG::CGPIO_GET_STATE, rx_data, -1);

  state[0] = rx_data[0];
  state[1] = rx_data[1];
  for (int i = 0; i < 4; i++) {
    digit_io[i] = bin8_to_16(&rx_data[2 + i * 2]);
    analog[i] = (float)(bin8_to_16(&rx_data[10 + i * 2]) / 4095.0 * 10.0);
  }
  for (int i = 0; i < 8; i++) {
    input_conf[i] = rx_data[18 + i];
    output_conf[i] = rx_data[26 + i];
  }
  if (input_conf2 != NULL) {
    for (int i = 0; i < 8; i++) {
      input_conf2[i] = rx_data[34 + i];
    }
  }
  if (output_conf2 != NULL) {
    for (int i = 0; i < 8; i++) {
      output_conf2[i] = rx_data[42 + i];
    }
  }
  delete[] rx_data;
  return ret;
}

int UxbusCmd::get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in, int orient_type_out) {
  float txdata[14] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = pose1[i]; }
  for (int i = 0; i < 6; i++) { txdata[6+i] = pose2[i]; }
  unsigned char *hexdata = new unsigned char[50]();
  nfp32_to_hex(txdata, hexdata, 12);
  hexdata[48] = orient_type_in;
  hexdata[49] = orient_type_out;

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::CAL_POSE_OFFSET, hexdata, 50);
  delete[] hexdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *datas = new unsigned char[24]();
  ret = _recv_modbus_response(UXBUS_RG::CAL_POSE_OFFSET, ret, datas, 24, G_TOUT_);
  hex_to_nfp32(datas, offset, 6);
  delete[] datas;
  return ret;
}

int UxbusCmd::get_position_aa(float pose[6]) {
  return _get_nfp32(UXBUS_RG::GET_TCP_POSE_AA, pose, 6);
}

int UxbusCmd::move_line_aa(float mvpose[6], float mvvelo, float mvacc, float mvtime, int mvcoord, int relative, unsigned char only_check_type, unsigned char *only_check_result, unsigned char motion_type) {
  float txdata[9] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  if (only_check_type <= 0 && motion_type == 0) {
    char additional[2] = { (char)mvcoord, (char)relative };
    return _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_AA, txdata, 9, additional, 2);
  }
  else {
    unsigned char rx_data[3] = { 0 };
    int ret = 0;
    if (motion_type == 0) {
      char additional[3] = { (char)mvcoord, (char)relative, (char)only_check_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_AA, txdata, 9, additional, 3, rx_data, 3, 10000);
    }
    else {
      char additional[4] = { (char)mvcoord, (char)relative, (char)only_check_type, (char)motion_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_AA, txdata, 9, additional, 4, rx_data, 3, 10000);
    }
    if (only_check_type > 0 && ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // char additional[2] = { (char)mvcoord, (char)relative };
  // return _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE_AA, txdata, 9, additional, 2);
}

int UxbusCmd::move_servo_cart_aa(float mvpose[6], float mvvelo, float mvacc, int tool_coord, int relative) {
  float txdata[9] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = (char)tool_coord;
  char additional[1] = { (char)relative };
  return _set_nfp32_with_bytes(UXBUS_RG::MOVE_SERVO_CART_AA, txdata, 9, additional, 1);
}

int UxbusCmd::move_relative(float mvpose[7], float mvvelo, float mvacc, float mvtime, float radius, int is_joint_motion, bool is_axis_angle, unsigned char only_check_type, unsigned char *only_check_result, unsigned char motion_type, std::string feedback_key)
{
  float txdata[11] = { 0 };
  for (int i = 0; i < 7; i++) { txdata[i] = mvpose[i]; }
  txdata[7] = mvvelo;
  txdata[8] = mvacc;
  txdata[9] = mvtime;
  txdata[10] = radius;
  if (only_check_type <= 0 && motion_type == 0) {
    char additional[2] = { (char)is_joint_motion, (char)is_axis_angle  };
    return _set_nfp32_with_bytes(UXBUS_RG::MOVE_RELATIVE, txdata, 11, additional, 2, NULL, 0, 2000, feedback_key);
  }
  else {
    unsigned char rx_data[3] = { 0 };
    int ret = 0;
    if (motion_type == 0) {
      char additional[3] = { (char)is_joint_motion, (char)is_axis_angle , (char)only_check_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_RELATIVE, txdata, 11, additional, 3, rx_data, 3, 10000, feedback_key);
    }
    else {
      char additional[4] = { (char)is_joint_motion, (char)is_axis_angle , (char)only_check_type, (char)motion_type };
      ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_RELATIVE, txdata, 11, additional, 4, rx_data, 3, 10000, feedback_key);
    }
    if (only_check_type > 0 && ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
    return ret;
  }
  // char additional[2] = { (char)is_joint_motion, (char)is_axis_angle };
  // return _set_nfp32_with_bytes(UXBUS_RG::MOVE_RELATIVE, txdata, 11, additional, 2);
}

int UxbusCmd::tgpio_delay_set_digital(int ionum, int value, float delay_sec) {
  unsigned char *txdata = new unsigned char[6]();
  txdata[0] = ionum;
  txdata[1] = value;
  fp32_to_hex(delay_sec, &txdata[2]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::DELAYED_TGPIO_SET, txdata, 6);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::DELAYED_TGPIO_SET, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::cgpio_delay_set_digital(int ionum, int value, float delay_sec) {
  unsigned char *txdata = new unsigned char[6]();
  txdata[0] = ionum;
  txdata[1] = value;
  fp32_to_hex(delay_sec, &txdata[2]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::DELAYED_CGPIO_SET, txdata, 6);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::DELAYED_CGPIO_SET, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::tgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r) {
  unsigned char *txdata = new unsigned char[18]();
  txdata[0] = ionum;
  txdata[1] = value;
  nfp32_to_hex(xyz, &txdata[2], 3);
  fp32_to_hex(tol_r, &txdata[14]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::POSITION_TGPIO_SET, txdata, 18);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::POSITION_TGPIO_SET, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::cgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r) {
  unsigned char *txdata = new unsigned char[18]();
  txdata[0] = ionum;
  txdata[1] = value;
  nfp32_to_hex(xyz, &txdata[2], 3);
  fp32_to_hex(tol_r, &txdata[14]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::POSITION_CGPIO_SET, txdata, 18);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::POSITION_CGPIO_SET, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::cgpio_position_set_analog(int ionum, float value, float xyz[3], float tol_r) {
  unsigned char *txdata = new unsigned char[19]();
  txdata[0] = ionum;
  int val = (int)(value / 10.0 * 4095.0);
  bin16_to_8(val, &txdata[1]);
  nfp32_to_hex(xyz, &txdata[3], 3);
  fp32_to_hex(tol_r, &txdata[15]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::POSITION_CGPIO_SET_ANALOG, txdata, 19);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  return _recv_modbus_response(UXBUS_RG::POSITION_CGPIO_SET_ANALOG, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::config_io_stop_reset(int io_type, int val) {
  int txdata[2] = { io_type, val };
  return _set_nu8(UXBUS_RG::SET_IO_STOP_RESET, txdata, 2);
}

int UxbusCmd::set_report_tau_or_i(int tau_or_i) {
  int txdata[1] = { tau_or_i };
  return _set_nu8(UXBUS_RG::REPORT_TAU_OR_I, txdata, 1);
}

int UxbusCmd::get_report_tau_or_i(int *rx_data) {
  return _get_nu8(UXBUS_RG::GET_REPORT_TAU_OR_I, rx_data, 1);
}

int UxbusCmd::set_self_collision_detection(int on_off) {
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::SET_SELF_COLLIS_CHECK, txdata, 1);
}

int UxbusCmd::set_collision_tool_model(int tool_type, int n, float *argv) {
  if (n > 0) {
    char additional[1] = { (char)tool_type };
    return _set_nfp32_with_bytes(UXBUS_RG::SET_COLLIS_TOOL, argv, n, additional, 1);
  }
  else {
    int txdata[1] = { tool_type };
    return _set_nu8(UXBUS_RG::SET_COLLIS_TOOL, txdata, 1);
  }
}

int UxbusCmd::set_simulation_robot(int on_off) {
  return _set_nu8(UXBUS_RG::SET_SIMULATION_ROBOT, &on_off, 1);
}

int UxbusCmd::vc_set_jointv(float jnt_v[7], int jnt_sync, float duration) {
  float txdata[7] = { 0 };
  for (int i = 0; i < 7; i++) { txdata[i] = jnt_v[i]; }
  char additional[5] = { (char)jnt_sync, 0, 0, 0, 0 };
  fp32_to_hex(duration, (unsigned char *)&additional[1]);
  return _set_nfp32_with_bytes(UXBUS_RG::VC_SET_JOINTV, txdata, 7, additional, duration >= 0 ? 5 : 1);
}

int UxbusCmd::vc_set_linev(float line_v[6], int coord, float duration) {
  float txdata[6] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = line_v[i]; }
  char additional[5] = { (char)coord, 0, 0, 0, 0 };
  fp32_to_hex(duration, (unsigned char *)&additional[1]);
  return _set_nfp32_with_bytes(UXBUS_RG::VC_SET_CARTV, txdata, 6, additional, duration >= 0 ? 5 : 1);
}

int UxbusCmd::cali_tcp_pose(float four_pnts[4][6], float ret_xyz[3])
{
  float txdata[24] = { 0 };
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 6; i++) { txdata[j*6 + i] = four_pnts[j][i]; }
  }
  return _swop_nfp32(UXBUS_RG::CALI_TCP_POSE, txdata, 24, ret_xyz, 3);
}

int UxbusCmd::cali_user_orient(float three_pnts[3][6], float ret_rpy[3], int mode, int trust_ind)
{
  float txdata[18] = { 0 };
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 6; i++) { txdata[j*6 + i] = three_pnts[j][i]; }
  }
  int rx_len = 12;
  unsigned char *rx_data = new unsigned char[rx_len]();
  char additional[2] = { (char)mode, (char)trust_ind };
  int ret = _set_nfp32_with_bytes(UXBUS_RG::CALI_WRLD_ORIENT, txdata, 18, additional, 2, rx_data, rx_len);
  hex_to_nfp32(rx_data, ret_rpy, 3);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::cali_tcp_orient(float rpy_be[3], float rpy_bt[3], float ret_rpy[3])
{
  float txdata[6] = { 0 };
  for (int j = 0; j < 3; j++) {
    txdata[j] = rpy_be[j];
    txdata[j+3] = rpy_bt[j];
  }
  return _swop_nfp32(UXBUS_RG::CALI_TCP_ORIENT, txdata, 6, ret_rpy, 3);
}

int UxbusCmd::cali_user_pos(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3])
{
  float txdata[6] = { 0 };
  for (int j = 0; j < 3; j++) {
    txdata[j] = rpy_ub[j];
    txdata[j+3] = pos_b_uorg[j];
  }
  return _swop_nfp32(UXBUS_RG::CALI_WRLD_POSE, txdata, 6, ret_xyz, 3);
}

int UxbusCmd::iden_load(int iden_type, float *rx_data, int num_get, int timeout, float estimated_mass)
{
  unsigned char tx_data[5] = {0};
  tx_data[0] = (unsigned char)iden_type;
  if (estimated_mass > 0)
    fp32_to_hex(estimated_mass, &tx_data[1]);
  return _get_nfp32_with_bytes(UXBUS_RG::IDEN_LOAD, tx_data, estimated_mass > 0 ? 5 : 1, rx_data, num_get, timeout);
  // return _get_nfp32_with_bytes(UXBUS_RG::IDEN_LOAD, (unsigned char *)&iden_type, 1, rx_data, num_get, timeout);
}

int UxbusCmd::iden_joint_friction(unsigned char sn[14], float *rx_data)
{
  return _get_nfp32_with_bytes(UXBUS_RG::IDEN_FRIC, sn, 14, rx_data, 1, 500000);
}

int UxbusCmd::set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6])
{
  unsigned char tx_data[79] = {0};
  tx_data[0] = (unsigned char)coord;
  for (unsigned int i = 0; i < 6; i++) {
    tx_data[i+1] = (unsigned char)c_axis[i];
  }

  nfp32_to_hex(M, &tx_data[7], 6);
  nfp32_to_hex(K, &tx_data[31], 6);
  nfp32_to_hex(B, &tx_data[55], 6);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::IMPEDANCE_CONFIG, tx_data, 79);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }

  return _recv_modbus_response(UXBUS_RG::IMPEDANCE_CONFIG, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::set_impedance_mbk(float M[6], float K[6], float B[6])
{
  unsigned char tx_data[72] = {0};

  nfp32_to_hex(M, &tx_data[0], 6);
  nfp32_to_hex(K, &tx_data[24], 6);
  nfp32_to_hex(B, &tx_data[48], 6);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::IMPEDANCE_CTRL_MBK, tx_data, 72);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }

  return _recv_modbus_response(UXBUS_RG::IMPEDANCE_CTRL_MBK, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::set_impedance_config(int coord, int c_axis[6])
{
  unsigned char tx_data[7] = {0};
  tx_data[0] = (unsigned char)coord;
  for (unsigned int i = 0; i < 6; i++) {
    tx_data[i+1] = (unsigned char)c_axis[i];
  }

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::IMPEDANCE_CTRL_CONFIG, tx_data, 7);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }

  return _recv_modbus_response(UXBUS_RG::IMPEDANCE_CTRL_CONFIG, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6])
{
  unsigned char tx_data[55] = {0};
  tx_data[0] = (unsigned char)coord;
  for (unsigned int i = 0; i < 6; i++) {
    tx_data[i+1] = (unsigned char)c_axis[i];
  }

  nfp32_to_hex(f_ref, &tx_data[7], 6);
  nfp32_to_hex(limits, &tx_data[31], 6);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::FORCE_CTRL_CONFIG, tx_data, 55);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }

  return _recv_modbus_response(UXBUS_RG::FORCE_CTRL_CONFIG, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
  unsigned char tx_data[96] = {0};

  nfp32_to_hex(kp, &tx_data[0], 6);
  nfp32_to_hex(ki, &tx_data[24], 6);
  nfp32_to_hex(kd, &tx_data[48], 6);
  nfp32_to_hex(xe_limit, &tx_data[72], 6);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::FORCE_CTRL_PID, tx_data, 96);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }

  return _recv_modbus_response(UXBUS_RG::FORCE_CTRL_PID, ret, NULL, 0, S_TOUT_);
}

int UxbusCmd::ft_sensor_set_zero(void)
{
  int txdata[1] = { 0 };
  return _set_nu8(UXBUS_RG::FTSENSOR_SET_ZERO, txdata, 0);
}

int UxbusCmd::ft_sensor_iden_load(float result[10])
{
  return iden_load(0, result, 10, 500000);
}

int UxbusCmd::ft_sensor_cali_load(float load[10])
{
  return _set_nfp32(UXBUS_RG::FTSENSOR_CALI_LOAD_OFFSET, load, 10);
}

int UxbusCmd::ft_sensor_enable(int on_off)
{
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::FTSENSOR_ENABLE, txdata, 1);
}

int UxbusCmd::ft_sensor_app_set(int app_code)
{
  int txdata[1] = { app_code };
  return _set_nu8(UXBUS_RG::FTSENSOR_SET_APP, txdata, 1);
}

int UxbusCmd::ft_sensor_app_get(int *app_code)
{
  return _get_nu8(UXBUS_RG::FTSENSOR_GET_APP, app_code, 1);
}

int UxbusCmd::ft_sensor_get_data(float ft_data[6], bool is_new)
{
  return _get_nfp32(is_new ? UXBUS_RG::FTSENSOR_GET_DATA : UXBUS_RG::FTSENSOR_GET_DATA_OLD, ft_data, 6);
}

int UxbusCmd::ft_sensor_get_config(int *ft_app_status, int *ft_is_started, int *ft_type, int *ft_id, int *ft_freq, 
  float *ft_mass, float *ft_dir_bias, float ft_centroid[3], float ft_zero[6], int *imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6],
  int *f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6])
{
  unsigned char data[280];
  int ret = _get_nu8(UXBUS_RG::FTSENSOR_GET_CONFIG, data, 280);
  if (ft_app_status != NULL) *ft_app_status = data[0];
  if (ft_is_started != NULL) *ft_is_started = data[1];
  if (ft_type != NULL) *ft_type = data[2];
  if (ft_id != NULL) *ft_id = data[3];
  if (ft_freq != NULL) *ft_freq = bin8_to_16(&data[4]);
  if (ft_mass != NULL) *ft_mass = hex_to_fp32(&data[6]);
  if (ft_dir_bias != NULL) *ft_dir_bias = hex_to_fp32(&data[10]);
  if (ft_centroid != NULL) hex_to_nfp32(&data[14], ft_centroid, 3);
  if (ft_zero != NULL) hex_to_nfp32(&data[26], ft_zero, 6);
  if (imp_coord != NULL) *imp_coord = data[50];
  if (imp_c_axis != NULL) { for(int i = 0; i < 6; i++) imp_c_axis[i] = data[51+i]; };
  if (M != NULL) hex_to_nfp32(&data[57], M, 6);
  if (K != NULL) hex_to_nfp32(&data[81], K, 6);
  if (B != NULL) hex_to_nfp32(&data[105], B, 6);

  if (f_coord != NULL) *f_coord = data[129];
  if (f_c_axis != NULL) { for(int i = 0; i < 6; i++) f_c_axis[i] = data[130+i]; };
  if (f_ref != NULL) hex_to_nfp32(&data[136], f_ref, 6);
  if (f_limits != NULL) hex_to_nfp32(&data[160], f_limits, 6);
  if (kp != NULL) hex_to_nfp32(&data[184], kp, 6);
  if (ki != NULL) hex_to_nfp32(&data[208], ki, 6);
  if (kd != NULL) hex_to_nfp32(&data[232], kd, 6);
  if (xe_limit != NULL) hex_to_nfp32(&data[256], xe_limit, 6);

  return ret;
}

int UxbusCmd::ft_sensor_get_error(int *err)
{
  unsigned char *txdata = new unsigned char[3]();
  txdata[0] = 8;
  bin16_to_8(0x0010, &txdata[1]);

  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(UXBUS_RG::SERVO_R16B, txdata, 3);
  delete[] txdata;
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  unsigned char *rx_data = new unsigned char[4]();
  ret = _recv_modbus_response(UXBUS_RG::SERVO_R16B, ret, rx_data, 4, G_TOUT_);
  if (ret == 0 || ret == 1 || ret == 2) {
    if (bin8_to_32(rx_data) == 27) {
      *err = 0;
    }
    else {
      *err = rx_data[2];
    }
  }
  return ret;
}

int UxbusCmd::iden_tcp_load(float result[4], float estimated_mass)
{
  return iden_load(1, result, 4, 500000, estimated_mass);
}

int UxbusCmd::track_modbus_r16s(int addr, unsigned char *rx_data, int len, unsigned char fcode)
{
  unsigned char *txdata = new unsigned char[6]();
  txdata[0] = UXBUS_CONF::TRACK_ID;
  txdata[1] = fcode;
  bin16_to_8(addr, &txdata[2]);
  bin16_to_8(len, &txdata[4]);
  int ret = tgpio_set_modbus(txdata, 6, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID, (float)0.001);
  delete[] txdata;
  return ret;
}

int UxbusCmd::track_modbus_w16s(int addr, unsigned char *send_data, int len, unsigned char *rx_data)
{
  unsigned char *txdata = new unsigned char[7 + len * 2]();
  txdata[0] = UXBUS_CONF::TRACK_ID;
  txdata[1] = 0x10;
  bin16_to_8(addr, &txdata[2]);
  bin16_to_8(len, &txdata[4]);
  txdata[6] = len * 2;
  memcpy(&txdata[7], send_data, len * 2);
  int ret = tgpio_set_modbus(txdata, len * 2 + 7, rx_data, UXBUS_CONF::LINEAR_TRACK_HOST_ID, (float)0.001);
  delete[] txdata;
  return ret;
}

int UxbusCmd::set_cartesian_velo_continuous(int on_off)
{
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::SET_CARTV_CONTINUE, txdata, 1);
}

int UxbusCmd::set_allow_approx_motion(int on_off)
{
  int txdata[1] = { on_off };
  return _set_nu8(UXBUS_RG::ALLOW_APPROX_MOTION, txdata, 1);
}

int UxbusCmd::move_line_common(float mvpose[6], float mvvelo, float mvacc, float mvtime, float radius, int coord, bool is_axis_angle, unsigned char only_check_type, unsigned char *only_check_result, unsigned char motion_type, std::string feedback_key)
{
  float txdata[10] = { 0 };
  for (int i = 0; i < 6; i++) { txdata[i] = mvpose[i]; }
  txdata[6] = mvvelo;
  txdata[7] = mvacc;
  txdata[8] = mvtime;
  txdata[9] = radius;

  int ret = 0;
  unsigned char rx_data[3] = { 0 };
  if (motion_type == 0) {
    char additional[3] = { (char)coord, (char)is_axis_angle, (char)only_check_type };
    ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE, txdata, 10, additional, 3, rx_data, 3, 10000, feedback_key);
  }
  else {
    char additional[4] = { (char)coord, (char)is_axis_angle, (char)only_check_type, (char)motion_type };
    ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_LINE, txdata, 10, additional, 4, rx_data, 3, 10000, feedback_key);
  }
  if (only_check_type > 0 && ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
  return ret;
}

int UxbusCmd::move_circle_common(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent, int coord, bool is_axis_angle, unsigned char only_check_type, unsigned char *only_check_result, std::string feedback_key)
{
  float txdata[16] = { 0 };
  for (int i = 0; i < 6; i++) {
    txdata[i] = pose1[i];
    txdata[6 + i] = pose2[i];
  }
  txdata[12] = mvvelo;
  txdata[13] = mvacc;
  txdata[14] = mvtime;
  txdata[15] = percent;
  char additional[3] = { (char)coord, (char)is_axis_angle, (char)only_check_type };
  unsigned char rx_data[3] = { 0 };
  int ret = _set_nfp32_with_bytes(UXBUS_RG::MOVE_CIRCLE, txdata, 16, additional, 3, rx_data, 3, 10000, feedback_key);
  if (ret == 0 && only_check_result != NULL) *only_check_result = rx_data[2];
  return ret;
}

int UxbusCmd::get_dh_params(float dh_params[28])
{
  return _get_nfp32(UXBUS_RG::GET_DH, dh_params, 28);
}

int UxbusCmd::set_dh_params(float dh_params[28], unsigned char flag)
{
  char additional[1] = { (char)flag };
  return _set_nfp32_with_bytes(UXBUS_RG::SET_DH, dh_params, 16, additional, 1);
}

int UxbusCmd::_set_feedback_type_no_lock(unsigned char feedback_type)
{
  unsigned char send_data[1] = {feedback_type};
  int ret = _send_modbus_request(UXBUS_RG::SET_FEEDBACK_TYPE, send_data, 1);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  ret = _recv_modbus_response(UXBUS_RG::SET_FEEDBACK_TYPE, ret, NULL, 0, S_TOUT_);
  return ret;
}

int UxbusCmd::set_feedback_type(unsigned char feedback_type)
{
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _set_feedback_type_no_lock(feedback_type);
  if (ret != UXBUS_STATE::ERR_NOTTCP) {
    feedback_type_ = feedback_type;
  }
  return ret;
}

int UxbusCmd::check_feedback(std::string feedback_key)
{
  return _set_nu8(UXBUS_RG::FEEDBACK_CHECK, (unsigned char *)NULL, 0, feedback_key, FeedbackType::MOTION_FINISH);
}

int UxbusCmd::set_common_param(unsigned char param_type, int param_val)
{
  unsigned char send_data[5] = {0};
  send_data[0] = param_type;
  int32_to_hex(param_val, &send_data[1]);
  return _set_nu8(UXBUS_RG::SET_COMMON_PARAM, send_data, 5);
}

int UxbusCmd::set_common_param(unsigned char param_type, float param_val)
{
  unsigned char send_data[5] = {0};
  send_data[0] = param_type;
  fp32_to_hex(param_val, &send_data[1]);
  return _set_nu8(UXBUS_RG::SET_COMMON_PARAM, send_data, 5);
}

int UxbusCmd::set_common_param(unsigned char param_type, float *param_vals, int n)
{
  unsigned char *send_data = new unsigned char[n * 4 + 1]();
  send_data[0] = param_type;
  nfp32_to_hex(param_vals, &send_data[1], n);
  int ret = _set_nu8(UXBUS_RG::SET_COMMON_PARAM, send_data,  n * 4 + 1);
  delete[] send_data;
  return ret;
}

int UxbusCmd::get_common_param(unsigned char param_type, int *param_val)
{
  unsigned char send_data[1] = {param_type};
  unsigned char rx_data[4] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_PARAM, send_data, 1, rx_data, 4);
  *param_val = bin8_to_32(rx_data);
  return ret;
}

int UxbusCmd::get_common_param(unsigned char param_type, float *param_val)
{
  unsigned char send_data[1] = {param_type};
  unsigned char rx_data[4] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_PARAM, send_data, 1, rx_data, 4);
  *param_val = hex_to_fp32(rx_data);
  return ret;
}

int UxbusCmd::get_common_param(unsigned char param_type, float *param_vals, int n)
{
  unsigned char send_data[1] = {param_type};
  unsigned char *rx_data = new unsigned char[n * 4]();
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_PARAM, send_data, 1, rx_data,  n * 4);
  hex_to_nfp32(rx_data, param_vals, n);
  delete[] rx_data;
  return ret;
}

int UxbusCmd::get_poe_status(int *status)
{
  unsigned char send_data[1] = {1};
  unsigned char rx_data[1] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 1);
  *status = rx_data[0];
  return ret;
}

int UxbusCmd::get_iden_status(int *status)
{
  unsigned char send_data[1] = {2};
  unsigned char rx_data[1] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 1);
  *status = rx_data[0];
  return ret;
}

int UxbusCmd::get_c31_error_info(int *id, float *theoretical_tau, float *actual_tau)
{
  unsigned char send_data[1] = {101};
  unsigned char rx_data[9] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 9);
  *id = rx_data[0];
  *theoretical_tau = hex_to_fp32(&rx_data[1]);
  *actual_tau = hex_to_fp32(&rx_data[5]);
  return ret;
}

int UxbusCmd::get_c54_error_info(int *dir, float *tau_threshold, float *actual_tau)
{
  unsigned char send_data[1] = {107};
  unsigned char rx_data[9] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 9);
  *dir = rx_data[0];
  *tau_threshold = hex_to_fp32(&rx_data[1]);
  *actual_tau = hex_to_fp32(&rx_data[5]);
  return ret;
}

int UxbusCmd::get_c37_error_info(int *id, float *diff_angle)
{
  unsigned char send_data[1] = {102};
  unsigned char rx_data[5] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 5);
  *id = rx_data[0];
  *diff_angle = hex_to_fp32(&rx_data[1]);
  return ret;
}

int UxbusCmd::get_c23_error_info(int *id, float *angle)
{
  unsigned char send_data[1] = {103};
  unsigned char rx_data[5] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 29);
  *id = rx_data[0];
  hex_to_nfp32(&rx_data[1], angle, 7);
  // *angle = hex_to_fp32(&rx_data[1]);
  return ret;
}

int UxbusCmd::get_c24_error_info(int *id, float *speed)
{
  unsigned char send_data[1] = {104};
  unsigned char rx_data[5] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 5);
  *id = rx_data[0];
  *speed = hex_to_fp32(&rx_data[1]);
  return ret;
}

int UxbusCmd::get_c60_error_info(float *max_velo, float *curr_velo)
{
  unsigned char send_data[1] = {105};
  unsigned char rx_data[8] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 8);
  *max_velo = hex_to_fp32(&rx_data[0]);
  *curr_velo = hex_to_fp32(&rx_data[4]);
  return ret;
}

int UxbusCmd::get_c38_error_info(int *id, float *angle)
{
  unsigned char send_data[1] = {106};
  unsigned char rx_data[5] = {0};
  int ret = _getset_nu8(UXBUS_RG::GET_COMMON_INFO, send_data, 1, rx_data, 29);
  *id = rx_data[0];
  hex_to_nfp32(&rx_data[1], angle, 7);
  // *angle = hex_to_fp32(&rx_data[1]);
  return ret;
}
