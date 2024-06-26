/**
 * Software License Agreement (MIT License)
 * 
 * @copyright Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef CORE_INSTRUCTION_UXBUS_CMD_H_
#define CORE_INSTRUCTION_UXBUS_CMD_H_

#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <vector>
#include <functional>
#ifdef _WIN32
#include <sys/timeb.h>
#include <windows.h>
#else
#include <unistd.h>
#include <time.h>
#endif
#include "xarm/core/common/data_type.h"
#include "xarm/core/instruction/uxbus_cmd_config.h"

inline long long get_us() {
#ifdef _WIN32
#define EPOCHFILETIME   (116444736000000000UL)
  FILETIME ft;
  LARGE_INTEGER li;
  GetSystemTimeAsFileTime(&ft);
  li.LowPart = ft.dwLowDateTime;
  li.HighPart = ft.dwHighDateTime;
  return (li.QuadPart - EPOCHFILETIME) / 10;
#else
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (t.tv_sec * 1000000000 + t.tv_nsec) * 0.001;
#endif
}

inline void sleep_ns(unsigned long long ns) {
  std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
}

inline void sleep_us(unsigned long long us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

inline void sleep_ms(unsigned long long ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void sleep_seconds(unsigned long long sec) {
  std::this_thread::sleep_for(std::chrono::seconds(sec));
}

inline void sleep_milliseconds(unsigned long milliseconds) {
#ifdef _WIN32
  Sleep(milliseconds);
#else
  usleep(milliseconds * 1000);
#endif
}

inline long long get_system_time()
{
#ifdef _WIN32
  struct timeb t;
  ftime(&t);
  return 1000 * t.time + t.millitm; // milliseconds
#else
  struct timespec t;
  // clock_gettime(CLOCK_REALTIME, &t);
  clock_gettime(CLOCK_MONOTONIC, &t);
  return 1000 * t.tv_sec + t.tv_nsec / 1000000; // milliseconds
#endif
}

class UxbusCmd {
public:
  UxbusCmd(void);
  UxbusCmd(std::function<void (std::string, int, unsigned char)> set_feedback_key_transid);
  ~UxbusCmd(void);

  int set_timeout(float timeout);

  int get_version(unsigned char rx_data[40]);
  int get_robot_sn(unsigned char rx_data[40]);
  int check_verification(int *rx_data);
  int system_control(int value);
  int set_record_traj(int value);
  int save_traj(char filename[81], std::string feedback_key = "");
  int load_traj(char filename[81], std::string feedback_key = "");
  int playback_traj(int times, int spdx = 1, std::string feedback_key = "");
  int playback_traj_old(int times);
  int get_traj_rw_status(int *rx_data);
  int set_reduced_mode(int on_off);
  int set_reduced_linespeed(float lspd_mm);
  int set_reduced_jointspeed(float jspd_rad);
  int get_reduced_mode(int *rx_data);
  int get_reduced_states(int *on, int xyz_list[6], float *tcp_speed, float *joint_speed, float jrange_rad[14] = NULL, int *fense_is_on = NULL, int *collision_rebound_is_on = NULL, int length = 21);
  int set_xyz_limits(int xyz_list[6]);
  int set_world_offset(float pose_offset[6]);
  int cnter_reset(void);
  int cnter_plus(void);
  int set_reduced_jrange(float jrange_rad[14]);
  int set_fense_on(int on_off);
  int set_collis_reb(int on_off);
  int motion_en(int id, int value);
  int set_state(int value);
  int get_state(int *rx_data);
  int get_cmdnum(int *rx_data);
  int get_err_code(int *rx_data);
  int get_hd_types(int *rx_data);
  int reload_dynamics(void);
  int clean_err(void);
  int clean_war(void);
  int set_brake(int axis, int en);
  int set_mode(int value, int detection_param = -1);
  int move_line(float mvpose[6], float mvvelo, float mvacc, float mvtime, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, unsigned char motion_type = 0);
  int move_lineb(float mvpose[6], float mvvelo, float mvacc, float mvtime,
    float mvradii, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, unsigned char motion_type = 0);
  int move_joint(float mvjoint[7], float mvvelo, float mvacc, float mvtime, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, std::string feedback_key = "");
  int move_jointb(float mvjoint[7], float mvvelo, float mvacc, float mvradii, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, std::string feedback_key = "");
  int move_line_tool(float mvpose[6], float mvvelo, float mvacc, float mvtime, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, unsigned char motion_type = 0);
  int move_gohome(float mvvelo, float mvacc, float mvtime, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, std::string feedback_key = "");
  int move_servoj(float mvjoint[7], float mvvelo, float mvacc, float mvtime);
  int move_servo_cartesian(float mvpose[6], float mvvelo, float mvacc, float mvtime);
  // // this interface is no longer supported
  // int set_servot(float jnt_taus[7]);
  int get_joint_tau(float jnt_taus[7]);
  int set_safe_level(int level);
  int get_safe_level(int *level);
  int sleep_instruction(float sltime);
  int move_circle(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL);
  int set_tcp_jerk(float jerk);
  int set_tcp_maxacc(float maxacc);
  int set_joint_jerk(float jerk);
  int set_joint_maxacc(float maxacc);
  int set_tcp_offset(float pose_offset[6]);
  int set_tcp_load(float mass, float load_offset[3], std::string feedback_key = "");
  int set_collis_sens(int value);
  int set_teach_sens(int value);
  int set_gravity_dir(float gravity_dir[3]);
  int clean_conf(void);
  int save_conf(void);

  int get_tcp_pose(float pose[6]);
  int get_joint_pose(float angles[7]);
  int get_joint_states(float position[7], float velocity[7], float effort[7], int num = 3);
  int get_ik(float pose[6], float angles[7]);
  int get_fk(float angles[7], float pose[6]);
  int is_joint_limit(float joint[7], int *value);
  int is_tcp_limit(float pose[6], int *value);
  int gripper_addr_w16(int addr, float value);
  int gripper_addr_r16(int addr, float *value);
  int gripper_addr_w32(int addr, float value);
  int gripper_addr_r32(int addr, float *value);
  int gripper_set_en(int value);
  int gripper_set_mode(int value);
  int gripper_set_zero(void);
  int gripper_get_pos(float *pulse);
  int gripper_set_pos(float pulse);
  int gripper_set_posspd(float speed);
  int gripper_get_errcode(int rx_data[2]);
  int gripper_clean_err(void);

  int tgpio_addr_w16(int addr, float value, unsigned char host_id = UXBUS_CONF::TGPIO_HOST_ID, char *add_data = NULL, int add_len = 0);
  int tgpio_addr_r16(int addr, float *value, unsigned char host_id = UXBUS_CONF::TGPIO_HOST_ID);
  int tgpio_addr_w32(int addr, float value, unsigned char host_id = UXBUS_CONF::TGPIO_HOST_ID);
  int tgpio_addr_r32(int addr, float *value, unsigned char host_id = UXBUS_CONF::TGPIO_HOST_ID);
  int tgpio_get_digital(int *io1, int *io2);
  int tgpio_set_digital(int ionum, int value, int sync = -1);
  int tgpio_get_analog1(float *value);
  int tgpio_get_analog2(float *value);

  int set_modbus_timeout(int value, bool is_transparent_transmission = false);
  int set_modbus_baudrate(int baud);
  int tgpio_set_modbus(unsigned char *send_data, int length, unsigned char *recv_data, unsigned char host_id = UXBUS_CONF::TGPIO_HOST_ID, float limit_sec = 0.0, bool is_transparent_transmission = false);
  int gripper_modbus_w16s(int addr, float value, int len);
  int gripper_modbus_r16s(int addr, int len, unsigned char *rx_data);
  int gripper_modbus_set_en(int value);
  int gripper_modbus_set_mode(int value);
  int gripper_modbus_set_zero(void);
  int gripper_modbus_get_pos(float *pulse);
  int gripper_modbus_set_pos(float pulse);
  int gripper_modbus_set_posspd(float speed);
  int gripper_modbus_get_errcode(int *err);
  int gripper_modbus_clean_err(void);

  int servo_set_zero(int id);
  int servo_get_dbmsg(int rx_data[16]);
  int servo_addr_w16(int id, int addr, float value);
  int servo_addr_r16(int id, int addr, float *value);
  int servo_addr_w32(int id, int addr, float value);
  int servo_addr_r32(int id, int addr, float *value);


  int cgpio_get_auxdigit(int *value);
  int cgpio_get_analog1(float *value);
  int cgpio_get_analog2(float *value);
  int cgpio_set_auxdigit(int ionum, int value, int sync = -1);
  int cgpio_set_analog1(float value, int sync = -1);
  int cgpio_set_analog2(float value, int sync = -1);
  int cgpio_set_infun(int ionum, int fun);
  int cgpio_set_outfun(int ionum, int fun);
  int cgpio_get_state(int *state, int *digit_io, float *analog, int *input_conf, int *output_conf, int *input_conf2=NULL, int *output_conf2=NULL);

  int get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in=0, int orient_type_out=0);
  int get_position_aa(float pose[6]);
  int move_line_aa(float mvpose[6], float mvvelo, float mvacc, float mvtime, int mvcoord=0, int relative=0, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, unsigned char motion_type = 0);
  int move_servo_cart_aa(float mvpose[6], float mvvelo, float mvacc, int tool_coord=0, int relative=0);
  int move_relative(float mvpose[7], float mvvelo, float mvacc, float mvtime, float radius, int is_joint_motion = false, bool is_axis_angle = false, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, unsigned char motion_type = 0, std::string feedback_key = "");

  int tgpio_delay_set_digital(int ionum, int value, float delay_sec);
  int cgpio_delay_set_digital(int ionum, int value, float delay_sec);
  int tgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r);
  int cgpio_position_set_digital(int ionum, int value, float xyz[3], float tol_r);
  int cgpio_position_set_analog(int ionum, float value, float xyz[3], float tol_r);
  int config_io_stop_reset(int io_type, int val);

  int set_report_tau_or_i(int tau_or_i);
  int get_report_tau_or_i(int *rx_data);

  int set_self_collision_detection(int on_off);
  int set_collision_tool_model(int tool_type, int n = 0, float *argv = NULL);
  int set_simulation_robot(int on_off);

  int vc_set_jointv(float jnt_v[7], int jnt_sync, float duration = -1.0);
  int vc_set_linev(float line_v[6], int coord, float duration = -1.0);

  int cali_tcp_pose(float four_pnts[4][6], float ret_xyz[3]);
  int cali_user_orient(float three_pnts[3][6], float ret_rpy[3], int mode = 0, int trust_ind = 0);
  int cali_tcp_orient(float rpy_be[3], float rpy_bt[3], float ret_rpy[3]);
  int cali_user_pos(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3]);

  int iden_load(int iden_type, float *rx_data, int num_get, int timeout=500000, float estimated_mass = 0.0);
  int set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6]);
  int set_impedance_mbk(float M[6], float K[6], float B[6]);
  int set_impedance_config(int coord, int c_axis[6]);
  int config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6]);
  int set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6]);
  int ft_sensor_set_zero(void);
  int ft_sensor_iden_load(float result[10]);
  int ft_sensor_cali_load(float load[10]);
  int ft_sensor_enable(int on_off);
  int ft_sensor_app_set(int app_code);
  int ft_sensor_app_get(int *app_code);
  int ft_sensor_get_data(float ft_data[6], bool is_new = true);
  int ft_sensor_get_config(int *ft_app_status = NULL, int *ft_is_started = NULL, int *ft_type = NULL, int *ft_id = NULL, int *ft_freq = NULL, 
    float *ft_mass = NULL, float *ft_dir_bias = NULL, float ft_centroid[3] = NULL, float ft_zero[6] = NULL, int *imp_coord = NULL, int imp_c_axis[6] = NULL, float M[6] = NULL, float K[6] = NULL, float B[6] = NULL,
    int *f_coord = NULL, int f_c_axis[6] = NULL, float f_ref[6] = NULL, float f_limits[6] = NULL, float kp[6] = NULL, float ki[6] = NULL, float kd[6] = NULL, float xe_limit[6] = NULL);
  int ft_sensor_get_error(int *err);
  int iden_tcp_load(float result[4], float estimated_mass = 0.5);

  int track_modbus_r16s(int addr, unsigned char *data, int len, unsigned char fcode = 0x03);
  int track_modbus_w16s(int addr, unsigned char *send_data, int len, unsigned char *rx_data);

  int set_cartesian_velo_continuous(int on_off);
  int set_allow_approx_motion(int on_off);

  int get_dh_params(float dh_params[28]);
  int set_dh_params(float dh_params[28], unsigned char flag = 0);

  int iden_joint_friction(unsigned char sn[14], float *result);

  int move_line_common(float mvpose[6], float mvvelo, float mvacc, float mvtime, float radius = -1.0, int coord = 0, bool is_axis_angle = false, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, unsigned char motion_type = 0, std::string feedback_key = "");
  int move_circle_common(float pose1[6], float pose2[6], float mvvelo, float mvacc, float mvtime, float percent, int coord = 0, bool is_axis_angle = false, unsigned char only_check_type = 0, unsigned char *only_check_result = NULL, std::string feedback_key = "");

  int set_feedback_type(unsigned char feedback_type);
  int check_feedback(std::string feedback_key = "");

  int set_common_param(unsigned char param_type, int param_val);
  int set_common_param(unsigned char param_type, float param_val);
  int get_common_param(unsigned char param_type, int *param_val);
  int get_common_param(unsigned char param_type, float *param_val);
  int get_poe_status(int *status);
  int get_iden_status(int *status);
  int get_c31_error_info(int *id, float *theoretical_tau, float *actual_tau);
  int get_c37_error_info(int *id, float *diff_angle);
  int get_c23_error_info(int *id, float *angle);
  int get_c24_error_info(int *id, float *speed);
  int get_c60_error_info(float *max_velo, float *curr_velo);
  int get_c38_error_info(int *id, float *angle);

  virtual void close(void);
  virtual int is_ok(void);
  virtual int get_protocol_identifier(void) { return 0; };
  virtual int set_protocol_identifier(int protocol_identifier = 2) { return -11; };

  /* modbus tcp func_code: 0x01 */
  virtual int read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits) { return -11; };
  /* modbus tcp func_code: 0x02 */
  virtual int read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits) { return -11; };
  /* modbus tcp func_code: 0x03 */
  virtual int read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false) { return -11; };
  /* modbus tcp func_code: 0x04 */
  virtual int read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false) { return -11; };
  /* modbus tcp func_code: 0x05 */
  virtual int write_single_coil_bit(unsigned short addr, unsigned char bit_val) { return -11; };
  /* modbus tcp func_code: 0x06 */
  virtual int write_single_holding_register(unsigned short addr, int reg_val) { return -11; };
  /* modbus tcp func_code: 0x0F */
  virtual int write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits) { return -11; };
  /* modbus tcp func_code: 0x10 */
  virtual int write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs) { return -11; };
  /* modbus tcp func_code: 0x16 */
  virtual int mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask) { return -11; };
  /* modbus tcp func_code: 0x17 */
  virtual int write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed = false) { return -11; };

private:
  virtual int _send_modbus_request(unsigned char unit_id, unsigned char *pdu_data, unsigned short pdu_len, int prot_id = -1);
  virtual int _recv_modbus_response(unsigned char t_unit_id, unsigned short t_trans_id, unsigned char *ret_data, unsigned short ret_len, int timeout, int t_prot_id = -1);
  virtual int _check_private_protocol(unsigned char *data);
  int _set_nu8(int funcode, unsigned char *datas, int num, std::string feedback_key = "", unsigned char feedback_type=FeedbackType::MOTION_FINISH);
  int _set_nu8(int funcode, int *datas, int num, std::string feedback_key = "", unsigned char feedback_type=FeedbackType::MOTION_FINISH);
  int _get_nu8(int funcode, int *rx_data, int num);
  int _get_nu8(int funcode, unsigned char *rx_data, int num);
  int _getset_nu8(int funcode, unsigned char *tx_data, int tx_num, unsigned char *rx_data, int rx_num);
  int _set_nu16(int funcode, int *datas, int num, char *add_data = NULL, int add_len = 0);
  int _get_nu16(int funcode, int *rx_data, int num);
  int _set_nfp32(int funcode, float *datas, int num, std::string feedback_key = "", unsigned char feedback_type=FeedbackType::MOTION_FINISH);
  int _set_nint32(int funcode, int *datas, int num, std::string feedback_key = "", unsigned char feedback_type=FeedbackType::MOTION_FINISH);
  int _get_nfp32(int funcode, float *rx_data, int num);
  int _swop_nfp32(int funcode, float tx_datas[], int txn, float *rx_data, int rxn);
  int _is_nfp32(int funcode, float datas[], int txn, int *value);
  int _set_nfp32_with_bytes(int funcode, float *tx_data, int tx_num, char *add_data, int add_len, unsigned char *rx_data = NULL, int rx_len=0, int timeout = UXBUS_CONF::SET_TIMEOUT, std::string feedback_key = "", unsigned char feedback_type=FeedbackType::MOTION_FINISH);
  int _get_nfp32_with_bytes(int funcode, unsigned char *tx_data, int tx_num, float *rx_data, int rxn, int timeout = UXBUS_CONF::GET_TIMEOUT);
  int _set_feedback_type_no_lock(unsigned char feedback_type);
  virtual int _get_trans_id() { return 0; }

public:
  bool state_is_ready;
  long long last_recv_ms;

protected:
  std::mutex mutex_;

private:
  int G_TOUT_ = UXBUS_CONF::GET_TIMEOUT;
  int S_TOUT_ = UXBUS_CONF::SET_TIMEOUT;

  long long last_modbus_comm_us_;
  unsigned char feedback_type_;
  bool has_feedback_key_transid_func_;
  std::function<void (std::string, int, unsigned char)> set_feedback_key_transid_;
};

#endif
