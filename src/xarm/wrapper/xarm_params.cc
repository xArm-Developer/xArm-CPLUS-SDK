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


int XArmAPI::set_collision_sensitivity(int sensitivity, bool wait) {
  _wait_until_not_pause();
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (wait) {
    if (support_feedback_)
      _wait_all_task_finish(NO_TIMEOUT);
    else
      _wait_move(NO_TIMEOUT);
  }
  return core->set_collis_sens(sensitivity);
}

int XArmAPI::set_teach_sensitivity(int sensitivity, bool wait) {
  _wait_until_not_pause();
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (wait) {
    if (support_feedback_)
      _wait_all_task_finish(NO_TIMEOUT);
    else
      _wait_move(NO_TIMEOUT);
  }
  return core->set_teach_sens(sensitivity);
}

int XArmAPI::set_gravity_direction(fp32 gravity_dir[3], bool wait) {
  _wait_until_not_pause();
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (wait) {
    if (support_feedback_)
      _wait_all_task_finish(NO_TIMEOUT);
    else
      _wait_move(NO_TIMEOUT);
  }
  return core->set_gravity_dir(gravity_dir);
}

int XArmAPI::clean_conf(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->clean_conf();
}

int XArmAPI::save_conf(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->save_conf();
}

int XArmAPI::set_tcp_offset(fp32 pose_offset[6], bool wait) {
  _wait_until_not_pause();
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 offset[6];
  for (int i = 0; i < 6; i++) {
    offset[i] = (float)(default_is_radian || i < 3 ? pose_offset[i] : to_radian(pose_offset[i]));
  }
  if (wait) {
    if (support_feedback_)
      _wait_all_task_finish(NO_TIMEOUT);
    else
      _wait_move(NO_TIMEOUT);
  }
  return core->set_tcp_offset(offset);
}

int XArmAPI::set_tcp_load(fp32 weight, fp32 center_of_gravity[3], bool wait) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  float _gravity[3];
  if (_version_is_ge(0, 2, 1)) {
    _gravity[0] = center_of_gravity[0];
    _gravity[1] = center_of_gravity[1];
    _gravity[2] = center_of_gravity[2];
  }
  else {
    _gravity[0] = (float)(center_of_gravity[0] / 1000.0);
    _gravity[1] = (float)(center_of_gravity[1] / 1000.0);
    _gravity[2] = (float)(center_of_gravity[2] / 1000.0);
  }
  std::string feedback_key = _gen_feedback_key(wait);
  int ret = core->set_tcp_load(weight, _gravity, feedback_key);
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret, true);
  if (wait && ret == 0) {
    ret = _wait_move(NO_TIMEOUT, trans_id);
  }

  return ret;
}

int XArmAPI::set_tcp_jerk(fp32 jerk) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->set_tcp_jerk(jerk);
}

int XArmAPI::set_tcp_maxacc(fp32 acc) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->set_tcp_maxacc(acc);
}

int XArmAPI::set_joint_jerk(fp32 jerk) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->set_joint_jerk(default_is_radian ? jerk : to_radian(jerk));
}

int XArmAPI::set_joint_maxacc(fp32 acc) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  return core->set_joint_maxacc(default_is_radian ? acc : to_radian(acc));
}

int XArmAPI::get_reduced_mode(int *mode) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_reduced_mode(mode);
}

int XArmAPI::get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14], int *fense_is_on, int *collision_rebound_is_on) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_reduced_states(on, xyz_list, tcp_speed, joint_speed, jrange, fense_is_on, collision_rebound_is_on, _version_is_ge() ? 79 : 21);
  if (!default_is_radian) {
    *joint_speed = to_degree(*joint_speed);
  }
  if (_version_is_ge()) {
    if (jrange != NULL && !default_is_radian) {
      for (int i = 0; i < 14; i++) {
        jrange[i] = to_degree(jrange[i]);
      }
    }
  }
  return ret;
}

int XArmAPI::set_reduced_mode(bool on) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_reduced_mode(int(on));
}

int XArmAPI::set_reduced_max_tcp_speed(float speed) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_reduced_linespeed(speed);
}

int XArmAPI::set_reduced_max_joint_speed(float speed) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_reduced_jointspeed(default_is_radian ? speed : to_radian(speed));
}


int XArmAPI::set_reduced_tcp_boundary(int boundary[6]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_xyz_limits(boundary);
}

int XArmAPI::set_reduced_joint_range(float jrange[14]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  float joint_range[14];
  for (int i = 0; i < 14; i++) {
    joint_range[i] = default_is_radian ? jrange[i] : to_radian(jrange[i]);
  }
  return core->set_reduced_jrange(joint_range);
}

int XArmAPI::set_fense_mode(bool on) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_fense_on(int(on));
}

int XArmAPI::set_collision_rebound(bool on) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_collis_reb(int(on));
}

int XArmAPI::set_world_offset(float pose_offset[6], bool wait) {
  _wait_until_not_pause();
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 offset[6];
  for (int i = 0; i < 6; i++) {
    offset[i] = default_is_radian || i < 3 ? pose_offset[i] : to_radian(pose_offset[i]);
  }
  if (wait) {
    if (support_feedback_)
      _wait_all_task_finish(NO_TIMEOUT);
    else
      _wait_move(NO_TIMEOUT);
  }
  return core->set_world_offset(offset);
}

int XArmAPI::config_tgpio_reset_when_stop(bool on_off) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->config_io_stop_reset(1, int(on_off));
}

int XArmAPI::config_cgpio_reset_when_stop(bool on_off) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->config_io_stop_reset(0, int(on_off));
}


int XArmAPI::get_report_tau_or_i(int *tau_or_i) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_report_tau_or_i(tau_or_i);
}

int XArmAPI::set_report_tau_or_i(int tau_or_i) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_report_tau_or_i(tau_or_i);
}

int XArmAPI::set_self_collision_detection(bool on) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_self_collision_detection((int)on);
}

int XArmAPI::set_collision_tool_model(int tool_type, int n, ...) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (tool_type < COLLISION_TOOL_TYPE::USE_PRIMITIVES) {
    return core->set_collision_tool_model(tool_type);
  }
  if (n <= (tool_type == COLLISION_TOOL_TYPE::BOX ? 2 : tool_type == COLLISION_TOOL_TYPE::CYLINDER ? 1 : 0))
    return API_CODE::PARAM_ERROR;
  fp32 *params = new fp32[n]();
  va_list args;
  va_start(args, n);
  int inx = 0;
  while (inx < n)
  {
    params[inx] = (fp32)va_arg(args, double);
    inx++;
  }
  va_end(args);
  int ret = core->set_collision_tool_model(tool_type, n, params);
  delete[] params;
  return ret;
}

int XArmAPI::_wait_all_task_finish(fp32 timeout)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (!support_feedback_) return API_CODE::CMD_NOT_EXIST;
  std::string feedback_key = _gen_feedback_key(true);
  int ret = core->check_feedback(feedback_key);
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret);
  if (ret == 0) {
    ret = _wait_feedback(timeout, trans_id);
    if (ret == 0)
      sleep_milliseconds(500);
  }
  return ret;
}

int XArmAPI::set_linear_spd_limit_factor(float factor)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_common_param(1, factor);
}

int XArmAPI::set_cmd_mat_history_num(int num)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_common_param(2, num);
}

int XArmAPI::set_fdb_mat_history_num(int num)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_common_param(3, num);
}

int XArmAPI::get_linear_spd_limit_factor(float *factor)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_common_param(1, factor);
}

int XArmAPI::get_cmd_mat_history_num(int *num)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_common_param(2, num);
}

int XArmAPI::get_fdb_mat_history_num(int *num)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_common_param(3, num);
}

int XArmAPI::get_tgpio_modbus_timeout(int *timeout, bool is_transparent_transmission)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (is_transparent_transmission)
    return core->get_common_param(5, timeout);
  else
    return core->get_common_param(4, timeout);
}

int XArmAPI::get_poe_status(int *status)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_poe_status(status);
}

int XArmAPI::get_iden_status(int *status)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_iden_status(status);
}

int XArmAPI::get_c31_error_info(int *servo_id, float *theoretical_tau, float *actual_tau)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_c31_error_info(servo_id, theoretical_tau, actual_tau);
}

int XArmAPI::get_c54_error_info(int *dir, float *tau_threshold, float *actual_tau)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_c54_error_info(dir, tau_threshold, actual_tau);
}

int XArmAPI::get_c37_error_info(int *servo_id, float *diff_angle)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_c37_error_info(servo_id, diff_angle);
  if (ret == 0 && !default_is_radian) {
    *diff_angle = to_degree(*diff_angle);
  }
  return ret;
}

int XArmAPI::get_c23_error_info(int *id_bits, float angles[7])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_c23_error_info(id_bits, angles);
  if (ret == 0 && !default_is_radian) {
    for (int i = 0; i < 7; i++)
      angles[i] = to_degree(angles[i]);
  }
  return ret;
}

int XArmAPI::get_c24_error_info(int *servo_id, float *speed)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_c24_error_info(servo_id, speed);
  if (ret == 0 && !default_is_radian) {
    *speed = to_degree(*speed);
  }
  return ret;
}

int XArmAPI::get_c60_error_info(float *max_velo, float *curr_velo)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_c60_error_info(max_velo, curr_velo);
}

int XArmAPI::get_c38_error_info(int *id_bits, float angles[7])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_c38_error_info(id_bits, angles);
  if (ret == 0 && !default_is_radian) {
    for (int i = 0; i < 7; i++)
      angles[i] = to_degree(angles[i]);
  }
  return ret;
}

int XArmAPI::set_ft_collision_detection(int on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_common_param(11, on_off);
}

int XArmAPI::set_ft_collision_rebound(int on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_common_param(13, on_off);
}

int XArmAPI::set_ft_collision_threshold(float thresholds[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_common_param(12, thresholds, 6);
}

int XArmAPI::set_ft_collision_reb_distance(float distances[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  float dis[6];
  for (int i = 0; i < 6; i++) {
    dis[i] = i < 3 || default_is_radian ? distances[i] : to_radian(distances[i]);
  }
  return core->set_common_param(14, dis, 6);
}

int XArmAPI::get_ft_collision_detection(int *on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_common_param(11, on_off);
}

int XArmAPI::get_ft_collision_rebound(int *on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_common_param(13, on_off);
}

int XArmAPI::get_ft_collision_threshold(float thresholds[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_common_param(12, thresholds, 6);
}

int XArmAPI::get_ft_collision_reb_distance(float distances[6])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_common_param(14, distances, 6);
  if (ret == 0 && !default_is_radian) {
    for (int i = 3; i < 6; i++) {
      distances[i] = to_degree(distances[i]);
    }
  }
  return ret;
}