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


int XArmAPI::set_position(fp32 pose[6], fp32 radius, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, bool relative, unsigned char motion_type) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  only_check_result = 0;
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (only_check_type_ > 0 && wait) {
    code = _wait_move(timeout);
    if (code != 0) return code;
  }
  int ret = 0;
  last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
  last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
  std::string feedback_key = _gen_feedback_key(wait);
  if (relative) {
    fp32 mvpose[7] = {0.0};
    for (int i = 0; i < 6; i++) {
      mvpose[i] = (float)(default_is_radian || i < 3 ? pose[i] : to_radian(pose[i]));
    }
    ret = core->move_relative(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius, 0, 0, only_check_type_, &only_check_result, motion_type, feedback_key);
  } 
  else {
    fp32 mvpose[6];
    for (int i = 0; i < 6; i++) {
      last_used_position[i] = pose[i];
      mvpose[i] = (float)(default_is_radian || i < 3 ? last_used_position[i] : to_radian(last_used_position[i]));
    }
    int ret = 0;
    if (_version_is_ge(1, 11, 100)) {
      ret = core->move_line_common(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius, 0, false, only_check_type_, &only_check_result, motion_type, feedback_key);
    }
    else {
      if (radius >= 0) {
        ret = core->move_lineb(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius, only_check_type_, &only_check_result, motion_type);
      }
      else {
        ret = core->move_line(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, only_check_type_, &only_check_result, motion_type);
      }
    }
  }
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret, true);
  if (ret == 0 && only_check_type_ > 0) {
    return only_check_result != 0 ? API_CODE::HAS_ERROR : ret;
  }
  else {
    only_check_result = 0;
  }
  if (wait && ret == 0) {
    ret = _wait_move(timeout, trans_id);
    _sync();
  }

  return ret;
}

int XArmAPI::set_position(fp32 pose[6], fp32 radius, bool wait, fp32 timeout, bool relative, unsigned char motion_type) {
  return set_position(pose, radius, 0, 0, 0, wait, timeout, relative, motion_type);
}

int XArmAPI::set_position(fp32 pose[6], bool wait, fp32 timeout, bool relative, unsigned char motion_type) {
  return set_position(pose, -1, 0, 0, 0, wait, timeout, relative, motion_type);
}

int XArmAPI::set_tool_position(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius, unsigned char motion_type) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  only_check_result = 0;
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (only_check_type_ > 0 && wait) {
    code = _wait_move(timeout);
    if (code != 0) return code;
  }
  last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
  last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
  fp32 mvpose[6];
  for (int i = 0; i < 6; i++) {
    mvpose[i] = (float)(default_is_radian || i < 3 ? pose[i] : to_radian(pose[i]));
  }
  int ret = 0;
  std::string feedback_key = _gen_feedback_key(wait);
  if (_version_is_ge(1, 11, 100)) {
    ret = core->move_line_common(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius, 1, false, only_check_type_, &only_check_result, motion_type, feedback_key);
  }
  else {
    ret = core->move_line_tool(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, only_check_type_, &only_check_result, motion_type);
  }
  int trans_id = _get_feedback_transid(feedback_key);

  ret = _check_code(ret, true);
  if (ret == 0 && only_check_type_ > 0) {
    return only_check_result != 0 ? API_CODE::HAS_ERROR : ret;
  }
  else {
    only_check_result = 0;
  }
  if (wait && ret == 0) {
    ret = _wait_move(timeout, trans_id);
    _sync();
  }

  return ret;
}

int XArmAPI::set_tool_position(fp32 pose[6], bool wait, fp32 timeout, fp32 radius, unsigned char motion_type) {
  return set_tool_position(pose, 0, 0, 0, wait, timeout, radius, motion_type);
}


int XArmAPI::set_servo_angle(fp32 angs[7], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius, bool relative) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  only_check_result = 0;
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (only_check_type_ > 0 && wait) {
    code = _wait_move(timeout);
    if (code != 0) return code;
  }
  last_used_joint_speed = speed > 0 ? speed : last_used_joint_speed;
  last_used_joint_acc = acc > 0 ? acc : last_used_joint_acc;
  fp32 speed_ = (float)(default_is_radian ? last_used_joint_speed : to_radian(last_used_joint_speed));
  fp32 acc_ = (float)(default_is_radian ? last_used_joint_acc : to_radian(last_used_joint_acc));
  fp32 mvjoint[7];
  int ret = 0;
  std::string feedback_key = _gen_feedback_key(wait);
  if (relative) {
    for (int i = 0; i < 7; i++) {
      mvjoint[i] = i >= axis ? 0 : (float)(default_is_radian ? angs[i] : to_radian(angs[i]));
    }
    ret = core->move_relative(mvjoint, speed_, acc_, mvtime, radius, 1, 0, only_check_type_, &only_check_result, 0, feedback_key);
  }
  else {
    for (int i = 0; i < 7; i++) {
      last_used_angles[i] = angs[i];
      mvjoint[i] =  i >= axis ? 0 : (float)(default_is_radian ? last_used_angles[i] : to_radian(last_used_angles[i]));
    }
    if (_version_is_ge(1, 5, 20) && radius >= 0) {
      ret = core->move_jointb(mvjoint, speed_, acc_, radius, only_check_type_, &only_check_result, feedback_key);
    }
    else {
      ret = core->move_joint(mvjoint, speed_, acc_, mvtime, only_check_type_, &only_check_result, feedback_key);
    }
  }
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret, true);
  if (ret == 0 && only_check_type_ > 0) {
    return only_check_result != 0 ? API_CODE::HAS_ERROR : ret;
  }
  else {
    only_check_result = 0;
  }
  if (wait && ret == 0) {
    ret = _wait_move(timeout, trans_id);
    _sync();
  }
  return ret;
}

int XArmAPI::set_servo_angle(fp32 angs[7], bool wait, fp32 timeout, fp32 radius, bool relative) {
  return set_servo_angle(angs, 0, 0, 0, wait, timeout, radius, relative);
}

int XArmAPI::set_servo_angle(int servo_id, fp32 angle, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius, bool relative) {
  if (servo_id <= 0 || servo_id > axis) return API_CODE::PARAM_ERROR;
  last_used_angles[servo_id - 1] = angle;
  return set_servo_angle(last_used_angles, speed, acc, mvtime, wait, timeout, radius), relative;
}

int XArmAPI::set_servo_angle(int servo_id, fp32 angle, bool wait, fp32 timeout, fp32 radius, bool relative) {
  return set_servo_angle(servo_id, angle, 0, 0, 0, wait, timeout, radius, relative);
}

int XArmAPI::set_servo_angle_j(fp32 angs[7], fp32 speed, fp32 acc, fp32 mvtime) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 mvjoint[7];
  for (int i = 0; i < 7; i++) {
    mvjoint[i] =  i >= axis ? 0 : (float)(default_is_radian ? angs[i] : to_radian(angs[i]));
  }
  int ret = core->move_servoj(mvjoint, last_used_joint_speed, last_used_joint_acc, mvtime);
  ret = _check_code(ret, true, 1);
  return ret;
}

int XArmAPI::set_servo_cartesian(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 mvpose[6];
  for (int i = 0; i < 6; i++) {
    mvpose[i] = (float)(i < 3 || default_is_radian ? pose[i] : to_radian(pose[i]));
  }
  mvtime = (float)(is_tool_coord ? 1.0 : 0.0);
  int ret = core->move_servo_cartesian(mvpose, speed, acc, mvtime);
  ret = _check_code(ret, true, 1);
  return ret;
}

int XArmAPI::move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, bool is_tool_coord, bool is_axis_angle) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  only_check_result = 0;
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (only_check_type_ > 0 && wait) {
    code = _wait_move(timeout);
    if (code != 0) return code;
  }
  last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
  last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
  fp32 pose_1[6];
  fp32 pose_2[6];
  for (int i = 0; i < 6; i++) {
    pose_1[i] = (float)(default_is_radian || i < 3 ? pose1[i] : to_radian(pose1[i]));
    pose_2[i] = (float)(default_is_radian || i < 3 ? pose2[i] : to_radian(pose2[i]));
  }
  int ret = 0;
  std::string feedback_key = _gen_feedback_key(wait);
  if (_version_is_ge(1, 11, 100)) {
    ret = core->move_circle_common(pose_1, pose_2, last_used_tcp_speed, last_used_tcp_acc, mvtime, percent, is_tool_coord ? 1 : 0, is_axis_angle, only_check_type_, &only_check_result, feedback_key);
  }
  else {
    ret = core->move_circle(pose_1, pose_2, last_used_tcp_speed, last_used_tcp_acc, mvtime, percent, only_check_type_, &only_check_result);
  }
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret, true);
  if (ret == 0 && only_check_type_ > 0) {
    return only_check_result != 0 ? API_CODE::HAS_ERROR : ret;
  }
  else {
    only_check_result = 0;
  }
  if (wait && ret == 0) {
    ret = _wait_move(timeout, trans_id);
    _sync();
  }

  return ret;
}

int XArmAPI::move_gohome(fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  only_check_result = 0;
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (only_check_type_ > 0 && wait) {
    code = _wait_move(timeout);
    if (code != 0) return code;
  }
  fp32 speed_ = (float)(default_is_radian ? speed : to_radian(speed));
  fp32 acc_ = (float)(default_is_radian ? acc : to_radian(acc));
  speed_ = speed_ > 0 ? speed_ : (float)0.8726646259971648; // 50 °/s
  acc_ = acc_ > 0 ? acc_ : (float)17.453292519943297; // 1000 °/s^2
  std::string feedback_key = _gen_feedback_key(wait);
  int ret = core->move_gohome(speed_, acc_, mvtime, only_check_type_, &only_check_result, feedback_key);
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret, true);
  if (ret == 0 && only_check_type_ > 0) {
    return only_check_result != 0 ? API_CODE::HAS_ERROR : ret;
  }
  else {
    only_check_result = 0;
  }
  if (wait && ret == 0) {
    ret = _wait_move(timeout, trans_id);
    _sync();
  }

  return ret;
}

int XArmAPI::move_gohome(bool wait, fp32 timeout) {
  return move_gohome(0, 0, 0, wait, timeout);
}

void XArmAPI::reset(bool wait, fp32 timeout) {
  int err_warn[2] = { error_code, warn_code };
  int state_ = state;
  if (!is_tcp_) {
    get_err_warn_code(err_warn);
    get_state(&state_);
  }
  if (warn_code != 0) {
    clean_warn();
  }
  if (error_code != 0) {
    clean_error();
    motion_enable(true, 8);
    set_mode(0);
    set_state(0);
  }
  if (!is_ready_) {
    motion_enable(true, 8);
    set_mode(0);
    set_state(0);
  }
  move_gohome(wait, timeout);
}

int XArmAPI::set_position_aa(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord, bool relative, bool wait, fp32 timeout, fp32 radius, unsigned char motion_type) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  only_check_result = 0;
  int code = _xarm_is_ready();
  if (code != 0) return code;
  if (only_check_type_ > 0 && wait) {
    code = _wait_move(timeout);
    if (code != 0) return code;
  }
  last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
  last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
  fp32 mvpose[6];
  for (int i = 0; i < 6; i++) {
    mvpose[i] = (float)(default_is_radian || i < 3 ? pose[i] : to_radian(pose[i]));
  }
  int ret = 0;
  std::string feedback_key = _gen_feedback_key(wait);
  if (_version_is_ge(1, 11, 100)) {
    if (!is_tool_coord && relative) {
      ret = core->move_relative(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius, 0, true, only_check_type_, &only_check_result, motion_type, feedback_key);
    }
    else {
      ret = core->move_line_common(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius, (int)is_tool_coord, true, only_check_type_, &only_check_result, motion_type, feedback_key);
    }
  }
  else {
    ret = core->move_line_aa(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, (int)is_tool_coord, (int)relative, only_check_type_, &only_check_result, motion_type);
  }
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret, true);
  if (ret == 0 && only_check_type_ > 0) {
    return only_check_result != 0 ? API_CODE::HAS_ERROR : ret;
  }
  else {
    only_check_result = 0;
  }
  if (wait && ret == 0) {
    ret = _wait_move(timeout, trans_id);
    _sync();
  }

  return ret;
}

int XArmAPI::set_position_aa(fp32 pose[6], bool is_tool_coord, bool relative, bool wait, fp32 timeout, fp32 radius, unsigned char motion_type) {
  return set_position_aa(pose, 0, 0, 0, is_tool_coord, relative, wait, timeout, radius, motion_type);
}

int XArmAPI::set_servo_cartesian_aa(fp32 pose[6], fp32 speed, fp32 acc, bool is_tool_coord, bool relative) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 mvpose[6];
  for (int i = 0; i < 6; i++) {
    mvpose[i] = (float)(i < 3 || default_is_radian ? pose[i] : to_radian(pose[i]));
  }
  int ret = core->move_servo_cart_aa(mvpose, speed, acc, (int)is_tool_coord, (int)relative);
  ret = _check_code(ret, true, 1);
  return ret;
}

int XArmAPI::set_servo_cartesian_aa(fp32 pose[6], bool is_tool_coord, bool relative) {
  return set_servo_cartesian_aa(pose, 0, 0, is_tool_coord, relative);
}

int XArmAPI::vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord, fp32 duration) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 line_v[6];
  for (int i = 0; i < 6; i++) {
    line_v[i] = (float)((i < 3 || default_is_radian) ? speeds[i] : to_radian(speeds[i]));
  }
  return core->vc_set_linev(line_v, is_tool_coord ? 1 : 0, _version_is_ge(1, 8, 0) ? duration : (fp32)-1.0);
}

int XArmAPI::vc_set_joint_velocity(fp32 speeds[7], bool is_sync, fp32 duration) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 jnt_v[7];
  for (int i = 0; i < 7; i++) {
    jnt_v[i] =  i >= axis ? 0 : (float)(default_is_radian ? speeds[i] : to_radian(speeds[i]));
  }
  return core->vc_set_jointv(jnt_v, is_sync ? 1 : 0, _version_is_ge(1, 8, 0) ? duration : (fp32)-1.0);
}