/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include <map>
#include "xarm_wrapper.h"


namespace XArmWrapper
{
  XArmAPI* arm = NULL;
  int id = 0;
  int active_instance_id = 0;
  std::map<int, XArmAPI*> xarm_map;

  int __stdcall switch_xarm(int instance_id) {
    std::map<int, XArmAPI*>::iterator iter = xarm_map.find(instance_id);
    if (iter != xarm_map.end()) {
      if (arm != NULL && arm != iter->second) {
        bool removed = true;
        for (std::map<int, XArmAPI*>::iterator it = xarm_map.begin(); it != xarm_map.end(); ++it) {
          if (it->second == arm) {
            removed = false;
            break;
          }
        }
        if (removed) {
          arm->disconnect();
          delete arm;
        }
        printf("current active instance_id: %d\n", iter->first);
      }
      arm = iter->second;
      active_instance_id = iter->first;
      return 0;
    }
    fprintf(stderr, "[switch failed], instance %d is not exist, ", instance_id);
    if (active_instance_id != 0) {
      printf("current active instance_id: %d\n", active_instance_id);
    }
    else {
      printf("no active instance\n");
    }
    return -1;
  }

  int __stdcall remove_instance(int instance_id) {
    std::map<int, XArmAPI*>::iterator iter = xarm_map.find(instance_id);
    if (iter != xarm_map.end()) {
      if (iter->second == arm) {
        printf("You removed the instance you are using, and the instance will be disconnected and destroyed when you successfully switch to another instance\n");
      }
      else {
        arm->disconnect();
        delete arm;
      }
      xarm_map.erase(iter);
      return 0;
    }
    return -1;
  }

  XArmAPI* __stdcall get_instance(int instance_id)
  {
    if (instance_id != -1) {
      std::map<int, XArmAPI*>::iterator iter = xarm_map.find(instance_id);
      if (iter != xarm_map.end()) {
        return iter->second;
      }
      else {
        printf("instance %d is not exist, use default instance\n", instance_id);
      }
    }
    return arm;
  }

  int __stdcall create_instance(
    char* port,
    bool is_radian,
    bool do_not_open,
    bool check_tcp_limit,
    bool check_joint_limit,
    bool check_cmdnum_limit,
    bool check_robot_sn,
    bool check_is_ready,
    bool check_is_pause,
    int max_callback_thread_count,
    int max_cmdnum,
    int init_axis,
    bool debug,
    char* report_type,
    bool baud_checkset) {
    arm = new XArmAPI(port, is_radian, do_not_open,
      check_tcp_limit, check_joint_limit, check_cmdnum_limit,
      check_robot_sn, check_is_ready, check_is_pause,
      max_callback_thread_count, max_cmdnum, init_axis, debug, report_type, baud_checkset);
    id++;
    xarm_map[id] = arm;
    return id;
  }

  int __stdcall connect_robot(char* port, int instance_id) {
    return get_instance(instance_id)->connect(port);
  }
  void __stdcall disconnect(int instance_id) {
    get_instance(instance_id)->disconnect();
  }
  int __stdcall motion_enable(bool enable, int servo_id, int instance_id) {
    return get_instance(instance_id)->motion_enable(enable, servo_id);
  }
  int __stdcall set_mode(int mode, int detection_param, int instance_id) {
    return get_instance(instance_id)->set_mode(mode, detection_param);
  }
  int __stdcall set_state(int state, int instance_id) {
    return get_instance(instance_id)->set_state(state);
  }
  int __stdcall clean_warn(int instance_id) {
    return get_instance(instance_id)->clean_warn();
  }
  int __stdcall clean_error(int instance_id) {
    return get_instance(instance_id)->clean_error();
  }
  int __stdcall set_position(fp32 pose[6], fp32 radius, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, bool relative, unsigned char motion_type, int instance_id) {
    return get_instance(instance_id)->set_position(pose, radius, speed, acc, mvtime, wait, timeout, relative, motion_type);
  }
  int __stdcall set_tool_position(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius, unsigned char motion_type, int instance_id) {
    return get_instance(instance_id)->set_tool_position(pose, speed, acc, mvtime, wait, timeout, radius, motion_type);
  }
  int __stdcall set_servo_angle(fp32 angles[7], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius, bool relative, int instance_id) {
    return get_instance(instance_id)->set_servo_angle(angles, speed, acc, mvtime, wait, timeout, radius, relative);
  }
  int __stdcall set_servo_angle_j(fp32 angles[7], fp32 speed, fp32 acc, fp32 mvtime, int instance_id) {
    return get_instance(instance_id)->set_servo_angle_j(angles, speed, acc, mvtime);
  }
  int __stdcall set_servo_cartesian(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord, int instance_id) {
    return get_instance(instance_id)->set_servo_cartesian(pose, speed, acc, mvtime, is_tool_coord);
  }
  int __stdcall move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, bool is_tool_coord, bool is_axis_angle, int instance_id) {
    return get_instance(instance_id)->move_circle(pose1, pose2, percent, speed, acc, mvtime, wait, timeout, is_tool_coord, is_axis_angle);
  }
  int __stdcall move_gohome(fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, int instance_id) {
    return get_instance(instance_id)->move_gohome(speed, acc, mvtime, wait, timeout);
  }
  void __stdcall reset(bool wait, fp32 timeout, int instance_id) {
    return get_instance(instance_id)->reset(wait, timeout);
  }
  void __stdcall emergency_stop(int instance_id) {
    get_instance(instance_id)->emergency_stop();
  }

  int __stdcall set_servo_attach(int servo_id, int instance_id) {
    return get_instance(instance_id)->set_servo_attach(servo_id);
  }
  int __stdcall set_servo_detach(int servo_id, int instance_id) {
    return get_instance(instance_id)->set_servo_attach(servo_id);
  }
  int __stdcall set_pause_time(fp32 sltime, int instance_id) {
    return get_instance(instance_id)->set_pause_time(sltime);
  }
  int __stdcall set_collision_sensitivity(int sensitivity, bool wait, int instance_id) {
    return get_instance(instance_id)->set_collision_sensitivity(sensitivity, wait);
  }
  int __stdcall set_teach_sensitivity(int sensitivity, bool wait, int instance_id) {
    return get_instance(instance_id)->set_teach_sensitivity(sensitivity, wait);
  }
  int __stdcall set_gravity_direction(fp32 gravity_dir[3], bool wait, int instance_id) {
    return get_instance(instance_id)->set_gravity_direction(gravity_dir, wait);
  }
  int __stdcall set_tcp_offset(fp32 pose_offset[6], bool wait, int instance_id) {
    return get_instance(instance_id)->set_tcp_offset(pose_offset, wait);
  }
  int __stdcall set_tcp_load(fp32 weight, fp32 center_of_gravity[3], bool wait, int instance_id) {
    return get_instance(instance_id)->set_tcp_load(weight, center_of_gravity, wait);
  }
  int __stdcall set_tcp_jerk(fp32 jerk, int instance_id) {
    return get_instance(instance_id)->set_tcp_jerk(jerk);
  }
  int __stdcall set_tcp_maxacc(fp32 acc, int instance_id) {
    return get_instance(instance_id)->set_tcp_maxacc(acc);
  }
  int __stdcall set_joint_jerk(fp32 jerk, int instance_id) {
    return get_instance(instance_id)->set_joint_jerk(jerk);
  }
  int __stdcall set_joint_maxacc(fp32 acc, int instance_id) {
    return get_instance(instance_id)->set_joint_maxacc(acc);
  }
  int __stdcall clean_conf(int instance_id) {
    return get_instance(instance_id)->clean_conf();
  }
  int __stdcall save_conf(int instance_id) {
    return get_instance(instance_id)->save_conf();
  }

  int __stdcall set_gripper_enable(bool enable, int instance_id) {
    return get_instance(instance_id)->set_gripper_enable(enable);
  }
  int __stdcall set_gripper_mode(int mode, int instance_id) {
    return get_instance(instance_id)->set_gripper_mode(mode);
  }
  int __stdcall set_gripper_speed(fp32 speed, int instance_id) {
    return get_instance(instance_id)->set_gripper_speed(speed);
  }
  int __stdcall set_gripper_position(fp32 pos, bool wait, fp32 timeout, bool wait_motion, int instance_id) {
    return get_instance(instance_id)->set_gripper_position(pos, wait, timeout, wait_motion);
  }
  int __stdcall get_gripper_position(fp32 *pos, int instance_id) {
    return get_instance(instance_id)->get_gripper_position(pos);
  }
  int __stdcall get_gripper_err_code(int *err, int instance_id) {
    return get_instance(instance_id)->get_gripper_err_code(err);
  }
  int __stdcall clean_gripper_error(int instance_id) {
    return get_instance(instance_id)->clean_gripper_error();
  }
  int __stdcall get_tgpio_digital(int *io0_value, int *io1_value, int *io2_value, int *io3_value, int *io4_value, int instance_id) {
    return get_instance(instance_id)->get_tgpio_digital(io0_value, io1_value, io2_value, io3_value, io4_value);
  }
  int __stdcall set_tgpio_digital(int ionum, int value, fp32 delay_sec, bool sync, int instance_id) {
    return get_instance(instance_id)->set_tgpio_digital(ionum, value, delay_sec, sync);
  }
  int __stdcall get_tgpio_analog(int ionum, fp32 *value, int instance_id) {
    return get_instance(instance_id)->get_tgpio_analog(ionum, value);
  }
  int __stdcall get_cgpio_digital(int *digitals, int *digitals2, int instance_id) {
    return get_instance(instance_id)->get_cgpio_digital(digitals, digitals2);
  }
  int __stdcall get_cgpio_analog(int ionum, fp32 *value, int instance_id) {
    return get_instance(instance_id)->get_cgpio_analog(ionum, value);
  }
  int __stdcall set_cgpio_digital(int ionum, int value, float delay_sec, bool sync, int instance_id) {
    return get_instance(instance_id)->set_cgpio_digital(ionum, value, delay_sec, sync);
  }
  int __stdcall set_cgpio_analog(int ionum, float value, bool sync, int instance_id) {
    return get_instance(instance_id)->set_cgpio_analog(ionum, value, sync);
  }
  int __stdcall set_cgpio_digital_input_function(int ionum, int fun, int instance_id) {
    return get_instance(instance_id)->set_cgpio_digital_input_function(ionum, fun);
  }
  int __stdcall set_cgpio_digital_output_function(int ionum, int fun, int instance_id) {
    return get_instance(instance_id)->set_cgpio_digital_output_function(ionum, fun);
  }
  int __stdcall get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf, int instance_id) {
    return get_instance(instance_id)->get_cgpio_state(state, digit_io, analog, input_conf, output_conf);
  }

  int __stdcall get_version(unsigned char version[40], int instance_id) {
    return get_instance(instance_id)->get_version(version);
  }
  int __stdcall get_robot_sn(unsigned char robot_sn[40], int instance_id) {
    return get_instance(instance_id)->get_robot_sn(robot_sn);
  }
  int __stdcall get_state(int *state, int instance_id) {
    return get_instance(instance_id)->get_state(state);
  }
  int __stdcall system_control(int value, int instance_id) {
    return get_instance(instance_id)->system_control(value);
  }
  int __stdcall shutdown_system(int value, int instance_id) {
    return get_instance(instance_id)->shutdown_system(value);
  }
  int __stdcall get_cmdnum(int *cmdnum, int instance_id) {
    return get_instance(instance_id)->get_cmdnum(cmdnum);
  }
  int __stdcall get_err_warn_code(int err_warn[2], int instance_id) {
    return get_instance(instance_id)->get_err_warn_code(err_warn);
  }
  int __stdcall get_position(fp32 pose[6], int instance_id) {
    return get_instance(instance_id)->get_position(pose);
  }
  int __stdcall get_servo_angle(fp32 angles[7], bool is_real, int instance_id) {
    return get_instance(instance_id)->get_servo_angle(angles, is_real);
  }

  int __stdcall get_suction_cup(int *val, int instance_id) {
    return get_instance(instance_id)->get_suction_cup(val);
  }
  int __stdcall get_vacuum_gripper(int *val, int instance_id) {
    return get_instance(instance_id)->get_vacuum_gripper(val);
  }
  int __stdcall set_suction_cup(bool on, bool wait, float timeout, float delay_sec, bool sync, int instance_id) {
    return get_instance(instance_id)->set_suction_cup(on, wait, timeout, delay_sec, sync);
  }
  int __stdcall set_vacuum_gripper(bool on, bool wait, float timeout, float delay_sec, bool sync, int instance_id) {
    return get_instance(instance_id)->set_vacuum_gripper(on, wait, timeout, delay_sec, sync);
  }
  int __stdcall set_reduced_mode(bool on, int instance_id) {
    return get_instance(instance_id)->set_reduced_mode(on);
  }
  int __stdcall set_reduced_max_tcp_speed(float speed, int instance_id) {
    return get_instance(instance_id)->set_reduced_max_tcp_speed(speed);
  }
  int __stdcall set_reduced_max_joint_speed(float speed, int instance_id) {
    return get_instance(instance_id)->set_reduced_max_joint_speed(speed);
  }
  int __stdcall get_reduced_mode(int *mode, int instance_id) {
    return get_instance(instance_id)->get_reduced_mode(mode);
  }
  int __stdcall get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14], int *fense_is_on, int *collision_rebound_is_on, int instance_id) {
    return get_instance(instance_id)->get_reduced_states(on, xyz_list, tcp_speed, joint_speed, jrange, fense_is_on, collision_rebound_is_on);
  }
  int __stdcall set_reduced_tcp_boundary(int boundary[6], int instance_id) {
    return get_instance(instance_id)->set_reduced_tcp_boundary(boundary);
  }
  int __stdcall set_reduced_joint_range(float jrange[14], int instance_id) {
    return get_instance(instance_id)->set_reduced_joint_range(jrange);
  }
  int __stdcall set_fense_mode(bool on, int instance_id) {
    return get_instance(instance_id)->set_fense_mode(on);
  }
  int __stdcall set_fence_mode(bool on, int instance_id) {
    return get_instance(instance_id)->set_fence_mode(on);
  }
  int __stdcall set_collision_rebound(bool on, int instance_id) {
    return get_instance(instance_id)->set_collision_rebound(on);
  }
  int __stdcall set_world_offset(float pose_offset[6], bool wait, int instance_id) {
    return get_instance(instance_id)->set_world_offset(pose_offset, wait);
  }
  int __stdcall start_record_trajectory(int instance_id) {
    return get_instance(instance_id)->start_record_trajectory();
  }
  int __stdcall stop_record_trajectory(char* filename, int instance_id) {
    return get_instance(instance_id)->stop_record_trajectory(filename);
  }
  int __stdcall save_record_trajectory(char* filename, float timeout, int instance_id) {
    return get_instance(instance_id)->save_record_trajectory(filename, timeout);
  }
  int __stdcall load_trajectory(char* filename, float timeout, int instance_id) {
    return get_instance(instance_id)->load_trajectory(filename, timeout);
  }
  int __stdcall playback_trajectory(int times, char* filename, bool wait, int double_speed, int instance_id) {
    return get_instance(instance_id)->playback_trajectory(times, filename, wait, double_speed);
  }
  int __stdcall get_trajectory_rw_status(int *status, int instance_id) {
    return get_instance(instance_id)->get_trajectory_rw_status(status);
  }
  int __stdcall set_counter_reset(int instance_id) {
    return get_instance(instance_id)->set_counter_reset();
  }
  int __stdcall set_counter_increase(int instance_id) {
    return get_instance(instance_id)->set_counter_increase();
  }
  int __stdcall set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r, int instance_id) {
    return get_instance(instance_id)->set_tgpio_digital_with_xyz(ionum, value, xyz, tol_r);
  }
  int __stdcall set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r, int instance_id) {
    return get_instance(instance_id)->set_cgpio_digital_with_xyz(ionum, value, xyz, tol_r);
  }
  int __stdcall set_cgpio_analog_with_xyz(int ionum, float value, float xyz[3], float tol_r, int instance_id) {
    return get_instance(instance_id)->set_cgpio_analog_with_xyz(ionum, value, xyz, tol_r);
  }

  int __stdcall get_inverse_kinematics(fp32 pose[6], fp32 angles[7], int instance_id) {
    return get_instance(instance_id)->get_inverse_kinematics(pose, angles);
  }
  int __stdcall get_forward_kinematics(fp32 angles[7], fp32 pose[6], int instance_id) {
    return get_instance(instance_id)->get_forward_kinematics(angles, pose);
  }
  int __stdcall is_joint_limit(fp32 angles[7], int *limit, int instance_id) {
    return get_instance(instance_id)->is_joint_limit(angles, limit);
  }
  int __stdcall is_tcp_limit(fp32 pose[6], int *limit, int instance_id) {
    return get_instance(instance_id)->is_tcp_limit(pose, limit);
  }
  int __stdcall set_position_aa(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord, bool relative, bool wait, fp32 timeout, fp32 radius, unsigned char motion_type, int instance_id) {
    return get_instance(instance_id)->set_position_aa(pose, speed, acc, mvtime, is_tool_coord, relative, wait, timeout, radius, motion_type);
  }
  int __stdcall set_servo_cartesian_aa(fp32 pose[6], fp32 speed, fp32 acc, bool is_tool_coord, bool relative, int instance_id) {
    return get_instance(instance_id)->set_servo_cartesian_aa(pose, speed, acc, is_tool_coord, relative);
  }

  int __stdcall robotiq_reset(unsigned char ret_data[6], int instance_id) {
    return get_instance(instance_id)->robotiq_reset(ret_data);
  }
  int __stdcall robotiq_set_activate(bool wait, fp32 timeout, unsigned char ret_data[6], int instance_id) {
    return get_instance(instance_id)->robotiq_set_activate(wait, timeout, ret_data);
  }
  int __stdcall robotiq_set_position(unsigned char pos, unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6], bool wait_motion, int instance_id) {
    return get_instance(instance_id)->robotiq_set_position(pos, speed, force, wait, timeout, ret_data, wait_motion);
  }
  int __stdcall robotiq_open(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6], bool wait_motion, int instance_id) {
    return get_instance(instance_id)->robotiq_open(speed, force, wait, timeout, ret_data, wait_motion);
  }
  int __stdcall robotiq_close(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6], bool wait_motion, int instance_id) {
    return get_instance(instance_id)->robotiq_close(speed, force, wait, timeout, ret_data, wait_motion);
  }
  int __stdcall robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers, int instance_id) {
    return get_instance(instance_id)->robotiq_get_status(ret_data, number_of_registers);
  }

  int __stdcall set_bio_gripper_enable(bool enable, bool wait, fp32 timeout, int instance_id) {
    return get_instance(instance_id)->set_bio_gripper_enable(enable, wait, timeout);
  }
  int __stdcall set_bio_gripper_speed(int speed, int instance_id) {
    return get_instance(instance_id)->set_bio_gripper_speed(speed);
  }
  int __stdcall set_bio_gripper_control_mode(int mode, int instance_id) {
    return get_instance(instance_id)->set_bio_gripper_control_mode(mode);
  }
  int __stdcall set_bio_gripper_force(int force, int instance_id) {
    return get_instance(instance_id)->set_bio_gripper_force(force);
  }
  int __stdcall set_bio_gripper_position(int pos, int speed, int force, bool wait, fp32 timeout, bool wait_motion, int instance_id) {
    return get_instance(instance_id)->set_bio_gripper_position(pos, speed, force, wait, timeout, wait_motion);
  }
  int __stdcall open_bio_gripper(int speed, bool wait, fp32 timeout, bool wait_motion, int instance_id) {
    return get_instance(instance_id)->open_bio_gripper(speed, wait, timeout, wait_motion);
  }
  int __stdcall close_bio_gripper(int speed, bool wait, fp32 timeout, bool wait_motion, int instance_id) {
    return get_instance(instance_id)->close_bio_gripper(speed, wait, timeout, wait_motion);
  }
  int __stdcall get_bio_gripper_status(int *status, int instance_id) {
    return get_instance(instance_id)->get_bio_gripper_status(status);
  }
  int __stdcall get_bio_gripper_error(int *err, int instance_id) {
    return get_instance(instance_id)->get_bio_gripper_error(err);
  }
  int __stdcall clean_bio_gripper_error(int instance_id) {
    return get_instance(instance_id)->clean_bio_gripper_error();
  }

  int __stdcall set_tgpio_modbus_timeout(int timeout, bool is_transparent_transmission, int instance_id) {
    return get_instance(instance_id)->set_tgpio_modbus_timeout(timeout, is_transparent_transmission);
  }
  int __stdcall set_tgpio_modbus_baudrate(int baud, int instance_id) {
    return get_instance(instance_id)->set_tgpio_modbus_baudrate(baud);
  }
  int __stdcall get_tgpio_modbus_baudrate(int *baud, int instance_id) {
    return get_instance(instance_id)->get_tgpio_modbus_baudrate(baud);
  }
  int __stdcall getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, unsigned char host_id, bool is_transparent_transmission, bool use_503_port, int instance_id) {
    return get_instance(instance_id)->getset_tgpio_modbus_data(modbus_data, modbus_length, ret_data, ret_length, host_id, is_transparent_transmission, use_503_port);
  }
  int __stdcall set_self_collision_detection(bool on, int instance_id) {
    return get_instance(instance_id)->set_self_collision_detection(on);
  }
  int __stdcall set_simulation_robot(bool on, int instance_id) {
    return get_instance(instance_id)->set_simulation_robot(on);
  }
  int __stdcall vc_set_joint_velocity(fp32 speeds[7], bool is_sync, fp32 duration, int instance_id) {
    return get_instance(instance_id)->vc_set_joint_velocity(speeds, is_sync, duration);
  }
  int __stdcall vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord, fp32 duration, int instance_id) {
    return get_instance(instance_id)->vc_set_cartesian_velocity(speeds, is_tool_coord, duration);
  }

  int __stdcall set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6], int instance_id) {
    return get_instance(instance_id)->set_impedance(coord, c_axis, M, K, B);
  }
  int __stdcall set_impedance_mbk(float M[6], float K[6], float B[6], int instance_id) {
    return get_instance(instance_id)->set_impedance_mbk(M, K, B);
  }
  int __stdcall set_impedance_config(int coord, int c_axis[6], int instance_id) {
    return get_instance(instance_id)->set_impedance_config(coord, c_axis);
  }
  int __stdcall config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6], int instance_id) {
    return get_instance(instance_id)->config_force_control(coord, c_axis, f_ref, limits);
  }
  int __stdcall set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6], int instance_id) {
    return get_instance(instance_id)->set_force_control_pid(kp, ki, kd, xe_limit);
  }
  int __stdcall ft_sensor_set_zero(int instance_id) {
    return get_instance(instance_id)->ft_sensor_set_zero();
  }
  int __stdcall ft_sensor_iden_load(float result[10], int instance_id) {
    return get_instance(instance_id)->ft_sensor_iden_load(result);
  }
  int __stdcall ft_sensor_cali_load(float load[10], bool association_setting_tcp_load, float m, float x, float y, float z, int instance_id) {
    return get_instance(instance_id)->ft_sensor_cali_load(load, association_setting_tcp_load, m, x, y, z);
  }
  int __stdcall ft_sensor_enable(int on_off, int instance_id) {
    return get_instance(instance_id)->ft_sensor_enable(on_off);
  }
  int __stdcall ft_sensor_app_set(int app_code, int instance_id) {
    return get_instance(instance_id)->ft_sensor_app_set(app_code);
  }
  int __stdcall ft_sensor_app_get(int *app_code, int instance_id) {
    return get_instance(instance_id)->ft_sensor_app_get(app_code);
  }
  int __stdcall get_ft_sensor_data(float ft_data[6], int instance_id) {
    return get_instance(instance_id)->get_ft_sensor_data(ft_data);
  }
  int __stdcall get_ft_sensor_config(int *ft_app_status, int *ft_is_started, int *ft_type, int *ft_id, int *ft_freq, 
    float *ft_mass, float *ft_dir_bias, float ft_centroid[3], float ft_zero[6], int *imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6],
    int *f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6], int instance_id) {
    return get_instance(instance_id)->get_ft_sensor_config(ft_app_status, ft_is_started, ft_type, ft_id, ft_freq,
      ft_mass, ft_dir_bias, ft_centroid, ft_zero, imp_coord, imp_c_axis, M, K, B,
      f_coord, f_c_axis, f_ref, f_limits, kp, ki, kd, xe_limit);
  }
  int __stdcall get_ft_sensor_error(int *err, int instance_id) {
    return get_instance(instance_id)->get_ft_sensor_error(err);
  }
  
  int __stdcall iden_tcp_load(float result[4], float estimated_mass, int instance_id) {
    return get_instance(instance_id)->iden_tcp_load(result, estimated_mass);
  }

  int __stdcall get_linear_track_error(int *err, int instance_id) {
    return get_instance(instance_id)->get_linear_track_error(err);
  }
  int __stdcall get_linear_track_status(int *status, int instance_id) {
    return get_instance(instance_id)->get_linear_track_status(status);
  }
  int __stdcall get_linear_track_pos(int *pos, int instance_id) {
    return get_instance(instance_id)->get_linear_track_pos(pos);
  }
  int __stdcall get_linear_track_is_enabled(int *status, int instance_id) {
    return get_instance(instance_id)->get_linear_track_is_enabled(status);
  }
  int __stdcall get_linear_track_on_zero(int *status, int instance_id) {
    return get_instance(instance_id)->get_linear_track_on_zero(status);
  }
  int __stdcall get_linear_track_sci(int *sci1, int instance_id) {
    return get_instance(instance_id)->get_linear_track_sci(sci1);
  }
  int __stdcall get_linear_track_sco(int sco[2], int instance_id) {
    return get_instance(instance_id)->get_linear_track_sco(sco);
  }
  int __stdcall clean_linear_track_error(int instance_id) {
    return get_instance(instance_id)->clean_linear_track_error();
  }
  int __stdcall set_linear_track_enable(bool enable, int instance_id) {
    return get_instance(instance_id)->set_linear_track_enable(enable);
  }
  int __stdcall set_linear_track_speed(int speed, int instance_id) {
    return get_instance(instance_id)->set_linear_track_speed(speed);
  }
  int __stdcall set_linear_track_back_origin(bool wait, bool auto_enable, int instance_id) {
    return get_instance(instance_id)->set_linear_track_back_origin(wait, auto_enable);
  }
  int __stdcall set_linear_track_pos(int pos, int speed, bool wait, fp32 timeout, bool auto_enable, int instance_id) {
    return get_instance(instance_id)->set_linear_track_pos(pos, speed, wait, timeout, auto_enable);
  }
  int __stdcall set_linear_track_stop(int instance_id) {
    return get_instance(instance_id)->set_linear_track_stop();
  }

  int __stdcall set_timeout(float timeout, int instance_id) {
    return get_instance(instance_id)->set_timeout(timeout);
  }

  int __stdcall set_baud_checkset_enable(bool enable, int instance_id) {
    return get_instance(instance_id)->set_baud_checkset_enable(enable);
  }

  int __stdcall set_checkset_default_baud(int type, int baud, int instance_id) {
    return get_instance(instance_id)->set_checkset_default_baud(type, baud);
  }

  int __stdcall get_checkset_default_baud(int type, int *baud, int instance_id) {
    return get_instance(instance_id)->get_checkset_default_baud(type, baud);
  }

  int __stdcall set_cartesian_velo_continuous(bool on_off, int instance_id) {
    return get_instance(instance_id)->set_cartesian_velo_continuous(on_off);
  }

  int __stdcall set_allow_approx_motion(bool on_off, int instance_id) {
    return get_instance(instance_id)->set_allow_approx_motion(on_off);
  }

  int __stdcall get_joint_states(fp32 position[7], fp32 velocity[7], fp32 effort[7], int num, int instance_id) {
    return get_instance(instance_id)->get_joint_states(position, velocity, effort, num);
  }

  int __stdcall iden_joint_friction(int *result, unsigned char *sn, int instance_id) {
    return get_instance(instance_id)->iden_joint_friction(result, sn);
  }

  int __stdcall set_only_check_type(unsigned char only_check_type, int instance_id) {
    return get_instance(instance_id)->set_only_check_type(only_check_type);
  }

  int __stdcall open_lite6_gripper(bool sync, int instance_id) {
    return get_instance(instance_id)->open_lite6_gripper(sync);
  }
  
  int __stdcall close_lite6_gripper(bool sync, int instance_id) {
    return get_instance(instance_id)->close_lite6_gripper(sync);
  }

  int __stdcall stop_lite6_gripper(bool sync, int instance_id) {
    return get_instance(instance_id)->stop_lite6_gripper(sync);
  }

  int __stdcall get_dh_params(fp32 dh_params[28], int instance_id)
  {
    return get_instance(instance_id)->get_dh_params(dh_params);
  }
  
  int __stdcall set_dh_params(fp32 dh_params[28], unsigned char flag, int instance_id)
  {
    return get_instance(instance_id)->set_dh_params(dh_params, flag);
  }

  int __stdcall set_feedback_type(unsigned char feedback_type, int instance_id)
  {
    return get_instance(instance_id)->set_feedback_type(feedback_type);
  }

  int __stdcall set_linear_spd_limit_factor(float factor, int instance_id)
  {
    return get_instance(instance_id)->set_linear_spd_limit_factor(factor);
  }

  int __stdcall set_cmd_mat_history_num(int num, int instance_id)
  {
    return get_instance(instance_id)->set_cmd_mat_history_num(num);
  }

  int __stdcall set_fdb_mat_history_num(int num, int instance_id)
  {
    return get_instance(instance_id)->set_fdb_mat_history_num(num);
  }

  int __stdcall get_linear_spd_limit_factor(float *factor, int instance_id)
  {
    return get_instance(instance_id)->get_linear_spd_limit_factor(factor);
  }

  int __stdcall get_cmd_mat_history_num(int *num, int instance_id)
  {
    return get_instance(instance_id)->get_cmd_mat_history_num(num);
  }

  int __stdcall get_fdb_mat_history_num(int *num, int instance_id)
  {
    return get_instance(instance_id)->get_fdb_mat_history_num(num);
  }

  int __stdcall get_tgpio_modbus_timeout(int *timeout, bool is_transparent_transmission, int instance_id)
  {
    return get_instance(instance_id)->get_tgpio_modbus_timeout(timeout, is_transparent_transmission);
  }

  int __stdcall get_poe_status(int *status, int instance_id)
  {
    return get_instance(instance_id)->get_poe_status(status);
  }

  int __stdcall get_iden_status(int *status, int instance_id)
  {
    return get_instance(instance_id)->get_iden_status(status);
  }

  int __stdcall get_c31_error_info(int *servo_id, float *theoretical_tau, float *actual_tau, int instance_id)
  {
    return get_instance(instance_id)->get_c31_error_info(servo_id, theoretical_tau, actual_tau);
  }

  int __stdcall get_c54_error_info(int *dir, float *tau_threshold, float *actual_tau, int instance_id)
  {
    return get_instance(instance_id)->get_c54_error_info(dir, tau_threshold, actual_tau);
  }

  int __stdcall get_c37_error_info(int *servo_id, float *diff_angle, int instance_id)
  {
    return get_instance(instance_id)->get_c37_error_info(servo_id, diff_angle);
  }

  int __stdcall get_c23_error_info(int *id_bits, float angles[7], int instance_id)
  {
    return get_instance(instance_id)->get_c23_error_info(id_bits, angles);
  }

  int __stdcall get_c24_error_info(int *servo_id, float *speed, int instance_id)
  {
    return get_instance(instance_id)->get_c24_error_info(servo_id, speed);
  }

  int __stdcall get_c60_error_info(float *max_velo, float *curr_velo, int instance_id)
  {
    return get_instance(instance_id)->get_c60_error_info(max_velo, curr_velo);
  }

  int __stdcall get_c38_error_info(int *id_bits, float angles[7], int instance_id)
  {
    return get_instance(instance_id)->get_c38_error_info(id_bits, angles);
  }

  int __stdcall set_ft_collision_detection(int on_off, int instance_id)
  {
    return get_instance(instance_id)->set_ft_collision_detection(on_off);
  }

  int __stdcall set_ft_collision_rebound(int on_off, int instance_id)
  {
    return get_instance(instance_id)->set_ft_collision_rebound(on_off);
  }

  int __stdcall set_ft_collision_threshold(float thresholds[6], int instance_id)
  {
    return get_instance(instance_id)->set_ft_collision_threshold(thresholds);
  }

  int __stdcall set_ft_collision_reb_distance(float distances[6], int instance_id)
  {
    return get_instance(instance_id)->set_ft_collision_reb_distance(distances);
  }

  int __stdcall get_ft_collision_detection(int *on_off, int instance_id)
  {
    return get_instance(instance_id)->get_ft_collision_detection(on_off);
  }

  int __stdcall get_ft_collision_rebound(int *on_off, int instance_id)
  {
    return get_instance(instance_id)->get_ft_collision_rebound(on_off);
  }

  int __stdcall get_ft_collision_threshold(float thresholds[6], int instance_id)
  {
    return get_instance(instance_id)->get_ft_collision_threshold(thresholds);
  }

  int __stdcall get_ft_collision_reb_distance(float distances[6], int instance_id)
  {
    return get_instance(instance_id)->get_ft_collision_reb_distance(distances);
  }

  /* modbus tcp func_code: 0x01 */
  int __stdcall read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, int instance_id)
  {
    return get_instance(instance_id)->read_coil_bits(addr, quantity, bits);
  }
  /* modbus tcp func_code: 0x02 */
  int __stdcall read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, int instance_id)
  {
    return get_instance(instance_id)->read_input_bits(addr, quantity, bits);
  }
  /* modbus tcp func_code: 0x03 */
  int __stdcall read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed, int instance_id)
  {
    return get_instance(instance_id)->read_holding_registers(addr, quantity, regs, is_signed);
  }
  /* modbus tcp func_code: 0x04 */
  int __stdcall read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed, int instance_id)
  {
    return get_instance(instance_id)->read_input_registers(addr, quantity, regs, is_signed);
  }
  /* modbus tcp func_code: 0x05 */
  int __stdcall write_single_coil_bit(unsigned short addr, unsigned char bit_val, int instance_id)
  {
    return get_instance(instance_id)->write_single_coil_bit(addr, bit_val);
  }
  /* modbus tcp func_code: 0x06 */
  int __stdcall write_single_holding_register(unsigned short addr, int reg_val, int instance_id)
  {
    return get_instance(instance_id)->write_single_holding_register(addr, reg_val);
  }
  /* modbus tcp func_code: 0x0F */
  int __stdcall write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, int instance_id)
  {
    return get_instance(instance_id)->write_multiple_coil_bits(addr, quantity, bits);
  }
  /* modbus tcp func_code: 0x10 */
  int __stdcall write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs, int instance_id)
  {
    return get_instance(instance_id)->write_multiple_holding_registers(addr, quantity, regs);
  }
  /* modbus tcp func_code: 0x16 */
  int __stdcall mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask, int instance_id)
  {
    return get_instance(instance_id)->mask_write_holding_register(addr, and_mask, or_mask);
  }
  /* modbus tcp func_code: 0x17 */
  int __stdcall write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed, int instance_id)
  {
    return get_instance(instance_id)->write_and_read_holding_registers(r_addr, r_quantity, r_regs, w_addr, w_quantity, w_regs, is_signed);
  }
}
