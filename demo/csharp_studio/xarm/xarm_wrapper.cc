/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
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
		printf("[switch failed], no instance with id %d, ", instance_id);
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

	int __stdcall connect(char* port) {
		return arm->connect(port);
	}
	void __stdcall disconnect(void) {
		arm->disconnect();
	}
	int __stdcall motion_enable(bool enable, int servo_id) {
		return arm->motion_enable(enable, servo_id);
	}
	int __stdcall set_mode(int mode) {
		return arm->set_mode(mode);
	}
	int __stdcall set_state(int state) {
		return arm->set_state(state);
	}
	int __stdcall clean_warn(void) {
		return arm->clean_warn();
	}
	int __stdcall clean_error(void) {
		return arm->clean_error();
	}
	int __stdcall set_position(fp32 pose[6], fp32 radius, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
		return arm->set_position(pose, radius, speed, acc, mvtime, wait, timeout);
	}
	int __stdcall set_tool_position(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
		return arm->set_tool_position(pose, speed, acc, mvtime, wait, timeout);
	}
	int __stdcall set_servo_angle(fp32 angles[7], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius) {
		return arm->set_servo_angle(angles, speed, acc, mvtime, wait, timeout, radius);
	}
	int __stdcall set_servo_angle_j(fp32 angles[7], fp32 speed, fp32 acc, fp32 mvtime) {
		return arm->set_servo_angle_j(angles, speed, acc, mvtime);
	}
	int __stdcall set_servo_cartesian(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord) {
		return arm->set_servo_cartesian(pose, speed, acc, mvtime, is_tool_coord);
	}
	int __stdcall move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
		return arm->move_circle(pose1, pose2, percent, speed, acc, mvtime, wait, timeout);
	}
	int __stdcall move_gohome(fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
		return arm->move_gohome(speed, acc, mvtime, wait, timeout);
	}
	void __stdcall reset(bool wait, fp32 timeout) {
		return arm->reset(wait, timeout);
	}
	void __stdcall emergency_stop(void) {
		arm->emergency_stop();
	}

	int __stdcall set_servo_attach(int servo_id) {
		return arm->set_servo_attach(servo_id);
	}
	int __stdcall set_servo_detach(int servo_id) {
		return arm->set_servo_attach(servo_id);
	}
	int __stdcall set_pause_time(fp32 sltime) {
		return arm->set_pause_time(sltime);
	}
	int __stdcall set_collision_sensitivity(int sensitivity) {
		return arm->set_collision_sensitivity(sensitivity);
	}
	int __stdcall set_teach_sensitivity(int sensitivity) {
		return arm->set_teach_sensitivity(sensitivity);
	}
	int __stdcall set_gravity_direction(fp32 gravity_dir[3]) {
		return arm->set_gravity_direction(gravity_dir);
	}
	int __stdcall set_tcp_offset(fp32 pose_offset[6]) {
		return arm->set_tcp_offset(pose_offset);
	}
	int __stdcall set_tcp_load(fp32 weight, fp32 center_of_gravity[3]) {
		return arm->set_tcp_load(weight, center_of_gravity);
	}
	int __stdcall set_tcp_jerk(fp32 jerk) {
		return arm->set_tcp_jerk(jerk);
	}
	int __stdcall set_tcp_maxacc(fp32 acc) {
		return arm->set_tcp_maxacc(acc);
	}
	int __stdcall set_joint_jerk(fp32 jerk) {
		return arm->set_joint_jerk(jerk);
	}
	int __stdcall set_joint_maxacc(fp32 acc) {
		return arm->set_joint_maxacc(acc);
	}
	int __stdcall clean_conf(void) {
		return arm->clean_conf();
	}
	int __stdcall save_conf(void) {
		return arm->save_conf();
	}

	int __stdcall set_gripper_enable(bool enable) {
		return arm->set_gripper_enable(enable);
	}
	int __stdcall set_gripper_mode(int mode) {
		return arm->set_gripper_mode(mode);
	}
	int __stdcall set_gripper_speed(fp32 speed) {
		return arm->set_gripper_speed(speed);
	}
	int __stdcall set_gripper_position(fp32 pos, bool wait, fp32 timeout, bool wait_motion) {
		return arm->set_gripper_position(pos, wait, timeout, wait_motion);
	}
	int __stdcall get_gripper_position(fp32 *pos) {
		return arm->get_gripper_position(pos);
	}
	int __stdcall get_gripper_err_code(int *err) {
		return arm->get_gripper_err_code(err);
	}
	int __stdcall clean_gripper_error(void) {
		return arm->clean_gripper_error();
	}
	int __stdcall get_tgpio_digital(int *io0_value, int *io1_value) {
		return arm->get_tgpio_digital(io0_value, io1_value);
	}
	int __stdcall set_tgpio_digital(int ionum, int value, fp32 delay_sec) {
		return arm->set_tgpio_digital(ionum, value, delay_sec);
	}
	int __stdcall get_tgpio_analog(int ionum, fp32 *value) {
		return arm->get_tgpio_analog(ionum, value);
	}
	int __stdcall get_cgpio_digital(int *digitals, int *digitals2) {
		return arm->get_cgpio_digital(digitals, digitals2);
	}
	int __stdcall get_cgpio_analog(int ionum, fp32 *value) {
		return arm->get_cgpio_analog(ionum, value);
	}
	int __stdcall set_cgpio_digital(int ionum, int value, float delay_sec) {
		return arm->set_cgpio_digital(ionum, value, delay_sec);
	}
	int __stdcall set_cgpio_analog(int ionum, float value) {
		return arm->set_cgpio_analog(ionum, value);
	}
	int __stdcall set_cgpio_digital_input_function(int ionum, int fun) {
		return arm->set_cgpio_digital_input_function(ionum, fun);
	}
	int __stdcall set_cgpio_digital_output_function(int ionum, int fun) {
		return arm->set_cgpio_digital_output_function(ionum, fun);
	}
	int __stdcall get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf) {
		return arm->get_cgpio_state(state, digit_io, analog, input_conf, output_conf);
	}

	int __stdcall get_version(unsigned char version[40]) {
		return arm->get_version(version);
	}
	int __stdcall get_robot_sn(unsigned char robot_sn[40]) {
		return arm->get_robot_sn(robot_sn);
	}
	int __stdcall get_state(int *state) {
		return arm->get_state(state);
	}
	int __stdcall shutdown_system(int value) {
		return arm->shutdown_system(value);
	}
	int __stdcall get_cmdnum(int *cmdnum) {
		return arm->get_cmdnum(cmdnum);
	}
	int __stdcall get_err_warn_code(int err_warn[2]) {
		return arm->get_err_warn_code(err_warn);
	}
	int __stdcall get_position(fp32 pose[6]) {
		return arm->get_position(pose);
	}
	int __stdcall get_servo_angle(fp32 angles[7]) {
		return arm->get_servo_angle(angles);
	}

	int __stdcall get_suction_cup(int *val) {
		return arm->get_suction_cup(val);
	}
	int __stdcall get_vacuum_gripper(int *val) {
		return arm->get_vacuum_gripper(val);
	}
	int __stdcall set_suction_cup(bool on, bool wait, float timeout, float delay_sec) {
		return arm->set_suction_cup(on, wait, timeout, delay_sec);
	}
	int __stdcall set_vacuum_gripper(bool on, bool wait, float timeout, float delay_sec) {
		return arm->set_vacuum_gripper(on, wait, timeout, delay_sec);
	}
	int __stdcall set_reduced_mode(bool on) {
		return arm->set_reduced_mode(on);
	}
	int __stdcall set_reduced_max_tcp_speed(float speed) {
		return arm->set_reduced_max_tcp_speed(speed);
	}
	int __stdcall set_reduced_max_joint_speed(float speed) {
		return arm->set_reduced_max_joint_speed(speed);
	}
	int __stdcall get_reduced_mode(int *mode) {
		return arm->get_reduced_mode(mode);
	}
	int __stdcall get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14], int *fense_is_on, int *collision_rebound_is_on) {
		return arm->get_reduced_states(on, xyz_list, tcp_speed, joint_speed, jrange, fense_is_on, collision_rebound_is_on);
	}
	int __stdcall set_reduced_tcp_boundary(int boundary[6]) {
		return arm->set_reduced_tcp_boundary(boundary);
	}
	int __stdcall set_reduced_joint_range(float jrange[14]) {
		return arm->set_reduced_joint_range(jrange);
	}
	int __stdcall set_fense_mode(bool on) {
		return arm->set_fense_mode(on);
	}
	int __stdcall set_fence_mode(bool on) {
		return arm->set_fence_mode(on);
	}
	int __stdcall set_collision_rebound(bool on) {
		return arm->set_collision_rebound(on);
	}
	int __stdcall set_world_offset(float pose_offset[6]) {
		return arm->set_world_offset(pose_offset);
	}
	int __stdcall start_record_trajectory(void) {
		return arm->start_record_trajectory();
	}
	int __stdcall stop_record_trajectory(char* filename) {
		return arm->stop_record_trajectory(filename);
	}
	int __stdcall save_record_trajectory(char* filename, float timeout) {
		return arm->save_record_trajectory(filename, timeout);
	}
	int __stdcall load_trajectory(char* filename, float timeout) {
		return arm->load_trajectory(filename, timeout);
	}
	int __stdcall playback_trajectory(int times, char* filename, bool wait, int double_speed) {
		return arm->playback_trajectory(times, filename, wait, double_speed);
	}
	int __stdcall get_trajectory_rw_status(int *status) {
		return arm->get_trajectory_rw_status(status);
	}
	int __stdcall set_counter_reset(void) {
		return arm->set_counter_reset();
	}
	int __stdcall set_counter_increase(void) {
		return arm->set_counter_increase();
	}
	int __stdcall set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r) {
		return arm->set_tgpio_digital_with_xyz(ionum, value, xyz, tol_r);
	}
	int __stdcall set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r) {
		return arm->set_cgpio_digital_with_xyz(ionum, value, xyz, tol_r);
	}
	int __stdcall set_cgpio_analog_with_xyz(int ionum, float value, float xyz[3], float tol_r) {
		return arm->set_cgpio_analog_with_xyz(ionum, value, xyz, tol_r);
	}

	int __stdcall get_inverse_kinematics(fp32 pose[6], fp32 angles[7]) {
		return arm->get_inverse_kinematics(pose, angles);
	}
	int __stdcall get_forward_kinematics(fp32 angles[7], fp32 pose[6]) {
		return arm->get_forward_kinematics(angles, pose);
	}
	int __stdcall is_joint_limit(fp32 angles[7], int *limit) {
		return arm->is_joint_limit(angles, limit);
	}
	int __stdcall is_tcp_limit(fp32 pose[6], int *limit) {
		return arm->is_tcp_limit(pose, limit);
	}
	int __stdcall set_position_aa(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord, bool relative, bool wait, fp32 timeout) {
		return arm->set_position_aa(pose, speed, acc, mvtime, is_tool_coord, relative, wait, timeout);
	}
	int __stdcall set_servo_cartesian_aa(fp32 pose[6], fp32 speed, fp32 acc, bool is_tool_coord, bool relative) {
		return arm->set_servo_cartesian_aa(pose, speed, acc, is_tool_coord, relative);
	}

	int __stdcall robotiq_reset(unsigned char ret_data[6]) {
		return arm->robotiq_reset(ret_data);
	}
	int __stdcall robotiq_set_activate(bool wait, fp32 timeout, unsigned char ret_data[6]) {
		return arm->robotiq_set_activate(wait, timeout, ret_data);
	}
	int __stdcall robotiq_set_position(unsigned char pos, unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6], bool wait_motion) {
		return arm->robotiq_set_position(pos, speed, force, wait, timeout, ret_data, wait_motion);
	}
	int __stdcall robotiq_open(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6], bool wait_motion) {
		return arm->robotiq_open(speed, force, wait, timeout, ret_data, wait_motion);
	}
	int __stdcall robotiq_close(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6], bool wait_motion) {
		return arm->robotiq_close(speed, force, wait, timeout, ret_data, wait_motion);
	}
	int __stdcall robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers) {
		return arm->robotiq_get_status(ret_data, number_of_registers);
	}

	int __stdcall set_bio_gripper_enable(bool enable, bool wait, fp32 timeout) {
		return arm->set_bio_gripper_enable(enable, wait, timeout);
	}
	int __stdcall set_bio_gripper_speed(int speed) {
		return arm->set_bio_gripper_speed(speed);
	}
	int __stdcall open_bio_gripper(int speed, bool wait, fp32 timeout, bool wait_motion) {
		return arm->open_bio_gripper(speed, wait, timeout, wait_motion);
	}
	int __stdcall close_bio_gripper(int speed, bool wait, fp32 timeout, bool wait_motion) {
		return arm->close_bio_gripper(speed, wait, timeout, wait_motion);
	}
	int __stdcall get_bio_gripper_status(int *status) {
		return arm->get_bio_gripper_status(status);
	}
	int __stdcall get_bio_gripper_error(int *err) {
		return arm->get_bio_gripper_error(err);
	}
	int __stdcall clean_bio_gripper_error(void) {
		return arm->clean_bio_gripper_error();
	}

	int __stdcall set_tgpio_modbus_timeout(int timeout) {
		return arm->set_tgpio_modbus_timeout(timeout);
	}
	int __stdcall set_tgpio_modbus_baudrate(int baud) {
		return arm->set_tgpio_modbus_baudrate(baud);
	}
	int __stdcall get_tgpio_modbus_baudrate(int *baud) {
		return arm->get_tgpio_modbus_baudrate(baud);
	}
	int __stdcall getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length) {
		return arm->getset_tgpio_modbus_data(modbus_data, modbus_length, ret_data, ret_length);
	}
	int __stdcall set_self_collision_detection(bool on) {
		return arm->set_self_collision_detection(on);
	}
	int __stdcall set_simulation_robot(bool on) {
		return arm->set_simulation_robot(on);
	}
	int __stdcall vc_set_joint_velocity(fp32 speeds[7], bool is_sync, fp32 duration) {
		return arm->vc_set_joint_velocity(speeds, is_sync, duration);
	}
	int __stdcall vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord, fp32 duration) {
		return arm->vc_set_cartesian_velocity(speeds, is_tool_coord, duration);
	}

	int __stdcall set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6]) {
		return arm->set_impedance(coord, c_axis, M, K, B);
	}
	int __stdcall set_impedance_mbk(float M[6], float K[6], float B[6]) {
		return arm->set_impedance_mbk(M, K, B);
	}
	int __stdcall set_impedance_config(int coord, int c_axis[6]) {
		return arm->set_impedance_config(coord, c_axis);
	}
	int __stdcall config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6]) {
		return arm->config_force_control(coord, c_axis, f_ref, limits);
	}
	int __stdcall set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6]) {
		return arm->set_force_control_pid(kp, ki, kd, xe_limit);
	}
	int __stdcall ft_sensor_set_zero(void) {
		return arm->ft_sensor_set_zero();
	}
	int __stdcall ft_sensor_iden_load(float result[10]) {
		return arm->ft_sensor_iden_load(result);
	}
	int __stdcall ft_sensor_cali_load(float load[10], bool association_setting_tcp_load, float m, float x, float y, float z) {
		return arm->ft_sensor_cali_load(load, association_setting_tcp_load, m, x, y, z);
	}
	int __stdcall ft_sensor_enable(int on_off) {
		return arm->ft_sensor_enable(on_off);
	}
	int __stdcall ft_sensor_app_set(int app_code) {
		return arm->ft_sensor_app_set(app_code);
	}
	int __stdcall ft_sensor_app_get(int *app_code) {
		return arm->ft_sensor_app_get(app_code);
	}
	int __stdcall get_ft_sensor_data(float ft_data[6]) {
		return arm->get_ft_sensor_data(ft_data);
	}
	int __stdcall get_ft_sensor_config(int *ft_app_status, int *ft_is_started, int *ft_type, int *ft_id, int *ft_freq, 
		float *ft_mass, float *ft_dir_bias, float ft_centroid[3], float ft_zero[6], int *imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6],
		int *f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6]) {
		return arm->get_ft_sensor_config(ft_app_status, ft_is_started, ft_type, ft_id, ft_freq,
			ft_mass, ft_dir_bias, ft_centroid, ft_zero, imp_coord, imp_c_axis, M, K, B,
			f_coord, f_c_axis, f_ref, f_limits, kp, ki, kd, xe_limit);
	}
	int __stdcall get_ft_sensor_error(int *err) {
		return arm->get_ft_sensor_error(err);
	}
	
	int __stdcall iden_tcp_load(float result[4]) {
		return arm->iden_tcp_load(result);
	}

	int __stdcall get_linear_track_error(int *err) {
		return arm->get_linear_track_error(err);
	}
	int __stdcall get_linear_track_status(int *status) {
		return arm->get_linear_track_status(status);
	}
	int __stdcall get_linear_track_pos(int *pos) {
		return arm->get_linear_track_pos(pos);
	}
	int __stdcall get_linear_track_is_enabled(int *status) {
		return arm->get_linear_track_is_enabled(status);
	}
	int __stdcall get_linear_track_on_zero(int *status) {
		return arm->get_linear_track_on_zero(status);
	}
	int __stdcall get_linear_track_sci(int *sci1) {
		return arm->get_linear_track_sci(sci1);
	}
	int __stdcall get_linear_track_sco(int sco[2]) {
		return arm->get_linear_track_sco(sco);
	}
	int __stdcall clean_linear_track_error(void) {
		return arm->clean_linear_track_error();
	}
	int __stdcall set_linear_track_enable(bool enable) {
		return arm->set_linear_track_enable(enable);
	}
	int __stdcall set_linear_track_speed(int speed) {
		return arm->set_linear_track_speed(speed);
	}
	int __stdcall set_linear_track_back_origin(bool wait, bool auto_enable) {
		return arm->set_linear_track_back_origin(wait, auto_enable);
	}
	int __stdcall set_linear_track_pos(int pos, int speed, bool wait, fp32 timeout, bool auto_enable) {
		return arm->set_linear_track_pos(pos, speed, wait, timeout, auto_enable);
	}
	int __stdcall set_linear_track_stop(void) {
		return arm->set_linear_track_stop();
	}

	int __stdcall set_timeout(float timeout) {
		return arm->set_timeout(timeout);
	}

	int __stdcall set_baud_checkset_enable(bool enable) {
		return arm->set_baud_checkset_enable(enable);
	}

	int __stdcall set_checkset_default_baud(int type, int baud) {
		return arm->set_checkset_default_baud(type, baud);
	}

	int __stdcall get_checkset_default_baud(int type, int *baud) {
		return arm->get_checkset_default_baud(type, baud);
	}
}
