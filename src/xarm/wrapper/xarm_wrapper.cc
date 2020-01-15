#include <map>
#include "xarm/wrapper/xarm_wrapper.h"


namespace XArmWrapper
{
	XArmAPI* arm;
	int id = 0;
	std::map<int, XArmAPI*> xarm_map;

	int __stdcall switch_xarm(int instance_id) {
		std::map<int, XArmAPI*>::iterator iter = xarm_map.find(instance_id);
		if (iter != xarm_map.end()) {
			arm = iter->second;
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
		bool check_is_pause) {
		arm = new XArmAPI(port, is_radian, do_not_open,
			check_tcp_limit, check_joint_limit, check_cmdnum_limit,
			check_robot_sn, check_is_ready, check_is_pause);
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
	int __stdcall set_servo_angle(fp32 angles[7], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
		return arm->set_servo_angle(angles, speed, acc, mvtime, wait, timeout);
	}
	int __stdcall set_servo_angle_j(fp32 angles[7], fp32 speed, fp32 acc, fp32 mvtime) {
		return arm->set_servo_angle_j(angles, speed, acc, mvtime);
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
	int __stdcall set_gripper_position(fp32 pos, bool wait, fp32 timeout) {
		return arm->set_gripper_position(pos, wait, timeout);
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
	int __stdcall set_tgpio_digital(int ionum, int value) {
		return arm->set_tgpio_digital(ionum, value);
	}
	int __stdcall get_tgpio_analog(int ionum, fp32 *value) {
		return arm->get_tgpio_analog(ionum, value);
	}
	int __stdcall get_cgpio_digital(int *digitals) {
		return arm->get_cgpio_digital(digitals);
	}
	int __stdcall get_cgpio_analog(int ionum, fp32 *value) {
		return arm->get_cgpio_analog(ionum, value);
	}
	int __stdcall set_cgpio_digital(int ionum, int value) {
		return arm->set_cgpio_digital(ionum, value);
	}
	int __stdcall set_cgpio_analog(int ionum, int value) {
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
	int __stdcall set_suction_cup(bool on, bool wait, float timeout) {
		return arm->set_suction_cup(on, wait, timeout);
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
}
