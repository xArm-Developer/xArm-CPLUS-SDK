#include "xarm/wrapper/xarm_api.h"

namespace XArmWrapper {
	extern "C" __declspec(dllexport) int __stdcall switch_xarm(int instance_id);
	extern "C" __declspec(dllexport) int __stdcall create_instance(
		char* port="", 
		bool is_radian = DEFAULT_IS_RADIAN,
		bool do_not_open = false,
		bool check_tcp_limit = true,
		bool check_joint_limit = true,
		bool check_cmdnum_limit = true,
		bool check_robot_sn = false,
		bool check_is_ready = true,
		bool check_is_pause = true);
	extern "C" __declspec(dllexport) int __stdcall connect(char* port="");
	extern "C" __declspec(dllexport) void __stdcall disconnect(void);
	extern "C" __declspec(dllexport) int __stdcall motion_enable(bool enable, int servo_id=8);
	extern "C" __declspec(dllexport) int __stdcall set_mode(int mode);
	extern "C" __declspec(dllexport) int __stdcall set_state(int state);
	extern "C" __declspec(dllexport) int __stdcall clean_warn(void);
	extern "C" __declspec(dllexport) int __stdcall clean_error(void);
	extern "C" __declspec(dllexport) int __stdcall set_position(fp32 pose[6], fp32 radius = -1, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	extern "C" __declspec(dllexport) int __stdcall set_tool_position(fp32 pose[6], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=0);
	extern "C" __declspec(dllexport) int __stdcall set_servo_angle(fp32 angles[7], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	extern "C" __declspec(dllexport) int __stdcall set_servo_angle_j(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0);
	extern "C" __declspec(dllexport) int __stdcall move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	extern "C" __declspec(dllexport) int __stdcall move_gohome(fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	extern "C" __declspec(dllexport) void __stdcall reset(bool wait = false, fp32 timeout = NO_TIMEOUT);
	extern "C" __declspec(dllexport) void __stdcall emergency_stop(void);

	extern "C" __declspec(dllexport) int __stdcall set_servo_attach(int servo_id);
	extern "C" __declspec(dllexport) int __stdcall set_servo_detach(int servo_id);
	extern "C" __declspec(dllexport) int __stdcall set_pause_time(fp32 sltime);
	extern "C" __declspec(dllexport) int __stdcall set_collision_sensitivity(int sensitivity);
	extern "C" __declspec(dllexport) int __stdcall set_teach_sensitivity(int sensitivity);
	extern "C" __declspec(dllexport) int __stdcall set_gravity_direction(fp32 gravity_dir[3]);
	extern "C" __declspec(dllexport) int __stdcall set_tcp_offset(fp32 pose_offset[6]);
	extern "C" __declspec(dllexport) int __stdcall set_tcp_load(fp32 weight, fp32 center_of_gravity[3]);
	extern "C" __declspec(dllexport) int __stdcall set_tcp_jerk(fp32 jerk);
	extern "C" __declspec(dllexport) int __stdcall set_tcp_maxacc(fp32 acc);
	extern "C" __declspec(dllexport) int __stdcall set_joint_jerk(fp32 jerk);
	extern "C" __declspec(dllexport) int __stdcall set_joint_maxacc(fp32 acc);
	extern "C" __declspec(dllexport) int __stdcall clean_conf(void);
	extern "C" __declspec(dllexport) int __stdcall save_conf(void);
	
	extern "C" __declspec(dllexport) int __stdcall set_gripper_enable(bool enable);
	extern "C" __declspec(dllexport) int __stdcall set_gripper_mode(int mode);
	extern "C" __declspec(dllexport) int __stdcall set_gripper_speed(fp32 speed);
	extern "C" __declspec(dllexport) int __stdcall set_gripper_position(fp32 pos, bool wait=false, fp32 timeout=10);
	extern "C" __declspec(dllexport) int __stdcall get_gripper_position(fp32 *pos);
	extern "C" __declspec(dllexport) int __stdcall get_gripper_err_code(int *err);
	extern "C" __declspec(dllexport) int __stdcall clean_gripper_error(void);
	extern "C" __declspec(dllexport) int __stdcall get_tgpio_digital(int *io0_value, int *io1_value);
	extern "C" __declspec(dllexport) int __stdcall set_tgpio_digital(int ionum, int value);
	extern "C" __declspec(dllexport) int __stdcall get_tgpio_analog(int ionum, fp32 *value);
	extern "C" __declspec(dllexport) int __stdcall get_cgpio_digital(int *digitals);
	extern "C" __declspec(dllexport) int __stdcall get_cgpio_analog(int ionum, fp32 *value);
	extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital(int ionum, int value);
	extern "C" __declspec(dllexport) int __stdcall set_cgpio_analog(int ionum, int value);
	extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital_input_function(int ionum, int fun);
	extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital_output_function(int ionum, int fun);
	extern "C" __declspec(dllexport) int __stdcall get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf);

	extern "C" __declspec(dllexport) int __stdcall get_version(unsigned char version[40]);
	extern "C" __declspec(dllexport) int __stdcall get_robot_sn(unsigned char robot_sn[40]);
	extern "C" __declspec(dllexport) int __stdcall get_state(int *state);
	extern "C" __declspec(dllexport) int __stdcall shutdown_system(int value = 1);
	extern "C" __declspec(dllexport) int __stdcall get_cmdnum(int *cmdnum);
	extern "C" __declspec(dllexport) int __stdcall get_err_warn_code(int err_warn[2]);
	extern "C" __declspec(dllexport) int __stdcall get_position(fp32 pose[6]);
	extern "C" __declspec(dllexport) int __stdcall get_servo_angle(fp32 angles[7]);

	extern "C" __declspec(dllexport) int __stdcall get_suction_cup(int *val);
	extern "C" __declspec(dllexport) int __stdcall set_suction_cup(bool on, bool wait = false, float timeout = 3);
	extern "C" __declspec(dllexport) int __stdcall set_reduced_mode(bool on);
	extern "C" __declspec(dllexport) int __stdcall set_reduced_max_tcp_speed(float speed);
	extern "C" __declspec(dllexport) int __stdcall set_reduced_max_joint_speed(float speed);
	extern "C" __declspec(dllexport) int __stdcall get_reduced_mode(int *mode);
	extern "C" __declspec(dllexport) int __stdcall get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14] = NULL, int *fense_is_on = NULL, int *collision_rebound_is_on = NULL);
	extern "C" __declspec(dllexport) int __stdcall set_reduced_tcp_boundary(int boundary[6]);
	extern "C" __declspec(dllexport) int __stdcall set_reduced_joint_range(float jrange[14]);
	extern "C" __declspec(dllexport) int __stdcall set_fense_mode(bool on);
	extern "C" __declspec(dllexport) int __stdcall set_collision_rebound(bool on);
	extern "C" __declspec(dllexport) int __stdcall set_world_offset(float pose_offset[6]);
	extern "C" __declspec(dllexport) int __stdcall start_record_trajectory(void);
	extern "C" __declspec(dllexport) int __stdcall stop_record_trajectory(char* filename = NULL);
	extern "C" __declspec(dllexport) int __stdcall save_record_trajectory(char* filename, float timeout = 10);
	extern "C" __declspec(dllexport) int __stdcall load_trajectory(char* filename, float timeout = 10);
	extern "C" __declspec(dllexport) int __stdcall playback_trajectory(int times = 1, char* filename = NULL, bool wait = false, int double_speed = 1);
	extern "C" __declspec(dllexport) int __stdcall get_trajectory_rw_status(int *status);
	extern "C" __declspec(dllexport) int __stdcall set_counter_reset(void);
	extern "C" __declspec(dllexport) int __stdcall set_counter_increase(void);
}

