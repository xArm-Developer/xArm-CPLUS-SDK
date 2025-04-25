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
    bool check_is_pause = true,
    int max_callback_thread_count = -1,
    int max_cmdnum = 512,
    int init_axis = 7,
    bool debug = false,
    char* report_type = "rich",
    bool baud_checkset = true);
  extern "C" __declspec(dllexport) int __stdcall remove_instance(int instance_id);
  extern "C" __declspec(dllexport) XArmAPI* __stdcall get_instance(int instance_id);
  extern "C" __declspec(dllexport) int __stdcall connect_robot(char* port="", int instance_id = -1);  // rename from connect to connect_robot, LNK2005 (ws2_32.lib)
  extern "C" __declspec(dllexport) void __stdcall disconnect(int instance_id = -1);
  
  extern "C" __declspec(dllexport) int __stdcall motion_enable(bool enable, int servo_id=8, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_mode(int mode, int detection_param = 0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_state(int state, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall clean_warn(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall clean_error(int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall set_position(fp32 pose[6], fp32 radius = -1, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, bool relative = false, unsigned char motion_type = 0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tool_position(fp32 pose[6], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=0, fp32 radius = -1, unsigned char motion_type = 0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_servo_angle(fp32 angles[7], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, bool relative = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_servo_angle_j(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_servo_cartesian(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, bool is_tool_coord = false, bool is_axis_angle = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall move_gohome(fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, int instance_id = -1);
  extern "C" __declspec(dllexport) void __stdcall reset(bool wait = false, fp32 timeout = NO_TIMEOUT, int instance_id = -1);
  extern "C" __declspec(dllexport) void __stdcall emergency_stop(int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall set_servo_attach(int servo_id, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_servo_detach(int servo_id, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_pause_time(fp32 sltime, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_collision_sensitivity(int sensitivity, bool wait = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_teach_sensitivity(int sensitivity, bool wait = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_gravity_direction(fp32 gravity_dir[3], bool wait = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tcp_offset(fp32 pose_offset[6], bool wait = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tcp_load(fp32 weight, fp32 center_of_gravity[3], bool wait = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tcp_jerk(fp32 jerk, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tcp_maxacc(fp32 acc, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_joint_jerk(fp32 jerk, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_joint_maxacc(fp32 acc, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall clean_conf(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall save_conf(int instance_id = -1);
  
  extern "C" __declspec(dllexport) int __stdcall set_gripper_enable(bool enable, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_gripper_mode(int mode, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_gripper_speed(fp32 speed, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_gripper_position(fp32 pos, bool wait=false, fp32 timeout=10, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_gripper_position(fp32 *pos, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_gripper_err_code(int *err, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall clean_gripper_error(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_tgpio_digital(int *io0_value, int *io1_value, int *io2_value = NULL, int *io3_value = NULL, int *io4_value = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tgpio_digital(int ionum, int value, fp32 delay_sec = 0, bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_tgpio_analog(int ionum, fp32 *value, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_cgpio_digital(int *digitals, int *digitals2 = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_cgpio_analog(int ionum, fp32 *value, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital(int ionum, int value, float delay_sec = 0, bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cgpio_analog(int ionum, float value, bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital_input_function(int ionum, int fun, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital_output_function(int ionum, int fun, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall get_version(unsigned char version[40], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_robot_sn(unsigned char robot_sn[40], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_state(int *state, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall system_control(int value = 1, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall shutdown_system(int value = 1, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_cmdnum(int *cmdnum, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_err_warn_code(int err_warn[2], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_position(fp32 pose[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_servo_angle(fp32 angles[7], bool is_real = false, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall get_inverse_kinematics(fp32 pose[6], fp32 angles[7], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_forward_kinematics(fp32 angles[7], fp32 pose[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall is_joint_limit(fp32 angles[7], int *limit, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall is_tcp_limit(fp32 pose[6], int *limit, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall get_suction_cup(int *val, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_vacuum_gripper(int *val, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_suction_cup(bool on, bool wait = false, float timeout = 3, float delay_sec = 0, bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_vacuum_gripper(bool on, bool wait = false, float timeout = 3, float delay_sec = 0, bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_reduced_mode(bool on, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_reduced_max_tcp_speed(float speed, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_reduced_max_joint_speed(float speed, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_reduced_mode(int *mode, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14] = NULL, int *fense_is_on = NULL, int *collision_rebound_is_on = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_reduced_tcp_boundary(int boundary[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_reduced_joint_range(float jrange[14], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_fense_mode(bool on, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_fence_mode(bool on, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_collision_rebound(bool on, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_world_offset(float pose_offset[6], bool wait = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall start_record_trajectory(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall stop_record_trajectory(char* filename = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall save_record_trajectory(char* filename, float timeout = 10, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall load_trajectory(char* filename, float timeout = 10, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall playback_trajectory(int times = 1, char* filename = NULL, bool wait = false, int double_speed = 1, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_trajectory_rw_status(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_counter_reset(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_counter_increase(int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cgpio_analog_with_xyz(int ionum, float value, float xyz[3], float tol_r, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall set_position_aa(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, unsigned char motion_type = 0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_servo_cartesian_aa(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, bool is_tool_coord = false, bool relative = false, int instance_id = -1);
  
  extern "C" __declspec(dllexport) int __stdcall robotiq_reset(unsigned char ret_data[6] = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall robotiq_set_activate(bool wait = true, fp32 timeout = 3, unsigned char ret_data[6] = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall robotiq_set_position(unsigned char pos, unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall robotiq_open(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall robotiq_close(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers = 3, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_bio_gripper_enable(bool enable, bool wait = true, fp32 timeout = 3, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_bio_gripper_speed(int speed, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_bio_gripper_control_mode(int mode, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_bio_gripper_force(int force, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_bio_gripper_position(int pos, int speed = 0, int force=100, bool wait = true, fp32 timeout = 5, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall open_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall close_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5, bool wait_motion = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_bio_gripper_status(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_bio_gripper_error(int *err, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall clean_bio_gripper_error(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tgpio_modbus_timeout(int timeout, bool is_transparent_transmission = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_tgpio_modbus_baudrate(int baud, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_tgpio_modbus_baudrate(int *baud, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, unsigned char host_id = 9, bool is_transparent_transmission = false, bool use_503_port = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_self_collision_detection(bool on, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_simulation_robot(bool on, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall vc_set_joint_velocity(fp32 speeds[7], bool is_sync = true, fp32 duration = -1.0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord = false, fp32 duration = -1.0, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall set_impedance(int coord, int c_axis[6], float M[6], float K[6], float B[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_impedance_mbk(float M[6], float K[6], float B[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_impedance_config(int coord, int c_axis[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall config_force_control(int coord, int c_axis[6], float f_ref[6], float limits[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall ft_sensor_set_zero(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall ft_sensor_iden_load(float result[10], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall ft_sensor_cali_load(float load[10], bool association_setting_tcp_load = false, float m = 0.270, float x = -17, float y = 9, float z = 11.8, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall ft_sensor_enable(int on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall ft_sensor_app_set(int app_code, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall ft_sensor_app_get(int *app_code, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_sensor_data(float ft_data[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_sensor_config(int *ft_app_status = NULL, int *ft_is_started = NULL, int *ft_type = NULL, int *ft_id = NULL, int *ft_freq = NULL, 
    float *ft_mass = NULL, float *ft_dir_bias = NULL, float ft_centroid[3] = NULL, float ft_zero[6] = NULL, int *imp_coord = NULL, int imp_c_axis[6] = NULL, float M[6] = NULL, float K[6] = NULL, float B[6] = NULL,
    int *f_coord = NULL, int f_c_axis[6] = NULL, float f_ref[6] = NULL, float f_limits[6] = NULL, float kp[6] = NULL, float ki[6] = NULL, float kd[6] = NULL, float xe_limit[6] = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_sensor_error(int *err, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall iden_tcp_load(float result[4], float estimated_mass = 0.0, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall get_linear_track_error(int *err, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_track_status(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_track_pos(int *pos, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_track_is_enabled(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_track_on_zero(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_track_sci(int *sci1, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_track_sco(int sco[2], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall clean_linear_track_error(int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_linear_track_enable(bool enable, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_linear_track_speed(int speed, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_linear_track_back_origin(bool wait = true, bool auto_enable = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_linear_track_pos(int pos, int speed = 0, bool wait = true, fp32 timeout = 100, bool auto_enable = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_linear_track_stop(int instance_id = -1);
  
  extern "C" __declspec(dllexport) int __stdcall set_timeout(float timeout, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_baud_checkset_enable(bool enable, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_checkset_default_baud(int type, int baud, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_checkset_default_baud(int type, int *baud, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cartesian_velo_continuous(bool on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_allow_approx_motion(bool on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_joint_states(fp32 position[7], fp32 velocity[7], fp32 effort[7], int num = 3, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall iden_joint_friction(int *result, unsigned char *sn = NULL, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_only_check_type(unsigned char only_check_type = 0, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall open_lite6_gripper(bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall close_lite6_gripper(bool sync = true, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall stop_lite6_gripper(bool sync = true, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall get_dh_params(fp32 dh_params[28], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_dh_params(fp32 dh_params[28], unsigned char flag = 0, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_feedback_type(unsigned char feedback_type, int instance_id = -1);

  extern "C" __declspec(dllexport) int __stdcall set_linear_spd_limit_factor(float factor, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_cmd_mat_history_num(int num, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_fdb_mat_history_num(int num, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_linear_spd_limit_factor(float *factor, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_cmd_mat_history_num(int *num, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_fdb_mat_history_num(int *num, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_tgpio_modbus_timeout(int *timeout, bool is_transparent_transmission = false, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_poe_status(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_iden_status(int *status, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c31_error_info(int *servo_id, float *theoretical_tau, float *actual_tau, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c54_error_info(int *dir, float *tau_threshold, float *actual_tau, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c37_error_info(int *servo_id, float *diff_angle, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c23_error_info(int *id_bits, float angle[7], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c24_error_info(int *servo_id, float *speed, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c60_error_info(float *max_velo, float *curr_velo, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_c38_error_info(int *id_bits, float angle[7], int instance_id = -1);
  
  extern "C" __declspec(dllexport) int __stdcall set_ft_collision_detection(int on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_ft_collision_rebound(int on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_ft_collision_threshold(float thresholds[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall set_ft_collision_reb_distance(float distances[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_collision_detection(int *on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_collision_rebound(int *on_off, int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_collision_threshold(float thresholds[6], int instance_id = -1);
  extern "C" __declspec(dllexport) int __stdcall get_ft_collision_reb_distance(float distances[6], int instance_id = -1);

  /* modbus tcp func_code: 0x01 */
  extern "C" __declspec(dllexport) int __stdcall read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, int instance_id = -1);
  /* modbus tcp func_code: 0x02 */
  extern "C" __declspec(dllexport) int __stdcall read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, int instance_id = -1);
  /* modbus tcp func_code: 0x03 */
  extern "C" __declspec(dllexport) int __stdcall read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false, int instance_id = -1);
  /* modbus tcp func_code: 0x04 */
  extern "C" __declspec(dllexport) int __stdcall read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false, int instance_id = -1);
  /* modbus tcp func_code: 0x05 */
  extern "C" __declspec(dllexport) int __stdcall write_single_coil_bit(unsigned short addr, unsigned char bit_val, int instance_id = -1);
  /* modbus tcp func_code: 0x06 */
  extern "C" __declspec(dllexport) int __stdcall write_single_holding_register(unsigned short addr, int reg_val, int instance_id = -1);
  /* modbus tcp func_code: 0x0F */
  extern "C" __declspec(dllexport) int __stdcall write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, int instance_id = -1);
  /* modbus tcp func_code: 0x10 */
  extern "C" __declspec(dllexport) int __stdcall write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs, int instance_id = -1);
  /* modbus tcp func_code: 0x16 */
  extern "C" __declspec(dllexport) int __stdcall mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask, int instance_id = -1);
  /* modbus tcp func_code: 0x17 */
  extern "C" __declspec(dllexport) int __stdcall write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed = false, int instance_id = -1);
}
