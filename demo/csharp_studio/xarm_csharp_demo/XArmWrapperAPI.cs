/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace xarm_csharp_demo
{
    class XArmAPIWrapper
    {
        public const float NO_TIMEOUT = -1;
        public int robot_instance_id = -1;

        public XArmAPIWrapper(int instance_id) {
            robot_instance_id = instance_id;
        }

        public XArmAPIWrapper(string ip = "",
            bool is_radian = false,
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
            string report_type = "rich",
            bool baud_checkset = true) {
            robot_instance_id = XArmAPI.create_instance(ip, is_radian, do_not_open, check_tcp_limit, check_joint_limit, 
                check_cmdnum_limit, check_robot_sn, check_is_ready, check_is_pause, max_callback_thread_count, max_cmdnum,
                init_axis, debug, report_type, baud_checkset);
        }

        public int connect(string port = "")
        {
            return XArmAPI.robot_connect(port, robot_instance_id);
        }

        public void disconnect()
        {
            XArmAPI.disconnect(robot_instance_id);
        }

        public int motion_enable(bool enable, int servo_id = 8)
        {
            return XArmAPI.motion_enable(enable, servo_id, robot_instance_id);
        }

        public int set_mode(int mode, int detection_param = 0)
        {
            return XArmAPI.set_mode(mode, detection_param, robot_instance_id);
        }

        public int set_state(int state)
        {
            return XArmAPI.set_state(state, robot_instance_id);
        }

        public int clean_warn()
        {
            return XArmAPI.clean_warn(robot_instance_id);
        }

        public int clean_error()
        {
            return XArmAPI.clean_error(robot_instance_id);
        }

        public int set_position(float[] pose, float radius = -1,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, bool relative = false, byte motion_type = 0)
        {
            return XArmAPI.set_position(pose, radius, speed, acc, mvtime, wait, timeout, relative, motion_type, robot_instance_id);
        }

        public int set_tool_position(float[] pose,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, byte motion_type = 0)
        {
            return XArmAPI.set_tool_position(pose, speed, acc, mvtime, wait, timeout, radius, motion_type, robot_instance_id);
        }

        public int set_servo_angle(float[] angles,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, bool relative = false)
        {
            return XArmAPI.set_servo_angle(angles, speed, acc, mvtime, wait, timeout, radius, relative, robot_instance_id);
        }

        public int set_servo_angle_j(float[] angles,
            float speed = 0, float acc = 0, float mvtime = 0)
        {
            return XArmAPI.set_servo_angle_j(angles, speed, acc, mvtime, robot_instance_id);
        }

        public int set_servo_cartesian(float[] pose,
            float speed = 0, float acc = 0, float mvtime = 0, bool is_tool_coord = false)
        {
            return XArmAPI.set_servo_cartesian(pose, speed, acc, mvtime, is_tool_coord, robot_instance_id);
        }

        public int move_circle(float[] pose1, float[] pose2, float percent,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, bool is_tool_coord = false, bool is_axis_angle = false)
        {
            return XArmAPI.move_circle(pose1, pose2, percent, speed, acc, mvtime, wait, timeout, is_tool_coord, is_axis_angle, robot_instance_id);
        }

        public int move_gohome(
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT)
        {
            return XArmAPI.move_gohome(speed, acc, mvtime, wait, timeout, robot_instance_id);
        }

        public void reset(bool wait = false, float timeout = NO_TIMEOUT)
        {
            XArmAPI.reset(wait, timeout, robot_instance_id);
        }

        public void emergency_stop()
        {
            XArmAPI.emergency_stop(robot_instance_id);
        }

        public int set_servo_attach(int servo_id)
        {
            return XArmAPI.set_servo_attach(servo_id, robot_instance_id);
        }

        public int set_servo_detach(int servo_id)
        {
            return XArmAPI.set_servo_detach(servo_id, robot_instance_id);
        }

        public int set_pause_time(float sltime)
        {
            return XArmAPI.set_pause_time(sltime, robot_instance_id);
        }

        public int set_collision_sensitivity(int sensitivity, bool wait = true)
        {
            return XArmAPI.set_collision_sensitivity(sensitivity, wait, robot_instance_id);
        }

        public int set_teach_sensitivity(int sensitivity, bool wait = true)
        {
            return XArmAPI.set_teach_sensitivity(sensitivity, wait, robot_instance_id);
        }

        public int set_gravity_direction(float[] gravity_dir, bool wait = true)
        {
            return XArmAPI.set_gravity_direction(gravity_dir, wait, robot_instance_id);
        }

        public int set_tcp_offset(float[] pose_offset, bool wait = true)
        {
            return XArmAPI.set_tcp_offset(pose_offset, wait, robot_instance_id);
        }

        public int set_tcp_load(float weight, float[] center_of_gravity, bool wait = false)
        {
            return XArmAPI.set_tcp_load(weight, center_of_gravity, wait, robot_instance_id);
        }

        public int set_tcp_jerk(float jerk)
        {
            return XArmAPI.set_tcp_jerk(jerk, robot_instance_id);
        }

        public int set_tcp_maxacc(float acc)
        {
            return XArmAPI.set_tcp_maxacc(acc, robot_instance_id);
        }

        public int set_joint_jerk(float jerk)
        {
            return XArmAPI.set_joint_jerk(jerk, robot_instance_id);
        }

        public int set_joint_maxacc(float acc)
        {
            return XArmAPI.set_joint_maxacc(acc, robot_instance_id);
        }

        public int clean_conf()
        {
            return XArmAPI.clean_conf(robot_instance_id);
        }

        public int save_conf()
        {
            return XArmAPI.save_conf(robot_instance_id);
        }

        public int set_gripper_enable(bool enable)
        {
            return XArmAPI.set_gripper_enable(enable, robot_instance_id);
        }

        public int set_gripper_mode(int mode)
        {
            return XArmAPI.set_gripper_mode(mode, robot_instance_id);
        }

        public int set_gripper_speed(float speed)
        {
            return XArmAPI.set_gripper_speed(speed, robot_instance_id);
        }

        public int set_gripper_position(float pos, bool wait = false, float timeout = 10, bool wait_motion = true)
        {
            return XArmAPI.set_gripper_position(pos, wait, timeout, wait_motion, robot_instance_id);
        }

        public int get_gripper_position(ref float pos)
        {
            return XArmAPI.get_gripper_position(ref pos, robot_instance_id);
        }

        public int get_gripper_err_code(ref int err)
        {
            return XArmAPI.get_gripper_err_code(ref err, robot_instance_id);
        }

        public int clean_gripper_error()
        {
            return XArmAPI.clean_gripper_error(robot_instance_id);
        }

        public int get_tgpio_digital(ref int io0_value, ref int io1_value, ref int io2_value, ref int io3_value, ref int io4_value)
        {
            return XArmAPI.get_tgpio_digital(ref io0_value, ref io1_value, ref io2_value, ref io3_value, ref io4_value, robot_instance_id);
        }

        public int get_tgpio_digital(ref int io0_value, ref int io1_value)
        {
            int tmp = 0;
            return XArmAPI.get_tgpio_digital(ref io0_value, ref io1_value, ref tmp, ref tmp, ref tmp, robot_instance_id);
        }

        public int set_tgpio_digital(int ionum, int value, float delay_sec = 0, bool sync = true)
        {
            return XArmAPI.set_tgpio_digital(ionum, value, delay_sec, sync, robot_instance_id);
        }

        public int get_tgpio_analog(int ionum, ref float value)
        {
            return XArmAPI.get_tgpio_analog(ionum, ref value, robot_instance_id);
        }

        public int get_cgpio_digital(int[] digitals, int[] digitals2 = null)
        {
            return XArmAPI.get_cgpio_digital(digitals, digitals2, robot_instance_id);
        }

        public int get_cgpio_analog(int ionum, ref float value)
        {
            return XArmAPI.get_cgpio_analog(ionum, ref value, robot_instance_id);
        }

        public int set_cgpio_digital(int ionum, int value, float delay_sec = 0, bool sync = true)
        {
            return XArmAPI.set_cgpio_digital(ionum, value, delay_sec, sync, robot_instance_id);
        }

        public int set_cgpio_analog(int ionum, int value, bool sync = true)
        {
            return XArmAPI.set_cgpio_analog(ionum, value, sync, robot_instance_id);
        }

        public int set_cgpio_digital_input_function(int ionum, int fun)
        {
            return XArmAPI.set_cgpio_digital_input_function(ionum, fun, robot_instance_id);
        }

        public int set_cgpio_digital_output_function(int ionum, int fun)
        {
            return XArmAPI.set_cgpio_digital_output_function(ionum, fun, robot_instance_id);
        }

        public int get_cgpio_state(ref int state, int[] digit_io, float[] analog, int[] input_conf, int[] output_conf)
        {
            return XArmAPI.get_cgpio_state(ref state, digit_io, analog, input_conf, output_conf, robot_instance_id);
        }

        public int get_version(byte[] version)
        {
            return XArmAPI.get_version(version, robot_instance_id);
        }

        public int get_robot_sn(byte[] robot_sn)
        {
            return XArmAPI.get_robot_sn(robot_sn, robot_instance_id);
        }

        public int get_state(ref int state)
        {
            return XArmAPI.get_state(ref state, robot_instance_id);
        }

        public int system_control(int value = 1)
        {
            return XArmAPI.system_control(value, robot_instance_id);
        }

        public int shutdown_system(int value = 1)
        {
            return XArmAPI.shutdown_system(value, robot_instance_id);
        }

        public int get_cmdnum(ref int cmdnum)
        {
            return XArmAPI.get_cmdnum(ref cmdnum, robot_instance_id);
        }

        public int get_err_warn_code(int[] err_warn)
        {
            return XArmAPI.get_err_warn_code(err_warn, robot_instance_id);
        }

        public int get_position(float[] pose)
        {
            return XArmAPI.get_position(pose, robot_instance_id);
        }

        public int get_servo_angle(float[] angles, bool is_real = false)
        {
            return XArmAPI.get_servo_angle(angles, is_real, robot_instance_id);
        }

        public int get_suction_cup(ref int val)
        {
            return XArmAPI.get_suction_cup(ref val, robot_instance_id);
        }

        public int set_suction_cup(bool on, bool wait = false, float timeout = 3, float delay_sec = 0, bool sync = true)
        {
            return XArmAPI.set_suction_cup(on, wait, timeout, delay_sec, sync, robot_instance_id);
        }

        public int set_vacuum_gripper(bool on, bool wait = false, float timeout = 3, float delay_sec = 0, bool sync = true)
        {
            return XArmAPI.set_vacuum_gripper(on, wait, timeout, delay_sec, sync, robot_instance_id);
        }

        public int set_reduced_mode(bool on)
        {
            return XArmAPI.set_reduced_mode(on, robot_instance_id);
        }

        public int set_reduced_max_tcp_speed(float speed)
        {
            return XArmAPI.set_reduced_max_tcp_speed(speed, robot_instance_id);
        }

        public int set_reduced_max_joint_speed(float speed)
        {
            return XArmAPI.set_reduced_max_joint_speed(speed, robot_instance_id);
        }

        public int get_reduced_mode(ref int mode)
        {
            return XArmAPI.get_reduced_mode(ref mode, robot_instance_id);
        }

        public int get_reduced_states(ref int on, int[] xyz_list, ref float tcp_speed, ref float joint_speed, float[] jrange, ref int fense_is_on, ref int collision_rebound_is_on)
        {
            return XArmAPI.get_reduced_states(ref on, xyz_list, ref tcp_speed, ref joint_speed, jrange, ref fense_is_on, ref collision_rebound_is_on, robot_instance_id);
        }

        public int set_reduced_tcp_boundary(int[] boundary)
        {
            return XArmAPI.set_reduced_tcp_boundary(boundary, robot_instance_id);
        }

        public int set_reduced_joint_range(float[] jrange)
        {
            return XArmAPI.set_reduced_joint_range(jrange, robot_instance_id);
        }

        public int set_fense_mode(bool on)
        {
            return XArmAPI.set_fense_mode(on, robot_instance_id);
        }

        public int set_collision_rebound(bool on)
        {
            return XArmAPI.set_collision_rebound(on, robot_instance_id);
        }

        public int set_world_offset(float[] pose_offset, bool wait = true)
        {
            return XArmAPI.set_world_offset(pose_offset, wait, robot_instance_id);
        }

        public int start_record_trajectory()
        {
            return XArmAPI.start_record_trajectory(robot_instance_id);
        }

        public int stop_record_trajectory(string filename)
        {
            return XArmAPI.stop_record_trajectory(filename, robot_instance_id);
        }

        public int save_record_trajectory(string filename, float timeout = 10)
        {
            return XArmAPI.save_record_trajectory(filename, timeout, robot_instance_id);
        }

        public int load_trajectory(string filename, float timeout = 10)
        {
            return XArmAPI.load_trajectory(filename, timeout, robot_instance_id);
        }

        public int playback_trajectory(int times, string filename, bool wait = false, int double_speed = 1)
        {
            return XArmAPI.playback_trajectory(times, filename, wait, double_speed, robot_instance_id);
        }

        public int get_trajectory_rw_status(ref int status)
        {
            return XArmAPI.get_trajectory_rw_status(ref status, robot_instance_id);
        }

        public int set_counter_reset()
        {
            return XArmAPI.set_counter_reset(robot_instance_id);
        }

        public int set_counter_increase()
        {
            return XArmAPI.set_counter_increase(robot_instance_id);
        }

        public int set_tgpio_digital_with_xyz(int ionum, int value, float[] xyz, float tol_r)
        {
            return XArmAPI.set_tgpio_digital_with_xyz(ionum, value, xyz, tol_r, robot_instance_id);
        }

        public int set_cgpio_digital_with_xyz(int ionum, int value, float[] xyz, float tol_r)
        {
            return XArmAPI.set_cgpio_digital_with_xyz(ionum, value, xyz, tol_r, robot_instance_id);
        }

        public int set_cgpio_analog_with_xyz(int ionum, float value, float[] xyz, float tol_r)
        {
            return XArmAPI.set_cgpio_analog_with_xyz(ionum, value, xyz, tol_r, robot_instance_id);
        }

        public int get_inverse_kinematics(float[] pose, float[] angles)
        {
            return XArmAPI.get_inverse_kinematics(pose, angles, robot_instance_id);
        }

        public int get_forward_kinematics(float[] angles, float[] pose)
        {
            return XArmAPI.get_forward_kinematics(angles, pose, robot_instance_id);
        }

        public int is_joint_limit(float[] angles, ref int limit)
        {
            return XArmAPI.is_joint_limit(angles, ref limit, robot_instance_id);
        }

        public int is_tcp_limit(float[] pose, ref int limit)
        {
            return XArmAPI.is_tcp_limit(pose, ref limit, robot_instance_id);
        }

        public int set_position_aa(float[] pose, float speed = 0, float acc = 0, float mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, byte motion_type = 0)
        {
            return XArmAPI.set_position_aa(pose, speed, acc, mvtime, is_tool_coord, relative, wait, timeout, radius, motion_type, robot_instance_id);
        }

        public int set_servo_cartesian_aa(float[] pose, float speed = 0, float acc = 0, bool is_tool_coord = false, bool relative = false)
        {
            return XArmAPI.set_servo_cartesian_aa(pose, speed, acc, is_tool_coord, relative, robot_instance_id);
        }
        
        public int robotiq_reset(byte[] ret_data = null)
        {
            return XArmAPI.robotiq_reset(ret_data, robot_instance_id);
        }

        public int robotiq_set_activate(bool wait = true, float timeout = 3, byte[] ret_data = null)
        {
            return XArmAPI.robotiq_set_activate(wait, timeout, ret_data, robot_instance_id);
        }

        public int robotiq_set_position(byte pos, byte speed = 0xFF, byte force = 0xFF, bool wait = true, float timeout = 5, byte[] ret_data = null, bool wait_motion = true)
        {
            return XArmAPI.robotiq_set_position(pos, speed, force, wait, timeout, ret_data, wait_motion, robot_instance_id);
        }

        public int robotiq_open(byte speed = 0xFF, byte force = 0xFF, bool wait = true, float timeout = 5, byte[] ret_data = null, bool wait_motion = true)
        {
            return XArmAPI.robotiq_open(speed, force, wait, timeout, ret_data, wait_motion, robot_instance_id);
        }

        public int robotiq_close(byte speed = 0xFF, byte force = 0xFF, bool wait = true, float timeout = 5, byte[] ret_data = null, bool wait_motion = true)
        {
            return XArmAPI.robotiq_close(speed, force, wait, timeout, ret_data, wait_motion, robot_instance_id);
        }

        public int robotiq_get_status(byte[] ret_data, byte number_of_registers = 3)
        {
            return XArmAPI.robotiq_get_status(ret_data, number_of_registers, robot_instance_id);
        }

        public int set_bio_gripper_enable(bool enable, bool wait = true, float timeout = 3)
        {
            return XArmAPI.set_bio_gripper_enable(enable, wait, timeout, robot_instance_id);
        }

        public int set_bio_gripper_speed(int speed)
        {
            return XArmAPI.set_bio_gripper_speed(speed, robot_instance_id);
        }

        public int set_bio_gripper_control_mode(int mode)
        {
            return XArmAPI.set_bio_gripper_control_mode(mode, robot_instance_id);
        }

        public int set_bio_gripper_force(int force)
        {
            return XArmAPI.set_bio_gripper_force(force, robot_instance_id);
        }

        public int set_bio_gripper_position(int pos, int speed = 0, int force=100, bool wait = true, float timeout = 5, bool wait_motion = true)
        {
            return XArmAPI.set_bio_gripper_position(pos, speed, force, wait, timeout, wait_motion, robot_instance_id);
        }

        public int open_bio_gripper(int speed = 0, bool wait = true, float timeout = 5, bool wait_motion = true)
        {
            return XArmAPI.open_bio_gripper(speed, wait, timeout, wait_motion, robot_instance_id);
        }

        public int close_bio_gripper(int speed = 0, bool wait = true, float timeout = 5, bool wait_motion = true)
        {
            return XArmAPI.close_bio_gripper(speed, wait, timeout, wait_motion, robot_instance_id);
        }

        public int get_bio_gripper_status(ref int status)
        {
            return XArmAPI.get_bio_gripper_status(ref status, robot_instance_id);
        }

        public int get_bio_gripper_error(ref int err)
        {
            return XArmAPI.get_bio_gripper_error(ref err, robot_instance_id);
        }

        public int clean_bio_gripper_error()
        {
            return XArmAPI.clean_bio_gripper_error(robot_instance_id);
        }

        public int set_tgpio_modbus_timeout(int timeout, bool is_transparent_transmission = false)
        {
            return XArmAPI.set_tgpio_modbus_timeout(timeout, is_transparent_transmission, robot_instance_id);
        }

        public int set_tgpio_modbus_baudrate(int baud)
        {
            return XArmAPI.set_tgpio_modbus_baudrate(baud, robot_instance_id);
        }

        public int get_tgpio_modbus_baudrate(ref int baud)
        {
            return XArmAPI.get_tgpio_modbus_baudrate(ref baud, robot_instance_id);
        }

        public int getset_tgpio_modbus_data(byte[] modbus_data, int modbus_length, byte[] ret_data, int ret_length, byte host_id = 9, bool is_transparent_transmission = false, bool use_503_port = false)
        {
            return XArmAPI.getset_tgpio_modbus_data(modbus_data, modbus_length, ret_data, ret_length, host_id, is_transparent_transmission, use_503_port, robot_instance_id);
        }

        public int set_self_collision_detection(bool on)
        {
            return XArmAPI.set_self_collision_detection(on, robot_instance_id);
        }

        public int set_simulation_robot(bool on)
        {
            return XArmAPI.set_simulation_robot(on, robot_instance_id);
        }

        public int vc_set_joint_velocity(float[] speeds, bool is_sync = true, float duration = -1)
        {
            return XArmAPI.vc_set_joint_velocity(speeds, is_sync, duration, robot_instance_id);
        }

        public int vc_set_cartesian_velocity(float[] speeds, bool is_tool_coord = false, float duration = -1)
        {
            return XArmAPI.vc_set_cartesian_velocity(speeds, is_tool_coord, duration, robot_instance_id);
        }

        public int set_impedance(int coord, int[] c_axis, float[] M, float[] K, float[] B)
        {
            return XArmAPI.set_impedance(coord, c_axis, M, K, B, robot_instance_id);
        }

        public int set_impedance_mbk(float[] M, float[] K, float[] B)
        {
            return XArmAPI.set_impedance_mbk(M, K, B, robot_instance_id);
        }

        public int set_impedance_config(int coord, int[] c_axis)
        {
            return XArmAPI.set_impedance_config(coord, c_axis, robot_instance_id);
        }

        public int config_force_control(int coord, int[] c_axis, float[] f_ref, float[] limits)
        {
            return XArmAPI.config_force_control(coord, c_axis, f_ref, limits, robot_instance_id);
        }

        public int set_force_control_pid(float[] kp, float[] ki, float[] kd, float[] xe_limit)
        {
            return XArmAPI.set_force_control_pid(kp, ki, kd, xe_limit, robot_instance_id);
        }

        public int ft_sensor_set_zero()
        {
            return XArmAPI.ft_sensor_set_zero(robot_instance_id);
        }

        public int ft_sensor_iden_load(float[] result)
        {
            return XArmAPI.ft_sensor_iden_load(result, robot_instance_id);
        }

        public int ft_sensor_cali_load(float[] load, bool association_setting_tcp_load = false, float m = (float)0.270, float x = -17, float y = 9, float z = (float)11.8)
        {
            return XArmAPI.ft_sensor_cali_load(load, association_setting_tcp_load, m, x, y, z, robot_instance_id);
        }

        public int ft_sensor_enable(int on_off)
        {
            return XArmAPI.ft_sensor_enable(on_off, robot_instance_id);
        }

        public int ft_sensor_app_set(int app_code)
        {
            return XArmAPI.ft_sensor_app_set(app_code, robot_instance_id);
        }

        public int ft_sensor_app_get(ref int app_code)
        {
            return XArmAPI.ft_sensor_app_get(ref app_code, robot_instance_id);
        }

        public int get_ft_sensor_data(float[] ft_data)
        {
            return XArmAPI.get_ft_sensor_data(ft_data, robot_instance_id);
        }

        public int get_ft_sensor_config(ref int ft_app_status, ref int ft_is_started, ref int ft_type, ref int ft_id, ref int ft_freq, 
            ref float ft_mass, ref float ft_dir_bias, float[] ft_centroid, float[] ft_zero, ref int imp_coord, int[] imp_c_axis, float[] M, float[] K, float[] B,
            ref int f_coord, int[] f_c_axis, float[] f_ref, float[] f_limits, float[] kp, float[] ki, float[] kd, float[] xe_limit)
        {
            return XArmAPI.get_ft_sensor_config(ref ft_app_status, ref ft_is_started, ref ft_type, ref ft_id, ref ft_freq, ref ft_mass, ref ft_dir_bias, 
                ft_centroid, ft_zero, ref imp_coord, imp_c_axis, M, K, B, ref f_coord, f_c_axis, f_ref, f_limits, kp, ki, kd, xe_limit, robot_instance_id);
        }

        public int get_ft_sensor_error(ref int err)
        {
            return XArmAPI.get_ft_sensor_error(ref err, robot_instance_id);
        }

        public int iden_tcp_load(float[] result, float estimated_mass = 0)
        {
            return XArmAPI.iden_tcp_load(result, estimated_mass, robot_instance_id);
        }

        public int get_linear_track_error(ref int err)
        {
            return XArmAPI.get_linear_track_error(ref err, robot_instance_id);
        }

        public int get_linear_track_status(ref int status)
        {
            return XArmAPI.get_linear_track_status(ref status, robot_instance_id);
        }

        public int get_linear_track_pos(ref int pos)
        {
            return XArmAPI.get_linear_track_pos(ref pos, robot_instance_id);
        }

        public int get_linear_track_is_enabled(ref int status)
        {
            return XArmAPI.get_linear_track_is_enabled(ref status, robot_instance_id);
        }

        public int get_linear_track_on_zero(ref int status)
        {
            return XArmAPI.get_linear_track_on_zero(ref status, robot_instance_id);
        }

        public int get_linear_track_sci(ref int sci1)
        {
            return XArmAPI.get_linear_track_sci(ref sci1, robot_instance_id);
        }

        public int get_linear_track_sco(int[] sco)
        {
            return XArmAPI.get_linear_track_sco(sco, robot_instance_id);
        }

        public int clean_linear_track_error()
        {
            return XArmAPI.clean_linear_track_error(robot_instance_id);
        }

        public int set_linear_track_enable(bool enable)
        {
            return XArmAPI.set_linear_track_enable(enable, robot_instance_id);
        }

        public int set_linear_track_speed(int speed)
        {
            return XArmAPI.set_linear_track_speed(speed, robot_instance_id);
        }

        public int set_linear_track_back_origin(bool wait = true, bool auto_enable = true)
        {
            return XArmAPI.set_linear_track_back_origin(wait, auto_enable, robot_instance_id);
        }

        public int set_linear_track_pos(int pos, int speed = 0, bool wait = true, float timeout = 100, bool auto_enable = true)
        {
            return XArmAPI.set_linear_track_pos(pos, speed, wait, timeout, auto_enable, robot_instance_id);
        }

        public int set_linear_track_stop()
        {
            return XArmAPI.set_linear_track_stop(robot_instance_id);
        }

        public int set_timeout(float timeout)
        {
            return XArmAPI.set_timeout(timeout, robot_instance_id);
        }

        public int set_baud_checkset_enable(bool enable)
        {
            return XArmAPI.set_baud_checkset_enable(enable, robot_instance_id);
        }

        public int set_checkset_default_baud(int type, int baud)
        {
            return XArmAPI.set_checkset_default_baud(type, baud, robot_instance_id);
        }

        public int get_checkset_default_baud(int type, ref int baud)
        {
            return XArmAPI.get_checkset_default_baud(type, ref baud, robot_instance_id);
        }

        public int set_cartesian_velo_continuous(bool on_off)
        {
            return XArmAPI.set_cartesian_velo_continuous(on_off, robot_instance_id);
        }

        public int set_allow_approx_motion(bool on_off)
        {
            return XArmAPI.set_allow_approx_motion(on_off, robot_instance_id);
        }

        public int get_joint_states(float[] position, float[] velocity, float[] effort, int num = 3)
        {
            return XArmAPI.get_joint_states(position, velocity, effort, num, robot_instance_id);
        }

        public int iden_joint_friction(ref int result, byte[] sn)
        {
            return XArmAPI.iden_joint_friction(ref result, sn, robot_instance_id);
        }

        public int set_only_check_type(byte only_check_type)
        {
            return XArmAPI.set_only_check_type(only_check_type, robot_instance_id);
        }

        public int open_lite6_gripper(bool sync = true)
        {
            return XArmAPI.open_lite6_gripper(sync, robot_instance_id);
        }

        public int close_lite6_gripper(bool sync = true)
        {
            return XArmAPI.close_lite6_gripper(sync, robot_instance_id);
        }

        public int stop_lite6_gripper(bool sync = true)
        {
            return XArmAPI.stop_lite6_gripper(sync, robot_instance_id);
        }

        public int get_dh_params(float[] dh_params)
        {
            return XArmAPI.get_dh_params(dh_params, robot_instance_id);
        }

        public int set_dh_params(float[] dh_params, byte flag = 0)
        {
            return XArmAPI.set_dh_params(dh_params, flag, robot_instance_id);
        }

        public int set_feedback_type(byte feedback_type)
        {
            return XArmAPI.set_feedback_type(feedback_type, robot_instance_id);
        }

        public int set_linear_spd_limit_factor(float factor)
        {
            return XArmAPI.set_linear_spd_limit_factor(factor, robot_instance_id);
        }

        public int set_cmd_mat_history_num(int num)
        {
            return XArmAPI.set_cmd_mat_history_num(num, robot_instance_id);
        }

        public int set_fdb_mat_history_num(int num)
        {
            return XArmAPI.set_fdb_mat_history_num(num, robot_instance_id);
        }

        public int get_linear_spd_limit_factor(ref float factor)
        {
            return XArmAPI.get_linear_spd_limit_factor(ref factor, robot_instance_id);
        }

        public int get_cmd_mat_history_num(ref int num)
        {
            return XArmAPI.get_cmd_mat_history_num(ref num, robot_instance_id);
        }

        public int get_fdb_mat_history_num(ref int num)
        {
            return XArmAPI.get_fdb_mat_history_num(ref num, robot_instance_id);
        }

        public int get_tgpio_modbus_timeout(ref int timeout, bool is_transparent_transmission = false)
        {
            return XArmAPI.get_tgpio_modbus_timeout(ref timeout, is_transparent_transmission, robot_instance_id);
        }

        public int get_poe_status(ref int status)
        {
            return XArmAPI.get_poe_status(ref status, robot_instance_id);
        }

        public int get_iden_status(ref int status)
        {
            return XArmAPI.get_iden_status(ref status, robot_instance_id);
        }

        public int get_c31_error_info(ref int servo_id, ref float theoretical_tau, ref float actual_tau)
        {
            return XArmAPI.get_c31_error_info(ref servo_id, ref theoretical_tau, ref actual_tau, robot_instance_id);
        }

        public int get_c54_error_info(ref int dir, ref float tau_threshold, ref float actual_tau)
        {
            return XArmAPI.get_c54_error_info(ref dir, ref tau_threshold, ref actual_tau, robot_instance_id);
        }

        public int get_c37_error_info(ref int servo_id, ref float diff_angle)
        {
            return XArmAPI.get_c37_error_info(ref servo_id, ref diff_angle, robot_instance_id);
        }

        public int get_c23_error_info(ref int id_bits, float[] angles)
        {
            return XArmAPI.get_c23_error_info(ref id_bits, angles, robot_instance_id);
        }

        public int get_c24_error_info(ref int servo_id, ref float speed)
        {
            return XArmAPI.get_c24_error_info(ref servo_id, ref speed, robot_instance_id);
        }

        public int get_c60_error_info(ref float max_velo, ref float curr_velo)
        {
            return XArmAPI.get_c60_error_info(ref max_velo, ref curr_velo, robot_instance_id);
        }

        public int get_c38_error_info(ref int id_bits, float[] angles)
        {
            return XArmAPI.get_c38_error_info(ref id_bits, angles, robot_instance_id);
        }

        public int set_ft_collision_detection(int on_off)
        {
            return XArmAPI.set_ft_collision_detection(on_off, robot_instance_id);
        }

        public int set_ft_collision_rebound(int on_off)
        {
            return XArmAPI.set_ft_collision_rebound(on_off, robot_instance_id);
        }

        public int set_ft_collision_threshold(float[] thresholds)
        {
            return XArmAPI.set_ft_collision_threshold(thresholds, robot_instance_id);
        }

        public int set_ft_collision_reb_distance(float[] distances)
        {
            return XArmAPI.set_ft_collision_reb_distance(distances, robot_instance_id);
        }

        public int get_ft_collision_detection(ref int on_off)
        {
            return XArmAPI.get_ft_collision_detection(ref on_off, robot_instance_id);
        }

        public int get_ft_collision_rebound(ref int on_off)
        {
            return XArmAPI.get_ft_collision_rebound(ref on_off, robot_instance_id);
        }

        public int get_ft_collision_threshold(float[] thresholds)
        {
            return XArmAPI.get_ft_collision_threshold(thresholds, robot_instance_id);
        }

        public int get_ft_collision_reb_distance(float[] distances)
        {
            return XArmAPI.get_ft_collision_reb_distance(distances, robot_instance_id);
        }

        public int read_coil_bits(UInt16 addr, UInt16 quantity, byte[] bits)
        {
            return XArmAPI.read_coil_bits(addr, quantity, bits, robot_instance_id);
        }
        
        public int read_input_bits(UInt16 addr, UInt16 quantity, byte[] bits)
        {
            return XArmAPI.read_input_bits(addr, quantity, bits, robot_instance_id);
        }
        
        public int read_holding_registers(UInt16 addr, UInt16 quantity, int[] regs, bool is_signed = false)
        {
            return XArmAPI.read_holding_registers(addr, quantity, regs, is_signed, robot_instance_id);
        }

        public int read_input_registers(UInt16 addr, UInt16 quantity, int[] regs, bool is_signed = false)
        {
            return XArmAPI.read_input_registers(addr, quantity, regs, is_signed, robot_instance_id);
        }

        public int write_single_coil_bit(UInt16 addr, byte bit_val)
        {
            return XArmAPI.write_single_coil_bit(addr, bit_val, robot_instance_id);
        }

        public int write_single_holding_register(UInt16 addr, int reg_val)
        {
            return XArmAPI.write_single_holding_register(addr, reg_val, robot_instance_id);
        }
        
        public int write_multiple_coil_bits(UInt16 addr, UInt16 quantity, byte[] bits)
        {
            return XArmAPI.write_multiple_coil_bits(addr, quantity, bits, robot_instance_id);
        }
        
        public int write_multiple_holding_registers(UInt16 addr, UInt16 quantity, int[] regs)
        {
            return XArmAPI.write_multiple_holding_registers(addr, quantity, regs, robot_instance_id);
        }

        public int mask_write_holding_register(UInt16 addr, UInt16 and_mask, UInt16 or_mask)
        {
            return XArmAPI.mask_write_holding_register(addr, and_mask, or_mask, robot_instance_id);
        }
        
        public int write_and_read_holding_registers(UInt16 r_addr, UInt16 r_quantity, int[] r_regs, UInt16 w_addr, UInt16 w_quantity, int[] w_regs, bool is_signed = false)
        {
            return XArmAPI.write_and_read_holding_registers(r_addr, r_quantity, r_regs, w_addr, w_quantity, w_regs, is_signed, robot_instance_id);
        }

        public int set_position(float[] pose, float radius = -1,
            bool wait = false, float timeout = NO_TIMEOUT, bool relative = false, byte motion_type = 0)
        {
            return set_position(pose, radius, 0, 0, 0, wait, timeout, relative, motion_type);
        }
        public int set_position(float[] pose, bool wait = false, float timeout = NO_TIMEOUT, bool relative = false, byte motion_type = 0)
        {
            return set_position(pose, -1, 0, 0, 0, wait, timeout, relative, motion_type);
        }
        public int set_tool_position(float[] pose,
            bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, byte motion_type = 0)
        {
            return set_tool_position(pose, 0, 0, 0, wait, timeout, radius, motion_type);
        }
        public int set_servo_angle(float[] angles, bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, bool relative = false)
        {
            return set_servo_angle(angles, 0, 0, 0, wait, timeout, radius, relative);
        }
    }
}
