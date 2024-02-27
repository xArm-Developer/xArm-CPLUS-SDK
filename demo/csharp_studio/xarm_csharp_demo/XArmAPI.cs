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
    class XArmAPI
    {
        public const float NO_TIMEOUT = -1;

        [DllImport("xarm.dll")]
        public static extern int create_instance(
            string port = "",
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
            bool baud_checkset = true);
        [DllImport("xarm.dll")]
        public static extern int remove_instance(int instance_id);
        [DllImport("xarm.dll")]
        public static extern int switch_xarm(int instance_id);
        [DllImport("xarm.dll")]
        public static extern int robot_connect(string port = "");
        [DllImport("xarm.dll")]
        public static extern int disconnect();
        [DllImport("xarm.dll")]
        public static extern int motion_enable(bool enable, int servo_id = 8);
        [DllImport("xarm.dll")]
        public static extern int set_mode(int mode, int detection_param = 0);
        [DllImport("xarm.dll")]
        public static extern int set_state(int state);
        [DllImport("xarm.dll")]
        public static extern int clean_warn();
        [DllImport("xarm.dll")]
        public static extern int clean_error();
        [DllImport("xarm.dll")]
        public static extern int set_position(float[] pose, float radius = -1,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, bool relative = false, byte motion_type = 0);
        [DllImport("xarm.dll")]
        public static extern int set_tool_position(float[] pose,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, byte motion_type = 0);
        [DllImport("xarm.dll")]
        public static extern int set_servo_angle(float[] angles,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, bool relative = false);
        [DllImport("xarm.dll")]
        public static extern int set_servo_angle_j(float[] angles,
            float speed = 0, float acc = 0, float mvtime = 0);
        [DllImport("xarm.dll")]
        public static extern int set_servo_cartesian(float[] pose,
            float speed = 0, float acc = 0, float mvtime = 0, bool is_tool_coord = false);
        [DllImport("xarm.dll")]
        public static extern int move_circle(float[] pose1, float[] pose2, float percent,
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT, bool is_tool_coord = false, bool is_axis_angle = false);
        [DllImport("xarm.dll")]
        public static extern int move_gohome(
            float speed = 0, float acc = 0, float mvtime = 0,
            bool wait = false, float timeout = NO_TIMEOUT);
        [DllImport("xarm.dll")]
        public static extern void reset(bool wait = false, float timeout = NO_TIMEOUT);
        [DllImport("xarm.dll")]
        public static extern void emergency_stop();

        [DllImport("xarm.dll")]
        public static extern int set_servo_attach(int servo_id);
        [DllImport("xarm.dll")]
        public static extern int set_servo_detach(int servo_id);
        [DllImport("xarm.dll")]
        public static extern int set_pause_time(float sltime);
        [DllImport("xarm.dll")]
        public static extern int set_collision_sensitivity(int sensitivity, bool wait = true);
        [DllImport("xarm.dll")]
        public static extern int set_teach_sensitivity(int sensitivity, bool wait = true);
        [DllImport("xarm.dll")]
        public static extern int set_gravity_direction(float[] gravity_dir, bool wait = true);
        [DllImport("xarm.dll")]
        public static extern int set_tcp_offset(float[] pose_offset, bool wait = true);
        [DllImport("xarm.dll")]
        public static extern int set_tcp_load(float weight, float[] center_of_gravity, bool wait = false);
        [DllImport("xarm.dll")]
        public static extern int set_tcp_jerk(float jerk);
        [DllImport("xarm.dll")]
        public static extern int set_tcp_maxacc(float acc);
        [DllImport("xarm.dll")]
        public static extern int set_joint_jerk(float jerk);
        [DllImport("xarm.dll")]
        public static extern int set_joint_maxacc(float acc);
        [DllImport("xarm.dll")]
        public static extern int clean_conf();
        [DllImport("xarm.dll")]
        public static extern int save_conf();

        [DllImport("xarm.dll")]
        public static extern int set_gripper_enable(bool enable);
        [DllImport("xarm.dll")]
        public static extern int set_gripper_mode(int mode);
        [DllImport("xarm.dll")]
        public static extern int set_gripper_speed(float speed);
        [DllImport("xarm.dll")]
        public static extern int set_gripper_position(float pos, bool wait = false, float timeout = 10, bool wait_motion = true);
        [DllImport("xarm.dll")]
        public static extern int get_gripper_position(ref float pos);
        [DllImport("xarm.dll")]
        public static extern int get_gripper_err_code(ref int err);
        [DllImport("xarm.dll")]
        public static extern int clean_gripper_error();
        [DllImport("xarm.dll")]
        public static extern int get_tgpio_digital(ref int io0_value, ref int io1_value);
        [DllImport("xarm.dll")]
        public static extern int set_tgpio_digital(int ionum, int value, float delay_sec = 0);
        [DllImport("xarm.dll")]
        public static extern int get_tgpio_analog(int ionum, ref float value);
        [DllImport("xarm.dll")]
        public static extern int get_cgpio_digital(int[] digitals, int[] digitals2 = null);
        [DllImport("xarm.dll")]
        public static extern int get_cgpio_analog(int ionum, ref float value);
        [DllImport("xarm.dll")]
        public static extern int set_cgpio_digital(int ionum, int value, float delay_sec = 0);
        [DllImport("xarm.dll")]
        public static extern int set_cgpio_analog(int ionum, int value);
        [DllImport("xarm.dll")]
        public static extern int set_cgpio_digital_input_function(int ionum, int fun);
        [DllImport("xarm.dll")]
        public static extern int set_cgpio_digital_output_function(int ionum, int fun);
        [DllImport("xarm.dll")]
        public static extern int get_cgpio_state(ref int state, int[] digit_io, float[] analog, int[] input_conf, int[] output_conf);

        [DllImport("xarm.dll")]
        public static extern int get_version(byte[] version);
        [DllImport("xarm.dll")]
        public static extern int get_robot_sn(byte[] robot_sn);
        [DllImport("xarm.dll")]
        public static extern int get_state(ref int state);
        [DllImport("xarm.dll")]
        public static extern int system_control(int value = 1);
        [DllImport("xarm.dll")]
        public static extern int shutdown_system(int value = 1);
        [DllImport("xarm.dll")]
        public static extern int get_cmdnum(ref int cmdnum);
        [DllImport("xarm.dll")]
        public static extern int get_err_warn_code(int[] err_warn);
        [DllImport("xarm.dll")]
        public static extern int get_position(float[] pose);
        [DllImport("xarm.dll")]
        public static extern int get_servo_angle(float[] angles, bool is_real = false);

        [DllImport("xarm.dll")]
        public static extern int get_suction_cup(ref int val);
        [DllImport("xarm.dll")]
        public static extern int set_suction_cup(bool on, bool wait = false, float timeout = 3, float delay_sec = 0);
        [DllImport("xarm.dll")]
        public static extern int set_vacuum_gripper(bool on, bool wait = false, float timeout = 3, float delay_sec = 0);
        [DllImport("xarm.dll")]
        public static extern int set_reduced_mode(bool on);
        [DllImport("xarm.dll")]
        public static extern int set_reduced_max_tcp_speed(float speed);
        [DllImport("xarm.dll")]
        public static extern int set_reduced_max_joint_speed(float speed);
        [DllImport("xarm.dll")]
        public static extern int get_reduced_mode(ref int mode);
        [DllImport("xarm.dll")]
        public static extern int get_reduced_states(ref int on, int[] xyz_list, ref float tcp_speed, ref float joint_speed, float[] jrange, ref int fense_is_on, ref int collision_rebound_is_on);
        [DllImport("xarm.dll")]
        public static extern int set_reduced_tcp_boundary(int[] boundary);
        [DllImport("xarm.dll")]
        public static extern int set_reduced_joint_range(float[] jrange);
        [DllImport("xarm.dll")]
        public static extern int set_fense_mode(bool on);
        [DllImport("xarm.dll")]
        public static extern int set_collision_rebound(bool on);
        [DllImport("xarm.dll")]
        public static extern int set_world_offset(float[] pose_offset, bool wait = true);
        [DllImport("xarm.dll")]
        public static extern int start_record_trajectory();
        [DllImport("xarm.dll")]
        public static extern int stop_record_trajectory(string filename);
        [DllImport("xarm.dll")]
        public static extern int save_record_trajectory(string filename, float timeout = 10);
        [DllImport("xarm.dll")]
        public static extern int load_trajectory(string filename, float timeout = 10);
        [DllImport("xarm.dll")]
        public static extern int playback_trajectory(int times, string filename, bool wait = false, int double_speed = 1);
        [DllImport("xarm.dll")]
        public static extern int get_trajectory_rw_status(ref int status);
        [DllImport("xarm.dll")]
        public static extern int set_counter_reset();
        [DllImport("xarm.dll")]
        public static extern int set_counter_increase();
        [DllImport("xarm.dll")]
        public static extern int set_tgpio_digital_with_xyz(int ionum, int value, float[] xyz, float tol_r);
        [DllImport("xarm.dll")]
        public static extern int set_cgpio_digital_with_xyz(int ionum, int value, float[] xyz, float tol_r);
        [DllImport("xarm.dll")]
        public static extern int set_cgpio_analog_with_xyz(int ionum, float value, float[] xyz, float tol_r);

        [DllImport("xarm.dll")]
        public static extern int get_inverse_kinematics(float[] pose, float[] angles);
        [DllImport("xarm.dll")]
        public static extern int get_forward_kinematics(float[] angles, float[] pose);
        [DllImport("xarm.dll")]
        public static extern int is_joint_limit(float[] angles, ref int limit);
        [DllImport("xarm.dll")]
        public static extern int is_tcp_limit(float[] pose, ref int limit);
        [DllImport("xarm.dll")]
        public static extern int set_position_aa(float[] pose, float speed = 0, float acc = 0, float mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, byte motion_type = 0);
        [DllImport("xarm.dll")]
        public static extern int set_servo_cartesian_aa(float[] pose, float speed = 0, float acc = 0, bool is_tool_coord = false, bool relative = false);
        
        [DllImport("xarm.dll")]
        public static extern int robotiq_reset(byte[] ret_data = null);
        [DllImport("xarm.dll")]
        public static extern int robotiq_set_activate(bool wait = true, float timeout = 3, byte[] ret_data = null);
        [DllImport("xarm.dll")]
        public static extern int robotiq_set_position(byte pos, byte speed = 0xFF, byte force = 0xFF, bool wait = true, float timeout = 5, byte[] ret_data = null, bool wait_motion = true);
        [DllImport("xarm.dll")]
        public static extern int robotiq_open(byte speed = 0xFF, byte force = 0xFF, bool wait = true, float timeout = 5, byte[] ret_data = null, bool wait_motion = true);
        [DllImport("xarm.dll")]
        public static extern int robotiq_close(byte speed = 0xFF, byte force = 0xFF, bool wait = true, float timeout = 5, byte[] ret_data = null, bool wait_motion = true);
        [DllImport("xarm.dll")]
        public static extern int robotiq_get_status(byte[] ret_data, byte number_of_registers = 3);
        [DllImport("xarm.dll")]
        public static extern int set_bio_gripper_enable(bool enable, bool wait = true, float timeout = 3);
        [DllImport("xarm.dll")]
        public static extern int set_bio_gripper_speed(int speed);
        [DllImport("xarm.dll")]
        public static extern int open_bio_gripper(int speed = 0, bool wait = true, float timeout = 5, bool wait_motion = true);
        [DllImport("xarm.dll")]
        public static extern int close_bio_gripper(int speed = 0, bool wait = true, float timeout = 5, bool wait_motion = true);
        [DllImport("xarm.dll")]
        public static extern int get_bio_gripper_status(ref int status);
        [DllImport("xarm.dll")]
        public static extern int get_bio_gripper_error(ref int err);
        [DllImport("xarm.dll")]
        public static extern int clean_bio_gripper_error();
        [DllImport("xarm.dll")]
        public static extern int set_tgpio_modbus_timeout(int timeout, bool is_transparent_transmission = false);
        [DllImport("xarm.dll")]
        public static extern int set_tgpio_modbus_baudrate(int baud);
        [DllImport("xarm.dll")]
        public static extern int get_tgpio_modbus_baudrate(ref int baud);
        [DllImport("xarm.dll")]
        public static extern int getset_tgpio_modbus_data(byte[] modbus_data, int modbus_length, byte[] ret_data, int ret_length, byte host_id = 9, bool is_transparent_transmission = false, bool use_503_port = false);
        [DllImport("xarm.dll")]
        public static extern int set_self_collision_detection(bool on);
        [DllImport("xarm.dll")]
        public static extern int set_simulation_robot(bool on);
        [DllImport("xarm.dll")]
        public static extern int vc_set_joint_velocity(float[] speeds, bool is_sync = true, float duration = -1);
        [DllImport("xarm.dll")]
        public static extern int vc_set_cartesian_velocity(float[] speeds, bool is_tool_coord = false, float duration = -1);

        [DllImport("xarm.dll")]
        public static extern int set_impedance(int coord, int[] c_axis, float[] M, float[] K, float[] B);
        [DllImport("xarm.dll")]
        public static extern int set_impedance_mbk(float[] M, float[] K, float[] B);
        [DllImport("xarm.dll")]
        public static extern int set_impedance_config(int coord, int[] c_axis);
        [DllImport("xarm.dll")]
        public static extern int config_force_control(int coord, int[] c_axis, float[] f_ref, float[] limits);
        [DllImport("xarm.dll")]
        public static extern int set_force_control_pid(float[] kp, float[] ki, float[] kd, float[] xe_limit);
        [DllImport("xarm.dll")]
        public static extern int ft_sensor_set_zero();
        [DllImport("xarm.dll")]
        public static extern int ft_sensor_iden_load(float[] result);
        [DllImport("xarm.dll")]
        public static extern int ft_sensor_cali_load(float[] load, bool association_setting_tcp_load = false, float m = (float)0.325, float x = -17, float y = 9, float z = (float)11.8);
        [DllImport("xarm.dll")]
        public static extern int ft_sensor_enable(int on_off);
        [DllImport("xarm.dll")]
        public static extern int ft_sensor_app_set(int app_code);
        [DllImport("xarm.dll")]
        public static extern int ft_sensor_app_get(ref int app_code);
        [DllImport("xarm.dll")]
        public static extern int get_ft_sensor_data(float[] ft_data);
        [DllImport("xarm.dll")]
        public static extern int get_ft_sensor_config(ref int ft_app_status, ref int ft_is_started, ref int ft_type, ref int ft_id, ref int ft_freq, 
            ref float ft_mass, ref float ft_dir_bias, float[] ft_centroid, float[] ft_zero, ref int imp_coord, int[] imp_c_axis, float[] M, float[] K, float[] B,
            ref int f_coord, int[] f_c_axis, float[] f_ref, float[] f_limits, float[] kp, float[] ki, float[] kd, float[] xe_limit);
        [DllImport("xarm.dll")]
        public static extern int get_ft_sensor_error(ref int err);

        [DllImport("xarm.dll")]
        public static extern int iden_tcp_load(float[] result, float estimated_mass = 0);

        [DllImport("xarm.dll")]
        public static extern int get_linear_track_error(ref int err);
        [DllImport("xarm.dll")]
        public static extern int get_linear_track_status(ref int status);
        [DllImport("xarm.dll")]
        public static extern int get_linear_track_pos(ref int pos);
        [DllImport("xarm.dll")]
        public static extern int get_linear_track_is_enabled(ref int status);
        [DllImport("xarm.dll")]
        public static extern int get_linear_track_on_zero(ref int status);
        [DllImport("xarm.dll")]
        public static extern int get_linear_track_sci(ref int sci1);
        [DllImport("xarm.dll")]
        public static extern int get_linear_track_sco(int[] sco);
        [DllImport("xarm.dll")]
        public static extern int clean_linear_track_error();
        [DllImport("xarm.dll")]
        public static extern int set_linear_track_enable(bool enable);
        [DllImport("xarm.dll")]
        public static extern int set_linear_track_speed(int speed);
        [DllImport("xarm.dll")]
        public static extern int set_linear_track_back_origin(bool wait = true, bool auto_enable = true);
        [DllImport("xarm.dll")]
        public static extern int set_linear_track_pos(int pos, int speed = 0, bool wait = true, float timeout = 100, bool auto_enable = true);
        [DllImport("xarm.dll")]
        public static extern int set_linear_track_stop();

        [DllImport("xarm.dll")]
        public static extern int set_timeout(float timeout);
        [DllImport("xarm.dll")]
        public static extern int set_baud_checkset_enable(bool enable);
        [DllImport("xarm.dll")]
        public static extern int set_checkset_default_baud(int type, int baud);
        [DllImport("xarm.dll")]
        public static extern int get_checkset_default_baud(int type, ref int baud);

        [DllImport("xarm.dll")]
        public static extern int set_cartesian_velo_continuous(bool on_off);
        [DllImport("xarm.dll")]
        public static extern int set_allow_approx_motion(bool on_off);
        [DllImport("xarm.dll")]
        public static extern int get_joint_states(float[] position, float[] velocity, float[] effort, int num = 3);
        [DllImport("xarm.dll")]
        public static extern int iden_joint_friction(ref int result, byte[] sn);

        [DllImport("xarm.dll")]
        public static extern int set_only_check_type(byte only_check_type);

        [DllImport("xarm.dll")]
        public static extern int get_dh_params(float[] dh_params);

        [DllImport("xarm.dll")]
        public static extern int set_dh_params(float[] dh_params, byte flag = 0);

        [DllImport("xarm.dll")]
        public static extern int set_feedback_type(byte feedback_type);

        [DllImport("xarm.dll")]
        public static extern int set_linear_spd_limit_factor(float factor);

        [DllImport("xarm.dll")]
        public static extern int set_cmd_mat_history_num(int num);

        [DllImport("xarm.dll")]
        public static extern int set_fdb_mat_history_num(int num);

        [DllImport("xarm.dll")]
        public static extern int get_linear_spd_limit_factor(ref float factor);
        
        [DllImport("xarm.dll")]
        public static extern int get_cmd_mat_history_num(ref int num);

        [DllImport("xarm.dll")]
        public static extern int get_fdb_mat_history_num(ref int num);
        
        [DllImport("xarm.dll")]
        public static extern int get_tgpio_modbus_timeout(ref int timeout, bool is_transparent_transmission = false);
        
        [DllImport("xarm.dll")]
        public static extern int get_poe_status(ref int status);

        [DllImport("xarm.dll")]
        public static extern int get_c31_error_info(ref int servo_id, ref float theoretical_tau, ref float actual_tau);
        
        [DllImport("xarm.dll")]
        public static extern int get_c37_error_info(ref int servo_id, ref float diff_angle);

        [DllImport("xarm.dll")]
        public static extern int get_c23_error_info(ref int servo_id, ref float angle);

        [DllImport("xarm.dll")]
        public static extern int get_c24_error_info(ref int servo_id, ref float speed);

        [DllImport("xarm.dll")]
        public static extern int get_c60_error_info(ref float max_velo, ref float curr_velo);

        [DllImport("xarm.dll")]
        public static extern int get_c38_error_info(ref int servo_id, ref float angle);

        /* modbus tcp func_code: 0x01 */
        [DllImport("xarm.dll")]
        public static extern int read_coil_bits(UInt16 addr, UInt16 quantity, byte[] bits);
        
        /* modbus tcp func_code: 0x02 */
        [DllImport("xarm.dll")]
        public static extern int read_input_bits(UInt16 addr, UInt16 quantity, byte[] bits);
        
        /* modbus tcp func_code: 0x03 */
        [DllImport("xarm.dll")]
        public static extern int read_holding_registers(UInt16 addr, UInt16 quantity, int[] regs, bool is_signed = false);
        
        /* modbus tcp func_code: 0x04 */
        [DllImport("xarm.dll")]
        public static extern int read_input_registers(UInt16 addr, UInt16 quantity, int[] regs, bool is_signed = false);
        
        /* modbus tcp func_code: 0x05 */
        [DllImport("xarm.dll")]
        public static extern int write_single_coil_bit(UInt16 addr, byte bit_val);

        /* modbus tcp func_code: 0x06 */
        [DllImport("xarm.dll")]
        public static extern int write_single_holding_register(UInt16 addr, int reg_val);
        
        /* modbus tcp func_code: 0x0F */
        [DllImport("xarm.dll")]
        public static extern int write_multiple_coil_bits(UInt16 addr, UInt16 quantity, byte[] bits);
        
        /* modbus tcp func_code: 0x10 */
        [DllImport("xarm.dll")]
        public static extern int write_multiple_holding_registers(UInt16 addr, UInt16 quantity, int[] regs);
        
        /* modbus tcp func_code: 0x16 */
        [DllImport("xarm.dll")]
        public static extern int mask_write_holding_register(UInt16 addr, UInt16 and_mask, UInt16 or_mask);
        
        /* modbus tcp func_code: 0x17 */
        [DllImport("xarm.dll")]
        public static extern int write_and_read_holding_registers(UInt16 r_addr, UInt16 r_quantity, int[] r_regs, UInt16 w_addr, UInt16 w_quantity, int[] w_regs, bool is_signed = false);

        public static int set_position(float[] pose, float radius = -1,
            bool wait = false, float timeout = NO_TIMEOUT, bool relative = false)
        {
            return set_position(pose, radius, 0, 0, 0, wait, timeout, relative);
        }
        public static int set_position(float[] pose, bool wait = false, float timeout = NO_TIMEOUT, bool relative = false)
        {
            return set_position(pose, -1, 0, 0, 0, wait, timeout, relative);
        }
        public static int set_tool_position(float[] pose,
            bool wait = false, float timeout = NO_TIMEOUT)
        {
            return set_tool_position(pose, 0, 0, 0, wait, timeout);
        }
        public static int set_servo_angle(float[] angles, bool wait = false, float timeout = NO_TIMEOUT, float radius = -1, bool relative = false)
        {
            return set_servo_angle(angles, 0, 0, 0, wait, timeout, radius, relative);
        }
    }
}
