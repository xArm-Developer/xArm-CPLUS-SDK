# xArm-C++-SDK API Documentation (V1.18.1)

## Contents

- [Constructor](#constructor)
- [Properties](#properties)
- [Methods](#methods)
  - [has_err_warn()](#method-has_err_warn)
  - [has_error()](#method-has_error)
  - [has_warn()](#method-has_warn)
  - [is_connected()](#method-is_connected)
  - [is_lite6()](#method-is_lite6)
  - [is_850()](#method-is_850)
  - [is_reported()](#method-is_reported)
  - [connect()](#method-connect)
  - [disconnect()](#method-disconnect)
  - [get_version()](#method-get_version)
  - [get_robot_sn()](#method-get_robot_sn)
  - [get_state()](#method-get_state)
  - [system_control()](#method-system_control)
  - [get_cmdnum()](#method-get_cmdnum)
  - [get_err_warn_code()](#method-get_err_warn_code)
  - [get_position()](#method-get_position)
  - [get_servo_angle()](#method-get_servo_angle)
  - [get_joint_states()](#method-get_joint_states)
  - [motion_enable()](#method-motion_enable)
  - [set_state()](#method-set_state)
  - [set_mode()](#method-set_mode)
  - [set_servo_attach()](#method-set_servo_attach)
  - [set_servo_detach()](#method-set_servo_detach)
  - [clean_error()](#method-clean_error)
  - [clean_warn()](#method-clean_warn)
  - [set_pause_time()](#method-set_pause_time)
  - [set_collision_sensitivity()](#method-set_collision_sensitivity)
  - [set_teach_sensitivity()](#method-set_teach_sensitivity)
  - [set_gravity_direction()](#method-set_gravity_direction)
  - [clean_conf()](#method-clean_conf)
  - [save_conf()](#method-save_conf)
  - [set_position()](#method-set_position)
  - [set_tool_position()](#method-set_tool_position)
  - [set_servo_angle()](#method-set_servo_angle)
  - [set_servo_angle_j()](#method-set_servo_angle_j)
  - [set_servo_cartesian()](#method-set_servo_cartesian)
  - [move_circle()](#method-move_circle)
  - [move_gohome()](#method-move_gohome)
  - [reset()](#method-reset)
  - [emergency_stop()](#method-emergency_stop)
  - [set_tcp_offset()](#method-set_tcp_offset)
  - [set_tcp_load()](#method-set_tcp_load)
  - [set_tcp_jerk()](#method-set_tcp_jerk)
  - [set_tcp_maxacc()](#method-set_tcp_maxacc)
  - [set_joint_jerk()](#method-set_joint_jerk)
  - [set_joint_maxacc()](#method-set_joint_maxacc)
  - [get_inverse_kinematics()](#method-get_inverse_kinematics)
  - [get_forward_kinematics()](#method-get_forward_kinematics)
  - [is_tcp_limit()](#method-is_tcp_limit)
  - [is_joint_limit()](#method-is_joint_limit)
  - [set_gripper_enable()](#method-set_gripper_enable)
  - [set_gripper_mode()](#method-set_gripper_mode)
  - [get_gripper_position()](#method-get_gripper_position)
  - [get_gripper_g2_position()](#method-get_gripper_g2_position)
  - [set_gripper_position()](#method-set_gripper_position)
  - [set_gripper_g2_position()](#method-set_gripper_g2_position)
  - [set_gripper_speed()](#method-set_gripper_speed)
  - [get_gripper_status()](#method-get_gripper_status)
  - [get_gripper_err_code()](#method-get_gripper_err_code)
  - [clean_gripper_error()](#method-clean_gripper_error)
  - [get_tgpio_digital()](#method-get_tgpio_digital)
  - [set_tgpio_digital()](#method-set_tgpio_digital)
  - [get_tgpio_analog()](#method-get_tgpio_analog)
  - [get_cgpio_digital()](#method-get_cgpio_digital)
  - [get_cgpio_analog()](#method-get_cgpio_analog)
  - [set_cgpio_digital()](#method-set_cgpio_digital)
  - [set_cgpio_analog()](#method-set_cgpio_analog)
  - [set_cgpio_digital_input_function()](#method-set_cgpio_digital_input_function)
  - [set_cgpio_digital_output_function()](#method-set_cgpio_digital_output_function)
  - [get_cgpio_state()](#method-get_cgpio_state)
  - [get_vacuum_gripper()](#method-get_vacuum_gripper)
  - [set_vacuum_gripper()](#method-set_vacuum_gripper)
  - [get_gripper_version()](#method-get_gripper_version)
  - [get_servo_version()](#method-get_servo_version)
  - [get_tgpio_version()](#method-get_tgpio_version)
  - [reload_dynamics()](#method-reload_dynamics)
  - [set_reduced_mode()](#method-set_reduced_mode)
  - [set_reduced_max_tcp_speed()](#method-set_reduced_max_tcp_speed)
  - [set_reduced_max_joint_speed()](#method-set_reduced_max_joint_speed)
  - [get_reduced_mode()](#method-get_reduced_mode)
  - [get_reduced_states()](#method-get_reduced_states)
  - [set_reduced_tcp_boundary()](#method-set_reduced_tcp_boundary)
  - [set_reduced_joint_range()](#method-set_reduced_joint_range)
  - [set_fence_mode()](#method-set_fence_mode)
  - [set_collision_rebound()](#method-set_collision_rebound)
  - [set_world_offset()](#method-set_world_offset)
  - [start_record_trajectory()](#method-start_record_trajectory)
  - [stop_record_trajectory()](#method-stop_record_trajectory)
  - [save_record_trajectory()](#method-save_record_trajectory)
  - [load_trajectory()](#method-load_trajectory)
  - [playback_trajectory()](#method-playback_trajectory)
  - [get_trajectory_rw_status()](#method-get_trajectory_rw_status)
  - [set_counter_reset()](#method-set_counter_reset)
  - [set_counter_increase()](#method-set_counter_increase)
  - [set_tgpio_digital_with_xyz()](#method-set_tgpio_digital_with_xyz)
  - [set_cgpio_digital_with_xyz()](#method-set_cgpio_digital_with_xyz)
  - [set_cgpio_analog_with_xyz()](#method-set_cgpio_analog_with_xyz)
  - [config_tgpio_reset_when_stop()](#method-config_tgpio_reset_when_stop)
  - [config_cgpio_reset_when_stop()](#method-config_cgpio_reset_when_stop)
  - [set_position_aa()](#method-set_position_aa)
  - [set_servo_cartesian_aa()](#method-set_servo_cartesian_aa)
  - [get_pose_offset()](#method-get_pose_offset)
  - [get_position_aa()](#method-get_position_aa)
  - [robotiq_reset()](#method-robotiq_reset)
  - [robotiq_set_activate()](#method-robotiq_set_activate)
  - [robotiq_set_position()](#method-robotiq_set_position)
  - [robotiq_open()](#method-robotiq_open)
  - [robotiq_close()](#method-robotiq_close)
  - [robotiq_get_status()](#method-robotiq_get_status)
  - [set_bio_gripper_enable()](#method-set_bio_gripper_enable)
  - [set_bio_gripper_speed()](#method-set_bio_gripper_speed)
  - [set_bio_gripper_control_mode()](#method-set_bio_gripper_control_mode)
  - [set_bio_gripper_force()](#method-set_bio_gripper_force)
  - [set_bio_gripper_g2_position()](#method-set_bio_gripper_g2_position)
  - [open_bio_gripper()](#method-open_bio_gripper)
  - [close_bio_gripper()](#method-close_bio_gripper)
  - [get_bio_gripper_status()](#method-get_bio_gripper_status)
  - [get_bio_gripper_g2_position()](#method-get_bio_gripper_g2_position)
  - [get_bio_gripper_error()](#method-get_bio_gripper_error)
  - [clean_bio_gripper_error()](#method-clean_bio_gripper_error)
  - [set_rs485_timeout()](#method-set_rs485_timeout)
  - [get_rs485_timeout()](#method-get_rs485_timeout)
  - [set_rs485_baudrate()](#method-set_rs485_baudrate)
  - [get_rs485_baudrate()](#method-get_rs485_baudrate)
  - [set_rs485_data()](#method-set_rs485_data)
  - [getset_tgpio_modbus_data()](#method-getset_tgpio_modbus_data)
  - [set_report_tau_or_i()](#method-set_report_tau_or_i)
  - [get_report_tau_or_i()](#method-get_report_tau_or_i)
  - [set_self_collision_detection()](#method-set_self_collision_detection)
  - [set_collision_tool_model()](#method-set_collision_tool_model)
  - [set_simulation_robot()](#method-set_simulation_robot)
  - [vc_set_joint_velocity()](#method-vc_set_joint_velocity)
  - [vc_set_cartesian_velocity()](#method-vc_set_cartesian_velocity)
  - [calibrate_tcp_coordinate_offset()](#method-calibrate_tcp_coordinate_offset)
  - [calibrate_tcp_orientation_offset()](#method-calibrate_tcp_orientation_offset)
  - [calibrate_user_orientation_offset()](#method-calibrate_user_orientation_offset)
  - [calibrate_user_coordinate_offset()](#method-calibrate_user_coordinate_offset)
  - [set_ft_sensor_admittance_parameters()](#method-set_ft_sensor_admittance_parameters)
  - [set_ft_sensor_force_parameters()](#method-set_ft_sensor_force_parameters)
  - [set_ft_sensor_zero()](#method-set_ft_sensor_zero)
  - [iden_ft_sensor_load_offset()](#method-iden_ft_sensor_load_offset)
  - [set_ft_sensor_load_offset()](#method-set_ft_sensor_load_offset)
  - [set_ft_sensor_enable()](#method-set_ft_sensor_enable)
  - [set_ft_sensor_mode()](#method-set_ft_sensor_mode)
  - [get_ft_sensor_mode()](#method-get_ft_sensor_mode)
  - [get_ft_sensor_data()](#method-get_ft_sensor_data)
  - [get_ft_sensor_config()](#method-get_ft_sensor_config)
  - [get_ft_sensor_error()](#method-get_ft_sensor_error)
  - [iden_tcp_load()](#method-iden_tcp_load)
  - [get_linear_motor_registers()](#method-get_linear_motor_registers)
  - [get_linear_motor_pos()](#method-get_linear_motor_pos)
  - [get_linear_motor_status()](#method-get_linear_motor_status)
  - [get_linear_motor_error()](#method-get_linear_motor_error)
  - [get_linear_motor_is_enabled()](#method-get_linear_motor_is_enabled)
  - [get_linear_motor_on_zero()](#method-get_linear_motor_on_zero)
  - [get_linear_motor_sci()](#method-get_linear_motor_sci)
  - [get_linear_motor_sco()](#method-get_linear_motor_sco)
  - [clean_linear_motor_error()](#method-clean_linear_motor_error)
  - [set_linear_motor_enable()](#method-set_linear_motor_enable)
  - [set_linear_motor_speed()](#method-set_linear_motor_speed)
  - [set_linear_motor_back_origin()](#method-set_linear_motor_back_origin)
  - [set_linear_motor_pos()](#method-set_linear_motor_pos)
  - [set_linear_motor_stop()](#method-set_linear_motor_stop)
  - [set_baud_checkset_enable()](#method-set_baud_checkset_enable)
  - [set_checkset_default_baud()](#method-set_checkset_default_baud)
  - [get_checkset_default_baud()](#method-get_checkset_default_baud)
  - [set_cartesian_velo_continuous()](#method-set_cartesian_velo_continuous)
  - [set_allow_approx_motion()](#method-set_allow_approx_motion)
  - [iden_joint_friction()](#method-iden_joint_friction)
  - [set_only_check_type()](#method-set_only_check_type)
  - [open_lite6_gripper()](#method-open_lite6_gripper)
  - [close_lite6_gripper()](#method-close_lite6_gripper)
  - [stop_lite6_gripper()](#method-stop_lite6_gripper)
  - [get_dh_params()](#method-get_dh_params)
  - [set_dh_params()](#method-set_dh_params)
  - [set_feedback_type()](#method-set_feedback_type)
  - [set_linear_spd_limit_factor()](#method-set_linear_spd_limit_factor)
  - [set_cmd_mat_history_num()](#method-set_cmd_mat_history_num)
  - [set_fdb_mat_history_num()](#method-set_fdb_mat_history_num)
  - [get_linear_spd_limit_factor()](#method-get_linear_spd_limit_factor)
  - [get_cmd_mat_history_num()](#method-get_cmd_mat_history_num)
  - [get_fdb_mat_history_num()](#method-get_fdb_mat_history_num)
  - [get_poe_status()](#method-get_poe_status)
  - [get_iden_status()](#method-get_iden_status)
  - [get_c31_error_info()](#method-get_c31_error_info)
  - [get_c54_error_info()](#method-get_c54_error_info)
  - [get_c37_error_info()](#method-get_c37_error_info)
  - [get_c23_error_info()](#method-get_c23_error_info)
  - [get_c24_error_info()](#method-get_c24_error_info)
  - [get_c60_error_info()](#method-get_c60_error_info)
  - [get_c38_error_info()](#method-get_c38_error_info)
  - [set_ft_collision_detection()](#method-set_ft_collision_detection)
  - [set_ft_collision_rebound()](#method-set_ft_collision_rebound)
  - [set_ft_collision_threshold()](#method-set_ft_collision_threshold)
  - [set_ft_collision_reb_distance()](#method-set_ft_collision_reb_distance)
  - [set_ft_admittance_ctrl_threshold()](#method-set_ft_admittance_ctrl_threshold)
  - [set_external_device_monitor_params()](#method-set_external_device_monitor_params)
  - [set_tgpio_monitor_params()](#method-set_tgpio_monitor_params)
  - [get_ft_collision_detection()](#method-get_ft_collision_detection)
  - [get_ft_collision_rebound()](#method-get_ft_collision_rebound)
  - [get_ft_collision_threshold()](#method-get_ft_collision_threshold)
  - [get_ft_collision_reb_distance()](#method-get_ft_collision_reb_distance)
  - [get_ft_admittance_ctrl_threshold()](#method-get_ft_admittance_ctrl_threshold)
  - [get_external_device_monitor_params()](#method-get_external_device_monitor_params)
  - [get_tgpio_monitor_params()](#method-get_tgpio_monitor_params)
  - [read_coil_bits()](#method-read_coil_bits)
  - [read_input_bits()](#method-read_input_bits)
  - [read_holding_registers()](#method-read_holding_registers)
  - [read_input_registers()](#method-read_input_registers)
  - [write_single_coil_bit()](#method-write_single_coil_bit)
  - [write_single_holding_register()](#method-write_single_holding_register)
  - [write_multiple_coil_bits()](#method-write_multiple_coil_bits)
  - [write_multiple_holding_registers()](#method-write_multiple_holding_registers)
  - [mask_write_holding_register()](#method-mask_write_holding_register)
  - [write_and_read_holding_registers()](#method-write_and_read_holding_registers)


## Constructor
<a id="constructor"></a>

```cpp
XArmAPI(const std::string &port = "", bool is_radian = DEFAULT_IS_RADIAN, bool do_not_open = false, bool check_tcp_limit = true, bool check_joint_limit = true, bool check_cmdnum_limit = true, bool check_robot_sn = false, bool check_is_ready = true, bool check_is_pause = true, int max_callback_thread_count = -1, int max_cmdnum = 512, int init_axis = 7, bool debug = false, std::string report_type = "rich", bool baud_checkset = true);
```

| Parameter | Description |
|-----------|-------------|
| `port` | ip-address(such as "192.168.1.185")<br> Note: this parameter is required if `do_not_open` is false |
| `is_radian` | set the default unit is radians or not, default is false |
| `do_not_open` | do not open, default is false. if true, call `connect()` manually later |
| `check_tcp_limit` | reserved, whether to check tcp limit, default is true |
| `check_joint_limit` | reserved, whether to check joint limit, default is true |
| `check_cmdnum_limit` | whether to check command num limit, default is true |
| `check_robot_sn` | whether to check robot sn, default is false |
| `check_is_ready` | whether to check robot ready state before motion, default is true<br>Note: only available if firmware version `< 1.5.20` |
| `check_is_pause` | whether to check robot pause state, default is true |
| `max_callback_thread_count` | max callback thread count, default is `-1`<br> greater than 0: maximum number of callback worker threads<br> equal to 0: callbacks are not dispatched by worker thread<br> less than 0: no limit on callback worker threads |
| `max_cmdnum` | max command cache threshold, default is 512<br>Note: only available in the param `check_cmdnum_limit` is true |

## Properties
<a id="properties"></a>

| Name | Type | Description |
|------|------|-------------|
| [state](#property-state) | `int` | state |
| [mode](#property-mode) | `int` | mode |
| [cmd_num](#property-cmd_num) | `int` | cmd cache count |
| [joints_torque](#property-joints_torque) | `fp32[7]` | joints torque, fp32[7]{servo-1, ..., servo-7} |
| [motor_brake_states](#property-motor_brake_states) | `bool[8]` | motor brake states, bool[8]{servo-1, ..., servo-7, reserved} |
| [motor_enable_states](#property-motor_enable_states) | `bool[8]` | motor enable states, bool[8]{servo-1, ..., servo-7, reserved} |
| [error_code](#property-error_code) | `int` | error code |
| [warn_code](#property-warn_code) | `int` | warn code |
| [tcp_load](#property-tcp_load) | `fp32[4]` | tcp load, fp32[4]{weight, x, y, z} |
| [collision_sensitivity](#property-collision_sensitivity) | `int` | collision sensitivity |
| [teach_sensitivity](#property-teach_sensitivity) | `int` | teach sensitivity |
| [device_type](#property-device_type) | `int` | device type |
| [axis](#property-axis) | `int` | robot axis |
| [master_id](#property-master_id) | `int` | - |
| [slave_id](#property-slave_id) | `int` | - |
| [motor_tid](#property-motor_tid) | `int` | - |
| [motor_fid](#property-motor_fid) | `int` | - |
| [version](#property-version) | `unsigned char[30]` | version |
| [sn](#property-sn) | `unsigned char[40]` | sn |
| [version_number](#property-version_number) | `int[3]` | version number |
| [tcp_jerk](#property-tcp_jerk) | `fp32` | tcp jerk |
| [joint_jerk](#property-joint_jerk) | `fp32` | joint jerk |
| [rot_jerk](#property-rot_jerk) | `fp32` | rot jerk |
| [max_rot_acc](#property-max_rot_acc) | `fp32` | max rot acc |
| [tcp_speed_limit](#property-tcp_speed_limit) | `fp32[2]` | fp32[2]{min, max} |
| [tcp_acc_limit](#property-tcp_acc_limit) | `fp32[2]` | fp32[2]{min, max} |
| [last_used_tcp_speed](#property-last_used_tcp_speed) | `fp32` | - |
| [last_used_tcp_acc](#property-last_used_tcp_acc) | `fp32` | - |
| [angles](#property-angles) | `fp32[7]` | fp32[7]{servo-1, ..., servo-7} |
| [last_used_angles](#property-last_used_angles) | `fp32[7]` | fp32[7]{servo-1, ..., servo-7} |
| [joint_speed_limit](#property-joint_speed_limit) | `fp32[2]` | fp32[2]{min, max} |
| [joint_acc_limit](#property-joint_acc_limit) | `fp32[2]` | fp32[2]{min, max} |
| [last_used_joint_speed](#property-last_used_joint_speed) | `fp32` | - |
| [last_used_joint_acc](#property-last_used_joint_acc) | `fp32` | - |
| [position](#property-position) | `fp32[6]` | fp32[6]{x, y, z, roll, pitch, yaw} |
| [position_aa](#property-position_aa) | `fp32[6]` | fp32[6]{x, y, z, rx, ry, rz} |
| [last_used_position](#property-last_used_position) | `fp32[6]` | fp32[6]{x, y, z, roll, pitch, yaw} |
| [tcp_offset](#property-tcp_offset) | `fp32[6]` | fp32[6]{x, y, z, roll, pitch, yaw} |
| [gravity_direction](#property-gravity_direction) | `fp32[3]` | fp32[3]{x_direction, y_direction, z_direction} |
| [realtime_tcp_speed](#property-realtime_tcp_speed) | `fp32` | - |
| [realtime_joint_speeds](#property-realtime_joint_speeds) | `fp32[7]` | - |
| [reduced_tcp_boundary](#property-reduced_tcp_boundary) | `int[6]` | - |
| [reduced_max_tcp_speed](#property-reduced_max_tcp_speed) | `fp32` | - |
| [reduced_max_joint_spped](#property-reduced_max_joint_spped) | `fp32` | - |
| [reduced_joint_limits](#property-reduced_joint_limits) | `fp32[14]` | - |
| [is_reduced_mode](#property-is_reduced_mode) | `bool` | - |
| [is_fence_mode](#property-is_fence_mode) | `bool` | - |
| [is_report_current](#property-is_report_current) | `bool` | - |
| [is_approx_motion](#property-is_approx_motion) | `bool` | - |
| [is_cart_continuous](#property-is_cart_continuous) | `bool` | - |
| [is_collision_rebound](#property-is_collision_rebound) | `bool` | - |
| [ft_sensor_is_enable](#property-ft_sensor_is_enable) | `bool` | - |
| [cgpio_alarm_code](#property-cgpio_alarm_code) | `int` | - |
| [monitor_device_type](#property-monitor_device_type) | `int` | - |
| [monitor_device_state](#property-monitor_device_state) | `int` | - |
| [monitor_device_pos](#property-monitor_device_pos) | `int` | - |
| [monitor_device_speed](#property-monitor_device_speed) | `int` | - |
| [monitor_device_current](#property-monitor_device_current) | `int` | - |
| [world_offset](#property-world_offset) | `fp32[6]` | fp32[6]{x, y, z, roll, pitch, yaw} |
| [temperatures](#property-temperatures) | `fp32[7]` | - |
| [count](#property-count) | `int` | - |
| [iden_progress](#property-iden_progress) | `int` | - |
| [gpio_reset_config](#property-gpio_reset_config) | `unsigned char[2]` | unsigned char[2]{cgpio_reset_enable, tgpio_reset_enable} |
| [ft_ext_force](#property-ft_ext_force) | `fp32[6]` | - |
| [ft_raw_force](#property-ft_raw_force) | `fp32[6]` | - |
| [default_is_radian](#property-default_is_radian) | `bool` | - |
| [core](#property-core) | `std::shared_ptr<UxbusCmd>` | - |
| [robotiq_status](#property-robotiq_status) | `struct RobotIqStatus` | - |
| [linear_motor_status](#property-linear_motor_status) | `LinearMotorStatus` | - |
| [voltages](#property-voltages) | `fp32[7]` | fp32[7]{servo-1, ..., servo-7} |
| [currents](#property-currents) | `fp32[7]` | fp32[7]{servo-1, ..., servo-7} |
| [is_simulation_robot](#property-is_simulation_robot) | `int` | 0: off, 1: on |
| [is_collision_detection](#property-is_collision_detection) | `int` | 0: off, 1: on |
| [collision_tool_type](#property-collision_tool_type) | `int` | - |
| [collision_model_params](#property-collision_model_params) | `fp32[6]` | fp32[6]{...} |
| [cgpio_state](#property-cgpio_state) | `int` | - |
| [cgpio_code](#property-cgpio_code) | `int` | - |
| [cgpio_input_digitals](#property-cgpio_input_digitals) | `int[2]` | int[2]{ digital-input-functional-gpio-state, digital-input-configuring-gpio-state } |
| [cgpio_output_digitals](#property-cgpio_output_digitals) | `int[2]` | int[2]{ digital-output-functional-gpio-state, digital-output-configuring-gpio-state } |
| [cgpio_input_analogs](#property-cgpio_input_analogs) | `fp32[2]` | fp32[2] {analog-1-input-value, analog-2-input-value} |
| [cgpio_output_analogs](#property-cgpio_output_analogs) | `fp32[2]` | fp32[2] {analog-1-output-value, analog-2-output-value} |
| [cgpio_input_conf](#property-cgpio_input_conf) | `int[16]` | int[16]{ CI0-conf, ... CI7-conf } |
| [cgpio_output_conf](#property-cgpio_output_conf) | `int[16]` | int[16]{ CO0-conf, ... CO7-conf } |
| [only_check_result](#property-only_check_result) | `unsigned char` | - |

<a id="property-state"></a>
<a id="property-mode"></a>
<a id="property-cmd_num"></a>
<a id="property-joints_torque"></a>
<a id="property-motor_brake_states"></a>
<a id="property-motor_enable_states"></a>
<a id="property-error_code"></a>
<a id="property-warn_code"></a>
<a id="property-tcp_load"></a>
<a id="property-collision_sensitivity"></a>
<a id="property-teach_sensitivity"></a>
<a id="property-device_type"></a>
<a id="property-axis"></a>
<a id="property-master_id"></a>
<a id="property-slave_id"></a>
<a id="property-motor_tid"></a>
<a id="property-motor_fid"></a>
<a id="property-version"></a>
<a id="property-sn"></a>
<a id="property-version_number"></a>
<a id="property-tcp_jerk"></a>
<a id="property-joint_jerk"></a>
<a id="property-rot_jerk"></a>
<a id="property-max_rot_acc"></a>
<a id="property-tcp_speed_limit"></a>
<a id="property-tcp_acc_limit"></a>
<a id="property-last_used_tcp_speed"></a>
<a id="property-last_used_tcp_acc"></a>
<a id="property-angles"></a>
<a id="property-last_used_angles"></a>
<a id="property-joint_speed_limit"></a>
<a id="property-joint_acc_limit"></a>
<a id="property-last_used_joint_speed"></a>
<a id="property-last_used_joint_acc"></a>
<a id="property-position"></a>
<a id="property-position_aa"></a>
<a id="property-last_used_position"></a>
<a id="property-tcp_offset"></a>
<a id="property-gravity_direction"></a>
<a id="property-realtime_tcp_speed"></a>
<a id="property-realtime_joint_speeds"></a>
<a id="property-reduced_tcp_boundary"></a>
<a id="property-reduced_max_tcp_speed"></a>
<a id="property-reduced_max_joint_spped"></a>
<a id="property-reduced_joint_limits"></a>
<a id="property-is_reduced_mode"></a>
<a id="property-is_fence_mode"></a>
<a id="property-is_report_current"></a>
<a id="property-is_approx_motion"></a>
<a id="property-is_cart_continuous"></a>
<a id="property-is_collision_rebound"></a>
<a id="property-ft_sensor_is_enable"></a>
<a id="property-cgpio_alarm_code"></a>
<a id="property-monitor_device_type"></a>
<a id="property-monitor_device_state"></a>
<a id="property-monitor_device_pos"></a>
<a id="property-monitor_device_speed"></a>
<a id="property-monitor_device_current"></a>
<a id="property-world_offset"></a>
<a id="property-temperatures"></a>
<a id="property-count"></a>
<a id="property-iden_progress"></a>
<a id="property-gpio_reset_config"></a>
<a id="property-ft_ext_force"></a>
<a id="property-ft_raw_force"></a>
<a id="property-default_is_radian"></a>
<a id="property-core"></a>
<a id="property-robotiq_status"></a>
<a id="property-linear_motor_status"></a>
<a id="property-voltages"></a>
<a id="property-currents"></a>
<a id="property-is_simulation_robot"></a>
<a id="property-is_collision_detection"></a>
<a id="property-collision_tool_type"></a>
<a id="property-collision_model_params"></a>
<a id="property-cgpio_state"></a>
<a id="property-cgpio_code"></a>
<a id="property-cgpio_input_digitals"></a>
<a id="property-cgpio_output_digitals"></a>
<a id="property-cgpio_input_analogs"></a>
<a id="property-cgpio_output_analogs"></a>
<a id="property-cgpio_input_conf"></a>
<a id="property-cgpio_output_conf"></a>
<a id="property-only_check_result"></a>

## Methods
<a id="methods"></a>

### has_err_warn()
<a id="method-has_err_warn"></a>

```cpp
bool has_err_warn(void);
```

  > xArm has error/warn or not, only available in socket way
  > @param port: ip-address(such as "192.168.1.185")
  > &ensp;&ensp;&ensp;&ensp; Note: this parameter is required if `do_not_open` is false
  > @param is_radian: set the default unit is radians or not, default is false
  > @param do_not_open: do not open, default is false. if true, call `connect()` manually later
  > @param check_tcp_limit: reserved, whether to check tcp limit, default is true
  > @param check_joint_limit: reserved, whether to check joint limit, default is true
  > @param check_cmdnum_limit: whether to check command num limit, default is true
  > @param check_robot_sn: whether to check robot sn, default is false
  > @param check_is_ready: whether to check robot ready state before motion, default is true
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware version `< 1.5.20`
  > @param check_is_pause: whether to check robot pause state, default is true
  > @param max_callback_thread_count: max callback thread count, default is `-1`
  > &ensp;&ensp;&ensp;&ensp; greater than 0: maximum number of callback worker threads
  > &ensp;&ensp;&ensp;&ensp; equal to 0: callbacks are not dispatched by worker thread
  > &ensp;&ensp;&ensp;&ensp; less than 0: no limit on callback worker threads
  > @param max_cmdnum: max command cache threshold, default is 512
  > &ensp;&ensp;&ensp;&ensp;Note: only available in the param `check_cmdnum_limit` is true
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  XArmAPI(const std::string &port = "",
  > &ensp;&ensp;&ensp;&ensp;    bool is_radian = DEFAULT_IS_RADIAN,
  > &ensp;&ensp;&ensp;&ensp;    bool do_not_open = false,
  > &ensp;&ensp;&ensp;&ensp;    bool check_tcp_limit = true,
  > &ensp;&ensp;&ensp;&ensp;    bool check_joint_limit = true,
  > &ensp;&ensp;&ensp;&ensp;    bool check_cmdnum_limit = true,
  > &ensp;&ensp;&ensp;&ensp;    bool check_robot_sn = false,
  > &ensp;&ensp;&ensp;&ensp;    bool check_is_ready = true,
  > &ensp;&ensp;&ensp;&ensp;    bool check_is_pause = true,
  > &ensp;&ensp;&ensp;&ensp;    int max_callback_thread_count = -1,
  > &ensp;&ensp;&ensp;&ensp;    int max_cmdnum = 512,
  > &ensp;&ensp;&ensp;&ensp;    int init_axis = 7,
  > &ensp;&ensp;&ensp;&ensp;    bool debug = false,
  > &ensp;&ensp;&ensp;&ensp;    std::string report_type = "rich",
  > &ensp;&ensp;&ensp;&ensp;    bool baud_checkset = true);
  > &ensp;&ensp;&ensp;&ensp;  ~XArmAPI(void);

### has_error()
<a id="method-has_error"></a>

```cpp
bool has_error(void);
```

  > xArm has error or not, only available in socket way

### has_warn()
<a id="method-has_warn"></a>

```cpp
bool has_warn(void);
```

  > xArm has warn or not, only available in socket way

### is_connected()
<a id="method-is_connected"></a>

```cpp
bool is_connected(void);
```

  > xArm is connected or not

### is_lite6()
<a id="method-is_lite6"></a>

```cpp
bool is_lite6(void);
```

  > Robot is lite6 or not

### is_850()
<a id="method-is_850"></a>

```cpp
bool is_850(void);
```

  > Robot is UF850 or not

### is_reported()
<a id="method-is_reported"></a>

```cpp
bool is_reported(void);
```

  > xArm is reported or not, only available in socket way

### connect()
<a id="method-connect"></a>

```cpp
int connect(const std::string &port = "");
```

  > Connect to xArm
  > @param port: port name or the ip address

### disconnect()
<a id="method-disconnect"></a>

```cpp
void disconnect(void);
```

  > Disconnect

### get_version()
<a id="method-get_version"></a>

```cpp
int get_version(unsigned char version[40]);
```

  > Get the xArm version
  > @param version:
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_robot_sn()
<a id="method-get_robot_sn"></a>

```cpp
int get_robot_sn(unsigned char robot_sn[40]);
```

  > Get the xArm sn
  > @param robot_sn:
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_state()
<a id="method-get_state"></a>

```cpp
int get_state(int *state);
```

  > Get the xArm state
  > @param state: the state of xArm
  > &ensp;&ensp;&ensp;&ensp; 1: in motion
  > &ensp;&ensp;&ensp;&ensp; 2: sleeping
  > &ensp;&ensp;&ensp;&ensp; 3: suspended
  > &ensp;&ensp;&ensp;&ensp; 4: stopping
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### system_control()
<a id="method-system_control"></a>

```cpp
int system_control(int value = 1);
```

  > Control the xArm controller system
  > @param value:
  > &ensp;&ensp;&ensp;&ensp; 1: shutdown
  > &ensp;&ensp;&ensp;&ensp; 2: reboot
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_cmdnum()
<a id="method-get_cmdnum"></a>

```cpp
int get_cmdnum(int *cmdnum);
```

  > Get the cmd count in cache
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_err_warn_code()
<a id="method-get_err_warn_code"></a>

```cpp
int get_err_warn_code(int err_warn[2]);
```

  > Get the controller error and warn code
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_position()
<a id="method-get_position"></a>

```cpp
int get_position(fp32 pose[6]);
```

  > Get the cartesian position
  > @param pose: the position of xArm, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_servo_angle()
<a id="method-get_servo_angle"></a>

```cpp
int get_servo_angle(fp32 angles[7], bool is_real = false);
```

  > Get the servo angle
  > @param angles: the angles of the servos, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_joint_states()
<a id="method-get_joint_states"></a>

```cpp
int get_joint_states(fp32 jposition[7], fp32 velocity[7], fp32 effort[7], int num = 3);
```

  > Get the joint states
  > @param position: the angles of the joints, like [angle-1, ..., angle-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of angle-1/.../angle-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of angle-1/.../angle-7 should be in degrees
  > @param velocity: the velocities of the joints, like [velo-1, ..., velo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of velo-1/.../velo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of velo-1/.../velo-7 should be in degrees
  > @param effort: the efforts of the joints, like [effort-1, ..., effort-7]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### motion_enable()
<a id="method-motion_enable"></a>

```cpp
int motion_enable(bool enable, int servo_id = 8);
```

  > Motion enable
  > @param enable: enable or not
  > @param servo_id: servo id, 1-8, 8(enable/disable all servo)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_state()
<a id="method-set_state"></a>

```cpp
int set_state(int state);
```

  > Set the xArm state
  > @param state: state
  > &ensp;&ensp;&ensp;&ensp; 0: motion state
  > &ensp;&ensp;&ensp;&ensp; 3: pause state
  > &ensp;&ensp;&ensp;&ensp; 4: stop state
  > &ensp;&ensp;&ensp;&ensp; 6: deceleration stop state
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_mode()
<a id="method-set_mode"></a>

```cpp
int set_mode(int mode, int detection_param = 0);
```

  > Set the xArm mode
  > @param mode: mode
  > &ensp;&ensp;&ensp;&ensp; 0: position control mode
  > &ensp;&ensp;&ensp;&ensp; 1: servo motion mode
  > &ensp;&ensp;&ensp;&ensp; 2: joint teaching mode
  > &ensp;&ensp;&ensp;&ensp; 3: cartesian teaching mode (invalid)
  > &ensp;&ensp;&ensp;&ensp; 4: joint velocity control mode
  > &ensp;&ensp;&ensp;&ensp; 5: cartesian velocity control mode
  > &ensp;&ensp;&ensp;&ensp; 6: joint online trajectory planning mode
  > &ensp;&ensp;&ensp;&ensp; 7: cartesian online trajectory planning mode
  > @param detection_param: teaching detection parameters, default is 0
  > &ensp;&ensp;&ensp;&ensp; 0: turn on motion detection
  > &ensp;&ensp;&ensp;&ensp; 1: turn off motion detection
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   1. only available if firmware_version >= 1.10.1
  > &ensp;&ensp;&ensp;&ensp;   2. only available if set_mode(2)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_servo_attach()
<a id="method-set_servo_attach"></a>

```cpp
int set_servo_attach(int servo_id);
```

  > Attach the servo
  > @param servo_id: servo id, 1-8, 8(attach all servo)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_servo_detach()
<a id="method-set_servo_detach"></a>

```cpp
int set_servo_detach(int servo_id);
```

  > Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.
  > @param servo_id: servo id, 1-8, 8(detach all servo)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### clean_error()
<a id="method-clean_error"></a>

```cpp
int clean_error(void);
```

  > Clean the controller error, need to be manually enabled motion and set state after clean error
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### clean_warn()
<a id="method-clean_warn"></a>

```cpp
int clean_warn(void);
```

  > Clean the controller warn
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_pause_time()
<a id="method-set_pause_time"></a>

```cpp
int set_pause_time(fp32 sltime);
```

  > Set the arm pause time, xArm will pause sltime second
  > @param sltime: sleep second
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_collision_sensitivity()
<a id="method-set_collision_sensitivity"></a>

```cpp
int set_collision_sensitivity(int sensitivity, bool wait = true);
```

  > Set the sensitivity of collision
  > @param sensitivity: sensitivity value, 0~5
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_teach_sensitivity()
<a id="method-set_teach_sensitivity"></a>

```cpp
int set_teach_sensitivity(int sensitivity, bool wait = true);
```

  > Set the sensitivity of drag and teach
  > @param sensitivity: sensitivity value, 1~5
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_gravity_direction()
<a id="method-set_gravity_direction"></a>

```cpp
int set_gravity_direction(fp32 gravity_dir[3], bool wait = true);
```

  > Set the gravity direction for proper torque compensation and collision detection.
  > @param gravity_dir: Gravity direction vector [x, y, z], e.g., [0, 0, -1] for a floor-mounted arm.
  > @param wait: Whether to wait for the robotic arm to stop or clear all previous queued commands before applying the setting.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### clean_conf()
<a id="method-clean_conf"></a>

```cpp
int clean_conf(void);
```

  > Clean current config and restore system default settings
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### save_conf()
<a id="method-save_conf"></a>

```cpp
int save_conf(void);
```

  > Save config
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_position()
<a id="method-set_position"></a>

```cpp
int set_position(fp32 pose[6], fp32 radius = -1, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, bool relative = false, unsigned char motion_type = 0);
int set_position(fp32 pose[6], fp32 radius, bool wait, fp32 timeout = NO_TIMEOUT, bool relative = false, unsigned char motion_type = 0);
int set_position(fp32 pose[6], bool wait, fp32 timeout = NO_TIMEOUT, bool relative = false, unsigned char motion_type = 0);
```

  > Set the position
  > @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
  > @param mvtime: reserved, 0
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @param relative: relative move or not
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.8.100
  > @param motion_type: motion planning type, default is 0
  > &ensp;&ensp;&ensp;&ensp; motion_type == 0: default, linear planning
  > &ensp;&ensp;&ensp;&ensp; motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible
  > &ensp;&ensp;&ensp;&ensp; motion_type == 2: direct transfer to IK using joint planning
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   1. only available if firmware_version >= 1.11.100
  > &ensp;&ensp;&ensp;&ensp;   2. when motion_type is 1 or 2, linear motion cannot be guaranteed
  > &ensp;&ensp;&ensp;&ensp;   3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage
  > &ensp;&ensp;&ensp;&ensp;     speed = speed / max_tcp_speed * max_joint_speed
  > &ensp;&ensp;&ensp;&ensp;     acc = acc / max_tcp_acc * max_joint_acc
  > &ensp;&ensp;&ensp;&ensp;   4. if there is no suitable IK, a C40 error will be triggered
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tool_position()
<a id="method-set_tool_position"></a>

```cpp
int set_tool_position(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, unsigned char motion_type = 0);
int set_tool_position(fp32 pose[6], bool wait, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, unsigned char motion_type = 0);
```

  > Movement relative to the tool coordinate system
  > @param pose: the coordinate relative to the current tool coordinate system, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
  > @param mvtime: reserved, 0
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @param radius: move radius, if radius less than 0, will MoveToolLine, else MoveToolArcLine
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.11.100
  > @param motion_type: motion planning type, default is 0
  > &ensp;&ensp;&ensp;&ensp; motion_type == 0: default, linear planning
  > &ensp;&ensp;&ensp;&ensp; motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible
  > &ensp;&ensp;&ensp;&ensp; motion_type == 2: direct transfer to IK using joint planning
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   1. only available if firmware_version >= 1.11.100
  > &ensp;&ensp;&ensp;&ensp;   2. when motion_type is 1 or 2, linear motion cannot be guaranteed
  > &ensp;&ensp;&ensp;&ensp;   3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage
  > &ensp;&ensp;&ensp;&ensp;     speed = speed / max_tcp_speed * max_joint_speed
  > &ensp;&ensp;&ensp;&ensp;     acc = acc / max_tcp_acc * max_joint_acc
  > &ensp;&ensp;&ensp;&ensp;   4. if there is no suitable IK, a C40 error will be triggered
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_servo_angle()
<a id="method-set_servo_angle"></a>

```cpp
int set_servo_angle(fp32 angles[7], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, bool relative = false);
int set_servo_angle(fp32 angles[7], bool wait, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, bool relative = false);
int set_servo_angle(int servo_id, fp32 angle, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, bool relative = false);
int set_servo_angle(int servo_id, fp32 angle, bool wait, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, bool relative = false);
```

  > Set the servo angle
  > @param angles: angles, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
  > @param servo_id: servo id, 1~7, specify the joint ID to set
  > @param angle: servo angle, use with servo_id parameters
  > @param speed: move speed (rad/s or °/s), default is this.last_used_joint_speed
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of speed should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of speed should be in degrees
  > @param acc: move acceleration (rad/s^2 or °/s^2), default is this.last_used_joint_acc
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of acc should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of acc should be in degrees
  > @param mvtime: reserved, 0
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @param radius: move radius, if radius less than 0, will MoveJoint, else MoveArcJoint
  > &ensp;&ensp;&ensp;&ensp; Note: the blending radius cannot be greater than the track length.
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.5.20
  > @param relative: relative move or not
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.8.100
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_servo_angle_j()
<a id="method-set_servo_angle_j"></a>

```cpp
int set_servo_angle_j(fp32 angles[7], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0);
```

  > Servo_j motion, execute only the last instruction, need to be set to servo motion mode(this.set_mode(1))
  > @param angles: angles, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
  > @param speed: reserved, move speed (rad/s or °/s)
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of speed should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of speed should be in degrees
  > @param acc: reserved, move acceleration (rad/s^2 or °/s^2)
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of acc should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of acc should be in degrees
  > @param mvtime: reserved, 0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_servo_cartesian()
<a id="method-set_servo_cartesian"></a>

```cpp
int set_servo_cartesian(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false);
```

  > Servo cartesian motion, execute only the last instruction, need to be set to servo motion mode(this.set_mode(1))
  > @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param speed: reserved, move speed (mm/s)
  > @param mvacc: reserved, move acceleration (mm/s^2)
  > @param mvtime: reserved, 0
  > @param is_tool_coord: is tool coordinate or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### move_circle()
<a id="method-move_circle"></a>

```cpp
int move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, bool is_tool_coord = false, bool is_axis_angle = false);
```

  > The motion calculates the trajectory of the space circle according to the three-point coordinates.
  > @param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param percent: the percentage of arc length and circumference of the movement
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
  > @param mvtime: 0, reserved
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @param is_tool_coord: is tool coord or not, default is false, only available if firmware_version >= 1.11.100
  > @param is_axis_angle: is axis angle or not, default is false, only available if firmware_version >= 1.11.100
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### move_gohome()
<a id="method-move_gohome"></a>

```cpp
int move_gohome(fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
int move_gohome(bool wait, fp32 timeout = NO_TIMEOUT);
```

  > Move to go home (Back to zero)
  > @param speed: move speed (rad/s or °/s), default is 50 °/s
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of speed should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of speed should be in degrees
  > @param acc: move acceleration (rad/s^2 or °/s^2), default is 1000 °/s^2
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of acc should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of acc should be in degrees
  > @param mvtime: reserved, 0
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### reset()
<a id="method-reset"></a>

```cpp
void reset(bool wait = false, fp32 timeout = NO_TIMEOUT);
```

  > Reset
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### emergency_stop()
<a id="method-emergency_stop"></a>

```cpp
void emergency_stop(void);
```

  > Emergency stop

### set_tcp_offset()
<a id="method-set_tcp_offset"></a>

```cpp
int set_tcp_offset(fp32 pose_offset[6], bool wait = true);
```

  > Set the tool coordinate system offset at the end
  > @param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tcp_load()
<a id="method-set_tcp_load"></a>

```cpp
int set_tcp_load(fp32 weight, fp32 center_of_gravity[3], bool wait = false);
```

  > Set the load
  > @param weight: load weight (unit: kg)
  > @param center_of_gravity: tcp load center of gravity, like [x(mm), y(mm), z(mm)]
  > @param wait: whether to wait for the command to be executed or the robotic arm to stop
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tcp_jerk()
<a id="method-set_tcp_jerk"></a>

```cpp
int set_tcp_jerk(fp32 jerk);
```

  > Set the translational jerk of Cartesian space
  > @param jerk: jerk (mm/s^3)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tcp_maxacc()
<a id="method-set_tcp_maxacc"></a>

```cpp
int set_tcp_maxacc(fp32 acc);
```

  > Set the max translational acceleration of Cartesian space
  > @param acc: max acceleration (mm/s^2)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_joint_jerk()
<a id="method-set_joint_jerk"></a>

```cpp
int set_joint_jerk(fp32 jerk);
```

  > Set the jerk of Joint space
  > @param jerk: jerk (°/s^3 or rad/s^3)
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of jerk should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of jerk should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_joint_maxacc()
<a id="method-set_joint_maxacc"></a>

```cpp
int set_joint_maxacc(fp32 acc);
```

  > Set the max acceleration of Joint space
  > @param acc: max acceleration (°/s^2 or rad/s^2)
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of acc should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of acc should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_inverse_kinematics()
<a id="method-get_inverse_kinematics"></a>

```cpp
int get_inverse_kinematics(fp32 pose[6], fp32 angles[7], bool limited = true, fp32 *ref_angles = nullptr);
```

  > Get inverse kinematics
  > @param pose: source pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param angles: target angles, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
  > @param limited: the result is limited to within ±180° or not, default is true
  > &ensp;&ensp;&ensp;&ensp; 1. only available if firmware_version >= 2.7.103
  > @param ref_angles: reference values for joint angles, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; 1. only available if firmware_version >= 2.7.103
  > &ensp;&ensp;&ensp;&ensp; 2. if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_forward_kinematics()
<a id="method-get_forward_kinematics"></a>

```cpp
int get_forward_kinematics(fp32 angles[7], fp32 pose[6]);
```

  > Get forward kinematics
  > @param angles: source angles, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
  > @param pose: target pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### is_tcp_limit()
<a id="method-is_tcp_limit"></a>

```cpp
int is_tcp_limit(fp32 pose[6], int *limit);
```

  > Check the tcp pose is in limit
  > @param pose: pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param limit: 1: limit, 0: no limit
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### is_joint_limit()
<a id="method-is_joint_limit"></a>

```cpp
int is_joint_limit(fp32 angles[7], int *limit);
```

  > Check the joint is in limit
  > @param angles: angles, like [servo-1, ..., servo-7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
  > @param limit: 1: limit, 0: no limit
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_gripper_enable()
<a id="method-set_gripper_enable"></a>

```cpp
int set_gripper_enable(bool enable);
```

  > Set the gripper enable
  > @param enable: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_gripper_mode()
<a id="method-set_gripper_mode"></a>

```cpp
int set_gripper_mode(int mode);
```

  > Set the gripper mode
  > @param mode: 1: location mode, 2: speed mode(no use), 3: torque mode(no use)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_gripper_position()
<a id="method-get_gripper_position"></a>

```cpp
int get_gripper_position(int *pos);
int get_gripper_position(fp32 *pos);
```

  > Get the gripper position (pulse)
  > @param pos: used to store the results obtained
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_gripper_g2_position()
<a id="method-get_gripper_g2_position"></a>

```cpp
int get_gripper_g2_position(int *pos);
```

  > Get the position (mm) of the xArm Gripper G2
  > @param pos: used to store the results obtained
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_gripper_position()
<a id="method-set_gripper_position"></a>

```cpp
int set_gripper_position(int pos, bool wait = false, fp32 timeout = 10, bool wait_motion = true);
int set_gripper_position(int pos, int speed, bool wait = false, fp32 timeout = 10, bool wait_motion = true);
```

  > Set the gripper position
  > @param pos: gripper position
  > @param wait: wait or not, default is false
  > @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_gripper_g2_position()
<a id="method-set_gripper_g2_position"></a>

```cpp
int set_gripper_g2_position(int pos, int speed = 100, int force = 50, bool wait = false, fp32 timeout = 10, bool wait_motion = true);
```

  > Set the position of the xArm Gripper G2
  > @param pos: gripper pos between 0 and 84, (unit: mm)
  > @param speed: gripper speed between 15 and 225, default is 100, (unit: mm/s)
  > @param force: gripper force between 1 and 100, default is 50
  > @param wait: whether to wait for the bio gripper motion complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_gripper_speed()
<a id="method-set_gripper_speed"></a>

```cpp
int set_gripper_speed(int speed);
```

  > Set the gripper speed
  > @param speed:
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_gripper_status()
<a id="method-get_gripper_status"></a>

```cpp
int get_gripper_status(int *status);
```

  > Get the status of the xArm Gripper
  > @param status: used to store the results obtained
  > &ensp;&ensp;&ensp;&ensp;   status & 0x03 == 0: stop state
  > &ensp;&ensp;&ensp;&ensp;   status & 0x03 == 1: move state
  > &ensp;&ensp;&ensp;&ensp;   status & 0x03 == 2: grasp state
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_gripper_err_code()
<a id="method-get_gripper_err_code"></a>

```cpp
int get_gripper_err_code(int *err);
```

  > Get the gripper error code
  > @param err: used to store the results obtained
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### clean_gripper_error()
<a id="method-clean_gripper_error"></a>

```cpp
int clean_gripper_error(void);
```

  > Clean the gripper error
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_tgpio_digital()
<a id="method-get_tgpio_digital"></a>

```cpp
int get_tgpio_digital(int *io0_value, int *io1_value, int *io2_value = nullptr, int *io3_value = nullptr, int *io4_value = nullptr);
```

  > Get the digital value of the Tool GPIO
  > @param io0_value: the digital value of Tool GPIO-0
  > @param io1_value: the digital value of Tool GPIO-1
  > @param io2_value: the digital value of Tool GPIO-2
  > @param io3_value: the digital value of Tool GPIO-3
  > @param io4_value: the digital value of Tool GPIO-4
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tgpio_digital()
<a id="method-set_tgpio_digital"></a>

```cpp
int set_tgpio_digital(int ionum, int value, float delay_sec = 0, bool sync = true);
```

  > Set the digital value of the specified Tool GPIO
  > @param ionum: ionum, 0 or 1
  > @param value: the digital value of the specified io
  > @param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > &ensp;&ensp;&ensp;&ensp;  2. only available if delay_sec <= 0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_tgpio_analog()
<a id="method-get_tgpio_analog"></a>

```cpp
int get_tgpio_analog(int ionum, float *value);
```

  > Get the analog value of the specified Tool GPIO
  > @param ionum: ionum, 0 or 1
  > @param value: the analog value of the specified tool io
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_cgpio_digital()
<a id="method-get_cgpio_digital"></a>

```cpp
int get_cgpio_digital(int *digitals, int *digitals2 = nullptr);
```

  > Get the digital value of the specified Controller GPIO
  > @param digitals: the values of the controller GPIO(0-7)
  > @param digitals2: the values of the controller GPIO(8-15)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_cgpio_analog()
<a id="method-get_cgpio_analog"></a>

```cpp
int get_cgpio_analog(int ionum, fp32 *value);
```

  > Get the analog value of the specified Controller GPIO
  > @param ionum: ionum, 0 or 1
  > @param value: the analog value of the specified controller io
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cgpio_digital()
<a id="method-set_cgpio_digital"></a>

```cpp
int set_cgpio_digital(int ionum, int value, float delay_sec = 0, bool sync = true);
```

  > Set the digital value of the specified Controller GPIO
  > @param ionum: ionum, 0 ~ 15
  > @param value: the digital value of the specified io
  > @param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > &ensp;&ensp;&ensp;&ensp;  2. only available if delay_sec <= 0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cgpio_analog()
<a id="method-set_cgpio_analog"></a>

```cpp
int set_cgpio_analog(int ionum, fp32 value, bool sync = true);
```

  > Set the analog value of the specified Controller GPIO
  > @param ionum: ionum, 0 or 1
  > @param value: the analog value of the specified io
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cgpio_digital_input_function()
<a id="method-set_cgpio_digital_input_function"></a>

```cpp
int set_cgpio_digital_input_function(int ionum, int fun);
```

  > Set the digital input functional mode of the Controller GPIO
  > @param ionum: ionum, 0 ~ 15
  > @param fun: functional mode
  > &ensp;&ensp;&ensp;&ensp;  0: general input
  > &ensp;&ensp;&ensp;&ensp;  1: external emergency stop
  > &ensp;&ensp;&ensp;&ensp;  2: protection reset
  > &ensp;&ensp;&ensp;&ensp; 11: offline task
  > &ensp;&ensp;&ensp;&ensp; 12: teaching mode
  > &ensp;&ensp;&ensp;&ensp; 13: reduced mode
  > &ensp;&ensp;&ensp;&ensp; 14: enable arm
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cgpio_digital_output_function()
<a id="method-set_cgpio_digital_output_function"></a>

```cpp
int set_cgpio_digital_output_function(int ionum, int fun);
```

  > Set the digital output functional mode of the specified Controller GPIO
  > @param ionum: ionum, 0 ~ 15
  > @param fun: functional mode
  > &ensp;&ensp;&ensp;&ensp;  0: general output
  > &ensp;&ensp;&ensp;&ensp;  1: emergency stop
  > &ensp;&ensp;&ensp;&ensp;  2: in motion
  > &ensp;&ensp;&ensp;&ensp; 11: has error
  > &ensp;&ensp;&ensp;&ensp; 12: has warn
  > &ensp;&ensp;&ensp;&ensp; 13: in collision
  > &ensp;&ensp;&ensp;&ensp; 14: in teaching
  > &ensp;&ensp;&ensp;&ensp; 15: in offline task
  > &ensp;&ensp;&ensp;&ensp; 16: in reduced mode
  > &ensp;&ensp;&ensp;&ensp; 17: is enabled
  > &ensp;&ensp;&ensp;&ensp; 18: emergency stop is pressed
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_cgpio_state()
<a id="method-get_cgpio_state"></a>

```cpp
int get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf, int *input_conf2 = nullptr, int *output_conf2 = nullptr);
```

  > Get the state of the Controller GPIO
  > @param state: controller gpio module state and controller gpio module error code
  > &ensp;&ensp;&ensp;&ensp; state[0]: controller gpio module state
  > &ensp;&ensp;&ensp;&ensp;   state[0] == 0: normal
  > &ensp;&ensp;&ensp;&ensp;   state[0] == 1：wrong
  > &ensp;&ensp;&ensp;&ensp;   state[0] == 6：communication failure
  > &ensp;&ensp;&ensp;&ensp; state[1]: controller gpio module error code
  > &ensp;&ensp;&ensp;&ensp;   state[1] == 0: normal
  > &ensp;&ensp;&ensp;&ensp;   state[1] != 0：error code
  > @param digit_io:
  > &ensp;&ensp;&ensp;&ensp; digit_io[0]: digital input functional gpio state
  > &ensp;&ensp;&ensp;&ensp; digit_io[1]: digital input configuring gpio state
  > &ensp;&ensp;&ensp;&ensp; digit_io[2]: digital output functional gpio state
  > &ensp;&ensp;&ensp;&ensp; digit_io[3]: digital output configuring gpio state
  > @param analog:
  > &ensp;&ensp;&ensp;&ensp; analog[0]: analog-0 input value
  > &ensp;&ensp;&ensp;&ensp; analog[1]: analog-1 input value
  > &ensp;&ensp;&ensp;&ensp; analog[2]: analog-0 output value
  > &ensp;&ensp;&ensp;&ensp; analog[3]: analog-1 output value
  > @param input_conf: digital(0-7) input functional info
  > @param output_conf: digital(0-7) output functional info
  > @param input_conf2: digital(8-15) input functional info
  > @param output_conf2: digital(8-15) output functional info
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_vacuum_gripper()
<a id="method-get_vacuum_gripper"></a>

```cpp
int get_vacuum_gripper(int *val, int hardware_version = 1);
```

  > Get the state of the Vacuum Gripper
  > @param callback: nullptr means to release all callbacks;
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_report_data_callback(void(*callback)(XArmReportData *report_data_ptr) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_report_data_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks;
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_report_location_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_connect_changed_callback(void(*callback)(bool connected, bool reported) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_connect_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_state_changed_callback(void(*callback)(int state) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_state_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_mode_changed_callback(void(*callback)(int mode) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_mode_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_mtable_mtbrake_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_error_warn_changed_callback(void(*callback)(int err_code, int warn_code) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_error_warn_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_cmdnum_changed_callback(void(*callback)(int cmdnum) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_cmdnum_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_temperature_changed_callback(void(*callback)(const fp32 *temps) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_temperature_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_count_changed_callback(void(*callback)(int count) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_count_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_iden_progress_changed_callback(void(*callback)(int progress) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_iden_progress_changed_callback(bool clear_all);
  > @param callback: nullptr means to release all callbacks for the same event
  > &ensp;&ensp;&ensp;&ensp;/
  > &ensp;&ensp;&ensp;&ensp;  int release_feedback_callback(void(*callback)(unsigned char *feedback_data) = nullptr);
  > &ensp;&ensp;&ensp;&ensp;  int release_feedback_callback(bool clear_all);
  > @param val:
  > &ensp;&ensp;&ensp;&ensp; -1: Vacuum Gripper is off
  > &ensp;&ensp;&ensp;&ensp; 0: Object not picked by vacuum gripper
  > &ensp;&ensp;&ensp;&ensp; 1: Object picked by vacuum gripper
  > @param hardware_version:
  > &ensp;&ensp;&ensp;&ensp; 1: Plug-in Connection, default
  > &ensp;&ensp;&ensp;&ensp; 2: Contact Connection
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_vacuum_gripper()
<a id="method-set_vacuum_gripper"></a>

```cpp
int set_vacuum_gripper(bool on, bool wait = false, float timeout = 3, float delay_sec = 0, bool sync = true, int hardware_version = 1);
```

  > Set the Vacuum Gripper ON/OFF
  > @param on: open vacuum gripper or not
  > @param wait: wait or not, default is false
  > @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
  > @param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > &ensp;&ensp;&ensp;&ensp;  2. only available if delay_sec <= 0
  > @param hardware_version:
  > &ensp;&ensp;&ensp;&ensp; 1: Plug-in Connection, default
  > &ensp;&ensp;&ensp;&ensp; 2: Contact Connection
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_gripper_version()
<a id="method-get_gripper_version"></a>

```cpp
int get_gripper_version(unsigned char versions[3]);
```

  > Get gripper version, only for debug
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_servo_version()
<a id="method-get_servo_version"></a>

```cpp
int get_servo_version(unsigned char versions[3], int servo_id = 1);
```

  > Get servo version, only for debug
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_tgpio_version()
<a id="method-get_tgpio_version"></a>

```cpp
int get_tgpio_version(unsigned char versions[3]);
```

  > Get tool gpio version, only for debug
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### reload_dynamics()
<a id="method-reload_dynamics"></a>

```cpp
int reload_dynamics(void);
```

  > Reload dynamics, only for debug
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_reduced_mode()
<a id="method-set_reduced_mode"></a>

```cpp
int set_reduced_mode(bool on);
```

  > Turn on/off reduced mode
  > @param on: on/off
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_reduced_max_tcp_speed()
<a id="method-set_reduced_max_tcp_speed"></a>

```cpp
int set_reduced_max_tcp_speed(float speed);
```

  > Set the maximum tcp speed of the reduced mode
  > @param speed: the maximum tcp speed
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_reduced_max_joint_speed()
<a id="method-set_reduced_max_joint_speed"></a>

```cpp
int set_reduced_max_joint_speed(float speed);
```

  > Set the maximum joint speed of the reduced mode
  > @param speed: the maximum joint speed
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of speed should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of speed should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_reduced_mode()
<a id="method-get_reduced_mode"></a>

```cpp
int get_reduced_mode(int *mode);
```

  > Get reduced mode
  > @param mode:
  > &ensp;&ensp;&ensp;&ensp; 0: reduced mode is on
  > &ensp;&ensp;&ensp;&ensp; 1: reduced mode is off
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_reduced_states()
<a id="method-get_reduced_states"></a>

```cpp
int get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14] = nullptr, int *fence_is_on = nullptr, int *collision_rebound_is_on = nullptr);
```

  > Get states of the reduced mode
  > @param on:
  > &ensp;&ensp;&ensp;&ensp; 0: reduced mode is on
  > &ensp;&ensp;&ensp;&ensp; 1: reduced mode is off
  > @param xyz_list: the tcp boundary, like [reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min],
  > @param tcp_speed: the maximum tcp speed of reduced mode
  > @param joint_speed: the maximum joint speed of reduced mode
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of speed should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of speed should be in degrees
  > @param jrange: the joint range of the reduced mode, like [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of joint range should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of joint range should be in degrees
  > @param fence_is_on:
  > &ensp;&ensp;&ensp;&ensp; 0: safety mode is on
  > &ensp;&ensp;&ensp;&ensp; 1: safety mode is off
  > @param collision_rebound_is_on:
  > &ensp;&ensp;&ensp;&ensp; 0: collision rebound is on
  > &ensp;&ensp;&ensp;&ensp; 1: collision rebound is off
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_reduced_tcp_boundary()
<a id="method-set_reduced_tcp_boundary"></a>

```cpp
int set_reduced_tcp_boundary(int boundary[6]);
```

  > Set the boundary of the safety boundary mode
  > @param boundary: like [x_max(mm), x_min(mm), y_max(mm), y_min(mm), z_max(mm), z_min(mm)]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_reduced_joint_range()
<a id="method-set_reduced_joint_range"></a>

```cpp
int set_reduced_joint_range(float jrange[14]);
```

  > Set the joint range of the reduced mode
  > @param jrange: like [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of joint range should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of joint range should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_fence_mode()
<a id="method-set_fence_mode"></a>

```cpp
int set_fence_mode(bool on);
```

  > Turn on/off safety mode
  > @param on: on/off
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_collision_rebound()
<a id="method-set_collision_rebound"></a>

```cpp
int set_collision_rebound(bool on);
```

  > Turn on/off collision rebound
  > @param on: on/off
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_world_offset()
<a id="method-set_world_offset"></a>

```cpp
int set_world_offset(float pose_offset[6], bool wait = true);
```

  > Set the base coordinate system offset at the end
  > @param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### start_record_trajectory()
<a id="method-start_record_trajectory"></a>

```cpp
int start_record_trajectory(void);
```

  > Start trajectory recording, only in teach mode, so you need to set joint teaching mode before.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### stop_record_trajectory()
<a id="method-stop_record_trajectory"></a>

```cpp
int stop_record_trajectory(const char* filename = nullptr);
```

  > Stop trajectory recording
  > @param filename: the name to save
  > &ensp;&ensp;&ensp;&ensp; If the filename is nullptr, just stop recording, do not save, you need to manually call `save_record_trajectory` save before changing the mode. otherwise it will be lost
  > &ensp;&ensp;&ensp;&ensp; the trajectory is saved in the controller box.
  > &ensp;&ensp;&ensp;&ensp; this action will overwrite the trajectory with the same name
  > &ensp;&ensp;&ensp;&ensp; empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### save_record_trajectory()
<a id="method-save_record_trajectory"></a>

```cpp
int save_record_trajectory(const char* filename, float timeout = 5);
```

  > Save the trajectory you just recorded
  > @param filename: the name to save
  > &ensp;&ensp;&ensp;&ensp; the trajectory is saved in the controller box.
  > &ensp;&ensp;&ensp;&ensp; this action will overwrite the trajectory with the same name
  > &ensp;&ensp;&ensp;&ensp; empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### load_trajectory()
<a id="method-load_trajectory"></a>

```cpp
int load_trajectory(const char* filename, float timeout = NO_TIMEOUT);
```

  > Load the trajectory
  > @param filename: the name of the trajectory to load
  > @param timeout: the maximum timeout waiting for loading to complete, default is 10 seconds.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### playback_trajectory()
<a id="method-playback_trajectory"></a>

```cpp
int playback_trajectory(int times = 1, const char* filename = nullptr, bool wait = false, int double_speed = 1);
```

  > Playback trajectory
  > @param times: number of playbacks.
  > @param filename: the name of the trajectory to play back
  > &ensp;&ensp;&ensp;&ensp; if filename is nullptr, you need to manually call the `load_trajectory` to load the trajectory.
  > @param wait: whether to wait for the arm to complete, default is false.
  > @param double_speed: double speed, only support 1/2/4, default is 1, only available if version > 1.2.11
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_trajectory_rw_status()
<a id="method-get_trajectory_rw_status"></a>

```cpp
int get_trajectory_rw_status(int *status);
```

  > Get trajectory read/write status
  > @param status:
  > &ensp;&ensp;&ensp;&ensp; 0: no read/write
  > &ensp;&ensp;&ensp;&ensp; 1: loading
  > &ensp;&ensp;&ensp;&ensp; 2: load success
  > &ensp;&ensp;&ensp;&ensp; 3: load failed
  > &ensp;&ensp;&ensp;&ensp; 4: saving
  > &ensp;&ensp;&ensp;&ensp; 5: save success
  > &ensp;&ensp;&ensp;&ensp; 6: save failed
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_counter_reset()
<a id="method-set_counter_reset"></a>

```cpp
int set_counter_reset(void);
```

  > Reset counter value
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_counter_increase()
<a id="method-set_counter_increase"></a>

```cpp
int set_counter_increase(void);
```

  > Set counter plus 1
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tgpio_digital_with_xyz()
<a id="method-set_tgpio_digital_with_xyz"></a>

```cpp
int set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r);
```

  > Set the digital value of the specified Tool GPIO when the robot has reached the specified xyz position
  > @param ionum: 0 or 1
  > @param value: value
  > @param xyz: position xyz, as [x, y, z]
  > @param tol_r: fault tolerance radius
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cgpio_digital_with_xyz()
<a id="method-set_cgpio_digital_with_xyz"></a>

```cpp
int set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r);
```

  > Set the digital value of the specified Controller GPIO when the robot has reached the specified xyz position
  > @param ionum: 0 ~ 7
  > @param value: value
  > @param xyz: position xyz, as [x, y, z]
  > @param tol_r: fault tolerance radius
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cgpio_analog_with_xyz()
<a id="method-set_cgpio_analog_with_xyz"></a>

```cpp
int set_cgpio_analog_with_xyz(int ionum, float value, float xyz[3], float tol_r);
```

  > Set the analog value of the specified Controller GPIO when the robot has reached the specified xyz position
  > @param ionum: 0 ~ 1
  > @param value: value, 0~10.0
  > @param xyz: position xyz, as [x, y, z]
  > @param tol_r: fault tolerance radius
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### config_tgpio_reset_when_stop()
<a id="method-config_tgpio_reset_when_stop"></a>

```cpp
int config_tgpio_reset_when_stop(bool on_off);
```

  > Config the Tool GPIO reset the digital output when the robot is in stop state
  > @param on_off: true/false
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### config_cgpio_reset_when_stop()
<a id="method-config_cgpio_reset_when_stop"></a>

```cpp
int config_cgpio_reset_when_stop(bool on_off);
```

  > Config the Controller GPIO reset the digital output when the robot is in stop state
  > @param on_off: true/false
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_position_aa()
<a id="method-set_position_aa"></a>

```cpp
int set_position_aa(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, unsigned char motion_type = 0);
int set_position_aa(fp32 pose[6], bool is_tool_coord, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, unsigned char motion_type = 0);
```

  > Set the pose represented by the axis angle pose
  > @param pose: the axis angle pose, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of rx/ry/rz should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of rx/ry/rz should be in degrees
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
  > @param mvtime: reserved, 0
  > @param is_tool_coord: is tool coordinate or not, if it is true, the relative parameter is no longer valid
  > @param relative: relative move or not
  > @param wait: whether to wait for the arm to complete, default is false
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
  > @param radius: move radius, if radius less than 0, will MoveLineAA, else MoveArcLineAA
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.11.100
  > @param motion_type: motion planning type, default is 0
  > &ensp;&ensp;&ensp;&ensp; motion_type == 0: default, linear planning
  > &ensp;&ensp;&ensp;&ensp; motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible
  > &ensp;&ensp;&ensp;&ensp; motion_type == 2: direct transfer to IK using joint planning
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   1. only available if firmware_version >= 1.11.100
  > &ensp;&ensp;&ensp;&ensp;   2. when motion_type is 1 or 2, linear motion cannot be guaranteed
  > &ensp;&ensp;&ensp;&ensp;   3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage
  > &ensp;&ensp;&ensp;&ensp;     speed = speed / max_tcp_speed * max_joint_speed
  > &ensp;&ensp;&ensp;&ensp;     acc = acc / max_tcp_acc * max_joint_acc
  > &ensp;&ensp;&ensp;&ensp;   4. if there is no suitable IK, a C40 error will be triggered
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_servo_cartesian_aa()
<a id="method-set_servo_cartesian_aa"></a>

```cpp
int set_servo_cartesian_aa(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, bool is_tool_coord = false, bool relative = false);
int set_servo_cartesian_aa(fp32 pose[6], bool is_tool_coord, bool relative = false);
```

  > Set the servo cartesian represented by the axis angle pose, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))
  > @param pose: the axis angle pose, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of rx/ry/rz should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of rx/ry/rz should be in degrees
  > @param speed: reserved, move speed (mm/s)
  > @param mvacc: reserved, move acceleration (mm/s^2)
  > @param is_tool_coord: is tool coordinate or not
  > @param relative: relative move or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_pose_offset()
<a id="method-get_pose_offset"></a>

```cpp
int get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in = 0, int orient_type_out = 0);
```

  > Calculate the pose offset of two given points
  > @param pose1: position, like [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/rx/pitch/ry/yaw/rz should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/rx/pitch/ry/yaw/rz should be in degrees
  > @param pose2: position, like [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/rx/pitch/ry/yaw/rz should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of roll/rx/pitch/ry/yaw/rz should be in degrees
  > @param offset: the offset between pose1 and pose2
  > @param orient_type_in: input attitude notation, 0 is RPY (default), 1 is axis angle
  > @param orient_type_out: notation of output attitude, 0 is RPY (default), 1 is axis angle
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_position_aa()
<a id="method-get_position_aa"></a>

```cpp
int get_position_aa(fp32 pose[6]);
```

  > Get the pose represented by the axis angle pose
  > @param pose: the pose represented by the axis angle pose of xArm, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of rx/ry/rz should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, The value of rx/ry/rz should be in degrees
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### robotiq_reset()
<a id="method-robotiq_reset"></a>

```cpp
int robotiq_reset(unsigned char ret_data[6] = nullptr);
```

  > Reset the robotiq gripper (clear previous activation if any)
  > @param ret_data: the response from robotiq
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### robotiq_set_activate()
<a id="method-robotiq_set_activate"></a>

```cpp
int robotiq_set_activate(bool wait = true, fp32 timeout = 3, unsigned char ret_data[6] = nullptr);
int robotiq_set_activate(bool wait, unsigned char ret_data[6]);
int robotiq_set_activate(unsigned char ret_data[6]);
```

  > If not already activated. Activate the robotiq gripper
  > @param wait: whether to wait for the robotiq activate complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 3, only available if wait=true
  > @param ret_data: the response from robotiq
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### robotiq_set_position()
<a id="method-robotiq_set_position"></a>

```cpp
int robotiq_set_position(unsigned char pos, unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = nullptr, bool wait_motion = true);
int robotiq_set_position(unsigned char pos, bool wait, fp32 timeout = 5, unsigned char ret_data[6] = nullptr, bool wait_motion = true);
int robotiq_set_position(unsigned char pos, bool wait, unsigned char ret_data[6], bool wait_motion = true);
int robotiq_set_position(unsigned char pos, unsigned char ret_data[6], bool wait_motion = true);
```

  > Go to the position with determined speed and force.
  > @param pos: position of the gripper. Integer between 0 and 255. 0 being the open position and 255 being the close position.
  > @param speed: gripper speed between 0 and 255
  > @param force: gripper force between 0 and 255
  > @param wait: whether to wait for the robotiq motion complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > @param ret_data: the response from robotiq
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### robotiq_open()
<a id="method-robotiq_open"></a>

```cpp
int robotiq_open(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = nullptr, bool wait_motion = true);
int robotiq_open(bool wait, fp32 timeout = 5, unsigned char ret_data[6] = nullptr, bool wait_motion = true);
int robotiq_open(bool wait, unsigned char ret_data[6], bool wait_motion = true);
int robotiq_open(unsigned char ret_data[6], bool wait_motion = true);
```

  > Open the robotiq gripper
  > @param speed: gripper speed between 0 and 255
  > @param force: gripper force between 0 and 255
  > @param wait: whether to wait for the robotiq motion complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > @param ret_data: the response from robotiq
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### robotiq_close()
<a id="method-robotiq_close"></a>

```cpp
int robotiq_close(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = nullptr, bool wait_motion = true);
int robotiq_close(bool wait, fp32 timeout = 5, unsigned char ret_data[6] = nullptr, bool wait_motion = true);
int robotiq_close(bool wait, unsigned char ret_data[6], bool wait_motion = true);
int robotiq_close(unsigned char ret_data[6], bool wait_motion = true);
```

  > Close the robotiq gripper
  > @param speed: gripper speed between 0 and 255
  > @param force: gripper force between 0 and 255
  > @param wait: whether to wait for the robotiq motion complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > @param ret_data: the response from robotiq
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### robotiq_get_status()
<a id="method-robotiq_get_status"></a>

```cpp
int robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers = 3);
```

  > Reading the status of robotiq gripper
  > @param ret_data: the response from robotiq
  > @param number_of_registers: number of registers, 1/2/3, default is 3
  > &ensp;&ensp;&ensp;&ensp; number_of_registers=1: reading the content of register 0x07D0
  > &ensp;&ensp;&ensp;&ensp; number_of_registers=2: reading the content of register 0x07D0/0x07D1
  > &ensp;&ensp;&ensp;&ensp; number_of_registers=3: reading the content of register 0x07D0/0x07D1/0x07D2
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   register 0x07D0: Register GRIPPER STATUS
  > &ensp;&ensp;&ensp;&ensp;   register 0x07D1: Register FAULT STATUS and register POSITION REQUEST ECHO
  > &ensp;&ensp;&ensp;&ensp;   register 0x07D2: Register POSITION and register CURRENT
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_bio_gripper_enable()
<a id="method-set_bio_gripper_enable"></a>

```cpp
int set_bio_gripper_enable(bool enable, bool wait = true, fp32 timeout = 3);
```

  > If not already enabled. Enable the bio gripper
  > @param enable: enable or not
  > @param wait: whether to wait for the bio gripper enable complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 3, only available if wait=true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_bio_gripper_speed()
<a id="method-set_bio_gripper_speed"></a>

```cpp
int set_bio_gripper_speed(int speed);
```

  > Set the speed of the bio gripper
  > @param speed: speed
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_bio_gripper_control_mode()
<a id="method-set_bio_gripper_control_mode"></a>

```cpp
int set_bio_gripper_control_mode(int mode);
```

  > Set the mode of the bio gripper
  > @param mode: mode
  > &ensp;&ensp;&ensp;&ensp;  0: bio gripper opening and closing mode
  > &ensp;&ensp;&ensp;&ensp;  1: position loop mode
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_bio_gripper_force()
<a id="method-set_bio_gripper_force"></a>

```cpp
int set_bio_gripper_force(int force);
```

  > Set the force of the bio gripper
  > @param force: gripper force between 10 and 100
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_bio_gripper_g2_position()
<a id="method-set_bio_gripper_g2_position"></a>

```cpp
int set_bio_gripper_g2_position(int pos, int speed = 2000, int force=100, bool wait = true, fp32 timeout = 5, bool wait_motion = true);
```

  > Set the position of the bio gripper
  > @param pos: gripper pos between 71 and 150, (unit: mm)
  > @param speed: gripper speed between 500 and 4000, default is 2000, (unit: pulse/s)
  > @param force: gripper force between 1 and 100, default is 100
  > @param wait: whether to wait for the bio gripper motion complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### open_bio_gripper()
<a id="method-open_bio_gripper"></a>

```cpp
int open_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5, bool wait_motion = true);
int open_bio_gripper(bool wait, fp32 timeout = 5, bool wait_motion = true);
```

  > Open the bio gripper
  > @param speed: speed value, default is 0 (not set the speed)
  > @param wait: whether to wait for the bio gripper motion complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### close_bio_gripper()
<a id="method-close_bio_gripper"></a>

```cpp
int close_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5, bool wait_motion = true);
int close_bio_gripper(bool wait, fp32 timeout = 5, bool wait_motion = true);
```

  > Close the bio gripper
  > @param speed: speed value, default is 0 (not set the speed)
  > @param wait: whether to wait for the bio gripper motion complete, default is true
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_bio_gripper_status()
<a id="method-get_bio_gripper_status"></a>

```cpp
int get_bio_gripper_status(int *status);
```

  > Get the status of the bio gripper
  > @param status: the result of the bio gripper status value
  > &ensp;&ensp;&ensp;&ensp; status & 0x03 == 0: stop
  > &ensp;&ensp;&ensp;&ensp; status & 0x03 == 1: motion
  > &ensp;&ensp;&ensp;&ensp; status & 0x03 == 2: catch
  > &ensp;&ensp;&ensp;&ensp; status & 0x03 == 3: error
  > &ensp;&ensp;&ensp;&ensp; (status >> 2) & 0x03 == 0: not enabled
  > &ensp;&ensp;&ensp;&ensp; (status >> 2) & 0x03 == 1: enabling
  > &ensp;&ensp;&ensp;&ensp; (status >> 2) & 0x03 == 2: enabled
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_bio_gripper_g2_position()
<a id="method-get_bio_gripper_g2_position"></a>

```cpp
int get_bio_gripper_g2_position(int *pos);
```

  > Get the position (mm) of the BIO Gripper G2
  > @param pos: the pos of the BIO gripper G2
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_bio_gripper_error()
<a id="method-get_bio_gripper_error"></a>

```cpp
int get_bio_gripper_error(int *err);
```

  > Get the error code of the bio gripper
  > @param err: the result of the bio gripper error code
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### clean_bio_gripper_error()
<a id="method-clean_bio_gripper_error"></a>

```cpp
int clean_bio_gripper_error(void);
```

  > Clean the error code of the bio gripper
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_rs485_timeout()
<a id="method-set_rs485_timeout"></a>

```cpp
int set_rs485_timeout(int timeout, std::string target = "robot", std::string protocol = "modbus_rtu");
int set_rs485_timeout(int timeout, bool is_transparent_transmission);
```

  > Set the timeout of the target RS485
  > @param timeout: timeout, milliseconds
  > @param target: "robot" or "control_box"
  > &ensp;&ensp;&ensp;&ensp; robot: Robot RS485
  > &ensp;&ensp;&ensp;&ensp; control_box: ControlBox RS485
  > @param protocol: "modbus_rtu" or "transparent"
  > &ensp;&ensp;&ensp;&ensp; modbus_rtu: Modbus RTU
  > &ensp;&ensp;&ensp;&ensp; transparent: Transparent Transmission
  > @param is_transparent_transmission: whether the set timeout is the timeout of transparent transmission (only for set_tgpio_modbus_timeout)
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.11.0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_rs485_timeout()
<a id="method-get_rs485_timeout"></a>

```cpp
int get_rs485_timeout(int *timeout, std::string target = "robot", std::string protocol = "modbus_rtu");
int get_rs485_timeout(int *timeout, bool is_transparent_transmission);
```

  > Get the timeout of the target RS485
  > @param timeout: timeout, milliseconds
  > @param target: "robot" or "control_box"
  > &ensp;&ensp;&ensp;&ensp; robot: Robot RS485
  > &ensp;&ensp;&ensp;&ensp; control_box: ControlBox RS485
  > @param protocol: "modbus_rtu" or "transparent"
  > &ensp;&ensp;&ensp;&ensp; modbus_rtu: Modbus RTU
  > &ensp;&ensp;&ensp;&ensp; transparent: Transparent Transmission
  > @param is_transparent_transmission: is transparent transmission or not  (only for get_tgpio_modbus_timeout)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_rs485_baudrate()
<a id="method-set_rs485_baudrate"></a>

```cpp
int set_rs485_baudrate(int baud, std::string target = "robot");
```

  > Set the baudrate of the target RS485
  > @param baud: baudrate, 4800/9600/19200/38400/57600/115200/230400/460800/921600/1000000/1500000/2000000/2500000
  > @param target: "robot" or "control_box"
  > &ensp;&ensp;&ensp;&ensp; robot: Robot RS485
  > &ensp;&ensp;&ensp;&ensp; control_box: ControlBox RS485
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_rs485_baudrate()
<a id="method-get_rs485_baudrate"></a>

```cpp
int get_rs485_baudrate(int *baud, std::string target = "robot");
```

  > Get the baudrate of the target RS485
  > @param baud: the result of baudrate
  > @param target: "robot" or "control_box"
  > &ensp;&ensp;&ensp;&ensp; robot: Robot RS485
  > &ensp;&ensp;&ensp;&ensp; control_box: ControlBox RS485
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_rs485_data()
<a id="method-set_rs485_data"></a>

```cpp
int set_rs485_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, std::string target = "robot", std::string protocol = "modbus_rtu", bool use_503_port = false);
int set_rs485_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, unsigned char host_id, bool is_transparent_transmission = false, bool use_503_port = false);
```

  > Send the modbus data to the Robot RS485
  > @param modbus_data: send data
  > @param modbus_length: the length of the modbus_data
  > @param ret_data: the response data of the modbus
  > @param ret_length: the length of the response data
  > @param target: "robot" or "control_box"
  > &ensp;&ensp;&ensp;&ensp; robot: Robot RS485
  > &ensp;&ensp;&ensp;&ensp; control_box: ControlBox RS485
  > @param protocol: "modbus_rtu" or "transparent"
  > &ensp;&ensp;&ensp;&ensp; modbus_rtu: Modbus RTU
  > &ensp;&ensp;&ensp;&ensp; transparent: Transparent Transmission
  > @param use_503_port: whether to use port 503 for communication, default is false
  > &ensp;&ensp;&ensp;&ensp; Note: if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.11.0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### getset_tgpio_modbus_data()
<a id="method-getset_tgpio_modbus_data"></a>

```cpp
int getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, unsigned char host_id = UXBUS_CONF::ROBOT_RS485_HOST_ID, bool is_transparent_transmission = false, bool use_503_port = false);
```

  > Send the modbus data to the RS485
  > @param modbus_data: send data
  > @param modbus_length: the length of the modbus_data
  > @param ret_data: the response data of the modbus
  > @param ret_length: the length of the response data
  > @param host_id: host id, default is 9
  > &ensp;&ensp;&ensp;&ensp;  9: Robot RS485
  > &ensp;&ensp;&ensp;&ensp; 11: ControlBox RS485
  > @param is_transparent_transmission: whether to choose transparent transmission, default is false
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.11.0
  > @param use_503_port: whether to use port 503 for communication, default is false
  > &ensp;&ensp;&ensp;&ensp; Note: if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission
  > &ensp;&ensp;&ensp;&ensp; Note: only available if firmware_version >= 1.11.0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_report_tau_or_i()
<a id="method-set_report_tau_or_i"></a>

```cpp
int set_report_tau_or_i(int tau_or_i = 0);
```

  > Set the reported torque or electric current
  > @param tau_or_i:
  > &ensp;&ensp;&ensp;&ensp; 0: torque
  > &ensp;&ensp;&ensp;&ensp; 1: electric current
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_report_tau_or_i()
<a id="method-get_report_tau_or_i"></a>

```cpp
int get_report_tau_or_i(int *tau_or_i);
```

  > Get the reported torque or electric current
  > @param tau_or_i: the result of the tau_or_i
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_self_collision_detection()
<a id="method-set_self_collision_detection"></a>

```cpp
int set_self_collision_detection(bool on);
```

  > Set whether to enable self-collision detection
  > @param on: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_collision_tool_model()
<a id="method-set_collision_tool_model"></a>

```cpp
int set_collision_tool_model(int tool_type, int n = 0, ...);
```

  > Set the geometric model of the end effector for self collision detection
  > @param tool_type: the geometric model type
  > &ensp;&ensp;&ensp;&ensp; 0: No end effector, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 1: xArm Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 2: xArm Vacuum Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 3: xArm Bio Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 4: Robotiq-2F-85 Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 5: Robotiq-2F-140 Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 7: Lite Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 8: Lite Vacuum Gripper, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 9: xArm Gripper G2, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 10:	PGC-140-50 of the DH-ROBOTICS, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 11: RH56DFX-2L of the INSPIRE-ROBOTS, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 12: RH56DFX-2R of the INSPIRE-ROBOTS, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 13: xArm Bio Gripper G2, no additional parameters required
  > &ensp;&ensp;&ensp;&ensp; 21: Cylinder, need additional parameters radius, height
  > &ensp;&ensp;&ensp;&ensp;   arm->set_collision_tool_model(21, 2, radius, height)
  > &ensp;&ensp;&ensp;&ensp;   @param radius: the radius of cylinder, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param height: the height of cylinder, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param x_offset: offset in the x direction, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param y_offset: offset in the y direction, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param z_offset: offset in the z direction, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp; 22: Cuboid, need additional parameters x, y, z
  > &ensp;&ensp;&ensp;&ensp;   arm->set_collision_tool_model(22, 3, x, y, z)
  > &ensp;&ensp;&ensp;&ensp;   @param x: the length of the cuboid in the x coordinate direction, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param y: the length of the cuboid in the y coordinate direction, (unit: mm)
  > &ensp;&ensp;&ensp;&ensp;   @param z: the length of the cuboid in the z coordinate direction, (unit: mm)
  > &ensp;&ensp;&ensp;&ensp;   @param x_offset: offset in the x direction, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param y_offset: offset in the y direction, (unit: mm), (float)
  > &ensp;&ensp;&ensp;&ensp;   @param z_offset: offset in the z direction, (unit: mm), (float)
  > @param n: the count of the additional parameters
  > @param ...: additional parameters
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_simulation_robot()
<a id="method-set_simulation_robot"></a>

```cpp
int set_simulation_robot(bool on);
```

  > Set the simulation robot
  > @param on: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### vc_set_joint_velocity()
<a id="method-vc_set_joint_velocity"></a>

```cpp
int vc_set_joint_velocity(fp32 speeds[7], bool is_sync = true, fp32 duration = -1.0);
```

  > Joint velocity control, need to be set to joint velocity control mode(this.set_mode(4))
  > @param speeds: [spd_J1, spd_J2, ..., spd_J7]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of spd_J1/.../spd_J7 should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, the value of spd_J1/.../spd_J7 should be in degrees
  > @param is_sync: whether all joints accelerate and decelerate synchronously, default is true
  > @param duration: the maximum duration of the speed, over this time will automatically set the speed to 0.
  > &ensp;&ensp;&ensp;&ensp; duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
  > &ensp;&ensp;&ensp;&ensp; duration == 0: always effective, will not stop automatically
  > &ensp;&ensp;&ensp;&ensp; duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   only available if firmware_version >= 1.8.0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### vc_set_cartesian_velocity()
<a id="method-vc_set_cartesian_velocity"></a>

```cpp
int vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord = false, fp32 duration = -1.0);
```

  > Cartesian velocity control, need to be set to cartesian velocity control mode(self.set_mode(5))
  > @param speeds: [spd_x, spd_y, spd_z, spd_rx, spd_ry, spd_rz]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of spd_rx/spd_ry/spd_rz should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, the value of spd_rx/spd_ry/spd_rz should be in degrees
  > @param is_tool_coord: is tool coordinate or not, default is false
  > @param duration: the maximum duration of the speed, over this time will automatically set the speed to 0.
  > &ensp;&ensp;&ensp;&ensp; duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
  > &ensp;&ensp;&ensp;&ensp; duration == 0: always effective, will not stop automatically
  > &ensp;&ensp;&ensp;&ensp; duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0
  > &ensp;&ensp;&ensp;&ensp; Note:
  > &ensp;&ensp;&ensp;&ensp;   only available if firmware_version >= 1.8.0
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### calibrate_tcp_coordinate_offset()
<a id="method-calibrate_tcp_coordinate_offset"></a>

```cpp
int calibrate_tcp_coordinate_offset(float four_points[4][6], float ret_xyz[3]);
```

  > Four-point method to calibrate tool coordinate system position offset
  > @param four_points: a list of four teaching coordinate positions [x, y, z, roll, pitch, yaw]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
  > @param ret_xyz: the result of the calculated xyz(mm) TCP offset, [x, y, z]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### calibrate_tcp_orientation_offset()
<a id="method-calibrate_tcp_orientation_offset"></a>

```cpp
int calibrate_tcp_orientation_offset(float rpy_be[3], float rpy_bt[3], float ret_rpy[3]);
```

  > An additional teaching point to calibrate the tool coordinate system attitude offset
  > @param rpy_be: the rpy value of the teaching point without TCP offset [roll, pitch, yaw]
  > @param rpy_bt: the rpy value of the teaching point with TCP offset [roll, pitch, yaw]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
  > @param ret_rpy: the result of the calculated rpy TCP offset, [roll, pitch, yaw]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### calibrate_user_orientation_offset()
<a id="method-calibrate_user_orientation_offset"></a>

```cpp
int calibrate_user_orientation_offset(float three_points[3][6], float ret_rpy[3], int mode = 0, int trust_ind = 0);
```

  > Three-point method teaches user coordinate system posture offset
  > @param four_points: a list of teaching TCP coordinate positions [x, y, z, roll, pitch, yaw]
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
  > @param ret_rpy: the result of the calculated rpy user offset, [roll, pitch, yaw]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### calibrate_user_coordinate_offset()
<a id="method-calibrate_user_coordinate_offset"></a>

```cpp
int calibrate_user_coordinate_offset(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3]);
```

  > An additional teaching point determines the position offset of the user coordinate system.
  > @param rpy_ub: the confirmed offset of the base coordinate system in the user coordinate system [roll, pitch, yaw], which is the result of calibrate_user_orientation_offset()
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  > &ensp;&ensp;&ensp;&ensp; if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
  > @param pos_b_uorg: the position of the teaching point in the base coordinate system [x, y, z], if the arm cannot reach the target position, the user can manually input the position of the target in the base coordinate.
  > @param ret_xyz: the result of the calculated xyz user offset, [x, y, z]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_sensor_admittance_parameters()
<a id="method-set_ft_sensor_admittance_parameters"></a>

```cpp
int set_ft_sensor_admittance_parameters(int coord, int c_axis[6], float M[6], float K[6], float B[6]);
int set_ft_sensor_admittance_parameters(int coord, int c_axis[6]);
int set_ft_sensor_admittance_parameters(float M[6], float K[6], float B[6]);
```

  > Set the parameters of admittance control through the Six-axis Force Torque Sensor.
  > @param coord: task frame. 0: base frame. 1: tool frame.
  > @param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be admittance in the corresponding axis of the task frame.
  > @param M: 6d vector, mass. (kg)
  > @param K: 6d vector, stiffness coefficient.
  > @param B: 6d vector, damping coefficient. invalid.
  > &ensp;&ensp;&ensp;&ensp; Note: the value is set to 2*sqrt(M*K) in controller.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_sensor_force_parameters()
<a id="method-set_ft_sensor_force_parameters"></a>

```cpp
int set_ft_sensor_force_parameters(int coord, int c_axis[6], float f_ref[6], float limits[6], float kp[6], float ki[6], float kd[6], float xe_limit[6]);
int set_ft_sensor_force_parameters(int coord, int c_axis[6], float f_ref[6], float limits[6]);
int set_ft_sensor_force_parameters(float kp[6], float ki[6], float kd[6], float xe_limit[6]);
```

  > Set the parameters of force control through the Six-axis Force Torque Sensor.
  > @param coord: task frame. 0: base frame. 1: tool frame.
  > @param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be compliant in the corresponding axis of the task frame.
  > @param f_ref: 6d vector, the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.
  > @param limits: 6d vector, for compliant axes, these values are the maximum allowed tcp speed along/about the axis.
  > @param kp: 6d vector, proportional gain
  > @param ki: 6d vector, integral gain.
  > @param kd: 6d vector, differential gain.
  > @param xe_limit: 6d vector, for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_sensor_zero()
<a id="method-set_ft_sensor_zero"></a>

```cpp
int set_ft_sensor_zero(void);
```

  > Set the current state to the zero point of the Six-axis Force Torque Sensor
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### iden_ft_sensor_load_offset()
<a id="method-iden_ft_sensor_load_offset"></a>

```cpp
int iden_ft_sensor_load_offset(float result[10]);
```

  > Identification the tcp load and offset with the Six-axis Force Torque Sensor
  > @param result: the result of identification, [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_offset]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_sensor_load_offset()
<a id="method-set_ft_sensor_load_offset"></a>

```cpp
int set_ft_sensor_load_offset(float load_offset[10], bool association_setting_tcp_load = false, float m = 0.270, float x = -17, float y = 9, float z = 11.8);
```

  > Write the load offset parameters identified by the Six-axis Force Torque Sensor
  > @param load_offset: iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_offset])
  > @param association_setting_tcp_load: whether to convert the parameter to the corresponding tcp load and set, default is false
  > &ensp;&ensp;&ensp;&ensp; if true, the value of tcp load will be modified
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_sensor_enable()
<a id="method-set_ft_sensor_enable"></a>

```cpp
int set_ft_sensor_enable(int on_off);
```

  > Used for enabling and disabling the use of the Six-axis Force Torque Sensor measurements in the controller.
  > @param on_off: enable or disable F/T data sampling.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_sensor_mode()
<a id="method-set_ft_sensor_mode"></a>

```cpp
int set_ft_sensor_mode(int mode);
```

  > Set robot to be controlled in force mode. (Through the Six-axis Force Torque Sensor)
  > @param mode: force mode.
  > &ensp;&ensp;&ensp;&ensp; 0: non-force mode
  > &ensp;&ensp;&ensp;&ensp; 1: admittance control
  > &ensp;&ensp;&ensp;&ensp; 2: force control
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_sensor_mode()
<a id="method-get_ft_sensor_mode"></a>

```cpp
int get_ft_sensor_mode(int *mode);
```

  > Get force mode
  > @param mode: the result of force mode.
  > &ensp;&ensp;&ensp;&ensp; 0: non-force mode
  > &ensp;&ensp;&ensp;&ensp; 1: admittance control
  > &ensp;&ensp;&ensp;&ensp; 2: force control
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_sensor_data()
<a id="method-get_ft_sensor_data"></a>

```cpp
int get_ft_sensor_data(float ft_data[6], bool is_raw = false);
```

  > Get the data of the Six-axis Force Torque Sensor
  > @param ft_data: the result of the Six-axis Force Torque Sensor.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_sensor_config()
<a id="method-get_ft_sensor_config"></a>

```cpp
int get_ft_sensor_config(int *ft_mode = nullptr, int *ft_is_started = nullptr, int *ft_type = nullptr, int *ft_id = nullptr, int *ft_freq = nullptr,;
```

  > Get the config of the Six-axis Force Torque Sensor
  > @param ft_mode: force mode
  > &ensp;&ensp;&ensp;&ensp; 0: non-force mode
  > &ensp;&ensp;&ensp;&ensp; 1: admittance control
  > &ensp;&ensp;&ensp;&ensp; 2: force control
  > @param ft_is_started: ft sensor is enable or not
  > @param ft_type: ft sensor type
  > @param ft_id: ft sensor id
  > @param ft_freq: ft sensor frequency
  > @param ft_mass: load mass
  > @param ft_dir_bias:
  > @param ft_centroid: [x_centroid，y_centroid，z_centroid]
  > @param ft_zero: [Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_offset]
  > @param imp_coord: task frame of admittance control mode.
  > &ensp;&ensp;&ensp;&ensp; 0: base frame.
  > &ensp;&ensp;&ensp;&ensp; 1: tool frame.
  > @param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be admittance in the corresponding axis of the task frame.
  > @param M: mass. (kg)
  > @param K: stiffness coefficient.
  > @param B: damping coefficient. invalid.   Note: the value is set to 2*sqrt(M*K) in controller.
  > @param f_coord: task frame of force control mode.
  > &ensp;&ensp;&ensp;&ensp; 0: base frame.
  > &ensp;&ensp;&ensp;&ensp; 1: tool frame.
  > @param f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be compliant in the corresponding axis of the task frame.
  > @param f_ref: the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.
  > @param f_limits: for compliant axes, these values are the maximum allowed tcp speed along/about the axis.
  > @param kp: proportional gain
  > @param ki: integral gain.
  > @param kd: differential gain.
  > @param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_sensor_error()
<a id="method-get_ft_sensor_error"></a>

```cpp
int get_ft_sensor_error(int *err);
```

  > Get the error code of the Six-axis Force Torque Sensor
  > @param err: the result of ft sensor error code
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### iden_tcp_load()
<a id="method-iden_tcp_load"></a>

```cpp
int iden_tcp_load(float result[4], float estimated_mass = 0.0);
```

  > Identification the tcp load with current
  > @param result: the result of identification. [mass，x_centroid，y_centroid，z_centroid]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_registers()
<a id="method-get_linear_motor_registers"></a>

```cpp
int get_linear_motor_registers(LinearMotorStatus *status = nullptr, int addr = 0x0A20, int number_of_registers = 8);
```

  > Get all status of the linear motor
  > @param status: the result of linear motor status
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_pos()
<a id="method-get_linear_motor_pos"></a>

```cpp
int get_linear_motor_pos(int *pos);
```

  > Get the pos of the linear motor
  > @param pos: the result of linear motor position
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_status()
<a id="method-get_linear_motor_status"></a>

```cpp
int get_linear_motor_status(int *status);
```

  > Get the motion status of the linear motor
  > @param status: the result of linear motor status
  > &ensp;&ensp;&ensp;&ensp; status & 0x00: motion finish.
  > &ensp;&ensp;&ensp;&ensp; status & 0x01: in motion
  > &ensp;&ensp;&ensp;&ensp; status & 0x02: has stop
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_error()
<a id="method-get_linear_motor_error"></a>

```cpp
int get_linear_motor_error(int *err);
```

  > Get the error code of the linear motor
  > @param err: the result of linear motor error
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_is_enabled()
<a id="method-get_linear_motor_is_enabled"></a>

```cpp
int get_linear_motor_is_enabled(int *status);
```

  > Get the linear motor is enabled or not
  > @param status: the result of linear motor status
  > &ensp;&ensp;&ensp;&ensp; status == 0: linear motor is not enabled
  > &ensp;&ensp;&ensp;&ensp; status == 1: linear motor is enabled
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_on_zero()
<a id="method-get_linear_motor_on_zero"></a>

```cpp
int get_linear_motor_on_zero(int *status);
```

  > Get the linear motor is on zero position or not
  > @param status: the result of linear motor status
  > &ensp;&ensp;&ensp;&ensp; status == 0: linear motor is not on zero
  > &ensp;&ensp;&ensp;&ensp; status == 1: linear motor is on zero
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_sci()
<a id="method-get_linear_motor_sci"></a>

```cpp
int get_linear_motor_sci(int *sci1);
```

  > Get the sci1 value of the linear motor
  > @param sci1: the result of linear motor sci1
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_motor_sco()
<a id="method-get_linear_motor_sco"></a>

```cpp
int get_linear_motor_sco(int sco[2]);
```

  > Get the sco value of the linear motor
  > @param sco: the result of linear motor sco0 and sco1
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### clean_linear_motor_error()
<a id="method-clean_linear_motor_error"></a>

```cpp
int clean_linear_motor_error(void);
```

  > Clean the linear motor error
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_linear_motor_enable()
<a id="method-set_linear_motor_enable"></a>

```cpp
int set_linear_motor_enable(bool enable);
```

  > Set the linear motor enable/disable
  > @param enable: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_linear_motor_speed()
<a id="method-set_linear_motor_speed"></a>

```cpp
int set_linear_motor_speed(int speed);
```

  > Set the speed of the linear motor
  > @param speed: Integer between 1 and 1000mm/s.
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_linear_motor_back_origin()
<a id="method-set_linear_motor_back_origin"></a>

```cpp
int set_linear_motor_back_origin(bool wait = true, bool auto_enable = true);
```

  > Set the linear motor go back to the origin position
  > @param wait: wait to motion finish or not, default is true
  > @param auto_enable: enable after back to origin or not, default is true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_linear_motor_pos()
<a id="method-set_linear_motor_pos"></a>

```cpp
int set_linear_motor_pos(int pos, int speed = 0, bool wait = true, fp32 timeout = 100, bool auto_enable = true);
```

  > Set the position of the linear motor
  > @param pos: position. Integer between 0 and 700/1000/1500.
  > &ensp;&ensp;&ensp;&ensp; If the SN of the linear motor is start with AL1300, the position range is 0~700mm.
  > &ensp;&ensp;&ensp;&ensp; If the SN of the linear motor is start with AL1301, the position range is 0~1000mm.
  > &ensp;&ensp;&ensp;&ensp; If the SN of the linear motor is start with AL1302, the position range is 0~1500mm.
  > @param speed: auto set the speed of the linear motor if the speed is changed, Integer between of 1 and 1000mm/s, default is -1(not set)
  > @param wait: wait to motion finish or not, default is true
  > @param timeout: wait timeout, seconds, default is 100s.
  > @param auto_enable: auto enable if not enabled, default is true
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_linear_motor_stop()
<a id="method-set_linear_motor_stop"></a>

```cpp
int set_linear_motor_stop(void);
```

  > Set the linear motor to stop
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_baud_checkset_enable()
<a id="method-set_baud_checkset_enable"></a>

```cpp
int set_baud_checkset_enable(bool enable);
```

  > Enable auto checkset the baudrate of the end IO board or not

### set_checkset_default_baud()
<a id="method-set_checkset_default_baud"></a>

```cpp
int set_checkset_default_baud(int type, int baud);
```

  > Set the checkset baud value
  > @param type: checkset type
  > &ensp;&ensp;&ensp;&ensp; 1: xarm gripper
  > &ensp;&ensp;&ensp;&ensp; 2: bio gripper
  > &ensp;&ensp;&ensp;&ensp; 3: robotiq gripper
  > &ensp;&ensp;&ensp;&ensp; 4: linear motor
  > @param baud: checkset baud value, less than or equal to 0 means disable checkset
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_checkset_default_baud()
<a id="method-get_checkset_default_baud"></a>

```cpp
int get_checkset_default_baud(int type, int *baud);
```

  > Get the checkset baud
  > @param type: checkset type
  > &ensp;&ensp;&ensp;&ensp; 1: xarm gripper
  > &ensp;&ensp;&ensp;&ensp; 2: bio gripper
  > &ensp;&ensp;&ensp;&ensp; 3: robotiq gripper
  > &ensp;&ensp;&ensp;&ensp; 4: linear motor
  > @param baud: checkset baud value, less than or equal to 0 means disable checkset
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cartesian_velo_continuous()
<a id="method-set_cartesian_velo_continuous"></a>

```cpp
int set_cartesian_velo_continuous(bool on_off);
```

  > Set cartesian motion velocity continuous
  > @param on_off: continuous or not, default is false
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_allow_approx_motion()
<a id="method-set_allow_approx_motion"></a>

```cpp
int set_allow_approx_motion(bool on_off);
```

  > Set allow to avoid overspeed near some singularities using approximate solutions
  > @param on_off: allow or not, default is false
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### iden_joint_friction()
<a id="method-iden_joint_friction"></a>

```cpp
int iden_joint_friction(int *result, unsigned char *sn = nullptr);
```

  > Identification the friction
  > @param result: the result of identification.
  > &ensp;&ensp;&ensp;&ensp;  0: success
  > &ensp;&ensp;&ensp;&ensp; -1: failure
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_only_check_type()
<a id="method-set_only_check_type"></a>

```cpp
int set_only_check_type(unsigned char only_check_type = 0);
```

  > Set the motion process detection type (valid for all motion interfaces of the current SDK instance)
  > @param only_check_type: Motion Detection Type
  > &ensp;&ensp;&ensp;&ensp; only_check_type == 0: Restore the original function of the motion interface, it will move, the default is 0
  > &ensp;&ensp;&ensp;&ensp; only_check_type == 1: Only check the self-collision without moving, take the actual state of the manipulator as the initial planned path, and check whether the path has self-collision (the intermediate state will be updated at this time)
  > &ensp;&ensp;&ensp;&ensp; only_check_type == 2: Only check the self-collision without moving, use the intermediate state as the starting planning path, check whether the path has self-collision (the intermediate state will be updated at this time), and restore the intermediate state to the actual state after the end
  > &ensp;&ensp;&ensp;&ensp; only_check_type == 3: Only check the self-collision without moving, use the intermediate state as the starting planning path, and check whether the path has self-collision (the intermediate state will be updated at this time)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### open_lite6_gripper()
<a id="method-open_lite6_gripper"></a>

```cpp
int open_lite6_gripper(bool sync = true);
```

  > Open the gripper of Lite6 series robotics arms
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### close_lite6_gripper()
<a id="method-close_lite6_gripper"></a>

```cpp
int close_lite6_gripper(bool sync = true);
```

  > Close the gripper of Lite6 series robotics arms
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### stop_lite6_gripper()
<a id="method-stop_lite6_gripper"></a>

```cpp
int stop_lite6_gripper(bool sync = true);
```

  > Stop the gripper of Lite6 series robotics arms
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)
  > &ensp;&ensp;&ensp;&ensp;  1. only available if firmware_version >= 2.4.101
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_dh_params()
<a id="method-get_dh_params"></a>

```cpp
int get_dh_params(fp32 dh_params[28]);
```

  > Get the DH parameters
  > @param dh_params: the result of DH parameters
  > &ensp;&ensp;&ensp;&ensp; dh_params[0:4]: DH parameters of Joint-1
  > &ensp;&ensp;&ensp;&ensp; dh_params[4:8]: DH parameters of Joint-2
  > &ensp;&ensp;&ensp;&ensp; ...
  > &ensp;&ensp;&ensp;&ensp; dh_params[24:28]: DH parameters of Joint-7
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_dh_params()
<a id="method-set_dh_params"></a>

```cpp
int set_dh_params(fp32 dh_params[28], unsigned char flag = 0);
```

  > Set the DH parameters
  > @param dh_params: DH parameters
  > @param flag:
  > &ensp;&ensp;&ensp;&ensp; 0: Use the set DH parameters, but do not write to the configuration file
  > &ensp;&ensp;&ensp;&ensp; 1: Use the set DH parameters and write to the configuration file
  > &ensp;&ensp;&ensp;&ensp; 2: Use the set DH parameters and delete the DH parameters of the configuration file
  > &ensp;&ensp;&ensp;&ensp; 3: Use the default DH parameters, but will not delete the DH parameters of the configuration file
  > &ensp;&ensp;&ensp;&ensp; 4: Use the default DH parameters and delete the DH parameters of the configuration file
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_feedback_type()
<a id="method-set_feedback_type"></a>

```cpp
int set_feedback_type(unsigned char feedback_type);
```

  > Set the feedback type
  > @param feedback_type:
  > &ensp;&ensp;&ensp;&ensp; 0: disable feedback
  > &ensp;&ensp;&ensp;&ensp; 1: feedback when the motion task starts executing
  > &ensp;&ensp;&ensp;&ensp; 2: feedback when the motion task execution ends or motion task is discarded(usually when the distance is too close to be planned)
  > &ensp;&ensp;&ensp;&ensp; 4: feedback when the non-motion task is triggered
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_linear_spd_limit_factor()
<a id="method-set_linear_spd_limit_factor"></a>

```cpp
int set_linear_spd_limit_factor(float factor);
```

  > Set linear speed limit factor (default is 1.2)
  > @param factor: speed limit factor
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_cmd_mat_history_num()
<a id="method-set_cmd_mat_history_num"></a>

```cpp
int set_cmd_mat_history_num(int num);
```

  > Set cmd mat history num
  > @param num: history num
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_fdb_mat_history_num()
<a id="method-set_fdb_mat_history_num"></a>

```cpp
int set_fdb_mat_history_num(int num);
```

  > Set fdb mat history num
  > @param num: history num
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_linear_spd_limit_factor()
<a id="method-get_linear_spd_limit_factor"></a>

```cpp
int get_linear_spd_limit_factor(float *factor);
```

  > Get linear speed limit factor
  > @param factor: speed limit factor
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_cmd_mat_history_num()
<a id="method-get_cmd_mat_history_num"></a>

```cpp
int get_cmd_mat_history_num(int *num);
```

  > Get cmd mat history num
  > @param num: history num
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_fdb_mat_history_num()
<a id="method-get_fdb_mat_history_num"></a>

```cpp
int get_fdb_mat_history_num(int *num);
```

  > Get fdb mat history num
  > @param num: history num
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_poe_status()
<a id="method-get_poe_status"></a>

```cpp
int get_poe_status(int *status);
```

  > Get poe status
  > @param status: poe status, 1 means poe valid, 0 means poe invalid
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_iden_status()
<a id="method-get_iden_status"></a>

```cpp
int get_iden_status(int *status);
```

  > Get iden status
  > @param status: iden status, 1 means in identifying, 0 means not in identifying
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c31_error_info()
<a id="method-get_c31_error_info"></a>

```cpp
int get_c31_error_info(int *servo_id, float *theoretical_tau, float *actual_tau);
```

  > Get collision error (C31) info
  > @param servo_id: servo id
  > @param theoretical_tau: theoretical tau
  > @param actual_tau: actual tau
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c54_error_info()
<a id="method-get_c54_error_info"></a>

```cpp
int get_c54_error_info(int *dir, float *tau_threshold, float *actual_tau);
```

  > Get (Six-axis Force Torque Sensor) collision error (C54) info
  > @param dir: trigger direction (XYZRxRyRz)
  > @param tau_threshold: tau threshold
  > @param actual_tau: actual tau
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c37_error_info()
<a id="method-get_c37_error_info"></a>

```cpp
int get_c37_error_info(int *servo_id, float *diff_angle);
```

  > Get payload error (C37) info
  > @param servo_id: servo id
  > @param diff_angle: diff angle
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c23_error_info()
<a id="method-get_c23_error_info"></a>

```cpp
int get_c23_error_info(int *id_bits, float angles[7]);
```

  > Get joint angle limit error (C23) info
  > @param id_bits: each bit corresponds to each joint (bit0 corresponds to joint 1), and a bit of 1 indicates that the corresponding joint exceeds the limit.
  > @param angles: current angles
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c24_error_info()
<a id="method-get_c24_error_info"></a>

```cpp
int get_c24_error_info(int *servo_id, float *speed);
```

  > Get joint angle speed limit (C24) error info
  > @param servo_id: servo id
  > @param speed: current speed
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c60_error_info()
<a id="method-get_c60_error_info"></a>

```cpp
int get_c60_error_info(float *max_velo, float *curr_velo);
```

  > Get linear angle speed limit (C60) error info
  > @param max_velo: max limit linear speed
  > @param curr_velo: current linear speed
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_c38_error_info()
<a id="method-get_c38_error_info"></a>

```cpp
int get_c38_error_info(int *id_bits, float angles[7]);
```

  > Get joint hard angle limit error (C38) info
  > @param id_bits: each bit corresponds to each joint (bit0 corresponds to joint 1), and a bit of 1 indicates that the corresponding joint exceeds the limit.
  > @param angles: current angles
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_collision_detection()
<a id="method-set_ft_collision_detection"></a>

```cpp
int set_ft_collision_detection(int on_off);
```

  > Set whether to enable collision detection with the Six-axis Force Torque Sensor
  > @param on_off: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_collision_rebound()
<a id="method-set_ft_collision_rebound"></a>

```cpp
int set_ft_collision_rebound(int on_off);
```

  > Set whether to enable collision rebound with the Six-axis Force Torque Sensor
  > @param on_off: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_collision_threshold()
<a id="method-set_ft_collision_threshold"></a>

```cpp
int set_ft_collision_threshold(float thresholds[6]);
```

  > Set the thresholds of the collision detection with the Six-axis Force Torque Sensor
  > @param thresholds: collision detection thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]
  > &ensp;&ensp;&ensp;&ensp;   x: [5, 200] (N)
  > &ensp;&ensp;&ensp;&ensp;   y: [5, 200] (N)
  > &ensp;&ensp;&ensp;&ensp;   z: [5, 200] (N)
  > &ensp;&ensp;&ensp;&ensp;   Rx: [0.1, 4] (Nm)
  > &ensp;&ensp;&ensp;&ensp;   Ry: [0.1, 4] (Nm)
  > &ensp;&ensp;&ensp;&ensp;   Rz: [0.1, 4] (Nm)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_collision_reb_distance()
<a id="method-set_ft_collision_reb_distance"></a>

```cpp
int set_ft_collision_reb_distance(float distances[6]);
```

  > Set the rebound distance of the collision rebound with the Six-axis Force Torque Sensor
  > @param distances: collision rebound distance, [x(mm), y(mm), z(mm), Rx(° or rad), Ry(° or rad), Rz(° or rad)]
  > &ensp;&ensp;&ensp;&ensp;   x: [2, 500] (mm)
  > &ensp;&ensp;&ensp;&ensp;   y: [2, 500] (mm)
  > &ensp;&ensp;&ensp;&ensp;   z: [2, 500] (mm)
  > &ensp;&ensp;&ensp;&ensp;   Rx: [0.2, 50] (°)
  > &ensp;&ensp;&ensp;&ensp;   Ry: [0.2, 50] (°)
  > &ensp;&ensp;&ensp;&ensp;   Rz: [0.2, 50] (°)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_ft_admittance_ctrl_threshold()
<a id="method-set_ft_admittance_ctrl_threshold"></a>

```cpp
int set_ft_admittance_ctrl_threshold(float thresholds[6]);
```

  > Set the reaction thresholds in each direction under the admittance control mode of the Six-axis Force Torque Sensor
  > @param thresholds: reaction thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]
  > &ensp;&ensp;&ensp;&ensp;   x: [0.1, 50] (N)
  > &ensp;&ensp;&ensp;&ensp;   y: [0.1, 50] (N)
  > &ensp;&ensp;&ensp;&ensp;   z: [0.1, 50] (N)
  > &ensp;&ensp;&ensp;&ensp;   Rx: [0.01, 2] (Nm)
  > &ensp;&ensp;&ensp;&ensp;   Ry: [0.01, 2] (Nm)
  > &ensp;&ensp;&ensp;&ensp;   Rz: [0.01, 2] (Nm)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_external_device_monitor_params()
<a id="method-set_external_device_monitor_params"></a>

```cpp
int set_external_device_monitor_params(int dev_type, int frequency);
```

  > Set the monitor params of the external device
  > @param dev_type: the type of the external device
  > &ensp;&ensp;&ensp;&ensp;   0: Turn off monitoring
  > &ensp;&ensp;&ensp;&ensp;   1: xArm Gripper
  > &ensp;&ensp;&ensp;&ensp;   2: xArm Gripper G2
  > &ensp;&ensp;&ensp;&ensp;   3: BIO Gripper G2
  > &ensp;&ensp;&ensp;&ensp;   4: Robotiq 2F-85/Robotiq 2F-140
  > @param frequency: the frequency of communication with the external device
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### set_tgpio_monitor_params()
<a id="method-set_tgpio_monitor_params"></a>

```cpp
int set_tgpio_monitor_params(int io_type, int frequency);
```

  > Set the monitor params of the TGPIO
  > @param io_type: the type of the TGPIO
  > &ensp;&ensp;&ensp;&ensp;   0: Turn off monitoring
  > &ensp;&ensp;&ensp;&ensp;   1: Turn on monitoring
  > @param frequency: the frequency of communication with the TGPIO
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_collision_detection()
<a id="method-get_ft_collision_detection"></a>

```cpp
int get_ft_collision_detection(int *on_off);
```

  > Get the collision detection with the Six-axis Force Torque Sensor is enable or not
  > @param on_off: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_collision_rebound()
<a id="method-get_ft_collision_rebound"></a>

```cpp
int get_ft_collision_rebound(int *on_off);
```

  > Get the collision rebound with the Six-axis Force Torque Sensor is enable or not
  > @param on_off: enable or not
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_collision_threshold()
<a id="method-get_ft_collision_threshold"></a>

```cpp
int get_ft_collision_threshold(float thresholds[6]);
```

  > Get the collision thresholds with the Six-axis Force Torque Sensor
  > @param thresholds: collision detection thresholds
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_collision_reb_distance()
<a id="method-get_ft_collision_reb_distance"></a>

```cpp
int get_ft_collision_reb_distance(float distances[6]);
```

  > Get the collision rebound distance with the Six-axis Force Torque Sensor
  > @param distances: rebound distance, [x(mm), y(mm), z(mm), Rx(° or rad), Ry(° or rad), Rz(° or rad)]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_ft_admittance_ctrl_threshold()
<a id="method-get_ft_admittance_ctrl_threshold"></a>

```cpp
int get_ft_admittance_ctrl_threshold(float thresholds[6]);
```

  > Get the reaction thresholds in each direction under the admittance control mode of the Six-axis Force Torque Sensor
  > @param thresholds: reaction thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_external_device_monitor_params()
<a id="method-get_external_device_monitor_params"></a>

```cpp
int get_external_device_monitor_params(int params[2]);
```

  > Get the monitor params of the external device
  > @param params: params, [dev_type, frequency]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### get_tgpio_monitor_params()
<a id="method-get_tgpio_monitor_params"></a>

```cpp
int get_tgpio_monitor_params(int params[2]);
```

  > Get the monitor params of the TGPIO
  > @param params: params, [io_type, frequency]
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### read_coil_bits()
<a id="method-read_coil_bits"></a>

```cpp
int read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits);
```

  > (Standard Modbus TCP) Read Coils (0x01)
  > @param addr: the starting address of the register to be read
  > @param quantity: number of registers
  > @param bits: store result
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### read_input_bits()
<a id="method-read_input_bits"></a>

```cpp
int read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits);
```

  > (Standard Modbus TCP) Read Discrete Inputs (0x02)
  > @param addr: the starting address of the register to be read
  > @param quantity: number of registers
  > @param bits: store result
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### read_holding_registers()
<a id="method-read_holding_registers"></a>

```cpp
int read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false);
```

  > (Standard Modbus TCP) Read Holding Registers (0x03)
  > @param addr: the starting address of the register to be read
  > @param quantity: number of registers
  > @param regs: store result
  > @param is_signed: whether to convert the read register value into a signed form
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### read_input_registers()
<a id="method-read_input_registers"></a>

```cpp
int read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false);
```

  > (Standard Modbus TCP) Read Input Registers (0x04)
  > @param addr: the starting address of the register to be read
  > @param quantity: number of registers
  > @param regs: store result
  > @param is_signed: whether to convert the read register value into a signed form
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### write_single_coil_bit()
<a id="method-write_single_coil_bit"></a>

```cpp
int write_single_coil_bit(unsigned short addr, unsigned char bit_val);
```

  > (Standard Modbus TCP) Write Single Coil (0x05)
  > @param addr: register address
  > @param bit_val: the value to write (0/1)
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### write_single_holding_register()
<a id="method-write_single_holding_register"></a>

```cpp
int write_single_holding_register(unsigned short addr, int reg_val);
```

  > (Standard Modbus TCP) Write Single Holding Register (0x06)
  > @param addr: register address
  > @param reg_val: the value to write
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### write_multiple_coil_bits()
<a id="method-write_multiple_coil_bits"></a>

```cpp
int write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits);
```

  > (Standard Modbus TCP) Write Multiple Coils (0x0F)
  > @param addr: the starting address of the register to be written
  > @param quantity: the number of registers to be written
  > @param bits: array of values to write
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### write_multiple_holding_registers()
<a id="method-write_multiple_holding_registers"></a>

```cpp
int write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs);
```

  > (Standard Modbus TCP) Write Multiple Holding Registers (0x10)
  > @param addr: the starting address of the register to be written
  > @param quantity: the number of registers to be written
  > @param regs: array of values to write
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### mask_write_holding_register()
<a id="method-mask_write_holding_register"></a>

```cpp
int mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask);
```

  > (Standard Modbus TCP) Mask Write Holding Register (0x16)
  > @param addr: register address
  > @param and_mask: mask to be AND with
  > @param or_mask: mask to be OR with
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.

### write_and_read_holding_registers()
<a id="method-write_and_read_holding_registers"></a>

```cpp
int write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed = false);
```

  > (Standard Modbus TCP) Write and Read Holding Registers (0x17)
  > @param r_addr: the starting address of the register to be read
  > @param r_quantity: number of registers to read
  > @param r_regs: store result
  > @param w_addr: the starting address of the register to be written
  > @param w_quantity: number of registers to write
  > @param w_regs: array of values to write to the register
  > @param is_signed: whether to convert the read register value into a signed form
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.
