/*
* \file xarm/wrapper/xarm_api.h
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#ifndef WRAPPER_XARM_API_H_
#define WRAPPER_XARM_API_H_

#include <iostream>
#include <functional>
#include <vector>
#include <assert.h>
#include <cmath>
#include "xarm/common/data_type.h"
#include "xarm/xarm_config.h"
#include "xarm/linux/thread.h"
#include "xarm/instruction/uxbus_cmd.h"
#include "xarm/instruction/uxbus_cmd_ser.h"
#include "xarm/instruction/uxbus_cmd_tcp.h"
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm/debug/debug_print.h"
#include "xarm/wrapper/utils.h"
#include "xarm/wrapper/timer.h"

#define DEFAULT_IS_RADIAN false
#define RAD_DEGREE 57.295779513082320876798154814105
#define TIMEOUT_10 10

class XArmAPI {
public:
    XArmAPI(const std::string &port="",
        bool is_radian=DEFAULT_IS_RADIAN,
        bool do_not_open=false,
        bool check_tcp_limit=true,
        bool check_joint_limit=true,
        bool check_cmdnum_limit=true);
    ~XArmAPI(void);
private:
    void init(void);
    void wait_stop(fp32 timeout);
    inline void _report_location_callback(void);
    inline void _report_connect_changed_callback(void);
    inline void _report_state_changed_callback(void);
    inline void _report_mode_changed_callback(void);
    inline void _report_mtable_mtbrake_changed_callback(void);
    inline void _report_error_warn_changed_callback(void);
    inline void _report_cmdnum_changed_callback(void);

public:
    u8 state;
    u8 mode;
    int cmd_num;
    fp32 *joints_torque;
    bool *motor_brake_states;
    bool *motor_enable_states;
    u8 error_code;
    u8 warn_code;
    fp32 *tcp_load;
    int collision_sensitivity;
    int teach_sensitivity;
    int device_type;
    int axis;
    int master_id;
    int slave_id;
    int motor_tid;
    int motor_fid;
    u8 version[30];
    fp32 tcp_jerk;
    fp32 rot_jerk;
    fp32 max_rot_acc;
    fp32 *tcp_speed_limit;
    fp32 *tcp_acc_limit;
    fp32 last_used_tcp_speed;
    fp32 last_used_tcp_acc;

    fp32 *angles;
    fp32 *last_used_angles;
    fp32 *joint_speed_limit;
    fp32 *joint_acc_limit;
    fp32 last_used_joint_speed;
    fp32 last_used_joint_acc;
    fp32 *position;
    fp32 *last_used_position;
    fp32 *tcp_offset;

    bool default_is_radian;
private:
    std::string port_;
    bool check_tcp_limit_;
    bool check_joint_limit_;
    bool check_cmdnum_limit_;
    pthread_t report_thread_;
    bool is_ready_;
    bool is_stop_;
    bool is_tcp_;

    int mt_brake_;
    int mt_able_;
    fp32 min_tcp_speed_;
    fp32 max_tcp_speed_;
    fp32 min_tcp_acc_;
    fp32 max_tcp_acc_;
    fp32 min_joint_speed_;
    fp32 max_joint_speed_;
    fp32 min_joint_acc_;
    fp32 max_joint_acc_;
    fp32 joint_jerk_;
    int sv3msg_[16];
    fp32 trs_msg_[5];
    fp32 p2p_msg_[5];
    fp32 rot_msg_[2];

    UxbusCmdTcp *cmd_tcp_;
    UxbusCmdSer *cmd_ser_;
    SocketPort *stream_tcp_;
    SocketPort *stream_tcp_report_;
    SerialPort *stream_ser_;
    Timer timer;

    std::vector<void(*)(const fp32*, const fp32*)> report_location_callbacks;
    std::vector<void(*)(bool, bool)> connect_changed_callbacks;
    std::vector<void(*)(u8)> state_changed_callbacks;
    std::vector<void(*)(u8)> mode_changed_callbacks;
    std::vector<void(*)(int, int)> mtable_mtbrake_changed_callbacks;
    std::vector<void(*)(u8, u8)> error_warn_changed_callbacks;
    std::vector<void(*)(int)> cmdnum_changed_callbacks;

public:
    bool has_err_warn(void);
    bool has_error(void);
    bool has_warn(void);
    bool is_connected(void);

public:
    /*
    * Connect to xArm
    * @param port: port name or the ip address
    * return:
        0: success
        -1: port is empty
        -2: tcp control connect failed
        -3: tcp report connect failed
    */
    int connect(const std::string &port="");
    void disconnect(void);
    void recv_report_data(void);
    void update(u8 *data);

    int get_version(u8 version[40]);
    int get_state(u8 *state);
    int get_cmdnum(u16 *cmdnum);
    int get_err_warn_code(u8 err_warn[2]);
    int get_position(fp32 pose[6]);
    int get_servo_angle(fp32 angles[7]);

    int motion_enable(bool enable, u8 servo_id=8);
    int set_state(u8 state);
    int set_mode(u8 mode);
    int set_servo_attach(u8 servo_id);
    int set_servo_detach(u8 servo_id);
    int clean_error(void);
    int clean_warn(void);
    int set_pause_time(fp32 sltime);
    int set_collision_sensitivity(u8 sensitivity);
    int set_teach_sensitivity(u8 sensitivity);
    int clean_conf(void);
    int save_conf(void);

    /*
    * Set the position
        MoveLine: Linear motion
        MoveArcLine: Linear arc motion with interpolation
    * @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    * @param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
    * @param speed: move speed (mm/s, rad/s)
    * @param acc: move acceleration (mm/s^2, rad/s^2)
    * @param mvtime: reserved, 0
    * @param wait: whether to wait for the arm to complete, default is False
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
    * return:
    */
    int set_position(fp32 pose[6], fp32 radius=-1, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_position(fp32 pose[6], fp32 radius=-1, bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_position(fp32 pose[6], bool wait=false, fp32 timeout=TIMEOUT_10);

    /*
    * Set the servo angle
    * @param angles: angles, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    * @param servo_id: servo id, 1~7, specify the joint ID to set
    * @param angle: servo angle, use with servo_id parameters
    * @param speed: move speed (rad/s or °/s)
        if default_is_radian is true, the value of speed should be in radians
        if default_is_radian is false, The value of speed should be in degrees
    * @param acc: move acceleration (rad/s^2 or °/s^2)
        if default_is_radian is true, the value of acc should be in radians
        if default_is_radian is false, The value of acc should be in degrees
    * @param mvtime: reserved, 0
    * @param wait: whether to wait for the arm to complete, default is False
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
    * return:
    */
    int set_servo_angle(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_servo_angle(fp32 angs[7], bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_servo_angle(u8 servo_id, fp32 angle, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_servo_angle(u8 servo_id, fp32 angle, bool wait=false, fp32 timeout=TIMEOUT_10);

    /*
    * Serbo_j motion
    * @param angles: angles, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    * @param speed: reserved, move speed (rad/s or °/s)
        if default_is_radian is true, the value of speed should be in radians
        if default_is_radian is false, The value of speed should be in degrees
    * @param acc: reserved, move acceleration (rad/s^2 or °/s^2)
        if default_is_radian is true, the value of acc should be in radians
        if default_is_radian is false, The value of acc should be in degrees
    * @param mvtime: reserved, 0
    * return:
    */
    int set_servo_angle_j(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0);

    /*
    * Move to go home (Back to zero)
    * @param speed: reserved, move speed (rad/s or °/s)
        if default_is_radian is true, the value of speed should be in radians
        if default_is_radian is false, The value of speed should be in degrees
    * @param acc: reserved, move acceleration (rad/s^2 or °/s^2)
        if default_is_radian is true, the value of acc should be in radians
        if default_is_radian is false, The value of acc should be in degrees
    * @param mvtime: reserved, 0
    * @param wait: whether to wait for the arm to complete, default is False
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
    * return:
    */
    int move_gohome(fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
    int move_gohome(bool wait=false, fp32 timeout=TIMEOUT_10);

    /*
    * Reset
    * @param wait: whether to wait for the arm to complete, default is False
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
    */
    void reset(bool wait=false, fp32 timeout=TIMEOUT_10);

    /*
    * Emergency stop
    */
    void emergency_stop(void);

    int set_tcp_offset(fp32 pose_offset[6]);
    int set_tcp_load(fp32 weight, fp32 center_of_gravity[3]);
    int set_tcp_jerk(fp32 jerk);
    int set_tcp_maxacc(fp32 acc);
    int set_joint_jerk(fp32 jerk);
    int set_joint_maxacc(fp32 acc);

    int get_inverse_kinematics(fp32 pose[6], fp32 angles[7]);
    int get_forward_kinematics(fp32 angles[7], fp32 pose[6]);
    int is_tcp_limit(fp32 pose[6], int *limit);
    int is_joint_limit(fp32 angles[7], int *limit);


    int set_gripper_enable(bool enable);
    int set_gripper_mode(u16 mode);
    int get_gripper_position(fp32 *pos);
    int set_gripper_position(fp32 pos, bool wait=false, fp32 timeout=10);
    int set_gripper_speed(fp32 speed);
    int get_gripper_err_code(u8 err_warn[2]);
    int clean_gripper_error(void);
    int get_gpio_digital(int *io1, int *io2);
    int set_gpio_digital(int ionum, int value);
    int get_gpio_analog(int ionum, fp32 *value);


    /*
    * Register the report location callback
    */
    int register_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles));

    /*
    * Register the connect status changed callback
    */
    int register_connect_changed_callback(void(*callback)(bool connected, bool reported));

    /*
    * Register the state status changed callback
    */
    int register_state_changed_callback(void(*callback)(u8 state));

    /*
    * Register the mode changed callback
    */
    int register_mode_changed_callback(void(*callback)(u8 mode));

    /*
    * Register the motor enable states or motor brake states changed callback
    */
    int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake));

    /*
    * Register the error code or warn code changed callback
    */
    int register_error_warn_changed_callback(void(*callback)(u8 err_code, u8 warn_code));

    /*
    * Register the cmdnum changed callback
    */
    int register_cmdnum_changed_callback(void(*callback)(int cmdnum));

    /*
    * Release the location report callback
    * @param callback: NULL means to release all callbacks;
    */
    int release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)=NULL);

    /*
    * Release the connect changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_connect_changed_callback(void(*callback)(bool connected, bool reported)=NULL);

    /*
    * Release the state changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_state_changed_callback(void(*callback)(u8 state)=NULL);

    /*
    * Release the mode changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_mode_changed_callback(void(*callback)(u8 mode)=NULL);

    /*
    * Release the motor enable states or motor brake states changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)=NULL);

    /*
    * Release the error warn changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_error_warn_changed_callback(void(*callback)(u8 err_code, u8 warn_code)=NULL);

    /*
    * Release the cmdnum changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_cmdnum_changed_callback(void(*callback)(int cmdnum)=NULL);
};

#endif
