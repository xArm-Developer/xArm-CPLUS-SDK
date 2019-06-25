/*
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
#include "xarm/core/common/data_type.h"
#include "xarm/core/xarm_config.h"
#include "xarm/core/linux/thread.h"
#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"
#include "xarm/core/instruction/uxbus_cmd_config.h"
#include "xarm/core/debug/debug_print.h"
#include "xarm/wrapper/common/utils.h"
#include "xarm/wrapper/common/timer.h"

#define DEFAULT_IS_RADIAN false
#define RAD_DEGREE 57.295779513082320876798154814105
#define TIMEOUT_10 10

typedef unsigned int u32;
typedef float fp32;

class XArmAPI {
public:
    /*
    * @param port: port name(such as "COM5"/"/dev/ttyUSB0") or ip-address(such as "192.168.1.185")
    *   Note: this parameter is required if parameter do_not_open is False
    * @param is_radian: set the default unit is radians or not, default is False
    * @param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.
    * @param check_tcp_limit: reversed
    * @param check_joint_limit: reversed
    * @param check_cmdnum_limit: reversed
    * @param check_robot_sn: reversed
    * @param check_is_ready: reversed
    */
    XArmAPI(const std::string &port="",
        bool is_radian=DEFAULT_IS_RADIAN,
        bool do_not_open=false,
        bool check_tcp_limit=true,
        bool check_joint_limit=true,
        bool check_cmdnum_limit=true,
        bool check_robot_sn=false,
        bool check_is_ready=true);
    ~XArmAPI(void);
private:
    void init(void);
    void _check_version(void);
    void wait_stop(fp32 timeout);
    void _update_old(unsigned char *data);
    void update(unsigned char *data);
    template<typename callable_vector, typename callable>
    inline int _register_event_callback(callable_vector&& callbacks, callable&& f);
    template<typename callable_vector, typename callable>
    inline int _release_event_callback(callable_vector&& callbacks, callable&& f);
    inline void _report_location_callback(void);
    inline void _report_connect_changed_callback(void);
    inline void _report_state_changed_callback(void);
    inline void _report_mode_changed_callback(void);
    inline void _report_mtable_mtbrake_changed_callback(void);
    inline void _report_error_warn_changed_callback(void);
    inline void _report_cmdnum_changed_callback(void);

private:
    std::string port_;
    bool check_tcp_limit_;
    bool check_joint_limit_;
    bool check_cmdnum_limit_;
    bool check_robot_sn_;
    bool check_is_ready_;
    pthread_t report_thread_;
    bool is_ready_;
    bool is_stop_;
    bool is_tcp_;
    bool is_old_protocol_;
    bool is_first_report_;
    bool is_sync_;

    int major_version_number_;
    int minor_version_number_;
    int revision_version_number_;

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

    std::vector<void(*)(const fp32*, const fp32*)> report_location_callbacks_;
    std::vector<void(*)(bool, bool)> connect_changed_callbacks_;
    std::vector<void(*)(int)> state_changed_callbacks_;
    std::vector<void(*)(int)> mode_changed_callbacks_;
    std::vector<void(*)(int, int)> mtable_mtbrake_changed_callbacks_;
    std::vector<void(*)(int, int)> error_warn_changed_callbacks_;
    std::vector<void(*)(int)> cmdnum_changed_callbacks_;

public:
    int state;
    int mode;
    int cmd_num;
    fp32 *joints_torque; // fp32[7]{servo-1, ..., servo-7}
    bool *motor_brake_states; // bool[8]{servo-1, ..., servo-7, reversed}
    bool *motor_enable_states; // bool[8]{servo-1, ..., servo-7, reversed}
    int error_code;
    int warn_code;
    fp32 *tcp_load; // fp32[4]{weight, x, y, z}
    int collision_sensitivity;
    int teach_sensitivity;
    int device_type;
    int axis;
    int master_id;
    int slave_id;
    int motor_tid;
    int motor_fid;
    unsigned char version[30];
    unsigned char sn[40];
    int *version_number;
    fp32 tcp_jerk;
    fp32 rot_jerk;
    fp32 max_rot_acc;
    fp32 *tcp_speed_limit; // fp32[2]{min, max}
    fp32 *tcp_acc_limit; // fp32[2]{min, max}
    fp32 last_used_tcp_speed;
    fp32 last_used_tcp_acc;

    fp32 *angles; // fp32[7]{servo-1, ..., servo-7}
    fp32 *last_used_angles; // fp32[7]{servo-1, ..., servo-7}
    fp32 *joint_speed_limit; // fp32[2]{min, max}
    fp32 *joint_acc_limit; // fp32[2]{min, max}
    fp32 last_used_joint_speed;
    fp32 last_used_joint_acc;
    fp32 *position; // fp32[6]{x, y, z, roll, pitch, yaw}
    fp32 *last_used_position; // fp32[6]{x, y, z, roll, pitch, yaw}
    fp32 *tcp_offset; // fp32[6]{x, y, z, roll, pitch, yaw}
    fp32 *gravity_direction; // fp32[3]{x_direction, y_direction, z_direction}

    bool default_is_radian;

public:
    /*
    * xArm has error/warn or not, only available in socket way
    */
    bool has_err_warn(void);

    /*
    * xArm has error or not, only available in socket way
    */
    bool has_error(void);

    /*
    * xArm has warn or not, only available in socket way
    */
    bool has_warn(void);

    /*
    * xArm is connected or not
    */
    bool is_connected(void);

    /*
    * xArm is reported or not, only available in socket way
    */
    bool is_reported(void);

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

    /*
    * Disconnect
    */
    void disconnect(void);

    /*no use please*/
    void recv_report_data(void);

    /*
    * Get the xArm version
    */
    int get_version(unsigned char version[40]);

    /*
    * Get the xArm sn
    */
    int get_robot_sn(unsigned char robot_sn[40]);

    /*
    * Get the xArm state
    * @param: the state of xArm
        1: in motion
        2: sleeping
        3: suspended
        4: stopping
    */
    int get_state(int *state);

    /*
    * Shutdown the xArm controller system
    * @param value:
        1: remote shutdown
    */
    int shutdown_system(int value=1);

    /*
    * Get the cmd count in cache
    */
    int get_cmdnum(int *cmdnum);

    /*
    * Get the controller error and warn code
    */
    int get_err_warn_code(int err_warn[2]);

    /*
    * Get the cartesian position
    * @param pose: the position of xArm, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    */
    int get_position(fp32 pose[6]);

    /*
    * Get the servo angle
    * @param angles: the angles of the servos, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    */
    int get_servo_angle(fp32 angles[7]);

    /*
    * Motion enable
    * @param enable: enable or not
    * @param servo_id: servo id, 1-8, 8(enable/disable all servo)
    */
    int motion_enable(bool enable, int servo_id=8);

    /*
    * Set the xArm state
    * @param state: state
        0: sport state
        3: pause state
        4: stop state
    */
    int set_state(int state);

    /*
    * Set the xArm mode
    * @param mode: mode
        0: position control mode
        1: servo motion mode
        2: joint teaching mode
        3: cartesian teaching mode (invalid)
    */
    int set_mode(int mode);

    /*
    * Attach the servo
    * @param servo_id: servo id, 1-8, 8(attach all servo)
    */
    int set_servo_attach(int servo_id);

    /*
    * Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.
    * @param servo_id: servo id, 1-8, 8(detach all servo)
    */
    int set_servo_detach(int servo_id);

    /*
    * Clean the controller error, need to be manually enabled motion and set state after clean error
    */
    int clean_error(void);

    /*
    * Clean the controller warn
    */
    int clean_warn(void);

    /*
    * Set the arm pause time, xArm will pause sltime second
    * @param sltime: sleep second
    */
    int set_pause_time(fp32 sltime);

    /*
    * Set the sensitivity of collision
    * @param sensitivity: sensitivity value, 0~255
    */
    int set_collision_sensitivity(int sensitivity);

    /*
    * Set the sensitivity of drag and teach
    * @param sensitivity: sensitivity value, 0~255
    */
    int set_teach_sensitivity(int sensitivity);

    /*
    * Set the direction of gravity
    * @param gravity_dir: direction of gravity, such as [x(mm), y(mm), z(mm)]
    */
    int set_gravity_direction(fp32 gravity_dir[3]);

    /*
    * Clean current config and restore system default settings
    * Note:
        1. This interface will clear the current settings and restore to the original settings (system default settings)
    */
    int clean_conf(void);

    /*
    * Save config
    * Note:
        1. This interface can record the current settings and will not be lost after the restart.
        2. The clean_conf interface can restore system default settings
    */
    int save_conf(void);

    /*
    * Set the position
        MoveLine: Linear motion
        MoveArcLine: Linear arc motion with interpolation
    * @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    * @param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
    * @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
    * @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
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
    * @param speed: move speed (rad/s or °/s), default is this.last_used_joint_speed
        if default_is_radian is true, the value of speed should be in radians
        if default_is_radian is false, The value of speed should be in degrees
    * @param acc: move acceleration (rad/s^2 or °/s^2), default is this.last_used_joint_acc
        if default_is_radian is true, the value of acc should be in radians
        if default_is_radian is false, The value of acc should be in degrees
    * @param mvtime: reserved, 0
    * @param wait: whether to wait for the arm to complete, default is False
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
    * return:
    */
    int set_servo_angle(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_servo_angle(fp32 angles[7], bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_servo_angle(int servo_id, fp32 angle, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
    int set_servo_angle(int servo_id, fp32 angle, bool wait=false, fp32 timeout=TIMEOUT_10);

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
    * The motion calculates the trajectory of the space circle according to the three-point coordinates.
      The three-point coordinates are (current starting point, pose1, pose2).
    * @param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    * @param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    * @param percent: the percentage of arc length and circumference of the movement
    * @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
    * @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
    * @param mvtime: 0, reserved
    * @param wait: whether to wait for the arm to complete, default is False
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
    */
    int move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);

    /*
    * Move to go home (Back to zero)
    * @param speed: move speed (rad/s or °/s), default is 50 °/s
        if default_is_radian is true, the value of speed should be in radians
        if default_is_radian is false, The value of speed should be in degrees
    * @param acc: move acceleration (rad/s^2 or °/s^2), default is 1000 °/s^2
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

    /*
    * Set the tool coordinate system offset at the end
    * @param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    */
    int set_tcp_offset(fp32 pose_offset[6]);

    /*
    * Set the load
    * @param weight: load weight (unit: kg)
    * @param center_of_gravity: tcp load center of gravity, like [x(mm), y(mm), z(mm)]
    */
    int set_tcp_load(fp32 weight, fp32 center_of_gravity[3]);

    /*
    * Set the translational jerk of Cartesian space
    * @param jerk: jerk (mm/s^3)
    */
    int set_tcp_jerk(fp32 jerk);

    /*
    * Set the max translational acceleration of Cartesian space
    * @param acc: max acceleration (mm/s^2)
    */
    int set_tcp_maxacc(fp32 acc);

    /*
    * Set the jerk of Joint space
    * @param jerk: jerk (°/s^3 or rad/s^3)
        if default_is_radian is true, the value of jerk should be in radians
        if default_is_radian is false, The value of jerk should be in degrees
    */
    int set_joint_jerk(fp32 jerk);

    /*
    * Set the max acceleration of Joint space
    * @param acc: max acceleration (°/s^2 or rad/s^2)
        if default_is_radian is true, the value of jerk should be in radians
        if default_is_radian is false, The value of jerk should be in degrees
    */
    int set_joint_maxacc(fp32 acc);

    /*
    * Get inverse kinematics
    * @param pose: source pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    * @param angles: target angles, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    */
    int get_inverse_kinematics(fp32 pose[6], fp32 angles[7]);

    /*
    * Get forward kinematics
    * @param angles: source angles, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    * @param pose: target pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    */
    int get_forward_kinematics(fp32 angles[7], fp32 pose[6]);

    /*
    * Check the tcp pose is in limit
    * @param pose: pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        if default_is_radian is true, the value of roll/pitch/yaw should be in radians
        if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
    * @param limit: 1: limit, 0: no limit
    */
    int is_tcp_limit(fp32 pose[6], int *limit);

    /*
    * Check the joint is in limit
    * @param angles: angles, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    * @param limit: 1: limit, 0: no limit
    */
    int is_joint_limit(fp32 angles[7], int *limit);

    /*
    * Set the gripper enable
    * @param enable: enable or not
    */
    int set_gripper_enable(bool enable);

    /*
    * Set the gripper mode
    * @param mode: 1: location mode, 2: speed mode(no use), 3: torque mode(no use)
    */
    int set_gripper_mode(int mode);

    /*
    * Get the gripper position
    * @param pos: used to store the results obtained
    */
    int get_gripper_position(fp32 *pos);

    /*
    * Set the gripper position
    * @param pos: gripper position
    * @param wait: wait or not, default is false
    * @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
    */
    int set_gripper_position(fp32 pos, bool wait=false, fp32 timeout=10);

    /*
    * Set the gripper speed
    * @param speed:
    */
    int set_gripper_speed(fp32 speed);

    /*
    * Get the gripper position
    * @param err_wan: used to store the results obtained
    */
    int get_gripper_err_code(int *err);

    /*
    * Clean the gripper error
    */
    int clean_gripper_error(void);

    /*
    * Get the digital value of the Tool GPIO
    * @param io1_value: the digital value of Tool GPIO1
    * @param io2_value: the digital value of Tool GPIO2
    */
    int get_tgpio_digital(int *io1_value, int *io2_value);

    /*
    * Set the digital value of the specified Tool GPIO
    * @param ionum: ionum, 1 or 2
    * @param value: the digital value of the specified io
    */
    int set_tgpio_digital(int ionum, int value);

    /*
    * Get the analog value of the specified Tool GPIO
    * @param ionum: ionum, 1 or 2
    * @param value: the analog value of the specified tool io
    */
    int get_tgpio_analog(int ionum, fp32 *value);

    int get_cgpio_digital(int *digitals);

    int get_cgpio_analog(int ionum, fp32 *value);

    int set_cgpio_digital(int ionum, int value);

    int set_cgpio_analog(int ionum, int value);

    int set_cgpio_digital_input_function(int ionum, int fun);

    int set_cgpio_digital_output_function(int ionum, int fun);

    int get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf);

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
    int register_state_changed_callback(void(*callback)(int state));

    /*
    * Register the mode changed callback
    */
    int register_mode_changed_callback(void(*callback)(int mode));

    /*
    * Register the motor enable states or motor brake states changed callback
    */
    int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake));

    /*
    * Register the error code or warn code changed callback
    */
    int register_error_warn_changed_callback(void(*callback)(int err_code, int warn_code));

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
    int release_state_changed_callback(void(*callback)(int state)=NULL);

    /*
    * Release the mode changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_mode_changed_callback(void(*callback)(int mode)=NULL);

    /*
    * Release the motor enable states or motor brake states changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)=NULL);

    /*
    * Release the error warn changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_error_warn_changed_callback(void(*callback)(int err_code, int warn_code)=NULL);

    /*
    * Release the cmdnum changed callback
    * @param callback: NULL means to release all callbacks for the same event
    */
    int release_cmdnum_changed_callback(void(*callback)(int cmdnum)=NULL);
};

#endif
