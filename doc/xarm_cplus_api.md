xArm-C++-SDK API Documentation

# class __XArmAPI__

__XArmAPI(const std::string &port="",
        bool is_radian=DEFAULT_IS_RADIAN,
        bool do_not_open=false,
        bool check_tcp_limit=true,
        bool check_joint_limit=true,
        bool check_cmdnum_limit=true,
        bool check_robot_sn=false,
        bool check_is_ready=false,
        bool check_is_pause=true,
        int max_callback_thread_count=10)__

```c++
:param port: ip-address(such as "192.168.1.185")
    Note: this parameter is required if parameter do_not_open is false
:param is_radian: set the default unit is radians or not, default is false
:param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.
:param check_tcp_limit: reversed, whether checking tcp limit, default is true
:param check_joint_limit: reversed, whether checking joint limit, default is true
:param check_cmdnum_limit: whether checking command num limit, default is true
:param check_robot_sn: whether checking robot sn, default is false
:param check_is_ready: reversed, check robot is ready to move or not, default is true
:param check_is_pause: check robot is pause or not, default is true
:param max_callback_thread_count: max callback thread count, default is -1
    Note: greater than 0 means the maximum number of threads that can be used to process callbacks
    Note: equal to 0 means no thread is used to process the callback
    Note: less than 0 means no limit on the number of threads used for callback
:param max_cmdnum: max cmdnum, default is 256
    Note: only available in the param `check_cmdnum_limit` is true
```

## Property

__int state__

```
xArm state

:return: 
    1: in motion
    2: sleeping
    3: suspended
    4: stopping
```

__int mode__

```
xArm mode，only available in socket way and  enable_report is True

:return: 
    0: position control mode
    1: servo motion mode
    2: joint teaching mode
    3: cartesian teaching mode (invalid)
```

__int cmd_num__
```
Number of command caches in the controller
```

__fp32 joints_torque[7]__
```
Joints torque, only available in socket way and  enable_report is True and report_type is 'rich'

:return: fp32[7]{servo-1, ..., servo-7}
```

__bool motor_brake_states[8]__
```
Motor brake state list, only available in socket way and  enable_report is True and report_type is 'rich'
Note:
    For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.

:return: bool[8]{servo-1, ..., servo-7, reversed}
```

__bool motor_enable_states[8]__
```
motor enable states, 
Motor enable state list, only available in socket way and  enable_report is True and report_type is 'rich'
Note:
    For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.

:return: bool[8]{servo-1, ..., servo-7, reversed}
```

__int error_code__
```
Controller error code. See the controller error code documentation for details.
```

__int warn_code__
```
Controller warn code. See the controller warn code documentation for details.
```

__fp32 tcp_load[4]__
```
xArm tcp load, only available in socket way and  enable_report is True and report_type is 'rich'

:return: fp32[4]{weight, x, y, z}
```

__int collision_sensitivity__
```
The sensitivity value of collision, only available in socket way and  enable_report is True and report_type is 'rich'
```

__int teach_sensitivity__
```
The sensitivity value of drag and teach, only available in socket way and  enable_report is True and report_type is 'rich'
```

__int device_type__
```
Device type, only available in socket way and  enable_report is True and report_type is 'rich'
```

__int axis__
```
Axis number, only available in socket way and enable_report is True and report_type is 'rich'
```

__unsigned char version[30]__
```
xArm version
```

__unsigned char sn[40]__
```
xArm sn
```

__int version_number[3]__
```
Frimware version number
```

__fp32 tcp_jerk__
```
tcp jerk
```

__fp32 joint_jerk__
```
joint jerk
```

__fp32 rot_jerk__
```
rot jerk
```

__fp32 max_rot_acc__
```
max rot acc
```

__fp32 tcp_speed_limit[2]__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 

:return: fp32[2]{min, max}
```

__fp32 tcp_acc_limit[2]__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 

:return: fp32[2]{min, max}
```

__fp32 last_used_tcp_speed__
```
The last used cartesion speed, default value of parameter speed of interface set_position/move_circle
```

__fp32 last_used_tcp_acc__
```
The last used cartesion acceleration, default value of parameter mvacc of interface set_position/move_circle
```

__fp32 angles[7]__
```
Servo angles

:return: fp32[7]{servo-1, ..., servo-7}
```

__fp32 last_used_angles[7]__
```
The last used servo angles, default value of parameter angle of interface set_servo_angle

:return: fp32[7]{servo-1, ..., servo-7}
```

__fp32 joint_speed_limit[2]__
```
Joint speed limit,  only available in socket way and enable_report is True and report_type is 'rich'

:return: fp32[2]{min, max}
```

__fp32 joint_acc_limit[2]__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 

:return: fp32[2]{min, max}
```

__fp32 last_used_joint_speed__
```
The last used joint speed, default value of parameter speed of interface set_servo_angle
```

__fp32 last_used_joint_acc__
```
The last used joint acceleration, default value of parameter mvacc of interface set_servo_angle
```

__fp32 position[6]__
```
Cartesion position

:return: fp32[6]{x, y, z, roll, pitch, yaw}
```

__fp32 last_used_position[6]__
```
The last used cartesion position, default value of parameter x/y/z/roll/pitch/yaw of interface set_position

:return: fp32[6]{x, y, z, roll, pitch, yaw}
```

__fp32 tcp_offset[6]__
```
Cartesion position offset, only available in socket way and enable_report is True 

:return: fp32[6]{x, y, z, roll, pitch, yaw}
```

__fp32 gravity_direction[3]__
```
gravity direction, only available in socket way and enable_report is True and report_type is 'rich'

:return: fp32[3]{x_direction, y_direction, z_direction}
```

__fp32 realtime_tcp_speed__
```
The real time speed of tcp motion, only available if version > 1.2.11
```

__fp32 realtime_joint_speeds[7]__
```
The real time speed of joint motion, only available if version > 1.2.11
```

__fp32 world_offset[6]__
```
Base coordinate offset, only available if version > 1.2.11

Note:
    1. If self.default_is_radian is True, the returned value(roll_offset/pitch_offset/yaw_offset) is in radians

:return: fp32[6]{x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)}
```

__int count__

```
Counter value
```

__fp32 temperatures[7]__

```
Motor temperature, only available if version > 1.2.11

:return: fp32[7]{motor-1-temperature, ..., motor-7-temperature}
```

__unsigned char gpio_reset_config[2]__
```
The gpio reset enable config, only available if version > 1.5.0

:return: unsigned char[2]{cgpio_reset_enable, tgpio_reset_enable}
```

__bool default_is_radian__
```
The default unit is radians or not
```

__fp32 voltages[7]__

```
Servos voltage
        
:return: fp32[7]{servo-1-voltage, ..., servo-7-voltage}
```

__fp32 currents[7]__

```
Servos electric current

:return: fp32[7]{servo-1-current, ..., servo-7-current}
```

__int is_collision_detection__

```
self collision detection or not 
```

__int collision_tool_type__

```
self collision tool type
```

__fp32 collision_model_params[6]__

```
self collision model params
```

__int iden_progress__

```
the progress of identifition
```

__fp32 ft_ext_force[6]__

```
the ext force data of ft sensor
```

__fp32 ft_raw_force[6]__

```
the raw force data of ft sensor
```


## Method

__bool has_err_warn(void)__
```
xArm has error/warn or not, only available in socket way
```

__bool has_error(void)__
```
xArm has error or not, only available in socket way
```

__bool has_warn(void)__
```
xArm has warn or not, only available in socket way
```

__bool is_connected(void)__
```
xArm is connected or not
```

__bool is_reported(void)__
```
xArm is reported or not, only available in socket way
```

__int connect(const std::string &port="")__
```
Connect to xArm

:param port: port name or the ip address
:return:
    0: success
    -1: port is empty
    -2: tcp control connect failed
    -3: tcp report connect failed
```

__void disconnect(void)__
```
Disconnect to xArm
```

__int get_version(unsigned char version[40])__
```
Get the xArm version

:param version: 
:return: see the API code documentation for details.
```

__int get_robot_sn(unsigned char robot_sn[40])__
```
Get the xArm sn

:param robot_sn:
:return: see the API code documentation for details.
```

__int get_state(int *state)__
```
Get the xArm state

:param: the state of xArm
        1: in motion
        2: sleeping
        3: suspended
        4: stopping
:return: see the API code documentation for details.
```

__int shutdown_system(int value=1)__
```
Shutdown the xArm controller system

:param value:
    1: remote shutdown
:return: see the API code documentation for details.
```

__int get_cmdnum(int *cmdnum)__
```
Get the cmd count in cache

:return: see the API code documentation for details.
```

__int get_err_warn_code(int err_warn[2])__
```
Get the controller error and warn code

:return: see the API code documentation for details.
```

__int get_position(fp32 pose[6])__
```
Get the cartesian position

:param pose: the position of xArm, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:return: see the API code documentation for details.
```

__int get_servo_angle(fp32 angles[7])__
```
Get the servo angle

:param angles: the angles of the servos, like [servo-1, ..., servo-7]
        if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
        if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
:return: see the API code documentation for details.
```

__int motion_enable(bool enable, int servo_id=8)__
```
Motion enable

:param enable: enable or not
:param servo_id: servo id, 1-8, 8(enable/disable all servo)
:return: see the API code documentation for details.
```

__int set_state(int state)__
```
Set the xArm state

:param state: state
    0: sport state
    3: pause state
    4: stop state
:return: see the API code documentation for details.
```

__int set_mode(int mode)__
```
Set the xArm mode

:param mode: mode
    0: position control mode
    1: servo motion mode
    2: joint teaching mode
    3: cartesian teaching mode (invalid)
:return: see the API code documentation for details.
```

__int set_servo_attach(int servo_id)__
```
Attach the servo

:param servo_id: servo id, 1-8, 8(attach all servo)
:return: see the API code documentation for details.
```

__int set_servo_detach(int servo_id)__
```
Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.

:param servo_id: servo id, 1-8, 8(detach all servo)
:return: see the API code documentation for details.
```

__int clean_error(void)__
```
Clean the controller error, need to be manually enabled motion and set state after clean error

:return: see the API code documentation for details.
```

__int clean_warn(void)__
```
Clean the controller warn

:return: see the API code documentation for details.
```

__int set_pause_time(fp32 sltime)__
```
Set the arm pause time, xArm will pause sltime second

:param sltime: sleep second
:return: see the API code documentation for details.
```

__int set_collision_sensitivity(int sensitivity)__
```
Set the sensitivity of collision

:param sensitivity: sensitivity value, 0~5
:return: see the API code documentation for details.
```

__int set_teach_sensitivity(int sensitivity)__
```
Set the sensitivity of drag and teach

:param sensitivity: sensitivity value, 1~5
:return: see the API code documentation for details.
```

__int set_gravity_direction(fp32 gravity_dir[3])__
```
Set the direction of gravity

:param gravity_dir: direction of gravity, such as [x(mm), y(mm), z(mm)]
:return: see the API code documentation for details.
```

__int clean_conf(void)__
```
Clean current config and restore system default settings
Note:
    1. This interface will clear the current settings and restore to the original settings (system default settings)

:return: see the API code documentation for details.  
```

__int save_conf(void)__
```
Save config
Note:
    1. This interface can record the current settings and will not be lost after the restart.
    2. The clean_conf interface can restore system default settings

:return: see the API code documentation for details.
```

__int set_position(fp32 pose[6], fp32 radius=-1, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=NO_TIMEOUT)__
__int set_position(fp32 pose[6], fp32 radius=-1, bool wait=false, fp32 timeout=NO_TIMEOUT)__
__int set_position(fp32 pose[6], bool wait=false, fp32 timeout=NO_TIMEOUT)__
```
Set the position
    MoveLine: Linear motion
    MoveArcLine: Linear arc motion with interpolation

:param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
:param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
:param mvtime: reserved, 0
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true

:return: see the API code documentation for details.   
```

__int set_tool_position(fp32 pose[6], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=NO_TIMEOUT)__
__int set_tool_position(fp32 pose[6], bool wait=false, fp32 timeout=NO_TIMEOUT)__
```
Movement relative to the tool coordinate system 

:param pose: the coordinate relative to the current tool coordinate systemion, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
:param mvtime: reserved, 0
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true

:return: see the API code documentation for details.
```

__int set_servo_angle(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=NO_TIMEOUT, fp32 radius = -1)__
__int set_servo_angle(fp32 angles[7], bool wait=false, fp32 timeout=NO_TIMEOUT, fp32 radius = -1)__
__int set_servo_angle(int servo_id, fp32 angle, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=NO_TIMEOUT, fp32 radius = -1)__
__int set_servo_angle(int servo_id, fp32 angle, bool wait=false, fp32 timeout=NO_TIMEOUT, fp32 radius = -1)__
```
Set the servo angle

:param angles: angles, like [servo-1, ..., servo-7]
    if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
    if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
:param servo_id: servo id, 1~7, specify the joint ID to set
:param angle: servo angle, use with servo_id parameters
:param speed: move speed (rad/s or °/s), default is this.last_used_joint_speed
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:param acc: move acceleration (rad/s^2 or °/s^2), default is this.last_used_joint_acc
    if default_is_radian is true, the value of acc should be in radians
    if default_is_radian is false, The value of acc should be in degrees
:param mvtime: reserved, 0
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
:param radius: move radius, if radius less than 0, will MoveJoint, else MoveArcJoint
    The blending radius cannot be greater than the track length.

:return: see the API code documentation for details.
```

__int set_servo_angle_j(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0)__
```
Set the servo angle, execute only the last instruction, need to be set to servo motion mode(this.set_mode(1))

:param angles: angles, like [servo-1, ..., servo-7]
    if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
    if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
:param speed: reserved, move speed (rad/s or °/s)
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:param acc: reserved, move acceleration (rad/s^2 or °/s^2)
    if default_is_radian is true, the value of acc should be in radians
    if default_is_radian is false, The value of acc should be in degrees
:param mvtime: reserved, 0
    
:return: see the API code documentation for details.
```

__int set_servo_cartesian(fp32 pose[6], fp32 speed=0, fp32 acc=0, fp32 mvtime=0,  bool is_tool_coord = false)__
```
Servo cartesian motion, execute only the last instruction, need to be set to servo motion mode(this.set_mode(1))

:param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param speed: reserved, move speed (mm/s)
:param mvacc: reserved, move acceleration (mm/s^2)
:param mvtime: reserved, 0
:param is_tool_coord: is tool coordinate or not
    
:return: see the API code documentation for details.
```

__int move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=NO_TIMEOUT)__
```
The motion calculates the trajectory of the space circle according to the three-point coordinates.
Note:
    The three-point coordinates are (current starting point, pose1, pose2).

:param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param percent: the percentage of arc length and circumference of the movement
:param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
:param mvtime: 0, reserved
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is True

:return: see the API code documentation for details.
```

__int move_gohome(fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=NO_TIMEOUT)__
__int move_gohome(bool wait=false, fp32 timeout=NO_TIMEOUT)__
```
Move to go home (Back to zero)

:param speed: move speed (rad/s or °/s), default is 50 °/s
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:param acc: move acceleration (rad/s^2 or °/s^2), default is 1000 °/s^2
    if default_is_radian is true, the value of acc should be in radians
    if default_is_radian is false, The value of acc should be in degrees
:param mvtime: reserved, 0
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true

:return: see the API code documentation for details.
```

__void reset(bool wait=false, fp32 timeout=NO_TIMEOUT)__
```
Reset

:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true
```

__void emergency_stop(void)__
```
Emergency stop
```

__int set_tcp_offset(fp32 pose_offset[6])__
```
Set the tool coordinate system offset at the end

:param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees

:return: see the API code documentation for details.  
```

__int set_tcp_load(fp32 weight, fp32 center_of_gravity[3])__
```
Set the tcp load

:param weight: load weight (unit: kg)
:param center_of_gravity: tcp load center of gravity, like [x(mm), y(mm), z(mm)]
:return: see the API code documentation for details.  
```

__int set_tcp_jerk(fp32 jerk)__
```
Set the translational jerk of Cartesian space

:param jerk: jerk (mm/s^3)
:return: see the API code documentation for details.
```

__int set_tcp_maxacc(fp32 acc)__
```
Set the max translational acceleration of Cartesian space

:param acc: max acceleration (mm/s^2)
:return: see the API code documentation for details.
```

__int set_joint_jerk(fp32 jerk)__
```
Set the jerk of Joint space

:param jerk: jerk (°/s^3 or rad/s^3)
    if default_is_radian is true, the value of jerk should be in radians
    if default_is_radian is false, The value of jerk should be in degrees
:return: see the API code documentation for details.
```

__int set_joint_maxacc(fp32 acc)__
```
Set the max acceleration of Joint space

:param acc: max acceleration (°/s^2 or rad/s^2)
    if default_is_radian is true, the value of jerk should be in radians
    if default_is_radian is false, The value of jerk should be in degrees
:return: see the API code documentation for details.
```

__int get_inverse_kinematics(fp32 pose[6], fp32 angles[7])__
```
Get inverse kinematics

:param pose: source pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param angles: target angles, like [servo-1, ..., servo-7]
    if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
    if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
    
:return: see the API code documentation for details.
```

__int get_forward_kinematics(fp32 angles[7], fp32 pose[6])__
```
Get forward kinematics

:param angles: source angles, like [servo-1, ..., servo-7]
    if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
    if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
:param pose: target pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees

:return: see the API code documentation for details.
```

__int is_tcp_limit(fp32 pose[6], int *limit)__
```
Check the tcp pose is in limit

:param pose: pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
:param limit: 1: limit, 0: no limit

:return: see the API code documentation for details.
```

__int is_joint_limit(fp32 angles[7], int *limit)__
```
Check the joint is in limit

:param angles: angles, like [servo-1, ..., servo-7]
    if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
    if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
:param limit: 1: limit, 0: no limit

:return: see the API code documentation for details.
```

__int set_gripper_enable(bool enable)__
```
Set the gripper enable

:param enable: enable or not
    :return: see the API code documentation for details.    
```

__int set_gripper_mode(int mode)__
```
Set the gripper mode

:param mode: 1: location mode, 2: speed mode(no use), 3: torque mode(no use)
:return: see the API code documentation for details.
```

__int get_gripper_position(fp32 *pos)__
```
Get the gripper position

:param pos: used to store the results obtained
:return: see the API code documentation for details.
```

__int set_gripper_position(fp32 pos, bool wait=false, fp32 timeout=10, bool wait_motion = true)__
```
Set the gripper position

:param pos: gripper position
:param wait: wait or not, default is false
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true

:return: see the API code documentation for details.
```

__int set_gripper_speed(fp32 speed)__
```
Set the gripper speed

:param speed:
:return: see the API code documentation for details.
```

__int get_gripper_err_code(int *err)__
```
Get the gripper error code

:param err: used to store the results obtained
:return: see the API code documentation for details.
```

__int clean_gripper_error(void)__
```
Clean the gripper error

:return: see the API code documentation for details.
```

__int get_tgpio_digital(int *io0_value, int *io1_value)__
```
Get the digital value of the Tool GPIO

:param io0_value: the digital value of Tool GPIO-0
:param io1_value: the digital value of Tool GPIO-1
:return: see the API code documentation for details.
```

__int set_tgpio_digital(int ionum, int value, float delay_sec=0)__
```
Set the digital value of the specified Tool GPIO

:param ionum: ionum, 0 or 1
:param value: the digital value of the specified io
:param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)
:return: see the API code documentation for details.
```

__int get_tgpio_analog(int ionum, fp32 *value)__
```
Get the analog value of the specified Tool GPIO

:param ionum: ionum, 0 or 1
:param value: the analog value of the specified tool io
:return: see the API code documentation for details.
```

__int get_cgpio_digital(int *digitals)__
```
Get the digital value of the specified Controller GPIO

:param digitals: the values of the controller GPIO
:return: see the API code documentation for details.
```

__int get_cgpio_analog(int ionum, fp32 *value)__
```
Get the analog value of the specified Controller GPIO

:param ionum: ionum, 0 or 1
:param value: the analog value of the specified controller io
:return: see the API code documentation for details.
```

__int set_cgpio_digital(int ionum, int value, float delay_sec=0)__
```
Set the digital value of the specified Controller GPIO

:param ionum: ionum, 0 ~ 7
:param value: the digital value of the specified io
:param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)
:return: see the API code documentation for details.
```

__int set_cgpio_analog(int ionum, float value)__

```
Set the analog value of the specified Controller GPIO

:param ionum: ionum, 0 or 1
:param value: the analog value of the specified io
:return: see the API code documentation for details.
```

__int set_cgpio_digital_input_function(int ionum, int fun)__
```
Set the digital input functional mode of the Controller GPIO

:param ionum: ionum, 0 ~ 7
:param fun: functional mode
    0: general input
    1: external emergency stop
    2: reversed, protection reset
    3: reversed, reduced mode
    4: reversed, operating mode
    5: reversed, three-state switching signal
    11: offline task
    12: teaching mode

:return: see the API code documentation for details.
```

__int set_cgpio_digital_output_function(int ionum, int fun)__
```
Set the digital output functional mode of the specified Controller GPIO

:param ionum: ionum, 0 ~ 7
:param fun: functional mode
    0: general output
    1: emergency stop
    2: in motion
    11: has error
    12: has warn
    13: in collision
    14: in teaching
    15: in offline task

:return: see the API code documentation for details.
```

__int get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf)__
```
 Get the state of the Controller GPIO

:param state: contorller gpio module state and controller gpio module error code
    state[0]: contorller gpio module state
    state[0] == 0: normal
    state[0] == 1：wrong
    state[0] == 6：communication failure
    state[1]: controller gpio module error code
    state[1] == 0: normal
    state[1] != 0：error code
:param digit_io:
    digit_io[0]: digital input functional gpio state
    digit_io[1]: digital input configuring gpio state
    digit_io[2]: digital output functional gpio state
    digit_io[3]: digital output configuring gpio state
:param analog:
    analog[0]: analog-0 input value
    analog[1]: analog-1 input value
    analog[2]: analog-0 output value
    analog[3]: analog-1 output value
:param input_conf: digital input functional info
:param output_conf: digital output functional info

:return: see the API code documentation for details.
```

__int register_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles))__
```
Register the report location callback
```

__int register_connect_changed_callback(void(*callback)(bool connected, bool reported))__
```
Register the connect status changed callback
```

__int register_state_changed_callback(void(*callback)(int state))__
```
Register the state status changed callback
```

__int register_mode_changed_callback(void(*callback)(int mode))__
```
Register the mode changed callback
```

__int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake))__
```
Register the motor enable states or motor brake states changed callback
```

__int register_error_warn_changed_callback(void(*callback)(int err_code, int warn_code))__
```
Register the error code or warn code changed callback
```

__int register_cmdnum_changed_callback(void(*callback)(int cmdnum))__
```
Register the cmdnum changed callback
```

__int register_temperature_changed_callback(void(*callback)(const fp32 *temps))__
```
Register the temperature changed callback
```

__int register_count_changed_callback(void(*callback)(int count))__
```
Register the value of counter changed callback
```

__int register_iden_progress_changed_callback(void(*callback)(int progress))__
```
Register the progress of identification changed callback
```

__int release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)=NULL)__
```
Release the location report callback

:param callback: NULL means to release all callbacks;
```

__int release_connect_changed_callback(void(*callback)(bool connected, bool reported)=NULL)__
```
Release the connect changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_state_changed_callback(void(*callback)(int state)=NULL)__
```
Release the state changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_mode_changed_callback(void(*callback)(int mode)=NULL)__
```
Release the mode changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)=NULL)__
```
Release the motor enable states or motor brake states changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_error_warn_changed_callback(void(*callback)(int err_code, int warn_code)=NULL)__
```
Release the error warn changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_cmdnum_changed_callback(void(*callback)(int cmdnum)=NULL)__
```
Release the cmdnum changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_temperature_changed_callback(void(*callback)(const fp32 *temps)=NULL)__
```
Release the temperature changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_count_changed_callback(void(*callback)(int count)=NULL)__
```
Release the value of counter changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int release_iden_progress_changed_callback(void(*callback)(int progress) = NULL)__
```
Release the progress of identification changed callback

:param callback: NULL means to release all callbacks for the same event
```

__int get_suction_cup(int *val)__
__int get_vacuum_gripper(int *val)__

```
Get suction cup state

:param val:
    0: suction cup is off
    1: suction cup is on
:return: see the API code documentation for details.
```

__int set_suction_cup(bool on, bool wait=false, float timeout=3, float delay_sec=0)__
__int set_vacuum_gripper(bool on, bool wait=false, float timeout=3, float delay_sec=0)__
```
Set suction cup

:param on: open suction cup or not
:param wait: wait or not, default is false
:param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true
:param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)

:return: see the API code documentation for details.
```

__int get_gripper_version(unsigned char versions[3])__
```
Get gripper version, only for debug

:return: see the API code documentation for details.
```

__int get_servo_version(unsigned char versions[3], int servo_id=1)__
```
Get servo version, only for debug

:return: see the API code documentation for details.
```

__int get_tgpio_version(unsigned char versions[3])__
```
Get tool gpio version, only for debug

:return: see the API code documentation for details.
```

__int reload_dynamics(void)__
```
Reload dynamics, only for debug

:return: see the API code documentation for details.
```

__int set_reduced_mode(bool on)__
```
Turn on/off reduced mode

:param on: on/off
:return: see the API code documentation for details.
```

__int set_reduced_max_tcp_speed(float speed)__
```
Set the maximum tcp speed of the reduced mode

:param speed: the maximum tcp speed
:return: see the API code documentation for details.
```

__int set_reduced_max_joint_speed(float speed)__
```
Set the maximum joint speed of the reduced mode

:param speed: the maximum joint speed
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:return: see the API code documentation for details.
```

__int get_reduced_mode(int *mode)__
```
Get reduced mode

:param mode:
    0: reduced mode is on
    1: reduced mode is off
:return: see the API code documentation for details.
```

__int get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14]=NULL, int *fense_is_on=NULL, int *collision_rebound_is_on=NULL)__
```
Get states of the reduced mode

:param on:
    0: reduced mode is on
    1: reduced mode is off
:param xyz_list: the tcp boundary, like [reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min],
:param tcp_speed: the maximum tcp speed of reduced mode
:param joint_speed: the maximum joint speed of reduced mode
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:param jrange: the joint range of the reduced mode, like [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:param fense_is_on:
    0: safety mode is on
    1: safety mode is off
:param collision_rebound_is_on:
    0: collision rebound is on
    1: collision rebound is off

:return: see the API code documentation for details.
```

__int set_reduced_tcp_boundary(int boundary[6])__
```
Set the boundary of the safety boundary mode

:param boundary: like [x_max(mm), x_min(mm), y_max(mm), y_min(mm), z_max(mm), z_min(mm)]
:return: see the API code documentation for details.
```

__int set_reduced_joint_range(float jrange[14])__
```
Set the joint range of the reduced mode

:param jrange: like [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]
    if default_is_radian is true, the value of speed should be in radians
    if default_is_radian is false, The value of speed should be in degrees
:return: see the API code documentation for details.
```

__int set_fense_mode(bool on)__
__int set_fence_mode(bool on)__

```
Turn on/off safety mode

:param on: on/off
:return: see the API code documentation for details.
```

__int set_collision_rebound(bool on)__
```
Turn on/off collision rebound

:param on: on/off
:return: see the API code documentation for details.
```

__int set_world_offset(float pose_offset[6])__
```
Set the base coordinate system offset at the end

:param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, The value of roll/pitch/yaw should be in degrees

:return: see the API code documentation for details.
```

__int start_record_trajectory(void)__
```
Start trajectory recording, only in teach mode, so you need to set joint teaching mode before.

:return: see the API code documentation for details.
```

__int stop_record_trajectory(char* filename=NULL)__
```
Stop trajectory recording

:param filename: the name to save
    If the filename is NULL, just stop recording, do not save, you need to manually call `save_record_trajectory` save before changing the mode. otherwise it will be lost    
    the trajectory is saved in the controller box.
    this action will overwrite the trajectory with the same name
    empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.

:return: see the API code documentation for details.
```

__int save_record_trajectory(char* filename, float timeout=10)__
```
Save the trajectory you just recorded

:param filename: the name to save
    the trajectory is saved in the controller box.
    this action will overwrite the trajectory with the same name
    empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory. 

:return: see the API code documentation for details.
```

__int load_trajectory(char* filename, float timeout=10)__
```
Load the trajectory

:param filename: the name of the trajectory to load
:param timeout: the maximum timeout waiting for loading to complete, default is 10 seconds.

:return: see the API code documentation for details.
```

__int playback_trajectory(int times=1, char* filename=NULL, bool wait=false, int double_speed=1)__
```
Playback trajectory

:param times: number of playbacks.
:param filename: the name of the trajectory to play back
    if filename is None, you need to manually call the `load_trajectory` to load the trajectory.
:param wait: whether to wait for the arm to complete, default is False.
:param double_speed: double speed, only support 1/2/4, default is 1, only available if version > 1.2.11

:return: see the API code documentation for details.
```

__int get_trajectory_rw_status(int *status)__
```
Get trajectory read/write status

:param status:
    0: no read/write
    1: loading
    2: load success
    3: load failed
    4: saving
    5: save success
    6: save failed
:return: see the API code documentation for details.
```

__int set_counter_reset(void)__
```
Reset counter value

:return: see the API code documentation for details.
```

__int set_counter_increase(void)__
```
Set counter plus 1

:return: see the API code documentation for details.
```

__int set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r)__
```
Set the digital value of the specified Tool GPIO when the robot has reached the specified xyz position

:param ionum: 0 or 1 
:param value: value
:param xyz: position xyz, as [x, y, z]
:param tol_r: fault tolerance radius

:return: see the API code documentation for details.
```

__int set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r)__
```
Set the digital value of the specified Controller GPIO when the robot has reached the specified xyz position

:param ionum: 0 ~ 7
:param value: value
:param xyz: position xyz, as [x, y, z]
:param tol_r: fault tolerance radius

:return: see the API code documentation for details.
```

__int set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r)__

```
Set the analog value of the specified Controller GPIO when the robot has reached the specified xyz position

:param ionum: 0 ~ 1
:param value: value, 0~10.0
:param xyz: position xyz, as [x, y, z]
:param tol_r: fault tolerance radius

:return: see the API code documentation for details.
```

__int config_tgpio_reset_when_stop(bool on_off)__

```
Config the Tool GPIO reset the digital output when the robot is in stop state

:param on_off: true/false

:return: see the API code documentation for details.
```

__int config_cgpio_reset_when_stop(bool on_off)__
```
Config the Controller GPIO reset the digital output when the robot is in stop state

:param on_off: true/false

:return: see the API code documentation for details.
```

__int set_position_aa(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT)__
__int set_position_aa(fp32 pose[6], bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT)__
```
Set the pose represented by the axis angle pose

:param pose: the axis angle pose, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
    Note: if default_is_radian is true, the value of rx/ry/rz should be in radians
    Note: if default_is_radian is false, The value of rx/ry/rz should be in degrees
:param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
:param mvtime: reserved, 0
:param is_tool_coord: is tool coordinate or not
:param relative: relative move or not
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true

:return: see the API code documentation for details.
```

__int set_servo_cartesian_aa(fp32 pose[6], fp32 speed = 0, fp32 acc = 0, bool is_tool_coord = false, bool relative = false)__
__int set_servo_cartesian_aa(fp32 pose[6], bool is_tool_coord = false, bool relative = false)__
```
Set the servo cartesian represented by the axis angle pose, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))
    Note: only available if firmware_version >= 1.4.7

:param pose: the axis angle pose, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
    Note: if default_is_radian is true, the value of rx/ry/rz should be in radians
    Note: if default_is_radian is false, The value of rx/ry/rz should be in degrees
:param speed: reserved, move speed (mm/s)
:param mvacc: reserved, move acceleration (mm/s^2)
:param is_tool_coord: is tool coordinate or not
:param relative: relative move or not

:return: see the API code documentation for details.
```

__int get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in = 0, int orient_type_out = 0)__
```
Calculate the pose offset of two given points

:param pose1: position, like [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
    Note: if default_is_radian is true, the value of roll/rx/pitch/ry/yaw/rz should be in radians
    Note: if default_is_radian is false, The value of roll/rx/pitch/ry/yaw/rz should be in degrees
:param pose2: position, like [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
    Note: if default_is_radian is true, the value of roll/rx/pitch/ry/yaw/rz should be in radians
    Note: if default_is_radian is false, The value of roll/rx/pitch/ry/yaw/rz should be in degrees
:param offset: the offset between pose1 and pose2
:param orient_type_in: input attitude notation, 0 is RPY (default), 1 is axis angle
:param orient_type_out: notation of output attitude, 0 is RPY (default), 1 is axis angle

:return: see the API code documentation for details.
```

__int get_position_aa(fp32 pose[6])__
```
Get the pose represented by the axis angle pose

:param pose: the pose represented by the axis angle pose of xArm, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
    Note: if default_is_radian is true, the value of rx/ry/rz should be in radians
    Note: if default_is_radian is false, The value of rx/ry/rz should be in degrees

:return: see the API code documentation for details.
```

__int robotiq_reset(unsigned char ret_data[6] = NULL)__
```
Reset the robotiq gripper (clear previous activation if any)

:param ret_data: the response from robotiq

:return: see the API code documentation for details.
```

__int robotiq_set_activate(bool wait = true, fp32 timeout = 3, unsigned char ret_data[6] = NULL)__
__int robotiq_set_activate(bool wait = true, unsigned char ret_data[6] = NULL)__
__int robotiq_set_activate(unsigned char ret_data[6] = NULL)__
```
If not already activated. Activate the robotiq gripper

:param wait: whether to wait for the robotiq activate complete, default is true
:param timeout: maximum waiting time(unit: second), default is 3, only available if wait=true
:param ret_data: the response from robotiq

:return: see the API code documentation for details.
```

__int robotiq_set_position(unsigned char pos, unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5,  unsigned char ret_data[6] = NULL)__
__int robotiq_set_position(unsigned char pos, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL)__
__int robotiq_set_position(unsigned char pos, bool wait = true, unsigned char ret_data[6] = NULL)__
__int robotiq_set_position(unsigned char pos, unsigned char ret_data[6] = NULL)__

```
Go to the position with determined speed and force.

:param pos: position of the gripper. Integer between 0 and 255. 0 being the open position and 255 being the close position.
:param speed: gripper speed between 0 and 255
:param force: gripper force between 0 and 255
:param wait: whether to wait for the robotion motion complete, default is true
:param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
:param ret_data: the response from robotiq

:return: see the API code documentation for details.
```

__int robotiq_open(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL)__
__int robotiq_open(bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL)__
__int robotiq_open(bool wait = true, unsigned char ret_data[6] = NULL)__
__int robotiq_open(unsigned char ret_data[6] = NULL)__

```
Open the robotiq gripper

:param speed: gripper speed between 0 and 255
:param force: gripper force between 0 and 255
:param wait: whether to wait for the robotion motion complete, default is true
:param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
:param ret_data: the response from robotiq

:return: see the API code documentation for details.
```

__int robotiq_close(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL)__
__int robotiq_close(bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL)__
__int robotiq_close(bool wait = true, unsigned char ret_data[6] = NULL)__
__int robotiq_close(unsigned char ret_data[6] = NULL)__

```
Close the robotiq gripper

:param speed: gripper speed between 0 and 255
:param force: gripper force between 0 and 255
:param wait: whether to wait for the robotion motion complete, default is true
:param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
:param ret_data: the response from robotiq

:return: see the API code documentation for details.
```

__int robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers = 3)__
```
Reading the status of robotiq gripper

:param ret_data: the response from robotiq
:param number_of_registers: number of registers, 1/2/3, default is 3
    number_of_registers=1: reading the content of register 0x07D0
    number_of_registers=2: reading the content of register 0x07D0/0x07D1
    number_of_registers=3: reading the content of register 0x07D0/0x07D1/0x07D2
        
    Note:
        register 0x07D0: Register GRIPPER STATUS
        register 0x07D1: Register FAULT STATUS and register POSITION REQUEST ECHO
        register 0x07D2: Register POSITION and register CURRENT

:return: see the API code documentation for details.
```

__int set_bio_gripper_enable(bool enable, bool wait = true, fp32 timeout = 3)__

```
If not already enabled. Enable the bio gripper

:param enable: enable or not
:param wait: whether to wait for the bio gripper enable complete, default is True
:param timeout: maximum waiting time(unit: second), default is 3, only available if wait=true

:return: See the code documentation for details.
```

__int set_bio_gripper_speed(int speed)__

```
Set the speed of the bio gripper

:param speed: speed

:return: See the code documentation for details.
```

__int open_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5)__

__int open_bio_gripper(bool wait = true, fp32 timeout = 5)__

```
Open the bio gripper

:param speed: speed value, default is 0 (not set the speed)
:param wait: whether to wait for the bio gripper motion complete, default is True
:param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
        
:return: See the code documentation for details.
```

__int close_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5)__

__int close_bio_gripper(bool wait = true, fp32 timeout = 5)__

```
Close the bio gripper

:param speed: speed value, default is 0 (not set the speed)
:param wait: whether to wait for the bio gripper motion complete, default is True
:param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true

:return: See the code documentation for details.
```

__int get_bio_gripper_status(int *status)__

```
Get the status of the bio gripper

:param status: the result of the bio gripper status value
    status & 0x03 == 0: stop
    status & 0x03 == 1: motion
    status & 0x03 == 2: catch
    status & 0x03 == 3: error
    (status >> 2) & 0x03 == 0: not enabled
    (status >> 2) & 0x03 == 1: enabling
    (status >> 2) & 0x03 == 2: enabled

return: See the code documentation for details.
```

__int get_bio_gripper_error(int *err)__

```
Get the error code of the bio gripper

:param err: the result of the bio gripper error code

:return: See the code documentation for details.
```

__int clean_bio_gripper_error(void)__

```
Clean the error code of the bio gripper

:return: See the code documentation for details.
```

__int set_tgpio_modbus_timeout(int timeout)__

```
Set the modbus timeout of the tool gpio

:param timeout: timeout, seconds

:return: See the code documentation for details.
```

__int set_tgpio_modbus_baudrate(int baud)__

```
Set the modbus baudrate of the tool gpio

:param baud: baudrate, 4800/9600/19200/38400/57600/115200/230400/460800/921600/1000000/1500000/2000000/2500000

return: See the code documentation for details.
```

__int get_tgpio_modbus_baudrate(int *baud)__

```
Get the modbus baudrate of the tool gpio

:param baud: the result of baudrate

:return: See the code documentation for details.
```

__int getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length)__

```
Send the modbus data to the tool gpio

:param modbus_data: send data
:param modbus_length: the length of the modbus_data
:param ret_data: the response data of the modbus
:param ret_length: the length of the response data

:return: See the code documentation for details.
```

__int set_report_tau_or_i(int tau_or_i = 0)__

```
Set the reported torque or electric current

:param tau_or_i:
    0: torque
    1: electric current

return: See the code documentation for details.
```

__int get_report_tau_or_i(int *tau_or_i)__

```
Get the reported torque or electric current

:param tau_or_i: the result of the tau_or_i

:return: See the code documentation for details.
```

__int set_self_collision_detection(bool on)__

```
Set whether to enable self-collision detection 

:param on: enable or not

:return: See the code documentation for details.
```

__int set_collision_tool_model(int tool_type, int n = 0, ...)__

```
Set the geometric model of the end effector for self collision detection

:param tool_type: the geometric model type
    0: No end effector, no additional parameters required
    1: xArm Gripper, no additional parameters required
    2: xArm Vacuum Gripper, no additional parameters required
    3: xArm Bio Gripper, no additional parameters required
    4: Robotiq-2F-85 Gripper, no additional parameters required
    5: Robotiq-2F-140 Gripper, no additional parameters required
    21: Cylinder, need additional parameters radius, height
        arm->set_collision_tool_model(21, 2, radius, height)
        :param radius: the radius of cylinder, (unit: mm)
        :param height: the height of cylinder, (unit: mm)
    22: Cuboid, need additional parameters x, y, z
        arm->set_collision_tool_model(22, 3, x, y, z)
        :param x: the length of the cuboid in the x coordinate direction, (unit: mm)
        :param y: the length of the cuboid in the y coordinate direction, (unit: mm)
        :param z: the length of the cuboid in the z coordinate direction, (unit: mm)
        
:param n: the count of the additional parameters
:param ...: additional parameters

:return: See the code documentation for details.
```

__int vc_set_joint_velocity(fp32 speeds[7], bool is_sync = true, fp32 duration = -1.0)__

```
Joint velocity control, need to be set to joint velocity control mode(this.set_mode(4))

:param speeds: [spd_J1, spd_J2, ..., spd_J7]
    if default_is_radian is true, the value of spd_J1/.../spd_J1 should be in radians
    if default_is_radian is false, the value of spd_J1/.../spd_J1 should be in degrees
:param is_sync: whether all joints accelerate and decelerate synchronously, default is true
:param duration: the maximum duration of the speed, over this time will automatically set the speed to 0.
    duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
    duration == 0: always effective, will not stop automativally
    duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0
    Note:
        only available if firmware_version >= 1.8.0

:return: See the code documentation for details.
```

__int vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord = false, fp32 duration = -1.0)__

```
Cartesian velocity control, need to be set to cartesian velocity control mode(self.set_mode(5))

:param speeds: [spd_x, spd_y, spd_z, spd_rx, spd_ry, spd_rz]
    if default_is_radian is true, the value of spd_rx/spd_ry/spd_rz should be in radians
    if default_is_radian is false, the value of spd_rx/spd_ry/spd_rz should be in degrees
:param is_tool_coord: is tool coordinate or not, default is false
:param duration: the maximum duration of the speed, over this time will automatically set the speed to 0.
    duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
    duration == 0: always effective, will not stop automativally
    duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0
    Note:
        only available if firmware_version >= 1.8.0

return: See the code documentation for details.
```

__int calibrate_tcp_coordinate_offset(float four_points[4][6], float ret_xyz[3])__

```
Four-point method to calibrate tool coordinate system position offset

:param four_points: a list of four teaching coordinate positions [x, y, z, roll, pitch, yaw]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
:param ret_xyz: the result of the calculated xyz(mm) TCP offset, [x, y, z]

:return: See the code documentation for details.
```

__int calibrate_tcp_orientation_offset(float rpy_be[3], float rpy_bt[3], float ret_rpy[3])__

```
An additional teaching point to calibrate the tool coordinate system attitude offset

:param rpy_be: the rpy value of the teaching point without TCP offset [roll, pitch, yaw]
:param rpy_bt: the rpy value of the teaching point with TCP offset [roll, pitch, yaw]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
:param ret_rpy: the result of the calculated rpy TCP offset, [roll, pitch, yaw]

:return: See the code documentation for details.
```

__int calibrate_user_orientation_offset(float three_points[3][6], float ret_rpy[3], int mode = 0, int trust_ind = 0)__

```
Three-point method teaches user coordinate system posture offset

:param four_points: a list of teaching TCP coordinate positions [x, y, z, roll, pitch, yaw]
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
:param ret_rpy: the result of the calculated rpy user offset, [roll, pitch, yaw]

:return: See the code documentation for details.
```

__int calibrate_user_coordinate_offset(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3])__

```
An additional teaching point determines the position offset of the user coordinate system.

:param rpy_ub: the confirmed offset of the base coordinate system in the user coordinate system [roll, pitch, yaw], which is the result of calibrate_user_orientation_offset()
    if default_is_radian is true, the value of roll/pitch/yaw should be in radians
    if default_is_radian is false, the value of roll/pitch/yaw should be in degrees
:param pos_b_uorg: the position of the teaching point in the base coordinate system [x, y, z], if the arm cannot reach the target position, the user can manually input the position of the target in the base coordinate.
:param ret_xyz: the result of the calculated xyz user offset, [x, y, z]

:return: See the code documentation for details.
```

__int set_impedance(int imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6])__
```
Set all parameters of impedance control through the Six-axis Force Torque Sensor.
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param imp_coord: task frame. 0: base frame. 1: tool frame.
:param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.
:param M: mass. (kg)
:param K: stiffness coefficient.
:param B: damping coefficient. invalid.
    Note: the value is set to 2*sqrt(M*K) in controller.

:return: See the code documentation for details.
```

__int set_impedance_mbk(float M[6], float K[6], float B[6])__
```
Set mbk parameters of impedance control through the Six-axis Force Torque Sensor.
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param M: mass. (kg)
:param K: stiffness coefficient.
:param B: damping coefficient. invalid.
    Note: the value is set to 2*sqrt(M*K) in controller.

:return: See the code documentation for details.
```

__int set_impedance_config(int imp_coord, int imp_c_axis[6])__
```
Set impedance control parameters of impedance control through the Six-axis Force Torque Sensor.
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param imp_coord: task frame. 0: base frame. 1: tool frame.
:param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.

:return: See the code documentation for details.
```

__int config_force_control(int f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6])__
```
Set force control parameters through the Six-axis Force Torque Sensor.
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param f_coord: task frame. 0: base frame. 1: tool frame.
:param f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.
:param f_ref: the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.
:param f_limits: for compliant axes, these values are the maximum allowed tcp speed along/about the axis.

:return: See the code documentation for details.
```

__int set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6])__
```
Set force control pid parameters through the Six-axis Force Torque Sensor.
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param kp: proportional gain
:param ki: integral gain.
:param kd: differential gain.
:param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s

:return: See the code documentation for details.
```

__int ft_sensor_set_zero(void)__
```
Set the current state to the zero point of the Six-axis Force Torque Sensor
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:return: See the code documentation for details.
```

__int ft_sensor_iden_load(float result[10])__
```
Identification the tcp load with the the Six-axis Force Torque Sensor
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param result: the result of identification, ([mass，x_centroid，y_centroid，z_centroid，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])

:return: See the code documentation for details.
```

__int ft_sensor_cali_load(float load[10], bool association_setting_tcp_load = false, float m = 0.325, float x = -17, float y = 9, float z = 11.8)__
```
Write the load offset parameters identified by the Six-axis Force Torque Sensor
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

：param load: iden result([mass，x_centroid，y_centroid，z_centroid，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])
：param association_setting_tcp_load: whether to convert the paramster to the corresponding tcp load and set, default is false
	if true, the value of tcp load will be modified

:return: See the code documentation for details.
```

__int ft_sensor_enable(int on_off)__
```
Used for enabling and disabling the use of the Six-axis Force Torque Sensor measurements in the controller.
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param on_off: enable or disable F/T data sampling.

:return: See the code documentation for details.
```

__int ft_sensor_app_set(int app_code)__
```
Set robot to be controlled in force mode. (Through the Six-axis Force Torque Sensor)
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param app_code: force mode. 
    0: non-force mode  
    1: impendance control  
    2: force control

:return: See the code documentation for details.
```

__int ft_sensor_app_get(int *app_code)__
```
Get force mode
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param app_code: the result of force mode.
    0: non-force mode  
    1: impendance control  
    2: force control

:return: See the code documentation for details.
```

__int get_ft_sensor_data(float ft_data[6])__
```
Get the data of the Six-axis Force Torque Sensor
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param ft_data: the result of the Six-axis Force Torque Sensor.

:return: See the code documentation for details.
```

__int get_ft_sensor_config(int *ft_app_status = NULL, int *ft_is_started = NULL, int *ft_type = NULL, int *ft_id = NULL, int *ft_freq = NULL, float *ft_mass = NULL, float *ft_dir_bias = NULL, float ft_centroid[3] = NULL, float ft_zero[6] = NULL, int *imp_coord = NULL, int imp_c_axis[6] = NULL, float M[6] = NULL, float K[6] = NULL, float B[6] = NULL, int *f_coord = NULL, int f_c_axis[6] = NULL, float f_ref[6] = NULL, float f_limits[6] = NULL, float kp[6] = NULL, float ki[6] = NULL, float kd[6] = NULL, float xe_limit[6] = NULL)__
```
Get the config of the Six-axis Force Torque Sensor
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param ft_app_status: force mode
    0: non-force mode
    1: impendance control
    2: force control
:param ft_is_started: ft sensor is enable or not
:param ft_type: ft sensor type
:param ft_id: ft sensor id
:param ft_freq: ft sensor frequency
:param ft_mass: load mass
:param ft_dir_bias:
:param ft_centroid: [x_centroid，y_centroid，z_centroid]
:param ft_zero: [Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]
:param imp_coord: task frame of impendance control mode.
    0: base frame.
    1: tool frame.
:param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.
:param M: mass. (kg)
:param K: stiffness coefficient.
:param B: damping coefficient. invalid.   Note: the value is set to 2*sqrt(M*K) in controller.
:param f_coord: task frame of force control mode. 
    0: base frame.
    1: tool frame.
:param f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.
:param f_ref:  the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.
:param f_limits:  for compliant axes, these values are the maximum allowed tcp speed along/about the axis.
:param kp: proportional gain
:param ki: integral gain.
:param kd: differential gain.
:param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s

:return: See the code documentation for details.
```

__int get_ft_sensor_error(int *err)__
```
Get the error code of the Six-axis Force Torque Sensor
    Note:
        1. only available if firmware_version >= 1.8.3
        2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)

:param err: the result of ft sensor error code

:return: See the code documentation for details.
```

__int iden_tcp_load(float result[4])__
```
Identification the tcp load with current
    Note: only available if firmware_version >= 1.8.0

:param result: the result of identification. [mass，x_centroid，y_centroid，z_centroid]

:return: See the code documentation for details.
```

__int get_linear_track_error(int *err)__
```
Get the error code of the linear track
    Note: only available if firmware_version >= 1.8.0

:param err: the result of linear track error

:return: See the code documentation for details.
```

__int get_linear_track_status(int *status)__
```
Get the status of the linear track
    Note: only available if firmware_version >= 1.8.0

:param status: the result of linear track status
    status & 0x00: motion finish.
    status & 0x01: in motion
    status & 0x02: has stop

:return: See the code documentation for details.
```

__int get_linear_track_pos(int *pos)__
```
Get the pos of the linear track
    Note: only available if firmware_version >= 1.8.0

:param pos: the result of linear track position

:return: See the code documentation for details.
```

__int get_linear_track_is_enabled(int *status)__
```
Get the linear track is enabled or not
    Note: only available if firmware_version >= 1.8.0

:param status: the result of linear track status
    status == 0: linear track is not enabled
    status == 1: linear track is enabled

:return: See the code documentation for details.
```

__int get_linear_track_on_zero(int *status)__
```
Get the linear track is on zero positon or not
    Note: only available if firmware_version >= 1.8.0

:param status: the result of linear track status
    status == 0: linear track is not on zero
    status == 1: linear track is on zero

:return: See the code documentation for details.
```

__int get_linear_track_sci(int *sci1)__
```
Get the sci1 value of the linear track
    Note: only available if firmware_version >= 1.8.0

:param sci1: the result of linear track sci1

:return: See the code documentation for details.
```

__int get_linear_track_sco(int sco[2])__
```
Get the sco value of the linear track
    Note: only available if firmware_version >= 1.8.0

:param sco: the result of linear track sco0 and sco1

:return: See the code documentation for details.
```

__int clean_linear_track_error(void)__
```
Clean the linear track error
    Note: only available if firmware_version >= 1.8.0

:return: See the code documentation for details.
```

__int set_linear_track_enable(bool enable)__
```
Set the linear track enable/disable
    Note: only available if firmware_version >= 1.8.0

:param enable: enable or not

:return: See the code documentation for details.
```

__int set_linear_track_speed(int speed)__
```
Set the speed of the linear track
    Note: only available if firmware_version >= 1.8.0

:param speed: Integer between 1 and 1000mm/s.

:return: See the code documentation for details.
```

__int set_linear_track_back_origin(bool wait = true, bool auto_enable = true)__
```
Set the linear track go back to the origin position
Note:
    1. only available if firmware_version >= 1.8.0
    2. only useful when powering on for the first time
    3. this operation must be performed at the first power-on

:param wait: wait to motion finish or not, default is True
:param auto_enable: enable after back to origin or not, default is True

:return: See the code documentation for details.
```

__int set_linear_track_pos(int pos, int speed = 0, bool wait = true, fp32 timeout = 100, bool auto_enable = true)__
```
Set the position of the linear track
    Note: only available if firmware_version >= 1.8.0

:param pos: position. Integer between 0 and 700/1000/1500.
    If the SN of the linear track is start with AL1300, the position range is 0~700mm.
    If the SN of the linear track is start with AL1301, the position range is 0~1000mm.
    If the SN of the linear track is start with AL1302, the position range is 0~1500mm.
:param speed: auto set the speed of the linear track if the speed is changed, Integer between of 1 and 1000mm/s, default is -1(not set)
:param wait: wait to motion finish or not, default is True
:param timeout: wait timeout, seconds, default is 100s.
:param auto_enable: auto enable if not enabled, default is true

:return: See the code documentation for details.
```

__int set_linear_track_stop(void)__
```
Set the linear track to stop
    Note: only available if firmware_version >= 1.8.0

:return: See the code documentation for details.
```

__int set_baud_checkset_enable(bool enable)__
```
Enable auto checkset the baudrate of the end IO board or not
    Note: only available in the API of gripper/bio/robotiq/linear_track.

:return: See the code documentation for details.
```

__int set_checkset_default_baud(int type, int baud)__
```
Set the checkset baud value
    Note: do not modify at will, only use when the baud rate of the corresponding peripheral is changed

:param type: checkset type
    1: xarm gripper
    2: bio gripper
    3: robotiq gripper
    4: linear track
:param baud: checkset baud value, less than or equal to 0 means disable checkset
:return: See the code documentation for details.
```

__int get_checkset_default_baud(int type, int *baud)__
```
Get the checkset baud value

:param type: checkset type
    1: xarm gripper
    2: bio gripper
    3: robotiq gripper
    4: linear track
:param baud: checkset baud value, less than or equal to 0 means disable checkset
:return: See the code documentation for details.
```
