xArm-C++-SDK API Documentation

## class  Swift ##

### class constructor ###

/\*!
- @param port: port name(such as "COM5"/"/dev/ttyUSB0") or ip-address(such as "192.168.1.185")

  Note: this parameter is required if parameter do_not_open is False

- @param is_radian: set the default unit is radians or not, default is False

- @param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.
- @param check_tcp_limit: reversed

- @param check_joint_limit: reversed

- @param check_cmdnum_limit: reversed

- @param check_robot_sn: reversed

- @param check_is_ready: reversed

\*/
```c++
XArmAPI(const std::string &port="",
        bool is_radian=DEFAULT_IS_RADIAN,
        bool do_not_open=false,
        bool check_tcp_limit=true,
        bool check_joint_limit=true,
        bool check_cmdnum_limit=true,
        bool check_robot_sn=false,
        bool check_is_ready=true);
```
--------------------------------------

### property ###

```c++
int state; // state
int mode; // mode
int cmd_num; // cmd cache count
fp32 *joints_torque; // joints torque, fp32[7]{servo-1, ..., servo-7}
bool *motor_brake_states; // motor brake states, bool[8]{servo-1, ..., servo-7, reversed}
bool *motor_enable_states; // motor enable states, bool[8]{servo-1, ..., servo-7, reversed}
int error_code; // error code
int warn_code; // warn code
fp32 *tcp_load; // tcp load, fp32[4]{weight, x, y, z}
int collision_sensitivity; // collision sensitivity
int teach_sensitivity; // teach sensitivity
int device_type; // device type
int axis; // robot axis
int master_id;
int slave_id;
int motor_tid;
int motor_fid;
unsigned char version[30]; // version
unsigned char sn[40]; // sn
int *version_number; // version numbre
fp32 tcp_jerk; // tcp jerk
fp32 rot_jerk; // rot jerk
fp32 max_rot_acc; // max rot acc
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
```
----------------------------------------

### method ###

/\*! Connect to xArm

- @param port: port name or the ip address
- return:
  - 0: success
  - -1: port is empty
  - -2: tcp control connect failed
  - -3: tcp report connect failed

\*/
```c++
int connect(const std::string &port="");
```

/\*! Disconnect to xArm
\*/

```c++
void disconnect();
```

/\*! Get the xArm version

\*/
```c++
int get_version(unsigned char version[40]);
```

/\*! Get the xArm sn

\*/
```c++
int get_robot_sn(unsigned char robot_sn[40]);
```

/\*! Get the xArm state

- @param: the state of xArm
  - 1: in motion
  - 2: sleeping
  - 3: suspended
  - 4: stopping

\*/
```c++
int get_state(int *state);
```

/\*! Shutdown the xArm controller system

\*/

```c++
int shutdown_system(int value=1);
```

/\*! Get the cmd count in cache

\*/
```c++
int get_cmdnum(int *cmdnum);
```

/\*! Get the controller error and warn code
\*/

```c++
int get_err_warn_code(int err_warn[2]);
```

/\*! Get the cartesian position

-  @param pose: the position of xArm, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  -  if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees

\*/
```c++
int get_position(fp32 pose[6]);
```

/\*! Get the servo angle

- @param angles: the angles of the servos, like [servo-1, ..., servo-7]
  -  if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  - if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees

\*/

```c++
int get_servo_angle(fp32 angles[7]);
```

/\*! Motion enable

- @param enable: enable or not
- @param servo_id: servo id, 1-8, 8(enable/disable all servo)

\*/
```c++
int motion_enable(bool enable, int servo_id=8);
```

/\*! Set the xArm state

- @param state: state
  - 0: sport state
  - 3: pause state
  - 4: stop state

\*/

```c++
int set_state(int state);
```

/\*! Set the xArm mode

- @param mode: mode
  - 0: position control mode
  - 1: servo motion mode
  - 2: joint teaching mode
  - 3: cartesian teaching mode (invalid)

\*/
```c++
int set_mode(int mode);
```

/\*! Attach the servo

- @param servo_id: servo id, 1-8, 8(attach all servo)

\*/

```c++
int set_servo_attach(int servo_id);
```

/\*! Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.

- @param servo_id: servo id, 1-8, 8(detach all servo)

\*/
```c++
int set_servo_detach(int servo_id);
```

/\*! Clean the controller error, need to be manually enabled motion and set state after clean error

\*/
```c++
int clean_error(void);
```

/\*! Clean the controller warn

\*/
```c++
int clean_warn(void);
```

/\*! Set the arm pause time, xArm will pause sltime second

-  @param sltime: sleep second

\*/
```c++
int set_pause_time(fp32 sltime);
```

/\*! Set the sensitivity of collision

    * @param sensitivity: sensitivity value, 0~5

\*/
```c++
int set_collision_sensitivity(int sensitivity);
```

/\*! Set the sensitivity of drag and teach

- @param sensitivity: sensitivity value, 1~5

\*/
```c++
int set_teach_sensitivity(int sensitivity);
```

/\*! Set the direction of gravity

- @param gravity_dir: direction of gravity, such as [x(mm), y(mm), z(mm)]

\*/
```c++
int set_gravity_direction(fp32 gravity_dir[3]);
```

/\*! Clean current config and restore system default settings

Note:

   1. This interface will clear the current settings and restore to the original settings (system default settings)

\*/
```c++
int clean_conf(void);
```

/\*! Save config

Note:

1. This interface can record the current settings and will not be lost after the restart.

        2. The clean_conf interface can restore system default settings

\*/
```c++
int save_conf(void);
```

/\*!  Set the position
        MoveLine: Linear motion
        MoveArcLine: Linear arc motion with interpolation

- @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  - if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  -  if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
- @param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
- @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
- @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
- @param mvtime: reserved, 0
- @param wait: whether to wait for the arm to complete, default is False
- @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true

\*/
```c++
int set_position(fp32 pose[6], fp32 radius=-1, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
int set_position(fp32 pose[6], fp32 radius=-1, bool wait=false, fp32 timeout=TIMEOUT_10);
int set_position(fp32 pose[6], bool wait=false, fp32 timeout=TIMEOUT_10);
```

/\*! Set the servo angle

- @param angles: angles, like [servo-1, ..., servo-7]
          if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
          if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
- @param servo_id: servo id, 1~7, specify the joint ID to set
-  @param angle: servo angle, use with servo_id parameters
- @param speed: move speed (rad/s or °/s), default is this.last_used_joint_speed
  - if default_is_radian is true, the value of speed should be in radians
  - if default_is_radian is false, The value of speed should be in degrees
- @param acc: move acceleration (rad/s^2 or °/s^2), default is this.last_used_joint_acc
  -  if default_is_radian is true, the value of acc should be in radians
  -  if default_is_radian is false, The value of acc should be in degrees
- @param mvtime: reserved, 0
- @param wait: whether to wait for the arm to complete, default is False
-  @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true

\*/
```c++
int set_servo_angle(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
int set_servo_angle(fp32 angles[7], bool wait=false, fp32 timeout=TIMEOUT_10);
int set_servo_angle(int servo_id, fp32 angle, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
int set_servo_angle(int servo_id, fp32 angle, bool wait=false, fp32 timeout=TIMEOUT_10);
```

/\*! Serbo_j motion

- @param angles: angles, like [servo-1, ..., servo-7]
  -  if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  - if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
- @param speed: reserved, move speed (rad/s or °/s)
  -  if default_is_radian is true, the value of speed should be in radians
  - if default_is_radian is false, The value of speed should be in degrees
- @param acc: reserved, move acceleration (rad/s^2 or °/s^2)
  - if default_is_radian is true, the value of acc should be in radians
  - if default_is_radian is false, The value of acc should be in degrees
- @param mvtime: reserved, 0

\*/
```c++
int set_servo_angle_j(fp32 angles[7], fp32 speed=0, fp32 acc=0, fp32 mvtime=0);
```

/\*! The motion calculates the trajectory of the space circle according to the three-point coordinates.
      The three-point coordinates are (current starting point, pose1, pose2).

- @param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  -  if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
- @param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  - if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
- @param percent: the percentage of arc length and circumference of the movement
- @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed
- @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc
-  @param mvtime: 0, reserved
- @param wait: whether to wait for the arm to complete, default is False
- @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True

\*/
```c++
int move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
```

/\*! Move to go home (Back to zero)

- @param speed: move speed (rad/s or °/s), default is 50 °/s
  - if default_is_radian is true, the value of speed should be in radians
  - if default_is_radian is false, The value of speed should be in degrees
- @param acc: move acceleration (rad/s^2 or °/s^2), default is 1000 °/s^2
  - if default_is_radian is true, the value of acc should be in radians
  - if default_is_radian is false, The value of acc should be in degrees
- @param mvtime: reserved, 0
- @param wait: whether to wait for the arm to complete, default is False
-  @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true

\*/
```c++
int move_gohome(fp32 speed=0, fp32 acc=0, fp32 mvtime=0, bool wait=false, fp32 timeout=TIMEOUT_10);
int move_gohome(bool wait=false, fp32 timeout=TIMEOUT_10);
```

/\*! Reset
   * @param wait: whether to wait for the arm to complete, default is False
* @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true

\*/
```c++
void reset(bool wait=false, fp32 timeout=TIMEOUT_10);
```

/\*! Emergency stop

*/
```c++
void emergency_stop(void);
```

/\*! Set the tool coordinate system offset at the end

- @param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  - if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees

\*/
```c++
int set_tcp_offset(fp32 pose_offset[6]);
```

/\*! Set the load

- @param weight: load weight (unit: kg)
- @param center_of_gravity: tcp load center of gravity, like [x(mm), y(mm), z(mm)]

\*/
```c++
int set_tcp_load(fp32 weight, fp32 center_of_gravity[3]);
```

/\*! Set the translational jerk of Cartesian space

- @param jerk: jerk (mm/s^3)

\*/
```c++
int set_tcp_jerk(fp32 jerk);
```

/\*! Set the max translational acceleration of Cartesian space

- @param acc: max acceleration (mm/s^2)

\*/
```C++
int set_tcp_maxacc(fp32 acc);
```

/\*! Set the jerk of Joint space

- @param jerk: jerk (°/s^3 or rad/s^3)
  - if default_is_radian is true, the value of jerk should be in radians
  - if default_is_radian is false, The value of jerk should be in degrees

\*/
```c++
int set_joint_jerk(fp32 jerk);
```

/\*! Set the max acceleration of Joint space

- @param acc: max acceleration (°/s^2 or rad/s^2)
  - if default_is_radian is true, the value of jerk should be in radians
  - if default_is_radian is false, The value of jerk should be in degrees

\*/
```c++
int set_joint_maxacc(fp32 acc);
```

/\*! Get inverse kinematics

- @param pose: source pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  - if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
- @param angles: target angles, like [servo-1, ..., servo-7]
  - if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  - if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees

\*/
```c++
int get_inverse_kinematics(fp32 pose[6], fp32 angles[7]);
```

/\*! Get forward kinematics

- @param angles: source angles, like [servo-1, ..., servo-7]
  - if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  - if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
- @param pose: target pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)
  - if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees

\*/
```c++
int get_forward_kinematics(fp32 angles[7], fp32 pose[6]);
```

/\*! Check the tcp pose is in limit

- @param pose: pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
  -  if default_is_radian is true, the value of roll/pitch/yaw should be in radians
  - if default_is_radian is false, The value of roll/pitch/yaw should be in degrees
- @param limit: 1: limit, 0: no limit

\*/
```c++
int is_tcp_limit(fp32 pose[6], int *limit);
```

/\*! Check the joint is in limit

- @param angles: angles, like [servo-1, ..., servo-7]
  - if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians
  - if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees
- @param limit: 1: limit, 0: no limit

\*/
```c++
int is_joint_limit(fp32 angles[7], int *limit);
```

/\*! Set the gripper enable

- @param enable: enable or not

\*/
```c++
int set_gripper_enable(bool enable);
```

/\*! Set the gripper mode

- @param mode: 
  - 1: location mode
  - 2: speed mode(no use)
  - 3: torque mode(no use)

\*/
```c++
int set_gripper_mode(int mode);
```

/\*! Get the gripper position

- @param pos: used to store the results obtained

\*/
```c++
int get_gripper_position(fp32 *pos);
```

/\*! Set the gripper position

- @param pos: gripper position
- @param wait: wait or not, default is false
- @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true

\*/

```c++
int set_gripper_position(fp32 pos, bool wait=false, fp32 timeout=10);
```
/\*! Set the gripper speed

- @param speed:

\*/
```c++
int set_gripper_speed(fp32 speed);
```

/\*! Get the gripper error code

- @param err: used to store the results obtained

\*/
```c++
int get_gripper_err_code(int *err);
```

/\*! Clean the gripper error

\*/
```c++
int clean_gripper_error(void);
```

/\*! Get the digital value of the Tool GPIO

- @param io0_value: the digital value of Tool GPIO-0
- @param io1_value: the digital value of Tool GPIO-1

\*/

```c++
int get_tgpio_digital(int *io0_value, int *io1_value);
```

/\*! Set the digital value of the specified Tool GPIO

- @param ionum: ionum, 0 or 1
- @param value: the digital value of the specified io

\*/
```c++
int set_tgpio_digital(int ionum, int value);
```

/\*! Get the analog value of the specified Tool GPIO

- @param ionum: ionum, 0 or 1
   - @param value: the analog value of the specified tool io

\*/
```c++
int get_tgpio_analog(int ionum, fp32 *value);
```

/\*! Get the digital value of the specified Controller GPIO

- @param digitals: the values of the controller GPIO

\*/
```c++
int get_cgpio_digital(int *digitals);
```

/\*! Get the analog value of the specified Controller GPIO

- @param ionum: ionum, 0 or 1
- @param value: the analog value of the specified controller io

\*/

```c++
int get_cgpio_analog(int ionum, fp32 *value);
```

/\*! Set the digital value of the specified Controller GPIO

- @param ionum: ionum, 0 ~ 7
- @param value: the digital value of the specified io

\*/

```c++
int set_cgpio_digital(int ionum, int value);
```

/\*! Set the analog value of the specified Controller GPIO

- @param ionum: ionum, 0 or 1
- @param value: the analog value of the specified io

\*/

```c++
int set_cgpio_analog(int ionum, int value);
```

/\*! Set the digital input functional mode of the Controller GPIO

- @param ionum: ionum, 0 ~ 7
- @param fun: functional mode
  - 0: general input
  - 1: external emergency stop
  - 2: reversed, protection reset
  - 3: reversed, reduced mode
  - 4: reversed, operating mode
  - 5: reversed, three-state switching signal
  - 11: offline task
  - 12: teaching mode

\*/

```c++
int set_cgpio_digital_input_function(int ionum, int fun);
```

/\*! Set the digital output functional mode of the specified Controller GPIO

- @param ionum: ionum, 0 ~ 7
- @param fun: functional mode
  - 0: general output
  - 1: emergency stop
  - 2: in motion
  - 11: has error
  - 12: has warn
  - 13: in collision
  - 14: in teaching
  - 15: in offline task

\*/

```c++
int set_cgpio_digital_output_function(int ionum, int fun);
```

/\*! Get the state of the Controller GPIO

- @param state:
- @param digit_io:
- @param analog:
- @param input_conf:
- @param output_conf:

\*/

```c++
int get_cgpio_state(int *state, int *digit_io, fp32 *analog, int *input_conf, int *output_conf);
```

/\*! Register the report location callback

\*/

```c++
int register_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles));
```

/\*! Register the connect status changed callback

\*/

```c++
int register_connect_changed_callback(void(*callback)(bool connected, bool reported));
```

/\*! Register the connect status changed callback

\*/

```c++
int register_connect_changed_callback(void(*callback)(bool connected, bool reported));
```

/\*! Register the state status changed callback

\*/

```c++
int register_state_changed_callback(void(*callback)(int state));
```

/\*! Register the state status changed callback

\*/

```c++
int register_state_changed_callback(void(*callback)(int state));
```

/\*! Register the mode changed callback

\*/

```c++
int register_mode_changed_callback(void(*callback)(int mode));
```

/\*! Register the motor enable states or motor brake states changed callback

\*/

```c++
int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake));
```

/\*! Register the error code or warn code changed callback

\*/

```c++
int register_error_warn_changed_callback(void(*callback)(int err_code, int warn_code));
```

/\*! Register the cmdnum changed callback

\*/

```c++
int register_cmdnum_changed_callback(void(*callback)(int cmdnum));
```

/\*! Register the motor enable states or motor brake states changed callback

\*/

```c++
int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake));
```

/\*! Release the location report callback

- @param callback: NULL means to release all callbacks;

\*/

```c++
int release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)=NULL);
```

/\*! Release the location report callback

- @param callback: NULL means to release all callbacks;

\*/

```c++
int release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)=NULL);
```

/\*! Register the motor enable states or motor brake states changed callback

\*/

```c++
int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake));
```

/\*! Release the connect changed callback

- @param callback: NULL means to release all callbacks for the same event

\*/

```c++
int release_connect_changed_callback(void(*callback)(bool connected, bool reported)=NULL);
```

/\*! Release the state changed callback

- @param callback: NULL means to release all callbacks for the same event

\*/

```c++
int release_state_changed_callback(void(*callback)(int state)=NULL);
```

/\*! Release the mode changed callback

- @param callback: NULL means to release all callbacks for the same event

\*/

```c++
int release_mode_changed_callback(void(*callback)(int mode)=NULL);
```

/\*! Release the motor enable states or motor brake states changed callback

- @param callback: NULL means to release all callbacks for the same event

\*/

```c++
int release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)=NULL);
```

/\*! Release the error warn changed callback

- @param callback: NULL means to release all callbacks for the same event

\*/

```c++
int release_error_warn_changed_callback(void(*callback)(int err_code, int warn_code)=NULL);
```

/\*! Release the cmdnum changed callback

- @param callback: NULL means to release all callbacks for the same event

\*/

```c++
int release_cmdnum_changed_callback(void(*callback)(int cmdnum)=NULL);
```


};