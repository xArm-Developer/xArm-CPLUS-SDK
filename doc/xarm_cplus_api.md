# xArm-C++-SDK API Documentation (V1.15.0)

## class __XArmAPI__
************************************
__XArmAPI(const std::string &robot_ip="", 
    bool is_radian=false,
    bool do_not_open=false,
    bool check_tcp_limit=true,
    bool check_joint_limit=true,
    bool check_cmdnum_limit=true,
    bool check_robot_sn=false,
    bool check_is_ready=false,
    bool check_is_pause=true,
    int max_callback_thread_count=100,
    int max_cmdnum = 512,
    int init_axis = 7,
    bool debug = false,
    std::string report_type = "rich",
    bool baud_checkset = true)__  
> @param robot_ip: ip-address(such as "192.168.1.185")  
> &ensp;&ensp;&ensp;&ensp;Note: this parameter is required if parameter do_not_open is false  
> @param is_radian: set the default unit is radians or not, default is false  
> @param do_not_open: do not open, default is false, if true, you need to manually call the connect interface.  
> @param check_tcp_limit: reversed, whether checking tcp limit, default is true  
> @param check_joint_limit: reversed, whether checking joint limit, default is true  
> @param check_cmdnum_limit: whether checking command num limit, default is true  
> @param check_robot_sn: whether checking robot sn, default is false  
> @param check_is_ready: check robot is ready to move or not, default is true  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version < 1.5.20  
> @param check_is_pause: check robot is pause or not, default is true  
> @param max_callback_thread_count: max callback thread count, default is -1  
> &ensp;&ensp;&ensp;&ensp;Note: greater than 0 means the maximum number of threads that can be used to process callbacks  
> &ensp;&ensp;&ensp;&ensp;Note: equal to 0 means no thread is used to process the callback  
> &ensp;&ensp;&ensp;&ensp;Note: less than 0 means no limit on the number of threads used for callback  
> @param max_cmdnum: max cmdnum, default is 512  
> &ensp;&ensp;&ensp;&ensp;Note: only available in the param `check_cmdnum_limit` is true  
> @param init_axis: init axis variable  
> @param debug:  reversed  
> @param report_type: report type   
> @param baud_checkset: auto check set the baud when use the gripper/bio/robotiq/lineartrack api or not  

************************************

## Property

- __int state__
  > xArm state  
  >  
  > @return:  
  > &ensp;&ensp;&ensp;&ensp;1: in motion  
  > &ensp;&ensp;&ensp;&ensp;2: sleeping  
  > &ensp;&ensp;&ensp;&ensp;3: suspended  
  > &ensp;&ensp;&ensp;&ensp;4: stopping  


- __int mode__
  >xArm mode，only available in socket way and  enable_report is true  
  >  
  > @return:   
  > &ensp;&ensp;&ensp;&ensp;0: position control mode  
  > &ensp;&ensp;&ensp;&ensp;1: servo motion mode  
  > &ensp;&ensp;&ensp;&ensp;2: joint teaching mode  
  > &ensp;&ensp;&ensp;&ensp;3: cartesian teaching mode (invalid)  
  > &ensp;&ensp;&ensp;&ensp;4: joint velocity control mode  
  > &ensp;&ensp;&ensp;&ensp;5: cartesian velocity control mode  
  > &ensp;&ensp;&ensp;&ensp;6: joint online trajectory planning mode  
  > &ensp;&ensp;&ensp;&ensp;7: cartesian online trajectory planning mode  


- __int cmd_num__
  > Number of command caches in the controller


- __float joints_torque[7]__
  > Joints torque, only available in socket way  
  >  
  > @return: float[7]{servo-1, ..., servo-7}


- __bool motor_brake_states[8]__
  > Motor brake state list, only available in socket way  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.  
  > 
  > @return: bool[8]{servo-1, ..., servo-7, reversed}


- __bool motor_enable_states[8]__
  > Motor enable state list, only available in socket way  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.
  >
  > @return: bool[8]{servo-1, ..., servo-7, reversed}


- __int error_code__
  > Controller error code. See the [Controller Error Code Documentation](./xarm_api_code.md#controller-error-code) for details.


- __int warn_code__
  > Controller warn code. See the [Controller Warn Code Documentation](./xarm_api_code.md#controller-warn-code) for details.


- __float tcp_load[4]__
  > xArm tcp load, only available in socket way  
  >
  > @return: float[4]{weight, x, y, z}


- __int collision_sensitivity__
  > The sensitivity value of collision, only available in socket way


- __int teach_sensitivity__
  > The sensitivity value of drag and teach, only available in socket way


- __int device_type__
  > Device type, only available in socket way  


- __int axis__
  > Axis number, only available in socket way  


- __unsigned char version[30]__
  > xArm firmware version


- __unsigned char sn[40]__
  > xArm sn


- __int version_number[3]__
  > Frimware version number


- __float tcp_jerk__
  > tcp jerk 


- __float joint_jerk__
  > joint jerk


- __float rot_jerk__
  > rot jerk


- __float max_rot_acc__
  > max rot acc


- __float tcp_speed_limit[2]__
  > Joint acceleration limit, only available in socket way  
  >
  > @return: float[2]{min, max}


- __float tcp_acc_limit[2]__
  > Joint acceleration limit, only available in socket way  
  > 
  > @return: float[2]{min, max}


- __float last_used_tcp_speed__
  > The last used cartesion speed, default value of parameter speed of interface set_position/move_circle


- __float last_used_tcp_acc__
  > The last used cartesion acceleration, default value of parameter mvacc of interface set_position/move_circle


- __float angles[7]__
  > Servo angles  
  >
  > @return: float[7]{servo-1, ..., servo-7}


- __float last_used_angles[7]__
  > The last used servo angles, default value of parameter angle of interface set_servo_angle
  >
  > @return: float[7]{servo-1, ..., servo-7}


- __float joint_speed_limit[2]__
  > Joint speed limit,  only available in socket way 
  >
  > @return: float[2]{min, max}


- __float joint_acc_limit[2]__
  > Joint acceleration limit, only available in socket way  
  >
  > @return: float[2]{min, max}


- __float last_used_joint_speed__
  > The last used joint speed, default value of parameter speed of interface set_servo_angle


- __float last_used_joint_acc__
  > The last used joint acceleration, default value of parameter mvacc of interface set_servo_angle


- __float position[6]__
  > Cartesion position
  >
  > @return: float[6]{x, y, z, roll, pitch, yaw}


- __float last_used_position[6]__
  > The last used cartesion position, default value of parameter x/y/z/roll/pitch/yaw of interface set_position
  > 
  > @return: float[6]{x, y, z, roll, pitch, yaw}


- __float tcp_offset[6]__
  > Cartesion position offset, only available in socket way and enable_report is true 
  >
  > @return: float[6]{x, y, z, roll, pitch, yaw}


- __float gravity_direction[3]__
  > gravity direction, only available in socket way 
  >
  >@return: float[3]{x_direction, y_direction, z_direction}


- __float realtime_tcp_speed__
  > The real time speed of tcp motion, only available if version > 1.2.11


- __float realtime_joint_speeds[7]__
  > The real time speed of joint motion, only available if version > 1.2.11


- __float world_offset[6]__
  > Base coordinate offset, only available if version > 1.2.11
  >
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. If self.default_is_radian is true, the returned value(roll_offset/pitch_offset/yaw_offset) is in radians
  >
  > @return: float[6]{x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)}


- __int count__
  > Counter value


- __float temperatures[7]__
  > Motor temperature, only available if version > 1.2.11
  > 
  > @return: float[7]{motor-1-temperature, ..., motor-7-temperature}


- __unsigned char gpio_reset_config[2]__
  > The gpio reset enable config, only available if version > 1.5.0
  >
  > @return: unsigned char[2]{cgpio_reset_enable, tgpio_reset_enable}


- __bool default_is_radian__
  > The default unit is radians or not


- __float voltages[7]__
  > Servos voltage
  >         
  > @return: float[7]{servo-1-voltage, ..., servo-7-voltage}


- __float currents[7]__
  > Servos electric current
  >
  > @return: float[7]{servo-1-current, ..., servo-7-current}


- __int is_collision_detection__
  > self collision detection or not 


- __int collision_tool_type__
  > self collision tool type


- __float collision_model_params[6]__
  > self collision model params


- __int iden_progress__
  > the progress of identifition


- __float ft_ext_force[6]__
  > the ext force data of ft sensor


- __float ft_raw_force[6]__
  > the raw force data of ft sensor


- __unsigned char only_check_result__
  > the result of only check motion

***********************************

## Method

- __bool has_err_warn(void)__

  >  xArm has error/warn or not, only available in socket way


- __bool has_error(void)__
  > xArm has error or not, only available in socket way


- __bool has_warn(void)__
  > xArm has warn or not, only available in socket way


- __bool is_connected(void)__
  > xArm is connected or not


- __bool is_reported(void)__
  > xArm is reported or not, only available in socket way


- __int connect(const std::string &port="")__
  > Connect to xArm
  >
  > @param port: port name or the ip address  
  > @return:  
  > &ensp;&ensp;&ensp;&ensp;0: success  
  > &ensp;&ensp;&ensp;&ensp;-1: port is empty  
  > &ensp;&ensp;&ensp;&ensp;-2: tcp control connect failed  
  > &ensp;&ensp;&ensp;&ensp;-3: tcp report connect failed


- __void disconnect(void)__
  > Disconnect to xArm


- __int get_version(unsigned char version[40])__
  > Get the xArm version
  >
  > @param version:  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_robot_sn(unsigned char robot_sn[40])__
  > Get the xArm sn
  > 
  > @param robot_sn:  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_state(int *state)__
  > Get the xArm state
  >
  > @param: the state of xArm  
  > &ensp;&ensp;&ensp;&ensp;1: in motion  
  > &ensp;&ensp;&ensp;&ensp;2: sleeping  
  > &ensp;&ensp;&ensp;&ensp;3: suspended  
  > &ensp;&ensp;&ensp;&ensp;4: stopping  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int system_control(int value=1)__
  > Control the xArm controller system
  >
  > @param value:  
  > &ensp;&ensp;&ensp;&ensp;1: shutdown  
  > &ensp;&ensp;&ensp;&ensp;2: reboot  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_cmdnum(int *cmdnum)__
  > Get the cmd count in cache
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_err_warn_code(int err_warn[2])__
  > Get the controller error and warn code
  >
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_position(float pose[6])__
  > Get the cartesian position
  >
  > @param pose: the position of xArm, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_servo_angle(float angles[7])__
  > Get the servo angle
  >
  > @param angles: the angles of the servos, like [servo-1, ..., servo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int motion_enable(bool enable, int servo_id=8)__
  > Motion enable
  > 
  > @param enable: enable or not  
  > @param servo_id: servo id, 1-8, 8(enable/disable all servo)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_state(int state)__
  > Set the xArm state
  > 
  > @param state: state  
  > &ensp;&ensp;&ensp;&ensp;0: sport state  
  > &ensp;&ensp;&ensp;&ensp;3: pause state  
  > &ensp;&ensp;&ensp;&ensp;4: stop state  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_mode(int mode, int detection_param = 0)__
  > Set the xArm mode
  >
  > @param mode: mode  
  > &ensp;&ensp;&ensp;&ensp;0: position control mode  
  > &ensp;&ensp;&ensp;&ensp;1: servo motion mode  
  > &ensp;&ensp;&ensp;&ensp;2: joint teaching mode  
  > &ensp;&ensp;&ensp;&ensp;3: cartesian teaching mode (invalid)  
  > &ensp;&ensp;&ensp;&ensp;4: joint velocity control mode  
  > &ensp;&ensp;&ensp;&ensp;5: cartesian velocity control mode  
  > &ensp;&ensp;&ensp;&ensp;6: joint online trajectory planning mode  
  > &ensp;&ensp;&ensp;&ensp;7: cartesian online trajectory planning mode  
  > @param detection_param: teaching detection parameters, default is 0  
  > &ensp;&ensp;&ensp;&ensp;0: turn on motion detection  
  > &ensp;&ensp;&ensp;&ensp;1: trun off motion detection  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.1  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. only available if set_mode(2)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_servo_attach(int servo_id)__
  > Attach the servo
  > 
  > @param servo_id: servo id, 1-8, 8(attach all servo)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_servo_detach(int servo_id)__
  > Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.
  >
  > @param servo_id: servo id, 1-8, 8(detach all servo)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int clean_error(void)__
  > Clean the controller error, need to be manually enabled motion and set state after clean error
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int clean_warn(void)__
  > Clean the controller warn
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_pause_time(float sltime)__
  > Set the arm pause time, xArm will pause sltime second
  > 
  > @param sltime: sleep second  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_collision_sensitivity(int sensitivity, bool wait = true)__
  > Set the sensitivity of collision
  >
  > @param sensitivity: sensitivity value, 0~5  
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_teach_sensitivity(int sensitivity, bool wait = true)__
  > Set the sensitivity of drag and teach
  > 
  > @param sensitivity: sensitivity value, 1~5  
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_gravity_direction(float gravity_dir[3], bool wait = true)__
  > Set the gravity direction for proper torque compensation and collision detection.
  > 
  > @param gravity_dir: Gravity direction vector [x, y, z], e.g., [0, 0, -1] for a floor-mounted arm.
  > @param wait: Whether to wait for the robotic arm to stop or clear all previous queued commands before applying the setting.  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int clean_conf(void)__
  > Clean current config and restore system default settings  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. This interface will clear the current settings and restore to the original settings (system default settings)
  >
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.  


- __int save_conf(void)__
  > Save config  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. This interface can record the current settings and will not be lost after the restart.  
  > &ensp;&ensp;&ensp;&ensp;2. The clean_conf interface can restore system default settings  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_position(float pose[6], float radius=-1, float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT, bool relative = false, unsigned char motion_type=0)__
- __int set_position(float pose[6], float radius, bool wait, float timeout=NO_TIMEOUT, bool relative=false, unsigned char motion_type=0)__
- __int set_position(float pose[6], bool wait, float timeout=NO_TIMEOUT, bool relative=false, unsigned char motion_type=0)__
  > Set the position  
  > &ensp;&ensp;&ensp;&ensp;MoveLine: Linear motion  
  > &ensp;&ensp;&ensp;&ensp;MoveArcLine: Linear arc motion with interpolation  
  >
  > @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine  
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed  
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc  
  > @param mvtime: reserved, 0  
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  
  > @param relative: relative move or not  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.100  
  > @param motion_type: motion planning type, default is 0  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 0: default, linear planning  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 2: direct transfer to IK using joint planning  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. when motion_type is 1 or 2, linear motion cannot be guaranteed  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`speed = speed / max_tcp_speed * max_joint_speed`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`acc = acc / max_tcp_acc * max_joint_acc`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. if there is no suitable IK, a C40 error will be triggered  
  >
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_tool_position(float pose[6], float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT, unsigned char motion_type=0)__
- __int set_tool_position(float pose[6], bool wait, float timeout=NO_TIMEOUT, unsigned char motion_type=0)__
  > Movement relative to the tool coordinate system  
  > &ensp;&ensp;&ensp;&ensp;MoveToolLine: Linear motion  
  > &ensp;&ensp;&ensp;&ensp;MoveToolArcLine: Linear arc motion with interpolation  
  > 
  > @param pose: the coordinate relative to the current tool coordinate systemion, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed  
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc  
  > @param mvtime: reserved, 0  
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  
  > @param radius: move radius, if radius less than 0, will MoveToolLine, else MoveToolArcLine  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.100  
  > @param motion_type: motion planning type, default is 0  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 0: default, linear planning   
  > &ensp;&ensp;&ensp;&ensp;motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 2: direct transfer to IK using joint planning  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. when motion_type is 1 or 2, linear motion cannot be guaranteed  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`speed = speed / max_tcp_speed * max_joint_speed`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`acc = acc / max_tcp_acc * max_joint_acc`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. if there is no suitable IK, a C40 error will be triggered  
  >  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_servo_angle(float angles[7], float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT, float radius = -1)__
- __int set_servo_angle(float angles[7], bool wait, float timeout=NO_TIMEOUT, float radius = -1, bool relative = false)__
- __int set_servo_angle(int servo_id, float angle, float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT, float radius = -1, bool relative = false)__
- __int set_servo_angle(int servo_id, float angle, bool wait, float timeout=NO_TIMEOUT, float radius = -1, bool relative = false)__
  > Set the servo angle
  > 
  > @param angles: angles, like [servo-1, ..., servo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees  
  > @param servo_id: servo id, 1~7, specify the joint ID to set  
  > @param angle: servo angle, use with servo_id parameters  
  > @param speed: move speed (rad/s or °/s), default is this.last_used_joint_speed  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @param acc: move acceleration (rad/s^2 or °/s^2), default is this.last_used_joint_acc  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of acc should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of acc should be in degrees  
  > @param mvtime: reserved, 0  
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  
  > @param radius: move radius, if radius less than 0, will MoveJoint, else MoveArcJoint  
  > &ensp;&ensp;&ensp;&ensp;The blending radius cannot be greater than the track length.  
  > @param relative: relative move or not  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.100  
  >
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_servo_angle_j(float angles[7], float speed=0, float acc=0, float mvtime=0)__
  > Set the servo angle, execute only the last instruction, need to be set to servo motion mode(this.set_mode(1))  
  > 
  > @param angles: angles, like [servo-1, ..., servo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees  
  > @param speed: reserved, move speed (rad/s or °/s)  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @param acc: reserved, move acceleration (rad/s^2 or °/s^2)  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of acc should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of acc should be in degrees  
  > @param mvtime: reserved, 0  
  >   
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_servo_cartesian(float pose[6], float speed=0, float acc=0, float mvtime=0,  bool is_tool_coord = false)__
  > Servo cartesian motion, execute only the last instruction, need to be set to servo motion mode(this.set_mode(1))  
  > 
  > @param pose: position, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param speed: reserved, move speed (mm/s)  
  > @param mvacc: reserved, move acceleration (mm/s^2)  
  > @param mvtime: reserved, 0  
  > @param is_tool_coord: is tool coordinate or not  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int move_circle(float pose1[6], float pose2[6], float percent, float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT, bool is_tool_coord=false, bool is_axis_angle=false)__
  > The motion calculates the trajectory of the space circle according to the three-point coordinates.  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;The three-point coordinates are (current starting point, pose1, pose2).  
  > 
  > @param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param percent: the percentage of arc length and circumference of the movement  
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed  
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc  
  > @param mvtime: 0, reserved  
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  
  > @param is_tool_coord: is tool coord or not, default is false, only available if firmware_version >= 1.11.100  
  > @param is_axis_angle: is axis angle or not, default is false, only available if firmware_version >= 1.11.100  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int move_gohome(float speed=0, float acc=0, float mvtime=0, bool wait=false, float timeout=NO_TIMEOUT)__
__int move_gohome(bool wait=false, float timeout=NO_TIMEOUT)__
  > Move to go home (Back to zero)  
  >
  > @param speed: move speed (rad/s or °/s), default is 50 °/s  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @param acc: move acceleration (rad/s^2 or °/s^2), default is 1000 °/s^2  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of acc should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of acc should be in degrees  
  > @param mvtime: reserved, 0  
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __void reset(bool wait=false, float timeout=NO_TIMEOUT)__
  > Reset  
  > 
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  


- __void emergency_stop(void)__
  > Emergency stop


- __int set_tcp_offset(float pose_offset[6], bool wait = true)__
  > Set the tool coordinate system offset at the end  
  > 
  > @param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.  


- __int set_tcp_load(float weight, float center_of_gravity[3], bool wait = false)__
  > Set the tcp load
  > 
  > @param weight: load weight (unit: kg)  
  > @param center_of_gravity: tcp load center of gravity, like [x(mm), y(mm), z(mm)]  
  > @param wait: whether to wait for the command to be executed or the robotic arm to stop  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.  


- __int set_tcp_jerk(float jerk)__
  > Set the translational jerk of Cartesian space  
  > 
  > @param jerk: jerk (mm/s^3)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_tcp_maxacc(float acc)__
  > Set the max translational acceleration of Cartesian space  
  > 
  > @param acc: max acceleration (mm/s^2)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_joint_jerk(float jerk)__
  > Set the jerk of Joint space  
  > 
  > @param jerk: jerk (°/s^3 or rad/s^3)  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of jerk should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of jerk should be in degrees  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_joint_maxacc(float acc)__
  > Set the max acceleration of Joint space
  > 
  > @param acc: max acceleration (°/s^2 or rad/s^2)  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of jerk should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of jerk should be in degrees  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_inverse_kinematics(float pose[6], float angles[7])__
  > Get inverse kinematics
  > 
  > @param pose: source pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param angles: target angles, like [servo-1, ..., servo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_forward_kinematics(float angles[7], float pose[6])__
  > Get forward kinematics
  > 
  > @param angles: source angles, like [servo-1, ..., servo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees  
  > @param pose: target pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int is_tcp_limit(float pose[6], int *limit)__
  > Check the tcp pose is in limit
  > 
  > @param pose: pose, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param limit: 1: limit, 0: no limit  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int is_joint_limit(float angles[7], int *limit)__
  > Check the joint is in limit
  > 
  > @param angles: angles, like [servo-1, ..., servo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of servo-1/.../servo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of servo-1/.../servo-7 should be in degrees  
  > @param limit: 1: limit, 0: no limit  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_gripper_enable(bool enable)__
  > Set the gripper enable
  > 
  > @param enable: enable or not  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.    


- __int set_gripper_mode(int mode)__
  > Set the gripper mode
  > 
  > @param mode:  
  > &ensp;&ensp;&ensp;&ensp;1: location mode  
  > &ensp;&ensp;&ensp;&ensp;2: speed mode(no use)  
  > &ensp;&ensp;&ensp;&ensp;3: torque mode(no use)  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_gripper_position(float *pos)__
  > Get the gripper position
  > 
  > @param pos: used to store the results obtained  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_gripper_position(float pos, bool wait=false, float timeout=10, bool wait_motion = true)__
  > Set the gripper position
  > 
  > @param pos: gripper position  
  > @param wait: wait or not, default is false  
  > @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_gripper_speed(float speed)__
  > Set the gripper speed
  > 
  > @param speed:  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_gripper_err_code(int *err)__
  > Get the gripper error code
  > 
  > @param err: used to store the results obtained  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int clean_gripper_error(void)__
  > Clean the gripper error
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_tgpio_digital(int *io0_value, int *io1_value)__
  > Get the digital value of the Tool GPIO
  > 
  > @param io0_value: the digital value of Tool GPIO-0  
  > @param io1_value: the digital value of Tool GPIO-1  
  > @param io2_value: the digital value of Tool GPIO-2  
  > @param io3_value: the digital value of Tool GPIO-3  
  > @param io4_value: the digital value of Tool GPIO-4  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_tgpio_digital(int ionum, int value, float delay_sec=0)__
  > Set the digital value of the specified Tool GPIO
  > 
  > @param ionum: ionum, 0 or 1  
  > @param value: the digital value of the specified io  
  > @param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)  
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  > &ensp;&ensp;&ensp;&ensp;2. only available if delay_sec <= 0  
  >  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_tgpio_analog(int ionum, float *value)__
  > Get the analog value of the specified Tool GPIO
  > 
  > @param ionum: ionum, 0 or 1  
  > @param value: the analog value of the specified tool io  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_cgpio_digital(int *digitals)__
  > Get the digital value of the specified Controller GPIO
  > 
  > @param digitals: the values of the controller GPIO  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_cgpio_analog(int ionum, float *value)__
  > Get the analog value of the specified Controller GPIO
  > 
  > @param ionum: ionum, 0 or 1  
  > @param value: the analog value of the specified controller io  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cgpio_digital(int ionum, int value, float delay_sec=0)__
  > Set the digital value of the specified Controller GPIO
  > 
  > @param ionum: ionum, 0 ~ 15  
  > @param value: the digital value of the specified io  
  > @param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)  
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  > &ensp;&ensp;&ensp;&ensp;2. only available if delay_sec <= 0  
  >  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cgpio_analog(int ionum, float value)__
  > Set the analog value of the specified Controller GPIO
  > 
  > @param ionum: ionum, 0 or 1  
  > @param value: the analog value of the specified io  
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  >  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cgpio_digital_input_function(int ionum, int fun)__
  > Set the digital input functional mode of the Controller GPIO
  > 
  > @param ionum: ionum, 0 ~ 15  
  > @param fun: functional mode  
  > &ensp;&ensp;&ensp;&ensp;0: general input  
  > &ensp;&ensp;&ensp;&ensp;1: external emergency stop  
  > &ensp;&ensp;&ensp;&ensp;2: protection reset   
  > &ensp;&ensp;&ensp;&ensp;11: offline task  
  > &ensp;&ensp;&ensp;&ensp;12: teaching mode  
  > &ensp;&ensp;&ensp;&ensp;13: reduced mode  
  > &ensp;&ensp;&ensp;&ensp;14: enable arm  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cgpio_digital_output_function(int ionum, int fun)__
  > Set the digital output functional mode of the specified Controller GPIO
  > 
  > @param ionum: ionum, 0 ~ 15  
  > @param fun: functional mode  
  > &ensp;&ensp;&ensp;&ensp;0: general output  
  > &ensp;&ensp;&ensp;&ensp;1: emergency stop  
  > &ensp;&ensp;&ensp;&ensp;2: in motion  
  > &ensp;&ensp;&ensp;&ensp;11: has error  
  > &ensp;&ensp;&ensp;&ensp;12: has warn  
  > &ensp;&ensp;&ensp;&ensp;13: in collision  
  > &ensp;&ensp;&ensp;&ensp;14: in teaching  
  > &ensp;&ensp;&ensp;&ensp;15: in offline task  
  > &ensp;&ensp;&ensp;&ensp;16: in reduced mode  
  > &ensp;&ensp;&ensp;&ensp;17: is enabled  
  > &ensp;&ensp;&ensp;&ensp;18: emergency stop is pressed  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_cgpio_state(int *state, int *digit_io, float *analog, int *input_conf, int *output_conf, int *input_conf2 = NULL, int *output_conf2 = NULL)__
  > Get the state of the Controller GPIO
  > 
  > @param state: contorller gpio module state and controller gpio module error code  
  > &ensp;&ensp;&ensp;&ensp;state[0]: contorller gpio module state  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;state[0] == 0: normal  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;state[0] == 1：wrong  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;state[0] == 6：communication failure  
  > &ensp;&ensp;&ensp;&ensp;state[1]: controller gpio module error code  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;state[1] == 0: normal  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;state[1] != 0：error code  
  > @param digit_io:  
  > &ensp;&ensp;&ensp;&ensp;digit_io[0]: digital input functional gpio state  
  > &ensp;&ensp;&ensp;&ensp;digit_io[1]: digital input configuring gpio state  
  > &ensp;&ensp;&ensp;&ensp;digit_io[2]: digital output functional gpio state  
  > &ensp;&ensp;&ensp;&ensp;digit_io[3]: digital output configuring gpio state  
  > @param analog:  
  > &ensp;&ensp;&ensp;&ensp;analog[0]: analog-0 input value  
  > &ensp;&ensp;&ensp;&ensp;analog[1]: analog-1 input value  
  > &ensp;&ensp;&ensp;&ensp;analog[2]: analog-0 output value  
  > &ensp;&ensp;&ensp;&ensp;analog[3]: analog-1 output value  
  > @param input_conf: digital(0-7) input functional info  
  > @param output_conf: digital(0-7) output functional info   
  > @param input_conf2: digital(8-15) input functional info  
  > @param output_conf2: digital(8-15) output functional info  
  >
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int register_report_location_callback(void(*callback)(const float *pose, const float *angles))__
- __int register_report_location_callback(std::function<void (const float*, const float*)> callback)__
  > Register the report location callback


- __int register_connect_changed_callback(void(*callback)(bool connected, bool reported))__
- __int register_connect_changed_callback(std::function<void (bool, bool)> callback)__
  > Register the connect status changed callback


- __int register_state_changed_callback(void(*callback)(int state))__
- __int register_state_changed_callback(std::function<void (int)> callback)__
  > Register the state status changed callback


- __int register_mode_changed_callback(void(*callback)(int mode))__
- __int register_mode_changed_callback(std::function<void (int)> callback)__
  > Register the mode changed callback


- __int register_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake))__
- __int register_mtable_mtbrake_changed_callback(std::function<void (int, int)> callback)__
  > Register the motor enable states or motor brake states changed callback


- __int register_error_warn_changed_callback(void(*callback)(int err_code, int warn_code))__
- __int register_error_warn_changed_callback(std::function<void (int, int)> callback)__
  > Register the error code or warn code changed callback


- __int register_cmdnum_changed_callback(void(*callback)(int cmdnum))__
- __int register_cmdnum_changed_callback(std::function<void (int)> callback)__
  > Register the cmdnum changed callback


- __int register_temperature_changed_callback(void(*callback)(const float *temps))__
- __int register_temperature_changed_callback(std::function<void (const float*)> callback)__
  > Register the temperature changed callback


- __int register_count_changed_callback(void(*callback)(int count))__
- __int register_count_changed_callback(std::function<void (int)> callback)__
  > Register the value of counter changed callback


- __int register_iden_progress_changed_callback(void(*callback)(int progress))__
- __int register_iden_progress_changed_callback(std::function<void (int)> callback)__
  > Register the progress of identification changed callback

- __int register_feedback_callback(void(*callback)(unsigned char *feedback_data))__
- __int register_feedback_callback(std::function<void (unsigned char *feedback_data)> callback)__
  > Register the feedback data callback  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;only available if firmware_version >= 2.1.0  
  >    
  > feedback_data: unsigned char data[]  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[0:2]: transaction id, (Big-endian conversion to unsigned 16-bit integer data), command ID corresponding to the feedback, consistent with issued instructions  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: this can be used to distinguish which instruction the feedback belongs to  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[4:6]: feedback_length, feedback_length == len(data) - 6, (Big-endian conversion to unsigned 16-bit integer data)  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[8]: feedback type  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: the motion task starts executing  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: the motion task execution ends or motion task is discarded(usually when the distance is too close to be planned)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4: the non-motion task is triggered  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[9]: feedback funcode, command code corresponding to feedback, consistent with issued instructions  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: this can be used to distinguish what instruction the feedback belongs to  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[10:12]: feedback taskid, (Big-endian conversion to unsigned 16-bit integer data)  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[12]: feedback code, execution status code, generally only meaningful when the feedback type is end, normally 0, 2 means motion task is discarded  
  > &ensp;&ensp;&ensp;&ensp;feedback_data[13:21]: feedback us, (Big-endian conversion to unsigned 64-bit integer data), time when feedback triggers (microseconds)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: this time is the corresponding controller system time when the feedback is triggered  


- __int release_report_location_callback(void(*callback)(const float *pose, const float *angles)=NULL)__
  > Release the location report callback
  > 
  > @param callback: NULL means to release all callbacks


- __int release_connect_changed_callback(void(*callback)(bool connected, bool reported)=NULL)__
  > Release the connect changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_state_changed_callback(void(*callback)(int state)=NULL)__
  > Release the state changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_mode_changed_callback(void(*callback)(int mode)=NULL)__
  > Release the mode changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_mtable_mtbrake_changed_callback(void(*callback)(int mtable, int mtbrake)=NULL)__
  > Release the motor enable states or motor brake states changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_error_warn_changed_callback(void(*callback)(int err_code, int warn_code)=NULL)__
  > Release the error warn changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_cmdnum_changed_callback(void(*callback)(int cmdnum)=NULL)__
  > Release the cmdnum changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_temperature_changed_callback(void(*callback)(const float *temps)=NULL)__
  > Release the temperature changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_count_changed_callback(void(*callback)(int count)=NULL)__
  > Release the value of counter changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int release_iden_progress_changed_callback(void(*callback)(int progress) = NULL)__
  > Release the progress of identification changed callback
  > 
  > @param callback: NULL means to release all callbacks for the same event

- __int release_feedback_callback(void(*callback)(unsigned char *feedback_data) = NULL)__
  > Release the feedback data callback    
  > 
  > @param callback: NULL means to release all callbacks for the same event


- __int get_vacuum_gripper(int *val)__
  > Get vacuum gripper state, alias for `get_suction_cup`
  > 
  > @param val:  
  > &ensp;&ensp;&ensp;&ensp;0: vacuum gripper is off  
  > &ensp;&ensp;&ensp;&ensp;1: vacuum gripper is on  
  > @param hardware_version:  
  > &ensp;&ensp;&ensp;&ensp;1: Plug-in Connection, default  
  > &ensp;&ensp;&ensp;&ensp;2: Contact Connection  
  >  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_vacuum_gripper(bool on, bool wait=false, float timeout=3, float delay_sec=0)__
  > Set vacuum gripper, alias for `set_suction_cup`
  > 
  > @param on: open vacuum gripper or not  
  > @param wait: wait or not, default is false  
  > @param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is true  
  > @param delay_sec: delay effective time from the current start, in seconds, default is 0(effective immediately)  
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  > &ensp;&ensp;&ensp;&ensp;2. only available if delay_sec <= 0  
  > @param hardware_version:  
  > &ensp;&ensp;&ensp;&ensp;1: Plug-in Connection, default  
  > &ensp;&ensp;&ensp;&ensp;2: Contact Connection  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_gripper_version(unsigned char versions[3])__
  > Get gripper version, only for debug
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_servo_version(unsigned char versions[3], int servo_id=1)__
  > Get servo version, only for debug
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_tgpio_version(unsigned char versions[3])__
  > Get tool gpio version, only for debug
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int reload_dynamics(void)__
  > Reload dynamics, only for debug
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_reduced_mode(bool on)__
  > Turn on/off reduced mode
  > 
  > @param on: on/off  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_reduced_max_tcp_speed(float speed)__
  > Set the maximum tcp speed of the reduced mode
  > 
  > @param speed: the maximum tcp speed  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_reduced_max_joint_speed(float speed)__
  > Set the maximum joint speed of the reduced mode
  > 
  > @param speed: the maximum joint speed  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_reduced_mode(int *mode)__
  > Get reduced mode
  > 
  > @param mode:  
  > &ensp;&ensp;&ensp;&ensp;0: reduced mode is on  
  > &ensp;&ensp;&ensp;&ensp;1: reduced mode is off  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14]=NULL, int *fense_is_on=NULL, int *collision_rebound_is_on=NULL)__
  > Get states of the reduced mode
  > 
  > @param on:  
  > &ensp;&ensp;&ensp;&ensp;0: reduced mode is on  
  > &ensp;&ensp;&ensp;&ensp;1: reduced mode is off  
  > @param xyz_list: the tcp boundary, like [reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min]  
  > @param tcp_speed: the maximum tcp speed of reduced mode  
  > @param joint_speed: the maximum joint speed of reduced mode  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @param jrange: the joint range of the reduced mode, like [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @param fense_is_on:  
  > &ensp;&ensp;&ensp;&ensp;0: safety mode is on  
  > &ensp;&ensp;&ensp;&ensp;1: safety mode is off  
  > @param collision_rebound_is_on:  
  > &ensp;&ensp;&ensp;&ensp;0: collision rebound is on  
  > &ensp;&ensp;&ensp;&ensp;1: collision rebound is off  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_reduced_tcp_boundary(int boundary[6])__
  > Set the boundary of the safety boundary mode
  > 
  > @param boundary: like [x_max(mm), x_min(mm), y_max(mm), y_min(mm), z_max(mm), z_min(mm)]  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_reduced_joint_range(float jrange[14])__
  > Set the joint range of the reduced mode
  > 
  > @param jrange: like [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of speed should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of speed should be in degrees  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_fence_mode(bool on)__
  > Turn on/off safety mode, alias for `set_fense_mode`
  > 
  > @param on: on/off  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_collision_rebound(bool on)__
  > Turn on/off collision rebound
  > 
  > @param on: on/off  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_world_offset(float pose_offset[6], bool wait = true)__
  > Set the base coordinate system offset at the end
  > 
  > @param pose_offset: tcp offset, like [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of roll/pitch/yaw should be in degrees  
  > @param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int start_record_trajectory(void)__
  > Start trajectory recording, only in teach mode, so you need to set joint teaching mode before.
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int stop_record_trajectory(char* filename=NULL)__
  > Stop trajectory recording
  > 
  > @param filename: the name to save  
  > &ensp;&ensp;&ensp;&ensp;If the filename is NULL, just stop recording, do not save, you need to manually call `save_record_trajectory` save before changing the mode. otherwise it will be lost the trajectory is saved in the controller box.  
  > &ensp;&ensp;&ensp;&ensp;This action will overwrite the trajectory with the same name
      empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int save_record_trajectory(char* filename, float timeout=10)__
  > Save the trajectory you just recorded
  > 
  > @param filename: the name to save  
  > &ensp;&ensp;&ensp;&ensp;The trajectory is saved in the controller box.  
  > &ensp;&ensp;&ensp;&ensp;This action will overwrite the trajectory with the same name empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int load_trajectory(char* filename, float timeout=10)__
  > Load the trajectory
  > 
  > @param filename: the name of the trajectory to load  
  > @param timeout: the maximum timeout waiting for loading to complete, default is 10 seconds.  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int playback_trajectory(int times=1, char* filename=NULL, bool wait=false, int double_speed=1)__
  > Playback trajectory
  > 
  > @param times: number of playbacks.  
  > @param filename: the name of the trajectory to play back  
  > &ensp;&ensp;&ensp;&ensp;if filename is None, you need to manually call the `load_trajectory` to load the trajectory.  
  > @param wait: whether to wait for the arm to complete, default is false.  
  > @param double_speed: double speed, only support 1/2/4, default is 1, only available if version > 1.2.11  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_trajectory_rw_status(int *status)__
  > Get trajectory read/write status
  > 
  > @param status:  
  > &ensp;&ensp;&ensp;&ensp;0: no read/write  
  > &ensp;&ensp;&ensp;&ensp;1: loading  
  > &ensp;&ensp;&ensp;&ensp;2: load success  
  > &ensp;&ensp;&ensp;&ensp;3: load failed  
  > &ensp;&ensp;&ensp;&ensp;4: saving  
  > &ensp;&ensp;&ensp;&ensp;5: save success  
  > &ensp;&ensp;&ensp;&ensp;6: save failed  
  >@return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_counter_reset(void)__
  > Reset counter value
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


  __int set_counter_increase(void)__
  > Set counter plus 1
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r)__
  > Set the digital value of the specified Tool GPIO when the robot has reached the specified xyz position
  > 
  > @param ionum: 0 or 1  
  > @param value: value  
  > @param xyz: position xyz, as [x, y, z]  
  > @param tol_r: fault tolerance radius
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r)__
  > Set the digital value of the specified Controller GPIO when the robot has reached the specified xyz position
  > 
  > @param ionum: 0 ~ 7  
  > @param value: value  
  > @param xyz: position xyz, as [x, y, z]  
  > @param tol_r: fault tolerance radius
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r)__
  > Set the analog value of the specified Controller GPIO when the robot has reached the specified xyz position
  > 
  > @param ionum: 0 ~ 1  
  > @param value: value, 0~10.0  
  > @param xyz: position xyz, as [x, y, z]  
  > @param tol_r: fault tolerance radius
  >
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int config_tgpio_reset_when_stop(bool on_off)__
  > Config the Tool GPIO reset the digital output when the robot is in stop state
  > 
  > @param on_off: true/false
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int config_cgpio_reset_when_stop(bool on_off)__
  > Config the Controller GPIO reset the digital output when the robot is in stop state
  > 
  > @param on_off: true/false
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_position_aa(float pose[6], float speed = 0, float acc = 0, float mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, float timeout = NO_TIMEOUT, unsigned char motion_type = 0)__
- __int set_position_aa(float pose[6], bool is_tool_coord, bool relative = false, bool wait = false, float timeout = NO_TIMEOUT, unsigned char motion_type = 0)__
  > Set the pose represented by the axis angle pose  
  > &ensp;&ensp;&ensp;&ensp;MoveLineAA: Linear motion  
  > &ensp;&ensp;&ensp;&ensp;MoveArcLineAA: Linear arc motion with interpolation  
  > 
  > @param pose: the axis angle pose, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is true, the value of rx/ry/rz should be in radians  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is false, The value of rx/ry/rz should be in degrees  
  > @param speed: move speed (mm/s, rad/s), default is this.last_used_tcp_speed  
  > @param mvacc: move acceleration (mm/s^2, rad/s^2), default is this.last_used_tcp_acc  
  > @param mvtime: reserved, 0  
  > @param is_tool_coord: is tool coordinate or not, if it is true, the relative parameter is no longer valid  
  > @param relative: relative move or not  
  > @param wait: whether to wait for the arm to complete, default is false  
  > @param timeout: maximum waiting time(unit: second), default is no timeout, only valid if wait is true  
  > @param radius: move radius, if radius less than 0, will MoveLineAA, else MoveArcLineAA  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.100  
  > @param motion_type: motion planning type, default is 0  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 0: default, linear planning  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible  
  > &ensp;&ensp;&ensp;&ensp;motion_type == 2: direct transfer to IK using joint planning  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. when motion_type is 1 or 2, linear motion cannot be guaranteed  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`speed = speed / max_tcp_speed * max_joint_speed`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`acc = acc / max_tcp_acc * max_joint_acc`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. if there is no suitable IK, a C40 error will be triggered  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_servo_cartesian_aa(float pose[6], float speed = 0, float acc = 0, bool is_tool_coord = false, bool relative = false)__
- __int set_servo_cartesian_aa(float pose[6], bool is_tool_coord, bool relative = false)__
  > Set the servo cartesian represented by the axis angle pose, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.4.7
  > 
  > @param pose: the axis angle pose, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is true, the value of rx/ry/rz should be in radians  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is false, The value of rx/ry/rz should be in degrees  
  > @param speed: reserved, move speed (mm/s)  
  > @param mvacc: reserved, move acceleration (mm/s^2)  
  > @param is_tool_coord: is tool coordinate or not  
  > @param relative: relative move or not  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in = 0, int orient_type_out = 0)__
  > Calculate the pose offset of two given points
  > 
  > @param pose1: position, like [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is true, the value of roll/rx/pitch/ry/yaw/rz should be in radians  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is false, The value of roll/rx/pitch/ry/yaw/rz should be in degrees  
  > @param pose2: position, like [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is true, the value of roll/rx/pitch/ry/yaw/rz should be in radians  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is false, The value of roll/rx/pitch/ry/yaw/rz should be in degrees  
  > @param offset: the offset between pose1 and pose2  
  > @param orient_type_in: input attitude notation, 0 is RPY (default), 1 is axis angle  
  > @param orient_type_out: notation of output attitude, 0 is RPY (default), 1 is axis angle
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_position_aa(float pose[6])__
  > Get the pose represented by the axis angle pose
  > 
  > @param pose: the pose represented by the axis angle pose of xArm, like [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is true, the value of rx/ry/rz should be in radians  
  > &ensp;&ensp;&ensp;&ensp;Note: if default_is_radian is false, The value of rx/ry/rz should be in degrees  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int robotiq_reset(unsigned char ret_data[6] = NULL)__
  > Reset the robotiq gripper (clear previous activation if any)
  > 
  > @param ret_data: the response from robotiq
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int robotiq_set_activate(bool wait = true, float timeout = 3, unsigned char ret_data[6] = NULL)__
- __int robotiq_set_activate(bool wait = true, unsigned char ret_data[6] = NULL)__
- __int robotiq_set_activate(unsigned char ret_data[6] = NULL)__
  > If not already activated. Activate the robotiq gripper
  > 
  > @param wait: whether to wait for the robotiq activate complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 3, only available if wait=true  
  > @param ret_data: the response from robotiq
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int robotiq_set_position(unsigned char pos, unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, float timeout = 5,  unsigned char ret_data[6] = NULL)__
- __int robotiq_set_position(unsigned char pos, bool wait = true, float timeout = 5, unsigned char ret_data[6] = NULL)__
- __int robotiq_set_position(unsigned char pos, bool wait = true, unsigned char ret_data[6] = NULL)__
- __int robotiq_set_position(unsigned char pos, unsigned char ret_data[6] = NULL)__
  > Go to the position with determined speed and force.
  > 
  > @param pos: position of the gripper. Integer between 0 and 255. 0 being the open position and 255 being the close position.  
  > @param speed: gripper speed between 0 and 255  
  > @param force: gripper force between 0 and 255  
  > @param wait: whether to wait for the robotion motion complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true  
  > @param ret_data: the response from robotiq
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int robotiq_open(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, float timeout = 5, unsigned char ret_data[6] = NULL)__
- __int robotiq_open(bool wait = true, float timeout = 5, unsigned char ret_data[6] = NULL)__
- __int robotiq_open(bool wait = true, unsigned char ret_data[6] = NULL)__
- __int robotiq_open(unsigned char ret_data[6] = NULL)__
  > Open the robotiq gripper
  > 
  > @param speed: gripper speed between 0 and 255  
  > @param force: gripper force between 0 and 255  
  > @param wait: whether to wait for the robotion motion complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true  
  > @param ret_data: the response from robotiq
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int robotiq_close(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, float timeout = 5, unsigned char ret_data[6] = NULL)__
- __int robotiq_close(bool wait = true, float timeout = 5, unsigned char ret_data[6] = NULL)__
- __int robotiq_close(bool wait = true, unsigned char ret_data[6] = NULL)__
- __int robotiq_close(unsigned char ret_data[6] = NULL)__
  > Close the robotiq gripper
  > 
  > @param speed: gripper speed between 0 and 255  
  > @param force: gripper force between 0 and 255  
  > @param wait: whether to wait for the robotion motion complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true  
  > @param ret_data: the response from robotiq
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers = 3)__
  > Reading the status of robotiq gripper
  > 
  > @param ret_data: the response from robotiq  
  > @param number_of_registers: number of registers, 1/2/3, default is 3  
  > &ensp;&ensp;&ensp;&ensp;number_of_registers=1: reading the content of register 0x07D0  
  > &ensp;&ensp;&ensp;&ensp;number_of_registers=2: reading the content of register 0x07D0/0x07D1  
  > &ensp;&ensp;&ensp;&ensp;number_of_registers=3: reading the content of register 0x07D0/0x07D1/0x07D2  
  > 
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;register 0x07D0: Register GRIPPER STATUS  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;register 0x07D1: Register FAULT STATUS and register POSITION REQUEST ECHO  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;register 0x07D2: Register POSITION and register CURRENT  
  > 
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_bio_gripper_enable(bool enable, bool wait = true, float timeout = 3)__
  > If not already enabled. Enable the bio gripper
  > 
  > @param enable: enable or not  
  > @param wait: whether to wait for the bio gripper enable complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 3, only available if wait=true  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_bio_gripper_speed(int speed)__
  > Set the speed of the bio gripper
  > 
  > @param speed: speed
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_bio_gripper_control_mode(int mode)__
  > Set the mode of the bio gripper
  > &ensp;&ensp;&ensp;&ensp;Note: Only available in the new version of BIO Gripper    
  > 
  > @param mode: mode
  > &ensp;&ensp;&ensp;&ensp;0: bio gripper opening and closing mode  
  > &ensp;&ensp;&ensp;&ensp;1: position loop mode  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_bio_gripper_force(int force)__
  > Set the force of the bio gripper
  > &ensp;&ensp;&ensp;&ensp;Note: Only available in the new version of BIO Gripper    
  > 
  > @param force: gripper force between 10 and 100  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_bio_gripper_position(int pos, int speed = 0, int force=100, bool wait = true, fp32 timeout = 5)__
  > Set the position of the bio gripper
  > &ensp;&ensp;&ensp;&ensp;Note: Only available in the new version of BIO Gripper    
  > 
  > @param pos: gripper pos between 71 and 150   
  > @param speed: gripper speed between 0 and 4500  
  > @param force: gripper force between 10 and 100  
  > @param wait: whether to wait for the bio gripper motion complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int open_bio_gripper(int speed = 0, bool wait = true, float timeout = 5)__
- __int open_bio_gripper(bool wait = true, float timeout = 5)__
  > Open the bio gripper
  > 
  > @param speed: speed value, default is 0 (not set the speed)  
  > @param wait: whether to wait for the bio gripper motion complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int close_bio_gripper(int speed = 0, bool wait = true, float timeout = 5)__
- __int close_bio_gripper(bool wait = true, float timeout = 5)__
  > Close the bio gripper
  > 
  > @param speed: speed value, default is 0 (not set the speed)  
  > @param wait: whether to wait for the bio gripper motion complete, default is true  
  > @param timeout: maximum waiting time(unit: second), default is 5, only available if wait=true  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_bio_gripper_status(int *status)__
  > Get the status of the bio gripper
  > 
  > @param status: the result of the bio gripper status value  
  > &ensp;&ensp;&ensp;&ensp;status & 0x03 == 0: stop  
  > &ensp;&ensp;&ensp;&ensp;status & 0x03 == 1: motion  
  > &ensp;&ensp;&ensp;&ensp;status & 0x03 == 2: catch  
  > &ensp;&ensp;&ensp;&ensp;status & 0x03 == 3: error  
  > &ensp;&ensp;&ensp;&ensp;(status >> 2) & 0x03 == 0: not enabled  
  > &ensp;&ensp;&ensp;&ensp;(status >> 2) & 0x03 == 1: enabling  
  > &ensp;&ensp;&ensp;&ensp;(status >> 2) & 0x03 == 2: enabled
  >
  > return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_bio_gripper_error(int *err)__
  > Get the error code of the bio gripper
  > 
  > @param err: the result of the bio gripper error code
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int clean_bio_gripper_error(void)__
  > Clean the error code of the bio gripper
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_tgpio_modbus_timeout(int timeout, bool is_transparent_transmission = false)__
  > Set the modbus timeout of the tool gpio
  > 
  > @param timeout: timeout, milliseconds  
  > @param is_transparent_transmission: whether the set timeout is the timeout of transparent transmission  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_tgpio_modbus_baudrate(int baud)__
  > Set the modbus baudrate of the tool gpio
  > 
  > @param baud: baudrate, 4800/9600/19200/38400/57600/115200/230400/460800/921600/1000000/1500000/2000000/2500000
  > 
  > return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_tgpio_modbus_baudrate(int *baud)__
  > Get the modbus baudrate of the tool gpio
  > 
  > @param baud: the result of baudrate
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length, unsigned char host_id = 9, bool is_transparent_transmission = false)__
  > Send the modbus data to the tool gpio
  > 
  > @param modbus_data: send data  
  > @param modbus_length: the length of the modbus_data  
  > @param ret_data: the response data of the modbus  
  > @param ret_length: the length of the response data  
  > @param host_id: host id  
  > &ensp;&ensp;&ensp;&ensp;9: END RS485  
  > &ensp;&ensp;&ensp;&ensp;10: Controller RS485  
  > @param is_transparent_transmission: whether to choose transparent transmission, default is false  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0 
  > @param use_503_port: whether to use port 503 for communication, default is false  
  > &ensp;&ensp;&ensp;&ensp;Note: if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_report_tau_or_i(int tau_or_i = 0)__
  > Set the reported torque or electric current
  > 
  > @param tau_or_i:  
  > &ensp;&ensp;&ensp;&ensp;0: torque  
  > &ensp;&ensp;&ensp;&ensp;1: electric current
  >
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_report_tau_or_i(int *tau_or_i)__
  > Get the reported torque or electric current
  > 
  > @param tau_or_i: the result of the tau_or_i
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_self_collision_detection(bool on)__
  > Set whether to enable self-collision detection 
  > 
  > @param on: enable or not
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_collision_tool_model(int tool_type, int n = 0, ...)__
  > Set the geometric model of the end effector for self collision detection
  > 
  > @param tool_type: the geometric model type  
  > &ensp;&ensp;&ensp;&ensp;0: No end effector, no additional parameters required  
  > &ensp;&ensp;&ensp;&ensp;1: xArm Gripper, no additional parameters required  
  > &ensp;&ensp;&ensp;&ensp;2: xArm Vacuum Gripper, no additional parameters required  
  > &ensp;&ensp;&ensp;&ensp;3: xArm Bio Gripper, no additional parameters required  
  > &ensp;&ensp;&ensp;&ensp;4: Robotiq-2F-85 Gripper, no additional parameters required  
  > &ensp;&ensp;&ensp;&ensp;5: Robotiq-2F-140 Gripper, no additional parameters required  
  > &ensp;&ensp;&ensp;&ensp;21: Cylinder, need additional parameters radius, height  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm->set_collision_tool_model(21, 2, radius, height)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;@param radius: the radius of cylinder, (unit: mm)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;@param height: the height of cylinder, (unit: mm)  
  > &ensp;&ensp;&ensp;&ensp;22: Cuboid, need additional parameters x, y, z  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm->set_collision_tool_model(22, 3, x, y, z)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;@param x: the length of the cuboid in the x coordinate direction, (unit: mm)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;@param y: the length of the cuboid in the y coordinate direction, (unit: mm)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;@param z: the length of the cuboid in the z coordinate direction, (unit: mm)  
  > 
  > @param n: the count of the additional parameters  
  > @param ...: additional parameters  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int vc_set_joint_velocity(float speeds[7], bool is_sync = true, float duration = -1.0)__
  > Joint velocity control, need to be set to joint velocity control mode(this.set_mode(4))
  > 
  > @param speeds: [spd_J1, spd_J2, ..., spd_J7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of spd_J1/.../spd_J1 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, the value of spd_J1/.../spd_J1 should be in degrees  
  > @param is_sync: whether all joints accelerate and decelerate synchronously, default is true  
  > @param duration: the maximum duration of the speed, over this time will automatically set the speed to 0.  
  > &ensp;&ensp;&ensp;&ensp;duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained  
  > &ensp;&ensp;&ensp;&ensp;duration == 0: always effective, will not stop automativally  
  > &ensp;&ensp;&ensp;&ensp;duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int vc_set_cartesian_velocity(float speeds[6], bool is_tool_coord = false, float duration = -1.0)__
  > Cartesian velocity control, need to be set to cartesian velocity control mode(self.set_mode(5))
  > 
  > @param speeds: [spd_x, spd_y, spd_z, spd_rx, spd_ry, spd_rz]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of spd_rx/spd_ry/spd_rz should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, the value of spd_rx/spd_ry/spd_rz should be in degrees  
  > @param is_tool_coord: is tool coordinate or not, default is false  
  > @param duration: the maximum duration of the speed, over this time will automatically set the speed to 0.  
  > &ensp;&ensp;&ensp;&ensp;duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained  
  > &ensp;&ensp;&ensp;&ensp;duration == 0: always effective, will not stop automativally  
  > &ensp;&ensp;&ensp;&ensp;duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int calibrate_tcp_coordinate_offset(float four_points[4][6], float ret_xyz[3])__
  > Four-point method to calibrate tool coordinate system position offset
  > 
  > @param four_points: a list of four teaching coordinate positions [x, y, z, roll, pitch, yaw]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, the value of roll/pitch/yaw should be in degrees  
  > @param ret_xyz: the result of the calculated xyz(mm) TCP offset, [x, y, z]  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int calibrate_tcp_orientation_offset(float rpy_be[3], float rpy_bt[3], float ret_rpy[3])__
  > An additional teaching point to calibrate the tool coordinate system attitude offset
  > 
  > @param rpy_be: the rpy value of the teaching point without TCP offset [roll, pitch, yaw]  
  > @param rpy_bt: the rpy value of the teaching point with TCP offset [roll, pitch, yaw]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, the value of roll/pitch/yaw should be in degrees  
  > @param ret_rpy: the result of the calculated rpy TCP offset, [roll, pitch, yaw]  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int calibrate_user_orientation_offset(float three_points[3][6], float ret_rpy[3], int mode = 0, int trust_ind = 0)__
  > Three-point method teaches user coordinate system posture offset
  > 
  > @param four_points: a list of teaching TCP coordinate positions [x, y, z, roll, pitch, yaw]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, the value of roll/pitch/yaw should be in degrees  
  > @param ret_rpy: the result of the calculated rpy user offset, [roll, pitch, yaw]
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int calibrate_user_coordinate_offset(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3])__
  > An additional teaching point determines the position offset of the user coordinate system.
  > 
  > @param rpy_ub: the confirmed offset of the base coordinate system in the user coordinate system [roll, pitch, yaw], which is the result of calibrate_user_orientation_offset()  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of roll/pitch/yaw should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, the value of roll/pitch/yaw should be in degrees  
  > @param pos_b_uorg: the position of the teaching point in the base coordinate system [x, y, z], if the arm cannot reach the target position, the user can manually input the position of the target in the base coordinate.  
  > @param ret_xyz: the result of the calculated xyz user offset, [x, y, z]
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_impedance(int imp_coord, int imp_c_axis[6], float M[6], float K[6], float B[6])__
  > Set all parameters of impedance control through the Six-axis Force Torque Sensor.  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3   
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param imp_coord: task frame. 0: base frame. 1: tool frame.  
  > @param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
  > @param M: mass. (kg)  
  > @param K: stiffness coefficient.  
  > @param B: damping coefficient. invalid.  
  > &ensp;&ensp;&ensp;&ensp;Note: the value is set to 2*sqrt(M*K) in controller.  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_impedance_mbk(float M[6], float K[6], float B[6])__
  > Set mbk parameters of impedance control through the Six-axis Force Torque Sensor.  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param M: mass. (kg)  
  > @param K: stiffness coefficient.  
  > @param B: damping coefficient. invalid.  
  > &ensp;&ensp;&ensp;&ensp;Note: the value is set to 2*sqrt(M*K) in controller.
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_impedance_config(int imp_coord, int imp_c_axis[6])__
  > Set impedance control parameters of impedance control through the Six-axis Force Torque Sensor.  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param imp_coord: task frame. 0: base frame. 1: tool frame.  
  > @param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int config_force_control(int f_coord, int f_c_axis[6], float f_ref[6], float f_limits[6])__
  > Set force control parameters through the Six-axis Force Torque Sensor.  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param f_coord: task frame. 0: base frame. 1: tool frame.  
  > @param f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
  > @param f_ref: the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.  
  > @param f_limits: for compliant axes, these values are the maximum allowed tcp speed along/about the axis.
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_force_control_pid(float kp[6], float ki[6], float kd[6], float xe_limit[6])__
  > Set force control pid parameters through the Six-axis Force Torque Sensor.  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param kp: proportional gain  
  > @param ki: integral gain.  
  > @param kd: differential gain.  
  > @param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int ft_sensor_set_zero(void)__
  > Set the current state to the zero point of the Six-axis Force Torque Sensor  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int ft_sensor_iden_load(float result[10])__
  > Identification the tcp load with the the Six-axis Force Torque Sensor  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;Note: the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
  > &ensp;&ensp;&ensp;&ensp;Note: tstarting from SDK v1.11.0, the centroid unit is millimeters (originally meters)  
  > 
  > @param result: the result of identification, ([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int ft_sensor_cali_load(float load[10], bool association_setting_tcp_load = false, float m = 0.325, float x = -17, float y = 9, float z = 11.8)__
  > Write the load offset parameters identified by the Six-axis Force Torque Sensor  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;Note: the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
  > &ensp;&ensp;&ensp;&ensp;Note: tstarting from SDK v1.11.0, the centroid unit is millimeters (originally meters)  
  > 
  > @param load: iden result([mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset])  
  > @param association_setting_tcp_load: whether to convert the paramster to the corresponding tcp load and set, default is false  
  > &ensp;&ensp;&ensp;&ensp;if true, the value of tcp load will be modified
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int ft_sensor_enable(int on_off)__
  > Used for enabling and disabling the use of the Six-axis Force Torque Sensor measurements in the controller.  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param on_off: enable or disable F/T data sampling.
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int ft_sensor_app_set(int app_code)__
  > Set robot to be controlled in force mode. (Through the Six-axis Force Torque Sensor)  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param app_code: force mode.  
  > &ensp;&ensp;&ensp;&ensp;0: non-force mode  
  > &ensp;&ensp;&ensp;&ensp;1: impendance control  
  > &ensp;&ensp;&ensp;&ensp;2: force control
  >
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int ft_sensor_app_get(int *app_code)__
  > Get force mode  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param app_code: the result of force mode.  
  > &ensp;&ensp;&ensp;&ensp;0: non-force mode  
  > &ensp;&ensp;&ensp;&ensp;1: impendance control  
  > &ensp;&ensp;&ensp;&ensp;2: force control
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_ft_sensor_data(float ft_data[6])__
  > Get the data of the Six-axis Force Torque Sensor  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param ft_data: the result of the Six-axis Force Torque Sensor.
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_ft_sensor_config(int *ft_app_status = NULL, int *ft_is_started = NULL, int *ft_type = NULL, int *ft_id = NULL, int *ft_freq = NULL, float *ft_mass = NULL, float *ft_dir_bias = NULL, float ft_centroid[3] = NULL, float ft_zero[6] = NULL, int *imp_coord = NULL, int imp_c_axis[6] = NULL, float M[6] = NULL, float K[6] = NULL, float B[6] = NULL, int *f_coord = NULL, int f_c_axis[6] = NULL, float f_ref[6] = NULL, float f_limits[6] = NULL, float kp[6] = NULL, float ki[6] = NULL, float kd[6] = NULL, float xe_limit[6] = NULL)__
  > Get the config of the Six-axis Force Torque Sensor  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param ft_app_status: force mode  
  > &ensp;&ensp;&ensp;&ensp;0: non-force mode  
  > &ensp;&ensp;&ensp;&ensp;1: impendance control  
  > &ensp;&ensp;&ensp;&ensp;2: force control  
  > @param ft_is_started: ft sensor is enable or not  
  > @param ft_type: ft sensor type  
  > @param ft_id: ft sensor id  
  > @param ft_freq: ft sensor frequency  
  > @param ft_mass: load mass  
  > @param ft_dir_bias:  
  > @param ft_centroid: [x_centroid，y_centroid，z_centroid]  
  > @param ft_zero: [Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]  
  > @param imp_coord: task frame of impendance control mode.  
  > &ensp;&ensp;&ensp;&ensp;0: base frame.  
  > &ensp;&ensp;&ensp;&ensp;1: tool frame.  
  > @param imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
  > @param M: mass. (kg)  
  > @param K: stiffness coefficient.  
  > @param B: damping coefficient. invalid.  
  > &ensp;&ensp;&ensp;&ensp;Note: the value is set to 2*sqrt(M*K) in controller.  
  > @param f_coord: task frame of force control mode.   
  > &ensp;&ensp;&ensp;&ensp;0: base frame.  
  > &ensp;&ensp;&ensp;&ensp;1: tool frame.  
  > @param f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
  > @param f_ref:  the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.  
  > @param f_limits:  for compliant axes, these values are the maximum allowed tcp speed along/about the axis.  
  > @param kp: proportional gain  
  > @param ki: integral gain.  
  > @param kd: differential gain.  
  > @param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_ft_sensor_error(int *err)__
  > Get the error code of the Six-axis Force Torque Sensor  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)
  > 
  > @param err: the result of ft sensor error code
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int iden_tcp_load(float result[4], float estimated_mass = 0.0)__
  > Identification the tcp load with current  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param result: the result of identification. [mass，x_centroid，y_centroid，z_centroid]  
  > @param estimated_mass: estimated mass  
  > &ensp;&ensp;&ensp;&ensp;Note: this parameter is only available on the Lite6 model manipulator, and this parameter must be specified for the Lite6 model manipulator  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_error(int *err)__
  > Get the error code of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param err: the result of linear track error
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_status(int *status)__
  > Get the status of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param status: the result of linear track status  
  > &ensp;&ensp;&ensp;&ensp;status & 0x00: motion finish.  
  > &ensp;&ensp;&ensp;&ensp;status & 0x01: in motion  
  > &ensp;&ensp;&ensp;&ensp;status & 0x02: has stop  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_pos(int *pos)__
  > Get the pos of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param pos: the result of linear track position
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_is_enabled(int *status)__
  > Get the linear track is enabled or not  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param status: the result of linear track status  
  > &ensp;&ensp;&ensp;&ensp;status == 0: linear track is not enabled  
  > &ensp;&ensp;&ensp;&ensp;status == 1: linear track is enabled  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_on_zero(int *status)__
  > Get the linear track is on zero positon or not  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param status: the result of linear track status  
  > &ensp;&ensp;&ensp;&ensp;status == 0: linear track is not on zero  
  > &ensp;&ensp;&ensp;&ensp;status == 1: linear track is on zero
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_sci(int *sci1)__
  > Get the sci1 value of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param sci1: the result of linear track sci1
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_linear_track_sco(int sco[2])__
  > Get the sco value of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param sco: the result of linear track sco0 and sco1
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int clean_linear_track_error(void)__
  > Clean the linear track error  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_linear_track_enable(bool enable)__
  > Set the linear track enable/disable  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param enable: enable or not
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_linear_track_speed(int speed)__
  > Set the speed of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param speed: Integer between 1 and 1000mm/s.
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_linear_track_back_origin(bool wait = true, bool auto_enable = true)__
  > Set the linear track go back to the origin position  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. only useful when powering on for the first time  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. this operation must be performed at the first power-on  
  > 
  > @param wait: wait to motion finish or not, default is true  
  > @param auto_enable: enable after back to origin or not, default is true
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_linear_track_pos(int pos, int speed = 0, bool wait = true, float timeout = 100, bool auto_enable = true)__
  > Set the position of the linear track  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0
  > 
  > @param pos: position. Integer between 0 and 700/1000/1500.  
  > &ensp;&ensp;&ensp;&ensp;If the SN of the linear track is start with AL1300, the position range is 0~700mm.  
  > &ensp;&ensp;&ensp;&ensp;If the SN of the linear track is start with AL1301, the position range is 0~1000mm.  
  > &ensp;&ensp;&ensp;&ensp;If the SN of the linear track is start with AL1302, the position range is 0~1500mm.  
  > @param speed: auto set the speed of the linear track if the speed is changed, Integer between of 1 and 1000mm/s, default is -1(not set)  
  > @param wait: wait to motion finish or not, default is true  
  > @param timeout: wait timeout, seconds, default is 100s.  
  > @param auto_enable: auto enable if not enabled, default is true  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_linear_track_stop(void)__
  > Set the linear track to stop  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_baud_checkset_enable(bool enable)__
  > Enable auto checkset the baudrate of the end IO board or not  
  > &ensp;&ensp;&ensp;&ensp;Note: only available in the API of gripper/bio/robotiq/linear_track.  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_checkset_default_baud(int type, int baud)__
  > Set the checkset baud value  
  > &ensp;&ensp;&ensp;&ensp;Note: do not modify at will, only use when the baud rate of the corresponding peripheral is changed
  > 
  > @param type: checkset type  
  > &ensp;&ensp;&ensp;&ensp;1: xarm gripper  
  > &ensp;&ensp;&ensp;&ensp;2: bio gripper  
  > &ensp;&ensp;&ensp;&ensp;3: robotiq gripper  
  > &ensp;&ensp;&ensp;&ensp;4: linear track  
  > @param baud: checkset baud value, less than or equal to 0 means disable checkset  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_checkset_default_baud(int type, int *baud)__
  > Get the checkset baud value
  > 
  > @param type: checkset type  
  > &ensp;&ensp;&ensp;&ensp;1: xarm gripper  
  > &ensp;&ensp;&ensp;&ensp;2: bio gripper  
  > &ensp;&ensp;&ensp;&ensp;3: robotiq gripper  
  > &ensp;&ensp;&ensp;&ensp;4: linear track  
  > @param baud: checkset baud value, less than or equal to 0 means disable checkset  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_cartesian_velo_continuous(bool on_off)__
  > Set cartesian motion velocity continuous  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.9.0  
  > 
  > @param on_off: continuous or not, default is false     
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_allow_approx_motion(bool on_off)__
  > Set allow to avoid overspeed near some singularities using approximate solutions  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.9.0  
  > 
  > @param on_off: allow or not, default is false  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int get_joint_states(float position[7], float velocity[7], float effort[7])__
  > Get the joint states  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.9.0  
  >
  > @param position: the angles of the joints, like [angle-1, ..., angle-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of angle-1/.../angle-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of angle-1/.../angle-7 should be in degrees  
  > @param velocity: the velocities of the joints, like [velo-1, ..., velo-7]  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is true, the value of velo-1/.../velo-7 should be in radians  
  > &ensp;&ensp;&ensp;&ensp;if default_is_radian is false, The value of velo-1/.../velo-7 should be in degrees  
  > @param effort: the efforts of the joints, like [effort-1, ..., effort-7]  
  > @return: see the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int iden_joint_friction(int *result, unsigned char *sn = NULL)__
  > Identification the friction  
  > &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.9.0  
  > 
  > @param result: the result of identification
  > &ensp;&ensp;&ensp;&ensp;0: success  
  > &ensp;&ensp;&ensp;&ensp;-1: failure  
  > @param sn: robot sn  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int open_lite6_gripper(void)__
  > Open the gripper of Lite6 series robotics arms  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
  >    
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  >  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int close_lite6_gripper(void)__
  > Close the gripper of Lite6 series robotics arms  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
  >    
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  >  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int stop_lite6_gripper(void)__
  > Stop the gripper of Lite6 series robotics arms  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
  >    
  > @param sync: whether to execute in the motion queue, set to false to execute immediately(default is true)  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
  >  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


- __int set_only_check_type(unsigned char only_check_type = 0)__
  > Set the motion process detection type (valid for all motion interfaces of the current SDK instance)  
  > &ensp;&ensp;&ensp;&ensp;Note1: only available if firmware_version >= 1.11.100   
  > &ensp;&ensp;&ensp;&ensp;Note2: This interface is a global configuration item of the current SDK, and affects all motion-related interfaces  
  > &ensp;&ensp;&ensp;&ensp;Note3: Generally, you only need to call when you don't want to move the robotic arm and only check whether some paths will have self-collision/angle-limit/cartesian-limit/overspeed. motion-related interfaces  
  > &ensp;&ensp;&ensp;&ensp;Note4: Currently only self-collision/angle-limit/cartesian-limit/overspeed are detected  
  > &ensp;&ensp;&ensp;&ensp;Note5: If only_check_type is set to be greater than 0, and the return value of calling the motion interface is not 0, you can view `arm.only_check_result` to view the specific error code  
  > &ensp;&ensp;&ensp;&ensp;Example: (Common scenarios, here is an example of the set_position interface)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;1. Check whether the process from point A to point B is normal (no self-collision and overspeed triggered)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1.1 Move to point A  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`arm->set_only_check_type(0)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(A)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1.2 Check if the process from point A to point B is normal (no self-collision and overspeed triggered)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`arm->set_only_check_type(1)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(B)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;// If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm->only_check_result  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`arm->set_only_check_type(0)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. Check whether the process from point A to point B, C, and D to point E is normal (no self-collision and overspeed are triggered)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2.1 Move to point A  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`arm->set_only_check_type(0)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(A)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2.2 Check whether the process of point A passing through points B, C, D to point E is normal (no self-collision and overspeed are triggered)  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`arm->set_only_check_type(3)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(B)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;// If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm->only_check_result  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(C)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;// If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm->only_check_result  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(D)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;// If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm->only_check_result  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`code = arm->set_position(E)`  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;// If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm->only_check_result  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;`arm->set_only_check_type(0)`
  > 
  > @param only_check_type: Motion Detection Type  
  > &ensp;&ensp;&ensp;&ensp;__only_check_type == 0__: Restore the original function of the motion interface, it will move, the default is 0  
  > &ensp;&ensp;&ensp;&ensp;__only_check_type == 1__: Only check the self-collision without moving, take the actual state of the manipulator as the initial planned path, and check whether the path has self-collision (the intermediate state will be updated at this time)  
  > &ensp;&ensp;&ensp;&ensp;__only_check_type == 2__: Only check the self-collision without moving, use the intermediate state as the starting planning path, check whether the path has self-collision (the intermediate state will be updated at this time), and restore the intermediate state to the actual state after the end  
  > &ensp;&ensp;&ensp;&ensp;__only_check_type == 3__: Only check the self-collision without moving, use the intermediate state as the starting planning path, and check whether the path has self-collision (the intermediate state will be updated at this time)  
  > 
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_dh_params(float dh_params[28])__
  > Stop the gripper of Lite6 series robotics arms  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  >  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.0.0  
  >    
  > @param dh_params: the result of DH parameters  
  > &ensp;&ensp;&ensp;&ensp;dh_params[0:4]: DH parameters of Joint-1  
  > &ensp;&ensp;&ensp;&ensp;dh_params[4:8]: DH parameters of Joint-2  
  > &ensp;&ensp;&ensp;&ensp;...  
  > &ensp;&ensp;&ensp;&ensp;dh_params[24:28]: DH parameters of Joint-7  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_dh_params(float dh_params[28], unsigned char flag = 0)__
  > Set the DH parameters  
  > &ensp;&ensp;&ensp;&ensp;Note:  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.0.0  
  > &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. this interface is only provided for users who need to use external DH parameters, ordinary users should not try to modify DH parameters.  
  >    
  > @param dh_params: DH parameters  
  > @param flag:  
  > &ensp;&ensp;&ensp;&ensp;0: Use the set DH parameters, but do not write to the configuration file  
  > &ensp;&ensp;&ensp;&ensp;1: Use the set DH parameters and write to the configuration file  
  > &ensp;&ensp;&ensp;&ensp;2: Use the set DH parameters and delete the DH parameters of the configuration file  
  > &ensp;&ensp;&ensp;&ensp;3: Use the default DH parameters, but will not delete the DH parameters of the configuration file  
  > &ensp;&ensp;&ensp;&ensp;4: Use the default DH parameters and delete the DH parameters of the configuration file  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_feedback_type(unsigned char feedback_type)__
  > Set the feedback type  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.1.0  
  > &ensp;&ensp;&ensp;&ensp;2. only works in position mode  
  > &ensp;&ensp;&ensp;&ensp;3. the setting will only affect subsequent tasks and will not affect previously cached tasks  
  > &ensp;&ensp;&ensp;&ensp;4. only valid for the current connection  
  > 
  > @param feedback_type: feedback type  
  >    &ensp;&ensp;&ensp;&ensp;0: disable feedback  
  >    &ensp;&ensp;&ensp;&ensp;1: feedback when the motion task starts executing  
  >    &ensp;&ensp;&ensp;&ensp;2: feedback when the motion task execution ends or motion task is discarded(usually when the distance is too close to be planned)  
  >    &ensp;&ensp;&ensp;&ensp;4: feedback when the non-motion task is triggered  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_linear_spd_limit_factor(float factor)__
  > Set linear speed limit factor (default is 1.2)  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
  > &ensp;&ensp;&ensp;&ensp;2. only available in mode 1  
  > 
  > @param factor: speed limit factor   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_cmd_mat_history_num(int num)__
  > Set cmd mat history num  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  > 
  > @param num: history num   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_fdb_mat_history_num(int num)__
  > Set fdb mat history num  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
  > 
  > @param num: history num   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_linear_spd_limit_factor(float *factor)__
  > Get linear speed limit factor 
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
  > 
  > @param factor: speed limit factor   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_cmd_mat_history_num(int *num)__
  > Get cmd mat history num  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  > 
  > @param num: history num   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_fdb_mat_history_num(int *num)__
  > Get fdb mat history num  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
  > 
  > @param num: history num   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_tgpio_modbus_timeout(int *timeout, bool is_transparent_transmission = false)__
  > Get tgpio modbus timeout  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
  > 
  > @param timeout: timeout, milliseconds   
  > @param is_transparent_transmission: is transparent transmission or not   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_poe_status(int *status)__
  > Get poe status  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
  > 
  > @param status: poe status, 1 means poe vaild, 0 means poe invalid   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_iden_status(int *status)__
  > Get iden status  
  > Note:  
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.5.0  
  > 
  > @param status: iden status, 1 means in identifying, 0 means not in identifying   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_c31_error_info(int *servo_id, float *theoretical_tau, float *actual_tau)__
  > Get collision error (C31) info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  >  
  > @param servo_id: servo id   
  > @param theoretical_tau: theoretical tau   
  > @param actual_tau: actual tau   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_c54_error_info(int *dir, float *tau_threshold, float *actual_tau)__
  > Get (Six-axis Force Torque Sensor) collision error (C54) info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103   
  >  
  > @param dir: trigger direction (XYZRxRyRz)   
  > @param tau_threshold: tau threshold   
  > @param actual_tau: actual tau   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_c37_error_info(int *servo_id, float *diff_angle)__
  > Get payload error (C37) info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  >   
  > @param servo_id: servo id   
  > @param diff_angle: diff angle   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_c23_error_info(int *id_bits, float angles[7])__
  > Get joint angle limit error (C23) info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  >    
  > @param id_bits: each bit corresponds to each joint (bit0 corresponds to joint 1), and a bit of 1 indicates that the corresponding joint exceeds the limit.   
  > @param angles: current angles   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
  
- __int get_c24_error_info(int *servo_id, float *speed)__
  > Get joint angle speed limit (C24) error info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  >   
  > @param servo_id: servo id   
  > @param speed: current speed   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_c60_error_info(float *max_velo, float *curr_velo)__
  > Get linear angle speed limit (C60) error info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0   
  > &ensp;&ensp;&ensp;&ensp;2. only available in mode 1   
  >   
  > @param max_velo: max limit linear speed   
  > @param curr_velo: current linear speed   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_c38_error_info(int *id_bits, float angles[7])__
  > Get joint hard angle limit error (C38) info   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.0   
  >    
  > @param id_bits: each bit corresponds to each joint (bit0 corresponds to joint 1), and a bit of 1 indicates that the corresponding joint exceeds the limit.   
  > @param angles: current angles   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_ft_collision_detection(int on_off)__
  > Set whether to enable collision detection with the Six-axis Force Torque Sensor   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103   
  > &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)   
  > &ensp;&ensp;&ensp;&ensp;3. the Six-axis Force Torque Sensor needs to be enabled and set force mode   
  >    
  > @param on_off: enable or not   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_ft_collision_rebound(int on_off)__
  > Set whether to enable collision rebound with the Six-axis Force Torque Sensor   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103     
  >    
  > @param on_off: enable or not   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_ft_collision_threshold(float thresholds[6])__
  > Set the threshold of the collision detection with the Six-axis Force Torque Sensor   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103     
  >    
  > @param thresholds: collision detection thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]   
  > &ensp;&ensp;&ensp;&ensp;x: [5, 200] (N)    
  > &ensp;&ensp;&ensp;&ensp;y: [5, 200] (N)    
  > &ensp;&ensp;&ensp;&ensp;z: [5, 200] (N)    
  > &ensp;&ensp;&ensp;&ensp;Rx: [0.1, 4] (Nm)    
  > &ensp;&ensp;&ensp;&ensp;Ry: [0.1, 4] (Nm)    
  > &ensp;&ensp;&ensp;&ensp;Rz: [0.1, 4] (Nm)    
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int set_ft_collision_reb_distance(float distances[6])__
  > Set the rebound distance of the collision rebound with the Six-axis Force Torque Sensor   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103     
  >    
  > @param distances: collision rebound distance, [x(mm), y(mm), z(mm), Rx(° or rad), Ry(° or rad), Rz(° or rad)]   
  > &ensp;&ensp;&ensp;&ensp;x: [2, 500] (mm)    
  > &ensp;&ensp;&ensp;&ensp;y: [2, 500] (mm)    
  > &ensp;&ensp;&ensp;&ensp;z: [2, 500] (mm)    
  > &ensp;&ensp;&ensp;&ensp;Rx: [0.2, 50] (°)    
  > &ensp;&ensp;&ensp;&ensp;Ry: [0.2, 50] (°)    
  > &ensp;&ensp;&ensp;&ensp;Rz: [0.2, 50] (°)    
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_ft_collision_detection(int *on_off)__
  > Get the collision detection with the Six-axis Force Torque Sensor is enable or not   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103      
  >    
  > @param on_off: enable or not   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_ft_collision_rebound(int *on_off)__
  > Get the collision rebound with the Six-axis Force Torque Sensor is enable or not   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103     
  >    
  > @param on_off: enable or not   
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_ft_collision_threshold(float thresholds[6])__
  > Get the collision threshold with the Six-axis Force Torque Sensor   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103     
  >    
  > @param thresholds: collision detection thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]    
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int get_ft_collision_reb_distance(float distances[6])__
  > Get the collision rebound distance with the Six-axis Force Torque Sensor   
  > Note:   
  > &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103     
  >    
  > @param distances: collision rebound distance, [x(mm), y(mm), z(mm), Rx(° or rad), Ry(° or rad), Rz(° or rad)]     
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.

- __int read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Read Coils (0x01)  
  > 
  > @param addr: the starting address of the register to be read  
  > @param quantity: number of registers  
  > @param bits: store result  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Read Discrete Inputs (0x02)  
  > 
  > @param addr: the starting address of the register to be read     
  > @param quantity: number of registers  
  > @param bits: store result  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Read Holding Registers (0x03)  
  > 
  > @param addr: the starting address of the register to be read  
  > @param quantity: number of registers  
  > @param regs: store result  
  > @param is_signed: whether to convert the read register value into a signed form      
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Read Input Registers (0x04)  
  > 
  > @param addr: the starting address of the register to be read  
  > @param quantity: number of registers  
  > @param regs: store result  
  > @param is_signed: whether to convert the read register value into a signed form  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int write_single_coil_bit(unsigned short addr, unsigned char bit_val)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Write Single Coil (0x05)  
  > 
  > @param addr: register address  
  > @param bit_val: the value to write (0/1)  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int write_single_holding_register(unsigned short addr, int reg_val)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Write Single Holding Register (0x06)  
  > 
  > @param addr: register address  
  > @param reg_val: the value to write  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Write Multiple Coils (0x0F)  
  > 
  > @param addr: the starting address of the register to be written  
  > @param quantity: the number of registers to be written  
  > @param bits: array of values to write  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Write Multiple Holding Registers (0x10)  
  > 
  > @param addr: the starting address of the register to be written  
  > @param quantity: the number of registers to be written  
  > @param regs: array of values to write  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Mask Write Holding Register (0x16)  
  > 
  > @param addr: register address  
  > @param and_mask: mask to be AND with  
  > @param or_mask: mask to be OR with  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means Modbus TCP exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)

- __int write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed = false)__
  > ([Standard Modbus TCP](./UF_ModbusTCP_Manual.md)) Mask Write Holding Register (0x17)  
  > 
  > @param r_addr: the starting address of the register to be read  
  > @param r_quantity: number of registers to read  
  > @param r_regs: store result  
  > @param w_addr: the starting address of the register to be written  
  > @param w_quantity: number of registers to write  
  > @param w_regs: array of values to write to the register  
  > @param is_signed: whether to convert the read register value into a signed form  
  > @return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
  > &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](./UF_ModbusTCP_Manual.md)
