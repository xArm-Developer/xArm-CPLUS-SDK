# xArm-C++-SDK Release Notes

## Update Summary

- > ### 1.14.2
  - Fix the 503 interface protocol identification error issue
  - Added sync parameters to some gpio interfaces to support immediate execution (requires firmware 2.4.101 or above)
  - Added XArmAPIWrapper to C# calls to support simultaneous control of multiple robotic arms
  - Added new interface to support obtaining some error information

- > ### 1.13.0
  - Compatible with the standard Modbus TCP protocol, providing part of the standard Modbus TCP protocol interface

- > ### 1.12.2
  - Support partial Task feedback (requires firmware version greater than or equal to v2.1.0)

- > ### 1.11.6
  - Correct the ambiguity that the `set_position_aa` interface is true when both relative and is_tool_coord are true. After the correction, when is_tool_coord is true, relative is invalid (previously is_tool_coord was invalid when relative was true)

- > ### 1.11.5
  - Optimization pause time is too long (wait=true)
  - Add common motion api (Enabled after firmware version 1.11.100)
  - The Cartesian motion-related interface adds the motion_type parameter to determine the planning method (Enabled after firmware version 1.11.100)

- > ### 1.11.0
  - Support transparent transmission (240/241)
  - Modified the centroid unit of `ft_sensor_iden_load` and `ft_sensor_cali_load` interfaces to millimeters (originally meters)

- > ### 1.9.10 
  - Support Lite6 Model
  - Fix several bugs

- > ### 1.9.0 
  - Support friction parameter identification interface
  - Support relative motion
  - Repair time-consuming interface (identification) failure due to heartbeat mechanism
  - Fix several bugs

- > ### 1.8.4
  - Support the Six-axis Force Torque Sensor (not a third party)
  - Modify the reporting processing logic and optimize the processing of sticky packets
  - Fixed frequent switching of the pause state causing the program to hang
  - Fix the program hangs when setting the mechanical claw position in speed mode


- > ### 1.8.0

  - The Velocity interface supports the duration parameter (requires firmware 1.8.0 or higher)
  - Added identification interface (current identification and torque identification) (requires firmware 1.8.0 or higher)
  - Support linear track interface (requires firmware 1.8.0 or higher)
  - Fix the problem of not waiting when the timeout parameter of the motion interface is greater than 0
  - Support macos compilation
  - Fix some bugs

- > ### 1.6.9
  - Support velocity control
  - Support calibrate tcp offset and user offset
  - Fix several bugs

- > ### 1.6.0

  - Support the xArm BIO gripper, Robotiq 2F-85 gripper and Robotiq 2F-140 gripper
  - Support position detection trigger the controller analog IO
  - Support self-collision model parameter setting
  - Support Modbus communication of end tools
  - Supports TCP timeout for setting instructions
  - Support joint motion with circular interpolation
  - Optimize logic, enhance API security, Fix several bugs

- > ### 1.5.0
  - The new parameter of `set_servo_cartisian` interface is used to support servo cartisian movement of tool coordinate system
  - Support delayed trigger digital IO
  - Support position detection trigger digital IO
  - Support configure the stop state to automatically reset IO signal
  - Support motion commands based on axis angle
  - Support to calculate the offset between two points
  - Support for blocky code conversion and operation of xArmStudio1.5.0