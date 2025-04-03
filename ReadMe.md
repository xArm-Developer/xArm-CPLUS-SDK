# xArm-C++-SDK


## Overview
- The current version supports Linux/windows/MacOS(beta), but the source code structure has changed.

## Caution
- During use, people should stay away from the robot arm to avoid accidental injury or damage to other items by the robot arm.
- Protect the arm before use.
- Before you exercise, please make sure you don't encounter obstacles.
- Protect the arm before unlocking the motor.

## Update Summary
- > ### 1.15.0
  - Added the Six-axis Force Torque Sensor collision detection related interfaces
  - Added support for the new version of BIO Gripper control interface

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

- > ### [More](./ReleaseNotes.md)

## Doc
- #### [API Document](doc/xarm_cplus_api.md)

- #### [API Code Document](doc/xarm_api_code.md)

- #### [UFACTORY ModbusTCP Manual](doc/UF_ModbusTCP_Manual.md)

## Linux

- Get the code:

  ```
  git clone https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
  ```

- Change directory

  ```
  cd ./xArm-CPLUS-SDK/
  ```


- Build library:
  ```bash
  make xarm
  ```

- Build all example

    ```bash
    make test
    ```

- Build a example

    ```bash
    make test-0002-get_property # build example/test-0002-get_property.cc
    ```

- Build all (build library and build all example)

    ```bash
    make clean
    make # make xarm && make test
    ```

- Install

    ```bash
    sudo make install
    ```

- Uninstall

    ```bash
    sudo make uninstall
    ```

- Run a example

    ```bash
    ./build/example/0002-get_property 192.168.1.221
    ```




## Windows

- Running environment

  ```
  It is recommended to run the project with visual studio 2015.Make sure your visual studio 2015 has a visual C++ development environment installed before running.
  ```
  
- Get the code:

  ```
  git clone https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
  ```

- Change directory

  ```
  Change your directory xArm-CPLUS-SDK/visual_studio
  ```

- Open project

  ```
  If you changed your directory, you can see a visual_studio.sln file. Click this file you will open the project.
  ```


- Check the xarm project properties

  ```markdown
  Open the xarm property pages and make sure your project configuration is the same as the following screenshot configuration.
  
  VC++ Directories path 
  	$(ProjectDir)..\..\include
  	$(ProjectDir)..\..\src
  ```

  ![image-20191217153415733](doc/img/image-20191217153415733.png)

  ![image-20191217152542587](doc/img/image-20191217152542587.png)

  

- Check the example project properties

  ```markdown
  Open the example property pages and make sure your project configuration is the same as the following screenshot configuration.
  Example project dependencies and xarm projects so references must be added to run.
  ```
  ![image-20191217154258804](doc/img/image-20191217154258804.png)

  ![image-20191217154111439](doc/img/image-20191217154111439.png)


- Build project

  ```markdown
  Right click example project and build the projiect.If the example project bulid successful you will get a .exe file in you project path.
  
  .exe path like this C:\Users\ufactory\Desktop\xArm-CPLUS-SDK\visual_studio\x64\Debug\0002-get_property.exe
  ```

  ![image-20191217155932743](doc/img/image-20191217155932743.png)


- Run project

  ```markdown
  You can use cmd run project.  
  ```

  ![image-20191217160911893](doc/img/image-20191217160911893.png)

- New project

  ```markdown
  If you want to create a new project, you need to pay attention to the dependent path of your new project. 
  You must make sure that the configuration of the project you create is the same as that of the example project.
  ```



## [Example](example/)

- ##### [0001-event_register](example/0001-event_register.cc)

- ##### [0002-get_property](example/0002-get_property.cc)

- ##### [0003-api_get](example/0003-api_get.cc)

- ##### [0004-servo_attach_detach](example/0004-servo_attach_detach.cc)

- ##### [1001-move_line](example/1001-move_line.cc)

- ##### [1004-move_arc_line](example/1004-move_arc_line.cc)

- ##### [1006-move_tool_line](example/1006-move_tool_line.cc)

- ##### [1007-counter](example/1007-counter.cc)

- ##### [1008-move_line_aa](example/1008-move_line_aa.cc)

- ##### [1009-cartesian_velocity_control](example/1009-cartesian_velocity_control.cc)

- ##### [2000-joint_velocity_control](example/2000-joint_velocity_control.cc)

- ##### [2001-move_joint](example/2001-move_joint.cc)

- ##### [3001-move_circle](example/3001-move_circle.cc)

- ##### [3002-record_trajectory](example/3002-record_trajectory.cc)

- ##### [3003-playback_trajectory](example/3003-playback_trajectory.cc)

- ##### [3004-get_report_data](example/3004-get_report_data.cc)

- ##### [3005-task_feedback](example/3005-task_feedback.cc)

- ##### [3006-standard_modbus_tcp](example/3006-standard_modbus_tcp.cc)

- ###### [5000-set_tgpio_modbus](example/5000-set_tgpio_modbus.cc)

- ##### [5001-get_tgpio_digital](example/5001-get_tgpio_digital.cc)

- ##### [5002-get_tgpio_analog](example/5002-get_tgpio_analog.cc)

- ##### [5003-set_tgpio_digital](example/5003-set_tgpio_digital.cc)

- ##### [5004-set_gripper](example/5004-set_gripper.cc)

- ##### [5005-get_cgpio_digital_analog](example/5005-get_cgpio_digital_analog.cc)

- ##### [5006-set_cgpio_dialog_analog](example/5006-set_cgpio_digital_analog.cc)

- ##### [5008-get_cgpio_state](example/5008-get_cgpio_state.cc)

- ##### [5009-set_bio_gripper](example/5009-set_bio_gripper.cc)

- ##### [6001-set_reduced_mode](example/6001-set_reduced_mode.cc)

- ##### [6002-set_fense_mode](example/6002-set_fense_mode.cc)

- ##### [7001-servo_j](example/7001-servo_j.cc)

- ##### [7002-servo_cartesian](example/7002-servo_cartesian.cc)

- ##### [7003-servo_cartesian_aa](example/7003-servo_cartesian_aa.cc)

- ##### [8000-load_identify_current](example/8000-load_identify_current.cc)

- ##### [8001-force_tech](example/8001-force_tech.cc)

- ##### [8002-impedance](example/8002-impedance.cc)

- ##### [8003-force_control](example/8003-force_control.cc)

- ##### [8004-load_identify](example/8004-load_identify.cc)

- ##### [8005-read_force_data](example/8005-read_force_data.cc)

- ##### [8006-save_force_zero](example/8006-save_force_zero.cc)

- ##### [8010-get_ft_sensor_config](example/8010-get_ft_sensor_config.cc)

- ##### [thirdparty-set_robotiq_gripper](example/thirdparty-set_robotiq_gripper.cc)

- ##### [thirdparty-set_yinshi_gripper](example/thirdparty-set_yinshi_gripper.cc)
