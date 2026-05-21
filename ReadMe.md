# xArm-C++-SDK


## Overview
- The current version supports Linux/windows/MacOS(beta), but the source code structure has changed.

## Caution
- During use, people should stay away from the robot arm to avoid accidental injury or damage to other items by the robot arm.
- Protect the arm before use.
- Before you exercise, please make sure you don't encounter obstacles.
- Protect the arm before unlocking the motor.

## Update Summary
- > ### 1.18.0/1.18.1 
  - Strengthen the code, fix some hidden bugs, and eliminate security risks.
  - Supports CMake compilation
  - Fix document errors and variable name errors

- > ### 1.17.0/1.17.1 
  - Change some API names

- > ### 1.16.0
  - Added parameter to support get raw data of the Six-axis Force Torque Sensor
  - Added an interface to control xArm Gripper G2
  - Optimize the interface for controlling BIO Gripper G2
  - Extend the get_joint_states interface

- > ### 1.15.0
  - Added the Six-axis Force Torque Sensor collision detection related interfaces
  - Added support for the new version of BIO Gripper control interface

- > ### 1.14.2
  - Fix the 503 interface protocol identification error issue
  - Added sync parameters to some gpio interfaces to support immediate execution (requires firmware 2.4.101 or above)
  - Added XArmAPIWrapper to C# calls to support simultaneous control of multiple robotic arms
  - Added new interface to support obtaining some error information

- > ### [More](./ReleaseNotes.md)

## API Change List
  | OLD API NAME   | NEW API NAME  |  SDK VERSION  |
  | -------------- | ------------- |  ------------  |
  | set_tgpio_modbus_timeout  |  set_rs485_timeout  |  1.17.1  |
  | get_tgpio_modbus_timeout  |  get_rs485_timeout  |  1.17.1  |
  | set_tgpio_modbus_baudrate  |  set_rs485_baudrate  |  1.17.1  |
  | get_tgpio_modbus_baudrate  |  get_rs485_baudrate  |  1.17.1  |
  | getset_tgpio_modbus_data  |  set_rs485_data  |  1.17.1  |
  | set_impedance  |  set_ft_sensor_admittance_parameters  |  1.17.0  |
  | set_impedance_mbk  |  set_ft_sensor_admittance_parameters  |  1.17.0  |
  | set_impedance_config  |  set_ft_sensor_admittance_parameters  |  1.17.0  |
  | set_force_control_pid  |  set_ft_sensor_force_parameters  |  1.17.0  |
  | config_force_control  |  set_ft_sensor_force_parameters  |  1.17.0  |
  | ft_sensor_set_zero  |  set_ft_sensor_zero  |  1.17.0  |
  | ft_sensor_iden_load  |  iden_ft_sensor_load_offset  |  1.17.0  |
  | ft_sensor_cali_load  |  set_ft_sensor_load_offset  |  1.17.0  |
  | ft_sensor_enable  |  set_ft_sensor_enable  |  1.17.0  |
  | ft_sensor_app_set  |  set_ft_sensor_mode  |  1.17.0  |
  | ft_sensor_app_get  |  get_ft_sensor_mode  |  1.17.0  |
  | get_linear_track_registers  |  get_linear_motor_registers  |  1.17.0  |
  | get_linear_track_pos  |  get_linear_motor_pos  |  1.17.0  |
  | get_linear_track_status  |  get_linear_motor_status  |  1.17.0  |
  | get_linear_track_error  |  get_linear_motor_error  |  1.17.0  |
  | get_linear_track_is_enabled  |  get_linear_motor_is_enabled  |  1.17.0  |
  | get_linear_track_on_zero  |  get_linear_motor_on_zero  |  1.17.0  |
  | get_linear_track_sci  |  get_linear_motor_sci  |  1.17.0  |
  | get_linear_track_sco  |  get_linear_motor_sco  |  1.17.0  |
  | clean_linear_track_error  |  clean_linear_motor_error  |  1.17.0  |
  | set_linear_track_enable  |  set_linear_motor_enable  |  1.17.0  |
  | set_linear_track_speed  |  set_linear_motor_speed  |  1.17.0  |
  | set_linear_track_back_origin  |  set_linear_motor_back_origin  |  1.17.0  |
  | set_linear_track_pos  |  set_linear_motor_pos  |  1.17.0  |
  | set_linear_track_stop  |  set_linear_motor_stop  |  1.17.0  |
  | shutdown_system  |  system_control  |  1.13.6  |
  | get_suction_cup  |  get_vacuum_gripper  |  1.8.0  |
  | set_suction_cup  |  set_vacuum_gripper  |  1.8.0  |
  

## Doc
- #### [API Document](doc/xarm_cplus_api.md)

- #### [API Code Document](doc/xarm_api_code.md)

- #### [UFACTORY ModbusTCP Manual](doc/UF_ModbusTCP_Manual.md)

## Build

### Requirements

- CMake 3.5 or newer
- A C++11 compiler
- Linux/macOS: `g++` or `clang++`
- Windows: Visual Studio 2015/2017/2019/2022 with Desktop development for C++

The top-level `CMakeLists.txt` is written against CMake 3.5 features.

### Get the code

```bash
git clone https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
cd xArm-CPLUS-SDK
```

### Output layout

By default, the current CMake build generates:

- libraries in `build/lib`
- executables in `build/bin`

When building shared libraries, make sure the runtime loader can find `build/lib`.

## Build With CMake

### Linux

Configure and build:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

Build static library instead of shared library:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
make -j
```

Disable examples:

```bash
mkdir -p build
cd build
cmake .. -DXARM_BUILD_EXAMPLES=OFF
make -j
```

Run an example with shared library output:

```bash
LD_LIBRARY_PATH=./build/lib ./build/bin/0002-get_property 192.168.1.221
```

Install:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install
```

### macOS

Configure and build:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

Run an example with shared library output:

```bash
DYLD_LIBRARY_PATH=./build/lib ./build/bin/0002-get_property 192.168.1.221
```

### Windows

Configure with Visual Studio 2015:

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 14 2015" -A x64
cmake --build . --config Release
```

Configure with Visual Studio 2017:

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 15 2017" -A x64
cmake --build . --config Release
```

Configure with Visual Studio 2019:

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 16 2019" -A x64
cmake --build . --config Release
```

If you use Visual Studio 2022:

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

Run an example:

```powershell
.\build\bin\Release\0002-get_property.exe 192.168.1.221
```

Build the optional C# wrapper:

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DXARM_BUILD_CSHARP_WRAPPER=ON
cmake --build . --config Release
```

## Build With Makefile

The repository still keeps the original `Makefile` for Linux-style builds.

Build library:

```bash
make xarm
```

Build all examples:

```bash
make test
```

Build one example:

```bash
make test-0002-get_property
```

Build everything:

```bash
make clean
make
```

Run an example with shared library output:

```bash
LD_LIBRARY_PATH=./build/lib ./build/bin/0002-get_property 192.168.1.221
```

Install:

```bash
sudo make install
```

Uninstall:

```bash
sudo make uninstall
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

- ##### [5006-set_cgpio_digital_analog](example/5006-set_cgpio_digital_analog.cc)

- ##### [5008-get_cgpio_state](example/5008-get_cgpio_state.cc)

- ##### [5009-set_bio_gripper](example/5009-set_bio_gripper.cc)

- ##### [6001-set_reduced_mode](example/6001-set_reduced_mode.cc)

- ##### [6002-set_fence_mode](example/6002-set_fence_mode.cc)

- ##### [7001-servo_j](example/7001-servo_j.cc)

- ##### [7002-servo_cartesian](example/7002-servo_cartesian.cc)

- ##### [7003-servo_cartesian_aa](example/7003-servo_cartesian_aa.cc)

- ##### [8000-load_identify_current](example/8000-load_identify_current.cc)

- ##### [8001-force_tech](example/8001-force_tech.cc)

- ##### [8002-admittance_control](example/8002-admittance_control.cc)

- ##### [8003-force_control](example/8003-force_control.cc)

- ##### [8004-load_identify](example/8004-load_identify.cc)

- ##### [8005-read_force_data](example/8005-read_force_data.cc)

- ##### [8006-save_force_zero](example/8006-save_force_zero.cc)

- ##### [8010-get_ft_sensor_config](example/8010-get_ft_sensor_config.cc)

- ##### [thirdparty-set_robotiq_gripper](example/thirdparty-set_robotiq_gripper.cc)

- ##### [thirdparty-set_yinshi_gripper](example/thirdparty-set_yinshi_gripper.cc)
