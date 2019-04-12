## 介绍:

​	xArm机械臂C++ API接口及使用示例
​	

### 示例使用方法

* 编译示例

  ```
  $ cd xarm_api_c++
  $ make all
  ```

  

* 运行示例：正常信息上报

  ```
  $ ./build/example1_report_norm 192.168.1.xxx
  ```

  以10HZ的频率上报机械臂基础信息

  

* 运行示例：调试信息上报

  ```
  $ ./build/example2_report_rich 192.168.1.xxx
  ```

  以10HZ的频率上报机械臂调试信息

  

* 运行示例：实时信息上报

  ```
  $ ./build/example3_report_develop 192.168.1.xxx
  ```

  以100HZ的频率上报机械臂基础信息

  

* 运行示例：运动控制（使用TCP连接）

  ```
  $ ./build/example4_control_tcp_motion 192.168.1.xxx
  ```

  对机械臂初始化、并发送位置运动指令

  

* 运行示例：运动控制（使用485连接）

  ```
  $ sudo ./build/example5_control_485_motion /dev/ttyUSBx
  ```

  对机械臂初始化、并发送运动指令

  

* 运行示例：获取信息的API接口测试（默认使用TCP连接）

  ```
  $ ./build/example6_test_fetch_instruction 192.168.1.xxx
  ```

  

* 运行示例：爪子测试（默认使用TCP连接）

  ```
  $ ./build/example7_test_gripper 192.168.1.xxx
  ```

  对机爪子初始化、并发送运动指令

  

* 运行示例：运动控制（使用TCP连接）

  ```
  $ ./build/example8_control_tcp_motion 192.168.1.xxx
  ```

  对机械臂初始化、并发送servoj运动指令

  

* 运行示例：tool gpio测试（使用TCP连接）

  ```
  $ ./build/example9_tgpio 192.168.1.xxx
  ```

  读取和设置数字io电平

  读取模拟io的数值

* 运行示例：运动控制（使用TCP连接，三点画圆）

  ```
  $ ./build/example10_control_tcp_motion_circle 192.168.1.xxx
  ```

* 运行示例：controler gpio测试（使用TCP连接）

  ```
  $ ./build/example11_cgpio 192.168.1.xxx
  ```

  设置/读取controler gpio

## 更新日志

​	V0.1 (2018-08-14)
​		1. 支持有TCP和485两种控制接口

​	V0.2 (2018-09-10)

​		1.新增servo模式

​		2.新增servoj的运动指令功能

​	V0.3 (xx-xx-xx)

​		1.新增工具gpio控制接口

​		2.新增力控功能、示教灵敏度设置、碰撞检测、碰撞检测灵敏度设置

​		3.新增设置重力方向

​		4.新增三点画圆运动

​	V0.4 (2019-04-10)

​		1.新增控制器gpio控制接口

## E-mail

​	jimy.zhang@ufactory.cc
​	jimy92@163.com