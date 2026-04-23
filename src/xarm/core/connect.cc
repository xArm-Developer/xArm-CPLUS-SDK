/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */
#include "xarm/core/connect.h"

#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"
#include "xarm/core/xarm_config.h"
#include "xarm/core/utils/log.h"

UxbusCmdSer *connect_rs485_control(const char *com) {
  auto arm_port = std::make_shared<SerialPort>(com, XARM_CONF::SERIAL_BAUD, 3, 128);
  if (!arm_port->is_connected()) {
    XARM_LOG_ERROR("Error: Serial RS485 connection failed\n");
    return nullptr;
  }
  UxbusCmdSer *arm_cmd = new UxbusCmdSer(arm_port);
  XARM_LOG_INFO("Serial RS485 connection successful\n");
  return arm_cmd;
}

UxbusCmdTcp *connect_tcp_control(const char *server_ip) {
  auto arm_port = std::make_shared<SocketPort>(server_ip, XARM_CONF::TCP_PORT_CONTROL, 3, 320, 0);
  if (!arm_port->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Control connection failed\n");
    return nullptr;
  }
  UxbusCmdTcp *arm_cmd = new UxbusCmdTcp(arm_port);
  XARM_LOG_INFO("Tcp Control connection successful\n");
  return arm_cmd;
}

SocketPort *__connect_tcp_report_norm(const char *server_ip) {
  SocketPort *arm_report = new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_NORM, 5, 256 + 4, 1); // 145 + 4
  if (!arm_report->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Report Norm connection failed, ip: %s\n", server_ip);
    delete arm_report;
    return nullptr;
  }
  XARM_LOG_INFO("Tcp Report Norm connection successful\n");
  return arm_report;
}

SocketPort *__connect_tcp_report_rich(const char *server_ip) {
  SocketPort *arm_report = new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_RICH, 5, 1024, 1);  // 494 + 4
  if (!arm_report->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Report Rich connection failed\n");
    delete arm_report;
    return nullptr;
  }
  XARM_LOG_INFO("Tcp Report Rich connection successful\n");
  return arm_report;
}

SocketPort *__connect_tcp_report_devl(const char *server_ip) {
  SocketPort *arm_report = new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_DEVL, 10, 256 + 4, 1); // 87 + 48 + 4
  if (!arm_report->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Report develop connection failed\n");
    delete arm_report;
    return nullptr;
  }
  XARM_LOG_INFO("Tcp Report develop connection successful\n");
  return arm_report;
}

SocketPort *connect_tcp_report_norm(const char *server_ip) {
  return __connect_tcp_report_norm(server_ip);
}

SocketPort *connect_tcp_report_rich(const char *server_ip) {
  return __connect_tcp_report_rich(server_ip);
}

SocketPort *connect_tcp_report_devl(const char *server_ip) {
  return __connect_tcp_report_devl(server_ip);
}

SocketPort *connect_tcp_report(const char *server_ip, const std::string &report_type) {
  if (report_type == "dev")
    return __connect_tcp_report_devl(server_ip);
  else if (report_type == "rich")
    return __connect_tcp_report_rich(server_ip);
  else
    return __connect_tcp_report_norm(server_ip);
}

std::shared_ptr<UxbusCmdSer> connect_rs485_control2(const char *com) {
  auto arm_port = std::make_shared<SerialPort>(com, XARM_CONF::SERIAL_BAUD, 3, 128);
  if (!arm_port->is_connected()) {
    XARM_LOG_ERROR("Error: Serial RS485 connection failed\n");
    return nullptr;
  }
  auto arm_cmd = std::make_shared<UxbusCmdSer>(arm_port);
  XARM_LOG_INFO("Serial RS485 connection successful\n");
  return arm_cmd;
}

std::shared_ptr<UxbusCmdTcp> connect_tcp_control2(const char *server_ip) {
  auto arm_port = std::make_shared<SocketPort>(server_ip, XARM_CONF::TCP_PORT_CONTROL, 3, 320, 0);
  if (!arm_port->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Control connection failed\n");
    return nullptr;
  }
  auto arm_cmd = std::make_shared<UxbusCmdTcp>(arm_port);
  XARM_LOG_INFO("Tcp Control connection successful\n");
  return arm_cmd;
}

std::shared_ptr<SocketPort> connect_tcp_report_norm2(const char *server_ip) {
  auto arm_report = std::make_shared<SocketPort>(server_ip, XARM_CONF::TCP_PORT_REPORT_NORM, 5, 256 + 4, 1);
  if (!arm_report->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Report Norm connection failed, ip: %s\n", server_ip);
    return nullptr;
  }
  XARM_LOG_INFO("Tcp Report Norm connection successful\n");
  return arm_report;
}

std::shared_ptr<SocketPort> connect_tcp_report_rich2(const char *server_ip) {
  auto arm_report = std::make_shared<SocketPort>(server_ip, XARM_CONF::TCP_PORT_REPORT_RICH, 5, 1024, 1);
  if (!arm_report->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Report Rich connection failed\n");
    return nullptr;
  }
  XARM_LOG_INFO("Tcp Report Rich connection successful\n");
  return arm_report;
}

std::shared_ptr<SocketPort> connect_tcp_report_devl2(const char *server_ip) {
  auto arm_report = std::make_shared<SocketPort>(server_ip, XARM_CONF::TCP_PORT_REPORT_DEVL, 10, 256 + 4, 1);
    if (!arm_report->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp Report develop connection failed\n");
    return nullptr;
  }
  XARM_LOG_INFO("Tcp Report develop connection successful\n");
  return arm_report;
}

std::shared_ptr<SocketPort> connect_tcp_report2(const char *server_ip, const std::string &report_type) {
  if (report_type == "dev")
    return connect_tcp_report_devl2(server_ip);
  else if (report_type == "rich")
    return connect_tcp_report_rich2(server_ip);
  else
    return connect_tcp_report_norm2(server_ip);
}
