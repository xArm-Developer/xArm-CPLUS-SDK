/**
 * Software License Agreement (MIT License)
 * 
 * @copyright Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef XARM_CONNECT_H_
#define XARM_CONNECT_H_

#include <memory>
#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"

SocketPort *__connect_tcp_report_norm(const char *server_ip);
SocketPort *__connect_tcp_report_rich(const char *server_ip);
SocketPort *__connect_tcp_report_devl(const char *server_ip);

[[deprecated("please use connect_rs485_control2 and manage lifetime with std::shared_ptr")]]
UxbusCmdSer *connect_rs485_control(const char *com);
[[deprecated("please use connect_tcp_control2 and manage lifetime with std::shared_ptr")]]
UxbusCmdTcp *connect_tcp_control(const char *server_ip);
[[deprecated("please use connect_tcp_report_norm2 and manage lifetime with std::shared_ptr")]]
SocketPort *connect_tcp_report_norm(const char *server_ip);
[[deprecated("please use connect_tcp_report_rich2 and manage lifetime with std::shared_ptr")]]
SocketPort *connect_tcp_report_rich(const char *server_ip);
[[deprecated("please use connect_tcp_report_devl2 and manage lifetime with std::shared_ptr")]]
SocketPort *connect_tcp_report_devl(const char *server_ip);
[[deprecated("please use connect_tcp_report2 and manage lifetime with std::shared_ptr")]]
SocketPort *connect_tcp_report(const char *server_ip, const std::string &report_type="normal");

std::shared_ptr<UxbusCmdSer> connect_rs485_control2(const char *com);
std::shared_ptr<UxbusCmdTcp> connect_tcp_control2(const char *server_ip);
std::shared_ptr<SocketPort> connect_tcp_report_norm2(const char *server_ip);
std::shared_ptr<SocketPort> connect_tcp_report_rich2(const char *server_ip);
std::shared_ptr<SocketPort> connect_tcp_report_devl2(const char *server_ip);
std::shared_ptr<SocketPort> connect_tcp_report2(const char *server_ip, const std::string &report_type="normal");

#define connext_tcp_report_norm connect_tcp_report_norm
#define connext_tcp_report_rich connect_tcp_report_rich
#define connext_tcp_report_devl connect_tcp_report_devl

#endif
