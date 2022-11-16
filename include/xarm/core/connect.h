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

#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"

UxbusCmdSer *connect_rs485_control(const char *com);
UxbusCmdTcp *connect_tcp_control(char *server_ip);
SocketPort *connect_tcp_report_norm(char *server_ip);
SocketPort *connect_tcp_report_rich(char *server_ip);
SocketPort *connect_tcp_report_devl(char *server_ip);
SocketPort *connect_tcp_report(char *server_ip, std::string report_type="normal");

#define connext_tcp_report_norm connect_tcp_report_norm
#define connext_tcp_report_rich connect_tcp_report_rich
#define connext_tcp_report_devl connect_tcp_report_devl

#endif
