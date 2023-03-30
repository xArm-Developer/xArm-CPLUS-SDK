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

#ifndef CORE_PORT_SOCKET_H_
#define CORE_PORT_SOCKET_H_

#include <iostream>
#include <thread>
#include "xarm/core/common/data_type.h"
#include "xarm/core/common/queue_memcpy.h"

class SocketPort {
public:
  SocketPort(char *server_ip, int server_port, int que_num, int que_maxlen, int tcp_type = 0, int feedback_que_num = 0, int feedback_que_maxlen = 100);
  ~SocketPort(void);
  int is_ok(void);
  void flush(void);
  void recv_proc(void);
  void recv_report_proc(void);
  int write_frame(unsigned char *data, int len);
  int read_frame(unsigned char *data);
  void close_port(void);
  int read_feedback_frame(unsigned char *data);
  int que_maxlen;
  bool is_report;

private:
  int port_;
  int fp_;
  int state_;
  int que_num_;
  int feedback_que_num_;
  QueueMemcpy *rx_que_;
  QueueMemcpy *feedback_que_;
};

#endif
