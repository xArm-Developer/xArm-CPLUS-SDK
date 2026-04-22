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

#ifndef CORE_PORT_SERIAL_H_
#define CORE_PORT_SERIAL_H_

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include "serial/serial.h"
#include "xarm/core/common/data_type.h"
#include "xarm/core/common/queue_memcpy.h"

class SerialPort {
public:
  SerialPort(const char *port, int baud, int que_num, int que_maxlen);
  ~SerialPort(void);
  int connect();
  void disconnect();
  bool is_connected();
  int is_ok(void);
  void flush(void);
  void recv_proc(void);
  int write_frame(unsigned char *data, int len);
  int read_frame(unsigned char *data);
  void close_port(void);

private:
  std::shared_ptr<serial::Serial> _init_serial(const char *port, int baud);
  int _read_char(unsigned char *ch);
  int _write_char(unsigned char ch);
  void _parse_put(unsigned char *data, int len);
  void _join_recv_thread();

public:
  int que_maxlen;
  std::shared_ptr<serial::Serial> ser;

private:
  std::string ser_port_;
  int ser_baud_;
  int que_num_;
  mutable std::mutex conn_mutex_;
  std::atomic<int> state_{-1};
  std::shared_ptr<QueueMemcpy> rx_que_;
  std::thread thread_id_;

  typedef enum _UXBUS_RECV_STATE {
  UXBUS_START_FROMID = 0,
  UXBUS_START_TOOID = 1,
  UXBUS_STATE_LENGTH = 2,
  UXBUS_STATE_DATA = 3,
  UXBUS_STATE_CRC1 = 4,
  UXBUS_STATE_CRC2 = 5,
  } UXBUS_RECV_STATE;

  unsigned char UXBUS_PROT_FROMID_;
  unsigned char UXBUS_PROT_TOID_;

  int rx_data_idx_;
  int rx_state_;
  unsigned char rx_buf_[128];
  int rx_length_;
};

#endif
