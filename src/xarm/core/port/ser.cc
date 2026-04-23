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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#ifdef _WIN32
#include <windows.h>
#include <sys/timeb.h>
#else
#include <unistd.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "xarm/core/port/ser.h"
#include "xarm/core/common/crc16.h"
#include "xarm/core/utils/log.h"

void SerialPort::recv_proc(void) {
  unsigned char ch;
  int ret;
  while (state_ == 0) {
    ret = _read_char(&ch);

    if (ret >= 0) {
      _parse_put(&ch, 1);
      continue;
    }
    //usleep(1000);
#ifdef _WIN32
    Sleep(1); // 1 ms
#else
    usleep(1000); // 1000us
#endif
  }
}

static void recv_proc_(void *arg) {
  SerialPort *my_this = (SerialPort *)arg;

  my_this->recv_proc();
}

SerialPort::SerialPort(const char *port, int baud, int que_num, int que_maxlen_) {
  ser_port_ = std::string(port);
  ser_baud_ = baud;
  que_num_ = que_num;
  que_maxlen = que_maxlen_;
  state_ = -1;
  rx_que_ = nullptr;
  UXBUS_PROT_FROMID_ = 0x55;
  UXBUS_PROT_TOID_ = 0xAA;
  connect();
}

SerialPort::~SerialPort(void) {
  disconnect();
  _join_recv_thread();
}

void SerialPort::_join_recv_thread() {
  if (!thread_id_.joinable()) return;
  if (thread_id_.get_id() == std::this_thread::get_id()) return;
  thread_id_.join();
}

int SerialPort::connect()
{
  std::unique_lock<std::mutex> lock(conn_mutex_);
  if (state_.load(std::memory_order_acquire) == 0) return 1;

  _join_recv_thread();
  auto new_ser = _init_serial(ser_port_.c_str(), ser_baud_);
  if (new_ser == nullptr)
  {
    return -1;
  }
  ser = new_ser;
  state_.store(0, std::memory_order_release);
  if (rx_que_ == nullptr)
    rx_que_ = std::make_shared<QueueMemcpy>(que_num_, que_maxlen);
  flush();
  thread_id_ = std::thread(recv_proc_, this);
  // thread_id_.detach();

  return 0;
}

void SerialPort::disconnect()
{
  std::unique_lock<std::mutex> lock(conn_mutex_);
  state_.store(-1, std::memory_order_release);
  try {
    ser->close();
  } 
  catch (const std::exception& e) {
    XARM_LOG_WARN("serial close exception: %s\n", e.what());
    return;
  }
  catch (...) {
    XARM_LOG_WARN("serial close exception: unknown\n");
    return;
  }
}

bool SerialPort::is_connected()
{
  return state_ == 0;
}

int SerialPort::is_ok(void) { return state_; }

void SerialPort::flush(void) {
  rx_que_->flush();
  rx_data_idx_ = 0;
  rx_state_ = UXBUS_START_FROMID;
}

int SerialPort::_read_char(unsigned char *ch) {
  try {
    std::string s = ser->read(1);
    if (s.empty()) return -1;
    *ch = static_cast<unsigned char>(s[0]);
    return 0;
  }
  catch (const std::exception& e) {
    XARM_LOG_WARN("serial read char exception: %s\n", e.what());
    return -1;
  }
  catch (...) {
    XARM_LOG_WARN("serial read char exception: unknown\n");
    return -1;
  }
}

int SerialPort::read_frame(unsigned char *data) {
  if (state_ != 0) { return -1; }
  return rx_que_->pop(data);
}

int SerialPort::_write_char(unsigned char ch) {
  //return ((write(fp_, &ch, 1) == 1) ? 0 : -1);
  try {
    char c = static_cast<char>(ch);
    return write_frame((unsigned char *)&c, 1);
  }
  catch (const std::exception& e) {
    XARM_LOG_WARN("serial write char exception: %s\n", e.what());
    return -1;
  }
  catch (...) {
    XARM_LOG_WARN("serial write char exception: unknown\n");
    return -1;
  }
}

int SerialPort::write_frame(unsigned char *data, int len) {
  try {
    std::string str_data(reinterpret_cast<const char *>(data), len);
    size_t size = ser->write(str_data);
    if (size != len) { return -1; }
    return 0;
  }
  catch (const std::exception& e) {
    XARM_LOG_WARN("serial write exception: %s\n", e.what());
    return -1;
  }
  catch (...) {
    XARM_LOG_WARN("serial write exception: unknown\n");
    return -1;
  }
}

void SerialPort::close_port(void) {
  disconnect();
}

void SerialPort::_parse_put(unsigned char *data, int len) {
  unsigned char ch;

  for (int i = 0; i < len; i++) {
    ch = data[i];
    switch (rx_state_) {
    case UXBUS_START_FROMID:
      if (UXBUS_PROT_FROMID_ == ch) {
        rx_buf_[0] = ch;
        rx_state_ = UXBUS_START_TOOID;
      }
      break;

    case UXBUS_START_TOOID:
      if (UXBUS_PROT_TOID_ == ch) {
        rx_buf_[1] = ch;
        rx_state_ = UXBUS_STATE_LENGTH;
      }
      else {
        rx_state_ = UXBUS_START_FROMID;
      }
      break;

    case UXBUS_STATE_LENGTH:
      if (0 < ch && ch < (que_maxlen - 5)) {
        rx_buf_[2] = ch;
        rx_length_ = ch;
        rx_data_idx_ = 3;
        rx_state_ = UXBUS_STATE_DATA;
      }
      else {
        rx_state_ = UXBUS_START_FROMID;
      }
      break;

    case UXBUS_STATE_DATA:
      if (rx_data_idx_ < rx_length_ + 3) {
        rx_buf_[rx_data_idx_++] = ch;
        if (rx_data_idx_ == rx_length_ + 3) {
          rx_state_ = UXBUS_STATE_CRC1;
        }
      }
      else {
        rx_state_ = UXBUS_START_FROMID;
      }
      break;

    case UXBUS_STATE_CRC1:
      rx_buf_[rx_length_ + 3] = ch;
      rx_state_ = UXBUS_STATE_CRC2;
      break;

    case UXBUS_STATE_CRC2:
      int crc, crc_r;
      rx_buf_[rx_length_ + 4] = ch;
      crc = modbus_crc(rx_buf_, rx_length_ + 3);
      crc_r = (rx_buf_[rx_length_ + 4] << 8) + rx_buf_[rx_length_ + 3];
      if (crc == crc_r) {
        rx_que_->push(rx_buf_);
      }
      rx_state_ = UXBUS_START_FROMID;
      break;

    default:
      rx_state_ = UXBUS_START_FROMID;
      break;
    }
  }
}

std::shared_ptr<serial::Serial> SerialPort::_init_serial(const char *port, int baud) {
  /* speed_t speed;
   struct termios options;

   fp_ = open((const char *)port, O_RDWR | O_NOCTTY);
   if (-1 == fp_) { return -1; }

   fcntl(fp_, F_SETFL, FNDELAY);
   tcgetattr(fp_, &options);
   bzero(&options, sizeof(options));

   switch (baud) {
   case 110:
     speed = B110;
     break;
   case 300:
     speed = B300;
     break;
   case 600:
     speed = B600;
     break;
   case 1200:
     speed = B1200;
     break;
   case 2400:
     speed = B2400;
     break;
   case 4800:
     speed = B4800;
     break;
   case 9600:
     speed = B9600;
     break;
   case 19200:
     speed = B19200;
     break;
   case 38400:
     speed = B38400;
     break;
   case 57600:
     speed = B57600;
     break;
   case 115200:
     speed = B115200;
     break;
   case 921600:
     speed = B921600;
     break;
   }

   cfsetispeed(&options, speed);
   cfsetospeed(&options, speed);

   options.c_oflag &= ~OPOST;
   options.c_cc[VTIME] = 200;
   options.c_cc[VMIN] = 10;
   tcsetattr(fp_, TCSANOW, &options);
   return 0;
   */
  try {
    auto new_ser = std::make_shared<serial::Serial>();
    new_ser->setPort(port);
    new_ser->setBaudrate(baud);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    new_ser->setTimeout(timeout);
    new_ser->open();
    return new_ser;
  }
  catch (const std::exception& e) {
    XARM_LOG_ERROR("serial open exception: %s\n", e.what());
    return nullptr;
  }
  catch (...) {
    XARM_LOG_ERROR("serial open exception: unknown\n");
    return nullptr;
  }
}
