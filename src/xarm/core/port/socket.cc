/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include <string.h>
#include <errno.h>
#include <vector>
#include "xarm/core/port/socket.h"
#include "xarm/core/os/network.h"
#include "xarm/core/xarm_config.h"
#include "xarm/core/utils/log.h"

#ifdef _WIN32
#include <ws2tcpip.h>
static int close(int fd)
{
  return closesocket(fd);
}

static int shutdown_socket(int fd)
{
  return shutdown(fd, SD_BOTH);
}

static bool is_ignore_errno(int sockfd, int port)
{
  if (WSAGetLastError() == WSAEINTR || WSAGetLastError() == WSAEWOULDBLOCK) {
    XARM_LOG_ERROR("EINTR occured, port=%d, sockfd=%d, errno=%d\n", port, sockfd, WSAGetLastError());
    return true;
  }
  XARM_LOG_ERROR("socket read failed, port=%d, sockfd=%d, errno=%d, exit\n", port, sockfd, WSAGetLastError());
  return false;
}
#else
#include <sys/socket.h>
#include <unistd.h>
#include <sys/socket.h>
#include <unistd.h>

static int shutdown_socket(int fd)
{
  return shutdown(fd, SHUT_RDWR);
}

static bool is_ignore_errno(int sockfd, int port)
{
  if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
    XARM_LOG_ERROR("EINTR occured, port=%d, sockfd=%d, errno=%d\n", port, sockfd, errno);
    return true;
  }
  XARM_LOG_ERROR("socket read failed, port=%d, sockfd=%d, errno=%d, exit\n", port, sockfd, errno);
  return false;
}
#endif

void SocketPort::recv_report_proc(void) {
  int ret;
  int size = 0;
  int num = 0, data_num = 0;
  std::vector<unsigned char> recv_data(que_maxlen, 0);
  bool size_is_not_confirm = false;

  unsigned long long recv_prev_ms = 0;
  unsigned long long recv_curr_ms = 0;
  unsigned long long recv_interval_ms = 0;
  unsigned long long recv_max_interval_ms = 0;
  unsigned long long recv_over_ms = 300;
  unsigned long recv_over_cnts = 0;

  unsigned long long data_prev_us = 0;
  unsigned long long data_curr_us = 0;
  unsigned long long data_interval_us = 0;
  unsigned long long data_max_interval_us = 0;
  unsigned long long data_over_us = 205 * 1000;
  unsigned long data_over_cnts = 0;

  bool debug = false; // log the debug msg

  int sockfd = sockfd_;

  while (state_ == 0)
  {
    num = recv(sockfd_, (char *)(recv_data.data() + 4 + data_num), (size == 0 ? 4 : size) - data_num, 0);
    if (num <= 0) {
      if (is_ignore_errno(sockfd, sin_port_)) {
        continue;
      }
      else {
        disconnect();
        break;
      }
    }
    if (size == 0) {
      // get report size at first
      data_num += num;
      if (data_num != 4) continue;
      size = bin8_to_32(recv_data.data() + 4);
      if (size <= 0 || size > que_maxlen - 4) {
        XARM_LOG_ERROR("recv_report_proc: invalid report size %d (max %d), port=%d\n", size, que_maxlen - 4, sin_port_);
        disconnect();
        break;
      }
      if (size == 233) {
        size_is_not_confirm = true;
        size = 245;
      }
      XARM_LOG_INFO("report_data_size: %d, size_is_not_confirm: %d\n", size, size_is_not_confirm);
    }
    else {
      data_num += num;
      if (data_num < size) continue;
      if (size_is_not_confirm) {
        size_is_not_confirm = false;
        if (que_maxlen >= 241 && bin8_to_32(recv_data.data() + 237) == 233) {
          size = 233;
          continue;
        }
      }
      if (bin8_to_32(recv_data.data() + 4) != size && !(size_is_not_confirm && size == 245 && bin8_to_32(recv_data.data() + 4) == 233)) {
        XARM_LOG_ERROR("report data error, disconnect, length=%d, size=%d\n", bin8_to_32(recv_data.data() + 4), size);
        disconnect();
        break;
      }

      // data_curr_us = bin8_to_64(&recv_data[size-4]);
      // recv_curr_ms = get_ms();
      // if (data_prev_us != 0 && recv_prev_ms != 0) {
      // 	data_interval_us = data_curr_us - data_prev_us;
      // 	data_over_cnts += data_interval_us > data_over_us ? 1 : 0;
      // 	recv_interval_ms = recv_curr_ms - recv_prev_ms;
      // 	recv_over_cnts += recv_interval_ms > recv_over_ms ? 1 : 0;

      // 	debug = false;

      // 	if (data_interval_us > data_max_interval_us) {
      // 		data_max_interval_us = data_interval_us;
      // 		debug = true;
      // 	}
      // 	else if (data_interval_us > data_over_us) {
      // 		debug = true;
      // 	}

      // 	if (recv_interval_ms > recv_max_interval_ms) {
      // 		recv_max_interval_ms = recv_interval_ms;
      // 		debug = true;
      // 	}
      // 	else if (recv_interval_ms > recv_over_ms) {
      // 		debug = true;
      // 	}

      // 	if (debug) {
      // 		XARM_LOG_DEBUG("[RECV] Di=%f, Dmax=%f, Dncts=%ld, Ri=%lld, Rmax=%lld, Rcnts=%ld\n",
      // 			data_interval_us / 1000.0, data_max_interval_us / 1000.0, data_over_cnts,
      // 			recv_interval_ms, recv_max_interval_ms, recv_over_cnts
      // 		);
      // 	}

      // }
      // data_prev_us = data_curr_us;
      // recv_prev_ms = recv_curr_ms;

      bin32_to_8(data_num, &recv_data[0]);
      ret = rx_que_->push(recv_data.data(), true);
      
      data_num = 0;
      std::fill(recv_data.begin(), recv_data.end(), 0); 
    }
  }
}

void SocketPort::recv_proc(void) {
  int ret;
  int failed_cnt = 0;
  int num = 0;
  int length = 0;
  int buf_len = 0;
  int buf_offset = 0;
  int buf_size = que_maxlen * 2;
  int sockfd = sockfd_;
  std::vector<unsigned char> recv_buf(buf_size, 0);
  std::vector<unsigned char> recv_data(que_maxlen, 0);
  while (state_ == 0) {
    num = recv(sockfd_, (char *)(recv_buf.data() + buf_len), buf_size - buf_len, 0);
    if (num <= 0) {
      if (is_ignore_errno(sockfd, sin_port_)) {
        continue;
      }
      else {
        disconnect();
        break;
      }
    }
    buf_len += num;
    buf_offset = 0;
    while (state_ == 0) {
      if (buf_len < 6) break;
      length = bin8_to_16(recv_buf.data() + buf_offset + 4) + 6;
      if (buf_len < length) break;
      if (length > que_maxlen) {
        XARM_LOG_ERROR("recv_proc: frame length %d exceeds buffer %d, port=%d\n", length, que_maxlen, sin_port_);
        disconnect();
        break;
      }

      memcpy(recv_data.data() + 4, recv_buf.data() + buf_offset, length);
      if (recv_data[10] == 0xFF) {
        if (feedback_que_num_ > 0) {
          if (feedback_que_->push(recv_data.data() + 4) != 0) {
            XARM_LOG_ERROR("feedback queue is full, discard\n");
          }
        }
      }
      else {
        bin32_to_8(length, recv_data.data());
        ret = rx_que_->push(recv_data.data());
        failed_cnt = 0;
        while (ret != 0 && state_ == 0 && failed_cnt < 1500)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          ret = rx_que_->push(recv_data.data());
          failed_cnt += 1;
        }
        if (ret != 0) {
          if (state_ == 0)
            XARM_LOG_ERROR("socket push data failed, exit, port=%d, sockfd=%d\n", sin_port_, sockfd);
          disconnect();
          break;
        };
      }
      buf_len -= length;
      buf_offset += length;
    }
    if (buf_len > 0 && buf_offset > 0) {
      memmove(recv_buf.data(), recv_buf.data() + buf_offset, buf_len);
    }
  }
}

static void *recv_proc_(void *arg) {
  SocketPort *my_this = (SocketPort *)arg;
  if (my_this->is_report) {
    my_this->recv_report_proc();
  }
  else {
    my_this->recv_proc();
  }
  return (void *)0;
}

SocketPort::SocketPort(const char *server_ip, const int server_port, int que_num,int que_maxlen_, int tcp_type, int feedback_que_num, int feedback_que_maxlen) {
  sin_addr_ = std::string(server_ip);
  sin_port_ = server_port;
  que_num_ = que_num;
  que_maxlen = que_maxlen_;
  feedback_que_num_ = feedback_que_num;
  feedback_que_maxlen_ = feedback_que_maxlen;
  state_ = -1;
  sockfd_ = -1;
  is_report = tcp_type == 1 ? true : false;
  rx_que_ = nullptr;
  feedback_que_ = nullptr;
  connect();
}

SocketPort::~SocketPort(void) {
  disconnect();
  _join_recv_thread();
}

void SocketPort::_join_recv_thread() {
  if (!thread_id_.joinable()) return;
  if (thread_id_.get_id() == std::this_thread::get_id()) return;
  thread_id_.join();
}

int SocketPort::connect()
{
  std::unique_lock<std::mutex> lock(conn_mutex_);
  if (state_.load(std::memory_order_acquire) == 0) return 1;

  _join_recv_thread();
  int new_fd = socket_init((char *)" ", 0, 0);
  if (new_fd == -1) {
    return -1;
  }
  int ret = socket_connect_server(&new_fd, sin_addr_.c_str(), sin_port_);
  if (ret == -1) {
    close(new_fd);
    return -2;
  }
  sockfd_ = new_fd;
  state_.store(0, std::memory_order_release);
  if (rx_que_ == nullptr)
    rx_que_ = std::make_shared<QueueMemcpy>(que_num_, que_maxlen);
  if (feedback_que_num_ > 0 && feedback_que_ == nullptr)
    feedback_que_ = std::make_shared<QueueMemcpy>(feedback_que_num_, feedback_que_maxlen_);
  state_ = 0;
  flush();
  thread_id_ = std::thread(recv_proc_, this);
  // thread_id_.detach();
  return 0;
}

void SocketPort::disconnect()
{
  int fd = -1;
  {
    std::unique_lock<std::mutex> lock(conn_mutex_);
    state_.store(-1, std::memory_order_release);
    fd = sockfd_;
    sockfd_ = -1;
  }
  if (fd != -1) {
    shutdown_socket(fd);
    close(fd);
  }
}

bool SocketPort::is_connected()
{
  return state_ == 0;
}

int SocketPort::is_ok(void) { return state_; }

void SocketPort::flush(void) { rx_que_->flush(); }

int SocketPort::read_frame(unsigned char *data) {
  if (state_ != 0) { return -1; }
  return rx_que_->pop(data);
}

int SocketPort::write_frame(unsigned char *data, int len) {
  int ret = socket_send_data(sockfd_, data, len);
  return ret;
}

void SocketPort::close_port(void) {
  disconnect();
}

int SocketPort::read_feedback_frame(unsigned char *data)
{
  if (state_ != 0 || feedback_que_num_ <= 0 || feedback_que_ == nullptr) { return -1; }
  return feedback_que_->pop(data);
}