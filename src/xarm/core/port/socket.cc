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
#include "xarm/core/port/socket.h"
#include "xarm/core/os/network.h"
#include "xarm/core/xarm_config.h"

#ifdef _WIN32
#include <ws2tcpip.h>
static int close(int fd)
{
  return closesocket(fd);
}

static bool is_ignore_errno(int fp, int port)
{
  if (WSAGetLastError() == WSAEINTR || WSAGetLastError() == WSAEWOULDBLOCK) {
    fprintf(stderr, "EINTR occured, port=%d, fp=%d, errno=%d\n", port, fp, WSAGetLastError());
    return true;
  }
  fprintf(stderr, "socket read failed, port=%d, fp=%d, errno=%d, exit\n", port, fp, WSAGetLastError());
  return false;
}
#else
#include <sys/socket.h>
#include <unistd.h>
static bool is_ignore_errno(int fp, int port)
{
  if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
    fprintf(stderr, "EINTR occured, port=%d, fp=%d, errno=%d\n", port, fp, errno);
    return true;
  }
  fprintf(stderr, "socket read failed, port=%d, fp=%d, errno=%d, exit\n", port, fp, errno);
  return false;
}
#endif


// inline unsigned long long get_ms()
// {
// #ifdef _WIN32
// 	struct timeb t;
// 	ftime(&t);
// 	return 1000 * t.time + t.millitm; // milliseconds
// #else
// 	struct timespec t;
// 	clock_gettime(CLOCK_REALTIME, &t);
// 	return 1000 * t.tv_sec + t.tv_nsec / 1000000; // milliseconds
// #endif
// }

void SocketPort::recv_report_proc(void) {
  int ret;
  int size = 0;
  int num = 0, data_num = 0;
  unsigned char *recv_data = new unsigned char[que_maxlen]();
  unsigned char *tmp_data = new unsigned char[que_maxlen]();
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

  bool print_log = false;

  while (state_ == 0)
  {
    num = recv(fp_, (char *)(&recv_data[4] + data_num), (size == 0 ? 4 : size) - data_num, 0);
    if (num <= 0) {
      if (is_ignore_errno(fp_, port_)) {
        continue;
      }
      else {
        close_port();
        break;
      }
    }
    if (size == 0) {
      // get report size at first
      data_num += num;
      if (data_num != 4) continue;
      size = bin8_to_32(&recv_data[4]);
      if (size == 233) {
        size_is_not_confirm = true;
        size = 245;
      }
      printf("report_data_size: %d, size_is_not_confirm: %d\n", size, size_is_not_confirm);
    }
    else {
      data_num += num;
      if (data_num < size) continue;
      if (size_is_not_confirm) {
        size_is_not_confirm = false;
        if (bin8_to_32(&recv_data[237]) == 233) {
          size = 233;
          continue;
        }
      }
      if (bin8_to_32(&recv_data[4]) != size && !(size_is_not_confirm && size == 245 && bin8_to_32(&recv_data[4]) == 233)) {
        fprintf(stderr, "report data error, close_port, length=%d, size=%d\n", bin8_to_32(&recv_data[4]), size);
        close_port();
        break;
      }

      // data_curr_us = bin8_to_64(&recv_data[size-4]);
      // recv_curr_ms = get_ms();
      // if (data_prev_us != 0 && recv_prev_ms != 0) {
      // 	data_interval_us = data_curr_us - data_prev_us;
      // 	data_over_cnts += data_interval_us > data_over_us ? 1 : 0;
      // 	recv_interval_ms = recv_curr_ms - recv_prev_ms;
      // 	recv_over_cnts += recv_interval_ms > recv_over_ms ? 1 : 0;

      // 	print_log = false;

      // 	if (data_interval_us > data_max_interval_us) {
      // 		data_max_interval_us = data_interval_us;
      // 		print_log = true;
      // 	}
      // 	else if (data_interval_us > data_over_us) {
      // 		print_log = true;
      // 	}

      // 	if (recv_interval_ms > recv_max_interval_ms) {
      // 		recv_max_interval_ms = recv_interval_ms;
      // 		print_log = true;
      // 	}
      // 	else if (recv_interval_ms > recv_over_ms) {
      // 		print_log = true;
      // 	}

      // 	if (print_log) {
      // 		printf("[RECV] Di=%f, Dmax=%f, Dncts=%ld, Ri=%lld, Rmax=%lld, Rcnts=%ld\n",
      // 			data_interval_us / 1000.0, data_max_interval_us / 1000.0, data_over_cnts,
      // 			recv_interval_ms, recv_max_interval_ms, recv_over_cnts
      // 		);
      // 	}

      // }
      // data_prev_us = data_curr_us;
      // recv_prev_ms = recv_curr_ms;

      bin32_to_8(data_num, &recv_data[0]);
      if (rx_que_->is_full()) {
        rx_que_->pop(tmp_data);
      }
      ret = rx_que_->push(recv_data);
      data_num = 0;
      memset(recv_data, 0, que_maxlen);
    }
  }
  delete[] recv_data;
  delete rx_que_;
}

void SocketPort::recv_proc(void) {
  int ret;
  int failed_cnt = 0;
  int num = 0;
  int length = 0;
  int buf_len = 0;
  int buf_offset = 0;
  int buf_size = que_maxlen * 2;
  unsigned char *recv_buf = new unsigned char[buf_size]();
  unsigned char *recv_data = new unsigned char[que_maxlen]();
  while (state_ == 0) {
    num = recv(fp_, (char *)(&recv_buf[buf_len]), buf_size - buf_len, 0);
    if (num <= 0) {
      if (is_ignore_errno(fp_, port_)) {
        continue;
      }
      else {
        close_port();
        break;
      }
    }
    buf_len += num;
    buf_offset = 0;
    while (state_ == 0) {
      if (buf_len < 6) break;
      length = bin8_to_16(&recv_buf[buf_offset + 4]) + 6;
      if (buf_len < length) break;

      memcpy(&recv_data[4], &recv_buf[buf_offset], length);
      if (recv_data[10] == 0xFF) {
        if (feedback_que_num_ > 0) {
          if (feedback_que_->push(&recv_data[4]) != 0) {
            fprintf(stderr, "feedback queue is full, discard\n");
          }
        }
      }
      else {
        bin32_to_8(length, &recv_data[0]);
        ret = rx_que_->push(recv_data);
        failed_cnt = 0;
        while (ret != 0 && state_ == 0 && failed_cnt < 1500)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          ret = rx_que_->push(recv_data);
          failed_cnt += 1;
        }
        if (ret != 0) {
          if (state_ == 0)
            fprintf(stderr, "socket push data failed, exit, port=%d, fp=%d\n", port_, fp_);
          close_port();
          break;
        };
      }
      buf_len -= length;
      buf_offset += length;
    }
    if (buf_len > 0) {
      memcpy(recv_data, &recv_buf[buf_offset], buf_len);
    }
  }
  delete[] recv_buf;
  delete[] recv_data;
  delete rx_que_;
  if (feedback_que_num_ > 0)
    delete feedback_que_;
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

SocketPort::SocketPort(char *server_ip, int server_port, int que_num,int que_maxlen_, int tcp_type, int feedback_que_num, int feedback_que_maxlen) {
  que_num_ = que_num;
  que_maxlen = que_maxlen_;
  state_ = -1;
  is_report = tcp_type == 1 ? true : false;
  rx_que_ = new QueueMemcpy(que_num_, que_maxlen);
  feedback_que_num_ = feedback_que_num;
  if (feedback_que_num_ > 0)
    feedback_que_ = new QueueMemcpy(feedback_que_num_, feedback_que_maxlen);
  fp_ = socket_init((char *)" ", 0, 0);
  if (fp_ == -1) { 
    delete rx_que_;
    if (feedback_que_num_ > 0)
      delete feedback_que_;
    return;
  }

  int ret = socket_connect_server(&fp_, server_ip, server_port);
  if (ret == -1) { 
    delete rx_que_;
    if (feedback_que_num_ > 0)
      delete feedback_que_;
    return;
  }
  port_ = server_port;
  state_ = 0;
  flush();
  std::thread th(recv_proc_, this);
  th.detach();
}

SocketPort::~SocketPort(void) {
  state_ = -1;
  close_port();
}

int SocketPort::is_ok(void) { return state_; }

void SocketPort::flush(void) { rx_que_->flush(); }

int SocketPort::read_frame(unsigned char *data) {
  if (state_ != 0) { return -1; }

  if (rx_que_->size() == 0) { return -1; }

  rx_que_->pop(data);
  return 0;
}

int SocketPort::write_frame(unsigned char *data, int len) {
  int ret = socket_send_data(fp_, data, len);
  return ret;
}

void SocketPort::close_port(void) {
  state_ = -1;
  close(fp_);
}

int SocketPort::read_feedback_frame(unsigned char *data)
{
  if (state_ != 0 || feedback_que_num_ <= 0) { return -1; }
  return feedback_que_->pop(data);
}