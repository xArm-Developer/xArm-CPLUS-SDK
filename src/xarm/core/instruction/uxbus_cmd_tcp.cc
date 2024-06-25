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
#include "xarm/core/instruction/uxbus_cmd_tcp.h"
#include "xarm/core/debug/debug_print.h"
#include "xarm/core/instruction/uxbus_cmd_config.h"

const unsigned short STANDARD_MODBUS_TCP_PROTOCOL = 0x00;
const unsigned short PRIVATE_MODBUS_TCP_PROTOCOL= 0x02;
const unsigned short TRANSACTION_ID_MAX = 65535;

UxbusCmdTcp::UxbusCmdTcp(SocketPort *arm_port) {
  arm_port_ = arm_port;
  transaction_id_ = 1;
  protocol_identifier_ = PRIVATE_MODBUS_TCP_PROTOCOL;
}

UxbusCmdTcp::UxbusCmdTcp(SocketPort *arm_port, std::function<void (std::string, int, unsigned char)> set_feedback_key_transid) : UxbusCmd(set_feedback_key_transid) {
  arm_port_ = arm_port;
  transaction_id_ = 1;
  protocol_identifier_ = PRIVATE_MODBUS_TCP_PROTOCOL;
}

UxbusCmdTcp::~UxbusCmdTcp(void) {}

int UxbusCmdTcp::get_protocol_identifier(void)
{
  return protocol_identifier_;
}

int UxbusCmdTcp::set_protocol_identifier(int protocol_identifier)
{
  std::lock_guard<std::mutex> locker(mutex_);
  if (protocol_identifier_ != protocol_identifier) {
    protocol_identifier_ = protocol_identifier;
    printf("change protocol identifier to %d\n", protocol_identifier_);
  }
  return 0;
}

void UxbusCmdTcp::close(void) { arm_port_->close_port(); }

int UxbusCmdTcp::is_ok(void) { return arm_port_->is_ok(); }

int UxbusCmdTcp::_send_modbus_request(unsigned char unit_id, unsigned char *pdu_data, unsigned short pdu_len, int prot_id)
{
  int len = pdu_len + 7;
  // unsigned char send_data[len];
  unsigned char *send_data = new unsigned char[len]();
  unsigned short curr_trans_id = transaction_id_;
  unsigned short curr_prot_id = prot_id < 0 ? protocol_identifier_ : prot_id;

  bin16_to_8(curr_trans_id, &send_data[0]);
  bin16_to_8(curr_prot_id, &send_data[2]);
  bin16_to_8(pdu_len + 1, &send_data[4]);
  send_data[6] = unit_id;
  if (pdu_len > 0)
    memcpy(&send_data[7], pdu_data, pdu_len);

  arm_port_->flush();
  // print_hex("send:", send_data, pdu_len + 7);
  int ret = arm_port_->write_frame(send_data, len);
  delete[] send_data;
  if (ret != len) { return -1; }

  transaction_id_ = transaction_id_ % TRANSACTION_ID_MAX + 1;

  return curr_trans_id;
}

int UxbusCmdTcp::_recv_modbus_response(unsigned char t_unit_id, unsigned short t_trans_id, unsigned char *ret_data, unsigned short ret_len, int timeout, int t_prot_id)
{
  unsigned short prot_id = t_prot_id < 0 ? protocol_identifier_ : t_prot_id;
  int ret = UXBUS_STATE::ERR_TOUT;
  int code;
  unsigned short length;
  // unsigned char rx_data[arm_port_->que_maxlen] = {0};
  unsigned char *rx_data = new unsigned char[arm_port_->que_maxlen]();
  long long expired = get_system_time() + (long long)timeout;
  while (get_system_time() < expired) {
    code = arm_port_->read_frame(rx_data);
    if (code == -1) {
      sleep_us(500);
      continue;
    }
    // print_hex("recv:", rx_data, arm_port_->que_maxlen);
    last_recv_ms = get_system_time();
    unsigned char *data = &rx_data[4];
    code = _check_protocol_header(data, t_trans_id, prot_id, t_unit_id);
    if (code != 0) {
      if (code != UXBUS_STATE::ERR_NUM) {
        ret = code;
        break;
      }
      continue;
    }
    if (prot_id != STANDARD_MODBUS_TCP_PROTOCOL) {
      // Private Modbus TCP Protocol
      code = _check_private_protocol(data);
      length = bin8_to_16(&data[4]) - 2;
      for (size_t i = 0; i < length; i++) {
        if (ret_len >= 0 && i >= ret_len) break;
        ret_data[i] = data[i + 8];
      }
      ret = code;
      break;
    }
    else {
      // Standard Modbus TCP Protocol
      length = bin8_to_16(&data[4]) + 6;
      for (size_t i = 0; i < length; i++) {
        if (ret_len >= 0 && i >= ret_len) break;
        ret_data[i] = data[i];
      }
      ret = code;
      break;
    }
  }
  delete[] rx_data;
  return ret;
}

int UxbusCmdTcp::_check_protocol_header(unsigned char *data, unsigned short t_trans_id, unsigned short t_prot_id, unsigned short t_unit_id)
{
  unsigned short trans_id = bin8_to_16(&data[0]);
  unsigned short prot_id = bin8_to_16(&data[2]);
  unsigned short length = bin8_to_16(&data[4]);
  unsigned char unit_id = data[6];

  if (trans_id != t_trans_id) return UXBUS_STATE::ERR_NUM;
  if (prot_id != t_prot_id) return UXBUS_STATE::ERR_PROT;
  if (unit_id != t_unit_id) return UXBUS_STATE::ERR_FUN;

  return 0;
}

int UxbusCmdTcp::_check_private_protocol(unsigned char *data)
{
  state_is_ready = !(data[7] & 0x10);
  if (data[7] & 0x08) { return UXBUS_STATE::INVALID; }
  if (data[7] & 0x40) { return UXBUS_STATE::ERR_CODE; }
  if (data[7] & 0x20) { return UXBUS_STATE::WAR_CODE; }
  return 0;
}

int UxbusCmdTcp::_standard_modbus_tcp_request(unsigned char *pdu_data, int pdu_len, unsigned char *rx_data, unsigned char unit_id)
{
  std::lock_guard<std::mutex> locker(mutex_);
  int ret = _send_modbus_request(unit_id, pdu_data, pdu_len, STANDARD_MODBUS_TCP_PROTOCOL);
  if (-1 == ret) { return UXBUS_STATE::ERR_NOTTCP; }
  ret = _recv_modbus_response(unit_id, ret, rx_data, -1, 10000, STANDARD_MODBUS_TCP_PROTOCOL);
  if (ret == 0 && rx_data[7] == pdu_data[0] + 0x80) {
    return rx_data[8] + 0x80;
  }
  return ret;
}

int UxbusCmdTcp::_read_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, unsigned char funcode)
{
  unsigned char pdu[5] = {0};
  pdu[0] = funcode;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(quantity, &pdu[3]);
  unsigned char *rx_data = new unsigned char[9 + (quantity + 7) / 8];
  int ret = _standard_modbus_tcp_request(pdu, 5, rx_data);
  if (ret == 0) {
    for (size_t i = 0; i < quantity; i++) {
      bits[i] = (rx_data[9 + i / 8] >> (i % 8) & 0x01);
    }
  }
  delete[] rx_data;
  return ret;
}

int UxbusCmdTcp::_read_registers(unsigned short addr, unsigned short quantity, int *regs, unsigned char funcode, bool is_signed)
{
  unsigned char pdu[5] = {0};
  pdu[0] = funcode;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(quantity, &pdu[3]);
  unsigned char *rx_data = new unsigned char[9 + quantity * 2];
  int ret = _standard_modbus_tcp_request(pdu, 5, rx_data);
  if (ret == 0) {
    if (is_signed) {
      bin8_to_ns16(&rx_data[9], regs, quantity);
    }
    else {
      for (size_t i = 0; i < quantity; i++) {
        regs[i] = bin8_to_16(&rx_data[9 + i * 2]);
      } 
    }
  }
  delete[] rx_data;
  return ret;
}

int UxbusCmdTcp::read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)
{
  return _read_bits(addr, quantity, bits, 0x01);
}

int UxbusCmdTcp::read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)
{
  return _read_bits(addr, quantity, bits, 0x02);
}

int UxbusCmdTcp::read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed)
{
  return _read_registers(addr, quantity, regs, 0x03, is_signed);
}

int UxbusCmdTcp::read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed)
{
  return _read_registers(addr, quantity, regs, 0x04, is_signed);
}

int UxbusCmdTcp::write_single_coil_bit(unsigned short addr, unsigned char bit_val)
{
  unsigned char pdu[5] = {0};
  pdu[0] = 0x05;
  unsigned short val = bit_val ? 0xFF00 : 0x0000;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(val, &pdu[3]);
  unsigned char rx_data[12];
  return _standard_modbus_tcp_request(pdu, 5, rx_data, 12);
}

int UxbusCmdTcp::write_single_holding_register(unsigned short addr, int reg_val)
{
  unsigned char pdu[5] = {0};
  pdu[0] = 0x06;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(reg_val, &pdu[3]);
  unsigned char rx_data[12];
  return _standard_modbus_tcp_request(pdu, 5, rx_data, 12);
}

int UxbusCmdTcp::write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)
{
  unsigned char data_len = (quantity + 7) / 8;
  unsigned char *pdu = new unsigned char[6 + data_len]();
  pdu[0] = 0x0F;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(quantity, &pdu[3]);
  pdu[5] = data_len;
  for (size_t i = 0; i < quantity; i++) {
    if (bits[i])
      pdu[6 + i / 8] |= (1 << (i % 8));
  }
  unsigned char rx_data[12];
  int ret = _standard_modbus_tcp_request(pdu, 6 + data_len, rx_data, 12);
  delete[] pdu;
  return ret;
}

int UxbusCmdTcp::write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs)
{
  unsigned char *pdu = new unsigned char[6 + quantity * 2]();
  pdu[0] = 0x10;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(quantity, &pdu[3]);
  pdu[5] = quantity * 2;
  for (size_t i = 0; i < quantity; i++) {
    bin16_to_8(regs[i], &pdu[6 + i * 2]);
  }
  unsigned char rx_data[12];
  int ret = _standard_modbus_tcp_request(pdu, 6 + quantity * 2, rx_data, 12);
  delete[] pdu;
  return ret;
}

int UxbusCmdTcp::mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask)
{
  unsigned char pdu[7] = {0};
  pdu[0] = 0x16;
  bin16_to_8(addr, &pdu[1]);
  bin16_to_8(and_mask, &pdu[3]);
  bin16_to_8(or_mask, &pdu[5]);
  unsigned char rx_data[14];
  return _standard_modbus_tcp_request(pdu, 7, rx_data, 14);
}

int UxbusCmdTcp::write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed)
{
  unsigned char *pdu = new unsigned char[10 + w_quantity * 2]();
  pdu[0] = 0x17;
  bin16_to_8(r_addr, &pdu[1]);
  bin16_to_8(r_quantity, &pdu[3]);
  bin16_to_8(w_addr, &pdu[5]);
  bin16_to_8(w_quantity, &pdu[7]);
  pdu[9] = w_quantity * 2;
  for (size_t i = 0; i < w_quantity; i++) {
    bin16_to_8(w_regs[i], &pdu[10 + i * 2]);
  }
  unsigned char *rx_data = new unsigned char[9 + r_quantity * 2]();
  int ret = _standard_modbus_tcp_request(pdu, 10 + w_quantity * 2, rx_data, 9 + r_quantity * 2);
  if (ret == 0) {
    if (is_signed) {
      bin8_to_ns16(&rx_data[9], r_regs, r_quantity);
    }
    else {
      for (size_t i = 0; i < r_quantity; i++) {
        r_regs[i] = bin8_to_16(&rx_data[9 + i * 2]);
      } 
    }
  }
  delete[] pdu;
  delete[] rx_data;
  return ret;
}