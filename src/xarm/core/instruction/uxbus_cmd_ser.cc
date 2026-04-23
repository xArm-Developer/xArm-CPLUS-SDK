/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/common/crc16.h"
#include "xarm/core/utils/log.h"
#include "xarm/core/instruction/uxbus_cmd_config.h"

// UxbusCmdSer::UxbusCmdSer(SerialPort *arm_port) {
//   throw std::logic_error("raw-pointer constructor is no longer supported");
//   std::shared_ptr<SerialPort> ptr(arm_port);
//   arm_port_ = ptr;
// }

UxbusCmdSer::UxbusCmdSer(const std::shared_ptr<SerialPort> &arm_port) {
  arm_port_ = arm_port;
}

UxbusCmdSer::~UxbusCmdSer(void) {}

void UxbusCmdSer::close(void) { arm_port_->disconnect(); }

int UxbusCmdSer::is_ok(void) { return arm_port_->is_ok(); }

int UxbusCmdSer::_send_modbus_request(unsigned char unit_id, unsigned char *pdu_data, unsigned short pdu_len, int prot_id)
{
  std::vector<unsigned char> send_data(pdu_len + 6, 0);

  send_data[0] = UXBUS_CONF::MASTER_ID;
  send_data[1] = UXBUS_CONF::SLAVE_ID;
  send_data[2] = pdu_len + 1;
  send_data[3] = unit_id;
  if (pdu_len > 0)
    memcpy(send_data.data() + 4, pdu_data, pdu_len);

  int crc = modbus_crc(send_data.data(), 4 + pdu_len);
  send_data[4 + pdu_len] = (unsigned char)(crc & 0xFF);
  send_data[5 + pdu_len] = (unsigned char)((crc >> 8) & 0xFF);

  arm_port_->flush();
  int ret = arm_port_->write_frame(send_data.data(), pdu_len + 6);
  return ret;
}

int UxbusCmdSer::_recv_modbus_response(unsigned char t_unit_id, unsigned short t_trans_id, unsigned char *ret_data, unsigned short ret_len, int timeout, int t_prot_id)
{
  int ret;
  std::vector<unsigned char> rx_data(arm_port_->que_maxlen, 0);
  long long expired = get_system_time() + (long long)timeout;
  while (get_system_time() < expired) {
    ret = arm_port_->read_frame(rx_data.data());
    if (ret == -1) {
      sleep_us(500);
      continue;
    }
    ret = _check_private_protocol(rx_data.data());
    for (int i = 0; i < ret_len; i++) { ret_data[i] = rx_data[i + 4]; }
    return ret;
  }
  return UXBUS_STATE::ERR_TOUT;
}

int UxbusCmdSer::_check_private_protocol(unsigned char *data)
{
  state_is_ready = !(data[3] & 0x10);
  if (data[3] & 0x08) { return UXBUS_STATE::INVALID; }
  if (data[3] & 0x40) { return UXBUS_STATE::ERR_CODE; }
  if (data[3] & 0x20) { return UXBUS_STATE::WAR_CODE; }
  return 0;
}