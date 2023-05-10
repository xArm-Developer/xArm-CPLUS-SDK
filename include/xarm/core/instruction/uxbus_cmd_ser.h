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

#ifndef CORE_INSTRUCTION_UXBUS_CMD_SER_H_
#define CORE_INSTRUCTION_UXBUS_CMD_SER_H_

#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/port/ser.h"

class UxbusCmdSer : public UxbusCmd {
public:
  UxbusCmdSer(SerialPort *arm_port);
  ~UxbusCmdSer(void);

  void close(void);
  int is_ok(void);

private:
  int _send_modbus_request(unsigned char unit_id, unsigned char *pdu_data, unsigned short pdu_len, int prot_id = -1);
  int _recv_modbus_response(unsigned char t_unit_id, unsigned short t_trans_id, unsigned char *ret_data, unsigned short ret_len, int timeout, int t_prot_id = -1);
  int _check_private_protocol(unsigned char *data);

private:
  SerialPort *arm_port_;
};

#endif
