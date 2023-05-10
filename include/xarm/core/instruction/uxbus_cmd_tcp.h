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

#ifndef CORE_INSTRUCTION_UXBUS_CMD_TCP_H_
#define CORE_INSTRUCTION_UXBUS_CMD_TCP_H_

#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/port/socket.h"

class UxbusCmdTcp : public UxbusCmd {
public:
  UxbusCmdTcp(SocketPort *arm_port);
  UxbusCmdTcp(SocketPort *arm_port, std::function<void (std::string, int, unsigned char)> set_feedback_key_transid);
  ~UxbusCmdTcp(void);

  void close(void);
  int is_ok(void);

  int get_protocol_identifier(void);
  int set_protocol_identifier(int protocol_identifier = 2);

  /* modbus tcp func_code: 0x01 */
  int read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits);
  /* modbus tcp func_code: 0x02 */
  int read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits);
  /* modbus tcp func_code: 0x03 */
  int read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false);
  /* modbus tcp func_code: 0x04 */
  int read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed = false);
  /* modbus tcp func_code: 0x05 */
  int write_single_coil_bit(unsigned short addr, unsigned char bit_val);
  /* modbus tcp func_code: 0x06 */
  int write_single_holding_register(unsigned short addr, int reg_val);
  /* modbus tcp func_code: 0x0F */
  int write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits);
  /* modbus tcp func_code: 0x10 */
  int write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs);
  /* modbus tcp func_code: 0x16 */
  int mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask);
  /* modbus tcp func_code: 0x17 */
  int write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed = false);

private:
  int _send_modbus_request(unsigned char unit_id, unsigned char *pdu_data, unsigned short pdu_len, int prot_id = -1);
  int _recv_modbus_response(unsigned char t_unit_id, unsigned short t_trans_id, unsigned char *ret_data, unsigned short ret_len, int timeout, int t_prot_id = -1);
  int _check_private_protocol(unsigned char *data);
  int _check_protocol_header(unsigned char *data, unsigned short t_trans_id, unsigned short t_prot_id, unsigned short t_unit_id);

  int _standard_modbus_tcp_request(unsigned char *pdu_data, int pdu_len, unsigned char *rx_data, unsigned char unit_id = 0x01);
  int _read_bits(unsigned short addr, unsigned short quantity, unsigned char *bits, unsigned char funcode = 0x01);
  int _read_registers(unsigned short addr, unsigned short quantity, int *regs, unsigned char funcode = 0x03, bool is_signed = false);

  int _get_trans_id() { return transaction_id_; };
private:
  SocketPort *arm_port_;

  unsigned short transaction_id_;       // transaction id 
  unsigned short protocol_identifier_;  // protocol identifier
};

#endif
