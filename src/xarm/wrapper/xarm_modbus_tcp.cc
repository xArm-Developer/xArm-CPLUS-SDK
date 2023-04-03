/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2023, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include "xarm/wrapper/xarm_api.h"

/* modbus tcp func_code: 0x01 */
int XArmAPI::read_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)
{
  return core->read_coil_bits(addr, quantity, bits);
}

/* modbus tcp func_code: 0x02 */
int XArmAPI::read_input_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)
{
  return core->read_input_bits(addr, quantity, bits);
}

/* modbus tcp func_code: 0x03 */
int XArmAPI::read_holding_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed)
{
  return core->read_holding_registers(addr, quantity, regs, is_signed);
}

/* modbus tcp func_code: 0x04 */
int XArmAPI::read_input_registers(unsigned short addr, unsigned short quantity, int *regs, bool is_signed)
{
  return core->read_input_registers(addr, quantity, regs, is_signed);
}

/* modbus tcp func_code: 0x05 */
int XArmAPI::write_single_coil_bit(unsigned short addr, unsigned char bit_val)
{
  return core->write_single_coil_bit(addr, bit_val);
}

/* modbus tcp func_code: 0x06 */
int XArmAPI::write_single_holding_register(unsigned short addr, int reg_val)
{
  return core->write_single_holding_register(addr, reg_val);
}

/* modbus tcp func_code: 0x0F */
int XArmAPI::write_multiple_coil_bits(unsigned short addr, unsigned short quantity, unsigned char *bits)
{
  return core->write_multiple_coil_bits(addr, quantity, bits);
}

/* modbus tcp func_code: 0x10 */
int XArmAPI::write_multiple_holding_registers(unsigned short addr, unsigned short quantity, int *regs)
{
  return core->write_multiple_holding_registers(addr, quantity, regs);
}

/* modbus tcp func_code: 0x16 */
int XArmAPI::mask_write_holding_register(unsigned short addr, unsigned short and_mask, unsigned short or_mask)
{
  return core->mask_write_holding_register(addr, and_mask, or_mask);
}

/* modbus tcp func_code: 0x17 */
int XArmAPI::write_and_read_holding_registers(unsigned short r_addr, unsigned short r_quantity, int *r_regs, unsigned short w_addr, unsigned short w_quantity, int *w_regs, bool is_signed)
{
  return core->write_and_read_holding_registers(r_addr, r_quantity, r_regs, w_addr, w_quantity, w_regs, is_signed);
}
