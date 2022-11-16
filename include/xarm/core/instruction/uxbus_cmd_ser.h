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

  int check_xbus_prot(unsigned char *datas, int funcode);
  int send_pend(int funcode, int num, int timeout, unsigned char *ret_data);
  int send_xbus(int funcode, unsigned char *datas, int num);
  void close(void);
  int is_ok(void);

  int get_prot_flag(void) { return 0; }
  int set_prot_flag(int prot_flag = 2) { return 0; }

private:
  SerialPort *arm_port_;
};

#endif
