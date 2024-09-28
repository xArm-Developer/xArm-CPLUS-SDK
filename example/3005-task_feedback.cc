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

void task_feedback_callback(unsigned char *feedback_data)
{
  unsigned short cmd_id = bin8_to_16(&feedback_data[0]);
  unsigned char feedback_type = feedback_data[8];
  unsigned char feedback_funcode = feedback_data[9];
  unsigned short feedback_taskid = bin8_to_16(&feedback_data[10]);
  unsigned char feedback_code = feedback_data[12];
  unsigned long long feedback_us = bin8_to_64(&feedback_data[13]);
  switch (feedback_type) {
    case 1:
      printf("[FB] motion task %d starts execution, funcode=%d, cmd_id=%d, us=%lld\n",feedback_taskid, feedback_funcode, cmd_id, feedback_us);
      break;
    case 2:
      if (feedback_code == 0)
        printf("[FB] motion task %d execution completed, funcode=%d, cmd_id=%d, us=%lld\n",feedback_taskid, feedback_funcode, cmd_id, feedback_us);
      else if (feedback_code == 2) 
        printf("[FB] motion task %d is discarded, funcode=%d, cmd_id=%d, us=%lld\n",feedback_taskid, feedback_funcode, cmd_id, feedback_us);
      break;
    case 4:
      printf("[FB] task %d is triggered, funcode=%d, cmd_id=%d, us=%lld\n",feedback_taskid, feedback_funcode, cmd_id, feedback_us);
      break;
    case 32:
      printf("[FB] cmd %d starts execution, funcode=%d, us=%lld\n",cmd_id, feedback_funcode, feedback_us);
      break;
    case 64:
      if (feedback_code == 0)
        printf("[FB] cmd %d execution success, funcode=%d, us=%lld\n",cmd_id, feedback_funcode, feedback_us);
      else if (feedback_code == 1) 
        printf("[FB] cmd %d execution failure, funcode=%d, us=%lld\n",cmd_id, feedback_funcode, feedback_us);
      break;
      
      break;
  }
}


int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  std::string port(argv[1]);

  XArmAPI *arm = new XArmAPI(port);
  sleep_milliseconds(500);
  if (arm->error_code != 0) arm->clean_error();
  if (arm->warn_code != 0) arm->clean_warn();
  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(4);
  arm->set_state(0);
  sleep_milliseconds(500);

  printf("=========================================\n");
  int ret;
  arm->move_gohome(true);

  arm->register_feedback_callback(task_feedback_callback);
  arm->set_feedback_type(14);  // task finish(2) + task discard(4) + task trigger(8)

  fp32 pose1[6] = {300, 0, 200, 180, 0, 0};
  fp32 pose2[6] = {300, 100, 200, 180, 0, 0};
  fp32 pose3[6] = {400, 100, 200, 180, 0, 0};
  fp32 pose4[6] = {400, -100, 200, 180, 0, 0};
  fp32 pose5[6] = {300, -100, 200, 180, 0, 0};
  fp32 pose6[6] = {300, 0, 200, 180, 0, 0};
  fp32 radius = 0;
  fp32 trigger_xyz[3] = {395, 100, 200};

  int cycles = 100;
  for (int i = 0; i < cycles; ++i) {
    ret = arm->set_cgpio_digital(0, 0);
    if (ret != 0) {
      printf("set_cgpio_digital, ret=%d\n", ret);
      break;
    }
    ret = arm->set_position(pose1, radius, false);
    if (ret != 0) {
      printf("set_position, ret=%d\n", ret);
      break;
    }
    if (ret != 0) break;
    arm->set_cgpio_analog_with_xyz(0, 1, trigger_xyz, 2);
    if (ret != 0) {
      printf("set_cgpio_analog_with_xyz, ret=%d\n", ret);
      break;
    }
    ret = arm->set_position(pose2, radius, false);
    if (ret != 0) {
      printf("set_position, ret=%d\n", ret);
      break;
    }
    ret = arm->set_position(pose3, radius, false);
    if (ret != 0) {
      printf("set_position, ret=%d\n", ret);
      break;
    }
    ret = arm->set_position(pose4, radius, false);
    if (ret != 0) {
      printf("set_position, ret=%d\n", ret);
      break;
    }
    ret = arm->set_position(pose5, radius, false);
    if (ret != 0) {
      printf("set_position, ret=%d\n", ret);
      break;
    }
    ret = arm->set_position(pose6, radius, false);
    if (ret != 0) {
      printf("set_position, ret=%d\n", ret);
      break;
    }
  }
  
  while (arm->is_connected() && arm->state < 4) {
    sleep_milliseconds(1000);
  }

  return 0;
}