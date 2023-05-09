/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include "xarm/wrapper/xarm_api.h"


int XArmAPI::start_record_trajectory(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_record_traj(1);
}

int XArmAPI::stop_record_trajectory(char* filename) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_record_traj(0);
  if (ret == 0 && filename != NULL) {
    int ret2 = save_record_trajectory(filename, 10);
    return ret2;
  }
  return ret;
}

int XArmAPI::save_record_trajectory(char* filename, float timeout) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int status;
  get_trajectory_rw_status(&status);
  std::string feedback_key = _gen_feedback_key(true);
  int ret = core->save_traj(filename, feedback_key);
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret);
  if (ret == 0) {
    return _wait_save_traj(timeout, trans_id, filename);
  }
  return ret;
}

int XArmAPI::load_trajectory(char* filename, float timeout) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int status;
  get_trajectory_rw_status(&status);
  std::string feedback_key = _gen_feedback_key(true);
  int ret = core->load_traj(filename, feedback_key);
  int trans_id = _get_feedback_transid(feedback_key);
  ret = _check_code(ret);
  if (ret == 0) {
    return _wait_load_traj(timeout, trans_id, filename);
  }
  return ret;
}

int XArmAPI::playback_trajectory(int times, char* filename, bool wait, int double_speed) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = 0;
  if (filename != NULL) {
    ret = load_trajectory(filename, -1);
    if (ret != 0) {
      return ret;
    }
  }
  if (state == 4) return API_CODE::NOT_READY;
  std::string feedback_key = _gen_feedback_key(true);
  ret = core->playback_traj(times, double_speed, feedback_key);
  int trans_id = _get_feedback_transid(feedback_key);
  if (ret == 0 && wait) {
    return _wait_play_traj(0, trans_id, times);
  }
  return ret;
}

int XArmAPI::get_trajectory_rw_status(int *status) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->get_traj_rw_status(status);
}

int XArmAPI::_check_traj_status(int status, std::string filename)
{
  if (status == TRAJ_STATE::LOAD_SUCCESS) {
    // printf("Load %s success\n", filename.c_str());
    return 0;
  }
  else if (status == TRAJ_STATE::LOAD_FAIL) {
    // fprintf(stderr, "Load %s failed\n", filename.c_str());
    return API_CODE::TRAJ_RW_FAILED;
  }
  if (status == TRAJ_STATE::SAVE_SUCCESS) {
    // printf("Save %s success\n", filename.c_str());
    return 0;
  }
  else if (status == TRAJ_STATE::SAVE_FAIL) {
    // fprintf(stderr, "Save %s failed\n", filename.c_str());
    return API_CODE::TRAJ_RW_FAILED;
  }
  return -1;
}

int XArmAPI::_wait_traj_op(fp32 timeout, int trans_id, std::string filename, std::string op)
{
  int code = 0;
  int status = 0;
  if (trans_id > 0 && support_feedback_) {
    int feedback_code;
    code = _wait_feedback(timeout, trans_id, &feedback_code);
    get_trajectory_rw_status(&status);
    if (code == 0) {
      int success_status = op == "Load" ? TRAJ_STATE::LOAD_SUCCESS : TRAJ_STATE::SAVE_SUCCESS; 
      int failure_status = op == "Load" ? TRAJ_STATE::LOAD_FAIL : TRAJ_STATE::SAVE_FAIL;
      status = feedback_code == FeedbackCode::SUCCESS ? success_status : FeedbackCode::FAILURE ? failure_status : status;
    }
    return _check_traj_status(status);
  }
  else {
    int idle_cnts = 0;
    long long expired = timeout <= 0 ? 0 : (get_system_time() + (long long)(timeout * 1000));
    while (timeout <= 0 || get_system_time() < expired) {
      sleep_milliseconds(100);
      code = get_trajectory_rw_status(&status);
      if (_check_code(code) == 0) {
        if (status == TRAJ_STATE::IDLE) {
          idle_cnts += 1;
          if (idle_cnts >= 5) {
            // fprintf(stderr, "%s %s failed\n", op.c_str(), filename.c_str());
            return API_CODE::TRAJ_RW_FAILED;
          }
        }
        else {
          code = _check_traj_status(status, filename);
          if (code >= 0) return code;
        }
      }
    }
    // fprintf(stderr, "%s %s timeout\n", op.c_str(), filename.c_str());
    return API_CODE::TRAJ_RW_TOUT;
  }
}

int XArmAPI::_wait_load_traj(fp32 timeout, int trans_id, std::string filename)
{
  return _wait_traj_op(timeout, trans_id, filename, "Load");
}

int XArmAPI::_wait_save_traj(fp32 timeout, int trans_id, std::string filename)
{
  return _wait_traj_op(timeout, trans_id, filename, "Save");
}

int XArmAPI::_wait_play_traj(fp32 timeout, int trans_id, int times)
{
  int code = 0;
  if (trans_id > 0 && support_feedback_) {
    int feedback_code = 0;
    code = _wait_feedback(timeout, trans_id, &feedback_code);
    if (feedback_code == FeedbackCode::FAILURE)
      code = API_CODE::TRAJ_PLAYBACK_FAILED;
  }
  else {
    long long start_time = get_system_time();
    while (state != 1) {
      if (state == 4) return API_CODE::NOT_READY;
      if (get_system_time() - start_time > 5000) return API_CODE::TRAJ_PLAYBACK_TOUT;
      sleep_milliseconds(100);
    }
    int max_count = int((get_system_time() - start_time) * 100);
    max_count = max_count > 10 ? max_count : 10;
    start_time = get_system_time();
    while (mode != 11) {
      if (state == 1) {
        start_time = get_system_time();
        sleep_milliseconds(100);
        continue;
      }
      if (state == 4) {
        return API_CODE::NOT_READY;
      }
      if (get_system_time() - start_time > 5000) {
        return API_CODE::TRAJ_PLAYBACK_TOUT;
      }
      sleep_milliseconds(100);
    }
    sleep_milliseconds(100);
    int cnt = 0;
    while (state != 4) {
      if (state == 2) {
        if (times == 1) break;
        cnt += 1;
      }
      else {
        cnt = 0;
      }
      if (cnt > max_count) break;
      sleep_milliseconds(100);
    }
    code = state != 4 ? 0 : API_CODE::STATE_NOT_READY;
  }
  if (state != 4) {
    set_mode(0);
    set_state(0);
  }
  _sync();
  return code;
}
