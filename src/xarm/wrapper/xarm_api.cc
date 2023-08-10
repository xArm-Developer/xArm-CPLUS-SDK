/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#define _CRT_SECURE_NO_WARNINGS 1
// #pragma warning(disable:4996)

#include <random>
#include "xarm/wrapper/xarm_api.h"

const int FEEDBACK_QUE_SIZE = 256;
const int FEEDBACK_DATA_MAX_LEN = 256;

fp32 to_radian(fp32 val) {
  return (fp32)(val / RAD_DEGREE);
}

fp32 to_degree(fp32 val) {
  return (fp32)(val * RAD_DEGREE);
}

static std::default_random_engine engine;

XArmAPI::XArmAPI(
  const std::string &port,
  bool is_radian,
  bool do_not_open,
  bool check_tcp_limit,
  bool check_joint_limit,
  bool check_cmdnum_limit,
  bool check_robot_sn,
  bool check_is_ready,
  bool check_is_pause,
  int max_callback_thread_count,
  int max_cmdnum,
  int init_axis,
  bool debug,
  std::string report_type,
  bool baud_checkset)
  : default_is_radian(is_radian), port_(port),
  check_tcp_limit_(check_tcp_limit), check_joint_limit_(check_joint_limit),
  check_cmdnum_limit_(check_cmdnum_limit), check_robot_sn_(check_robot_sn),
  check_is_ready_(check_is_ready), check_is_pause_(check_is_pause), baud_checkset_flag_(baud_checkset) {
  // default_is_radian = is_radian;
  // check_tcp_limit_ = check_tcp_limit;
  max_callback_thread_count_ = max_callback_thread_count;
  if (max_callback_thread_count_ < 0) max_callback_thread_count_ = 100;
  pool_.set_max_thread_count(max_callback_thread_count_);
  callback_in_thread_ = max_callback_thread_count_ != 0;
  max_cmdnum_ = max_cmdnum > 0 ? max_cmdnum : 256;
  axis = init_axis;
  report_type_ = report_type;
  debug_ = debug;
  _init();
  printf("SDK_VERSION: %s\n", SDK_VERSION);
  if (!do_not_open) {
    connect();
  }
}

XArmAPI::~XArmAPI() {
  disconnect();
  _destroy();
}

void XArmAPI::_destroy(void) {
  if (version_number != NULL) {
    delete[] version_number;
    version_number = NULL;
  }
  if (angles != NULL) {
    delete[] angles;
    angles = NULL;
  }
  if (last_used_angles != NULL) {
    delete[] last_used_angles;
    last_used_angles = NULL;
  }
  if (tcp_offset != NULL) {
    delete[] tcp_offset;
    tcp_offset = NULL;
  }
  if (joint_speed_limit != NULL) {
    delete[] joint_speed_limit;
    joint_speed_limit = NULL;
  }
  if (joint_acc_limit != NULL) {
    delete[] joint_acc_limit;
    joint_acc_limit = NULL;
  }
  if (position != NULL) {
    delete[] position;
    position = NULL;
  }
  if (position_aa != NULL) {
    delete[] position_aa;
    position_aa = NULL;
  }
  if (last_used_position != NULL) {
    delete[] last_used_position;
    last_used_position = NULL;
  }
  if (joints_torque != NULL) {
    delete[] joints_torque;
    joints_torque = NULL;
  }
  if (motor_brake_states != NULL) {
    delete[] motor_brake_states;
    motor_brake_states = NULL;
  }
  if (motor_enable_states != NULL) {
    delete[] motor_enable_states;
    motor_enable_states = NULL;
  }
  if (tcp_load != NULL) {
    delete[] tcp_load;
    tcp_load = NULL;
  }
  if (tcp_speed_limit != NULL) {
    delete[] tcp_speed_limit;
    tcp_speed_limit = NULL;
  }
  if (tcp_acc_limit != NULL) {
    delete[] tcp_acc_limit;
    tcp_acc_limit = NULL;
  }
  if (gravity_direction != NULL) {
    delete[] gravity_direction;
    gravity_direction = NULL;
  }
  if (realtime_joint_speeds != NULL) {
    delete[] realtime_joint_speeds;
    realtime_joint_speeds = NULL;
  }
  if (world_offset != NULL) {
    delete[] world_offset;
    world_offset = NULL;
  }
  if (temperatures != NULL) {
    delete[] temperatures;
    temperatures = NULL;
  }
  if (gpio_reset_config != NULL) {
    delete[] gpio_reset_config;
    gpio_reset_config = NULL;
  }
  if (ft_ext_force != NULL) {
    delete[] ft_ext_force;
    ft_ext_force = NULL;
  }
  if (ft_raw_force != NULL) {
    delete[] ft_raw_force;
    ft_raw_force = NULL;
  }
  if (voltages != NULL) {
    delete[] voltages;
    voltages = NULL;
  }
  if (currents != NULL) {
    delete[] currents;
    currents = NULL;
  }
  if (collision_model_params != NULL) {
    delete[] collision_model_params;
    collision_model_params = NULL;
  }
  if (cgpio_input_digitals != NULL) {
    delete[] cgpio_input_digitals;
    cgpio_input_digitals = NULL;
  }
  if (cgpio_output_digitals != NULL) {
    delete[] cgpio_output_digitals;
    cgpio_output_digitals = NULL;
  }
  if (cgpio_intput_anglogs != NULL) {
    delete[] cgpio_intput_anglogs;
    cgpio_intput_anglogs = NULL;
  }
  if (cgpio_output_anglogs != NULL) {
    delete[] cgpio_output_anglogs;
    cgpio_output_anglogs = NULL;
  }
  if (cgpio_input_conf != NULL) {
    delete[] cgpio_input_conf;
    cgpio_input_conf = NULL;
  }
  if (cgpio_output_conf != NULL) {
    delete[] cgpio_output_conf;
    cgpio_output_conf = NULL;
  }
  if (report_rich_data_ptr_ != NULL) {
    delete report_rich_data_ptr_;
    report_rich_data_ptr_ = NULL;
  }
}

void XArmAPI::_init(void) {
  core = NULL;
  stream_tcp_ = NULL;
  core503_ = NULL;
  stream_tcp503_ = NULL;
  stream_tcp_report_ = NULL;
  stream_tcp_rich_report_ = NULL;
  stream_ser_ = NULL;
  is_ready_ = true;
  is_tcp_ = true;
  is_old_protocol_ = false;
  is_first_report_ = true;
  is_sync_ = false;
  arm_type_is_1300_ = false;
  control_box_type_is_1300_ = false;

  major_version_number_ = 0;
  minor_version_number_ = 0;
  revision_version_number_ = 0;
  version_number = new int[3]{ major_version_number_, minor_version_number_, revision_version_number_ };

  mt_brake_ = 0;
  mt_able_ = 0;
  min_tcp_speed_ = (float)0.1;    // mm/s
  max_tcp_speed_ = 1000;   // mm/s
  min_tcp_acc_ = 1.0;      // mm/s^2
  max_tcp_acc_ = 50000;    // mm/s^2
  min_joint_speed_ = (float)0.01; // rad/s
  max_joint_speed_ = 4.0;  // rad/s
  min_joint_acc_ = (float)0.01;   // rad/s^2
  max_joint_acc_ = 20.0;   // rad/s^2
  count = -1;
  iden_progress = 0;
  keep_heart_ = true;

  sleep_finish_time_ = get_system_time();

  angles = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
  last_used_angles = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
  tcp_offset = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
  if (default_is_radian) {
    joint_speed_limit = new fp32[2]{ min_joint_speed_, max_joint_speed_ };
    joint_acc_limit = new fp32[2]{ min_joint_acc_, max_joint_acc_ };
    last_used_joint_speed = (fp32)0.3490658503988659; // rad/s (20°/s);
    last_used_joint_acc = (fp32)8.726646259971648;    // rad/s^2 (500°/s^2);
    position = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
    position_aa = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
    last_used_position = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
  }
  else {
    joint_speed_limit = new fp32[2]{ to_degree(min_joint_speed_), to_degree(max_joint_speed_) };
    joint_acc_limit = new fp32[2]{ to_degree(min_joint_acc_), to_degree(max_joint_acc_) };
    last_used_joint_speed = to_degree((fp32)0.3490658503988659); // rad/s (20°/s);
    last_used_joint_acc = to_degree((fp32)8.726646259971648);    // rad/s^2 (500°/s^2);
    position = new fp32[6]{ 201.5, 0, 140.5, to_degree((fp32)3.1415926), 0, 0 };
    position_aa = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
    last_used_position = new fp32[6]{ 201.5, 0, 140.5, to_degree((fp32)3.1415926), 0, 0 };
  }

  state = 4;
  mode = 0;
  cmd_num = 0;
  joints_torque = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
  motor_brake_states = new bool[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };
  motor_enable_states = new bool[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };
  error_code = 0;
  warn_code = 0;
  tcp_load = new fp32[4]{ 0, 0, 0, 0 };
  collision_sensitivity = 0;
  teach_sensitivity = 0;
  device_type = 7;
  axis = 7;
  master_id = 0;
  slave_id = 0;
  motor_tid = 0;
  motor_fid = 0;
  tcp_jerk = 1000;        // mm/s^3
  joint_jerk = default_is_radian ? (fp32)20.0 : to_degree((fp32)20.0); // 20 rad/s^3
  rot_jerk = (float)2.3;
  max_rot_acc = (float)2.7;
  tcp_speed_limit = new fp32[2]{ min_tcp_speed_, max_tcp_speed_ };
  tcp_acc_limit = new fp32[2]{ min_tcp_acc_, max_tcp_acc_ };
  last_used_tcp_speed = 100;  // mm/s
  last_used_tcp_acc = 2000;   // mm/s^2
  gravity_direction = new fp32[3]{ 0, 0, -1 };
  realtime_tcp_speed = 0;
  realtime_joint_speeds = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
  world_offset = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
  temperatures = new fp32[7]{ 0, 0, 0, 0, 0, 0 };
  gpio_reset_config = new unsigned char[2]{0, 0};
  ft_ext_force = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
  ft_raw_force = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
  modbus_baud_ = -1;
  ignore_error_ = false;
  ignore_state_ = false;

  gripper_is_enabled_ = false;
  bio_gripper_is_enabled_ = false;
  robotiq_is_activated_ = false;
  last_report_time_ = get_system_time();
  max_report_interval_ = 0;
  voltages = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
  currents = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
  is_simulation_robot = 0;
  is_collision_detection = 0;
  collision_tool_type = 0;
  collision_model_params = new fp32[6]{ 0, 0, 0, 0, 0, 0};
  cgpio_state = 0;
  cgpio_code = 0;
  cgpio_input_digitals = new int[2]{ 0, 0 };
  cgpio_output_digitals = new int[2]{ 0, 0 };
  cgpio_intput_anglogs = new fp32[2]{ 0, 0 };
  cgpio_output_anglogs = new fp32[2]{ 0, 0 };
  cgpio_input_conf = new int[16]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  cgpio_output_conf = new int[16]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  cmd_timeout_ = -1;

  xarm_gripper_error_code_ = 0;
  bio_gripper_error_code_ = 0;
  robotiq_error_code_ = 0;
  gripper_version_numbers_[0] = -1;
  gripper_version_numbers_[1] = -1;
  gripper_version_numbers_[2] = -1;

  linear_track_baud_ = -1;
  linear_track_speed_ = 0;

  memset(&linear_track_status, 0, sizeof(linear_track_status));
  linear_track_status.sci = 1;

  default_bio_baud_ = 2000000;
  default_gripper_baud_ = 2000000;
  default_robotiq_baud_ = 115200;
  default_linear_track_baud_ = 2000000;

  only_check_type_ = 0;
  only_check_result = 0;
  support_feedback_ = false;

  is_reduced_mode = false;
  is_fence_mode = false;
  is_report_current = false;
  is_approx_motion = false;
  is_cart_continuous = false;

  report_rich_data_ptr_ = new XArmReportData("rich");
  if (report_type_ != "rich") {
    report_data_ptr_ = new XArmReportData(report_type_);
  }
  else {
    report_data_ptr_ = report_rich_data_ptr_;
  }
}

int XArmAPI::set_baud_checkset_enable(bool enable)
{
  baud_checkset_flag_ = enable;
  return 0;
}

int XArmAPI::set_checkset_default_baud(int type, int baud)
{
  switch (type) {
    case 1:
      default_gripper_baud_ = baud;
      break;
    case 2:
      default_bio_baud_ = baud;
      break;
    case 3:
      default_robotiq_baud_ = baud;
      break;
    case 4:
      default_linear_track_baud_ = baud;
      break;
    default:
      return API_CODE::API_EXCEPTION;
  }
  return 0;
}

int XArmAPI::get_checkset_default_baud(int type, int *baud)
{
  switch (type) {
    case 1:
      *baud = default_gripper_baud_;
      break;
    case 2:
      *baud = default_bio_baud_;
      break;
    case 3:
      *baud = default_robotiq_baud_;
      break;
    case 4:
      *baud = default_linear_track_baud_;
      break;
    default:
      return API_CODE::API_EXCEPTION;
  }
  return 0;
}

bool XArmAPI::has_err_warn(void) {
  return has_error() || has_warn();
}

bool XArmAPI::has_error(void) {
  return error_code != 0;
}

bool XArmAPI::has_warn(void) {
  return warn_code != 0;
}

bool XArmAPI::is_connected(void) {
  return is_tcp_ ? (stream_tcp_ == NULL ? false : stream_tcp_->is_ok() == 0) : (stream_ser_ == NULL ? false : stream_ser_->is_ok() == 0);
}

bool XArmAPI::_is_connected_503(void) {
  return stream_tcp503_ == NULL ? false : stream_tcp503_->is_ok() == 0;
}

bool XArmAPI::is_lite6(void) {
  return axis == 6 && device_type == 9;
}

bool XArmAPI::is_850(void) {
  return axis == 6 && device_type == 12;
}

bool XArmAPI::is_reported(void) {
  return is_tcp_ ? (stream_tcp_report_ == NULL ? false : stream_tcp_report_->is_ok() == 0) : false;
}

bool XArmAPI::_is_rich_reported(void) {
  return is_tcp_ ? (stream_tcp_rich_report_ == NULL ? false : stream_tcp_rich_report_->is_ok() == 0) : false;
}

static void report_rich_thread_handle(void *arg) {
  XArmAPI *my_this = (XArmAPI *)arg;
  my_this->_handle_report_rich_data();
}

static void report_thread_handle(void *arg) {
  XArmAPI *my_this = (XArmAPI *)arg;
  my_this->_handle_report_data();
}

static void feedback_thread_handle(void *arg) {
  XArmAPI *my_this = (XArmAPI *)arg;
  my_this->_handle_feedback_data();
}

void XArmAPI::_sync(void) {
  memcpy(last_used_position, position, 24);
  memcpy(last_used_angles, angles, 28);
}

void XArmAPI::_check_version(void) {
  int cnt = 5;
  unsigned char version_[40] = { 0 };
  int ret = -1;
  while ((ret < 0 || ret > 2) && cnt > 0) {
    ret = get_version(version_);
    sleep_milliseconds(100);
    cnt -= 1;
  }
  std::string v((const char *)version_);
  // std::regex pattern_new(".*(\\d+),(\\d+),(\\S+),(\\S+),.*[vV](\\d+)\\.(\\d+)\\.(\\d+).*");
  std::regex pattern_new(".*(\\d+),(\\d+),(.*),(.*),.*[vV]*(\\d+)\\.(\\d+)\\.(\\d+).*");
  std::regex pattern(".*[vV]*(\\d+)\\.(\\d+)\\.(\\d+).*");
  // std::regex pattern(".*[vV](\\d+)[.](\\d+)[.](\\d+).*");
  std::smatch result;
  int arm_type = 0;
  int control_type = 0;
  if (std::regex_match(v, result, pattern_new)) {
    auto it = result.begin();
    auto str_axis = std::string(*++it);
    auto str_device_type = std::string(*++it);
    auto str_arm_type = std::string(*++it);
    auto str_control_type = std::string(*++it);

    sscanf(str_axis.data(), "%d", &axis);
    sscanf(str_device_type.data(), "%d", &device_type);
    if (str_arm_type.size() >= 6) {
      sscanf(str_arm_type.substr(2, 4).data(), "%d", &arm_type);
    }
    if (str_control_type.size() >= 6) {
      sscanf(str_control_type.substr(2, 4).data(), "%d", &control_type);
    }

    arm_type_is_1300_ = arm_type >= 1300;
    control_box_type_is_1300_ = control_type >= 1300;

    sscanf(std::string(*++it).data(), "%d", &major_version_number_);
    sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
    sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
  }
  else if (std::regex_match(v, result, pattern)) {
    auto it = result.begin();
    sscanf(std::string(*++it).data(), "%d", &major_version_number_);
    sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
    sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
  }
  else {
    std::vector<std::string> tmpList = split(v, "-");
    size_t size = tmpList.size();
    if (size >= 3) {
      int year = atoi(tmpList[size - 3].c_str());
      int month = atoi(tmpList[size - 2].c_str());
      if (year < 2019) is_old_protocol_ = true;
      else if (year == 2019) {
        is_old_protocol_ = month >= 2 ? false : true;
      }
      else {
        is_old_protocol_ = false;
      }
    }
    if (is_old_protocol_) {
      major_version_number_ = 0;
      minor_version_number_ = 0;
      revision_version_number_ = 1;
    }
    else {
      major_version_number_ = 0;
      minor_version_number_ = 1;
      revision_version_number_ = 0;
    }
  }
  version_number[0] = major_version_number_;
  version_number[1] = minor_version_number_;
  version_number[2] = revision_version_number_;
  if (check_robot_sn_) {
    cnt = 5;
    int err_warn[2] = { 0 };
    ret = -1;
    while ((ret < 0 || ret > 2) && cnt > 0 && warn_code == 0) {
      ret = get_robot_sn(version_);
      get_err_warn_code(err_warn);
      sleep_milliseconds(100);
      cnt -= 1;
    }
    printf("ROBOT_SN: %s\n", sn);
  }
  printf("ROBOT_IP: %s, VERSION: v%d.%d.%d, PROTOCOL: V%d, DETAIL: %s, TYPE1300: [%d, %d]\n", port_.c_str(), major_version_number_, minor_version_number_, revision_version_number_, is_old_protocol_ ? 0 : 1, version_, control_box_type_is_1300_, arm_type_is_1300_);
}

void XArmAPI::_wait_until_not_pause(void) {
  if (is_connected() && check_is_pause_ && state == 3) {
    std::unique_lock<std::mutex> locker(mutex_);
    cond_.wait(locker, [this] { return state != 3 || !is_connected(); });
    locker.unlock();
  }
}

void XArmAPI::_wait_until_cmdnum_lt_max(void) {
  if (!check_cmdnum_limit_) return;
  int cmdnum_ = 0;
  while (is_connected() && cmd_num >= max_cmdnum_) {
    if (get_system_time() - last_report_time_ > 400) {
      get_cmdnum(&cmdnum_);
    }
    sleep_milliseconds(50);
  }
}

int XArmAPI::_xarm_is_ready(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (check_is_ready_ && !_version_is_ge(1, 5, 20)) {
    if (error_code != 0) return API_CODE::HAS_ERROR;
    if (state == 4 || state == 5) return API_CODE::NOT_READY;
    return is_ready_ ? 0 : API_CODE::NOT_READY;
  }
  // no check if version >= 1.5.20
  return 0;
}

int XArmAPI::_check_code(int code, bool is_move_cmd, int mode_) {
  if (is_move_cmd) {
    if (code == 0 || code == UXBUS_STATE::WAR_CODE) {
      if (core->state_is_ready) {
        if (mode_ >= 0 && mode != mode_) {
          fprintf(stderr, "The mode may be incorrect, just as a reminder, mode: %d (%d)\n", mode_, mode);
        }
        return 0;
        // return (mode_ < 0 || mode == mode_) ? 0 : API_CODE::MODE_IS_NOT_CORRECT;
      }
      else {
        return UXBUS_STATE::STATE_NOT_READY;
      }
    }
    else {
      return code;
    }
    // return ((code == 0 || code == UXBUS_STATE::WAR_CODE) && core->state_is_ready) ? 0 : !core->state_is_ready ? UXBUS_STATE::STATE_NOT_READY : code;
  }
  else {
    return (code == 0 || code == UXBUS_STATE::ERR_CODE || code == UXBUS_STATE::WAR_CODE || code == UXBUS_STATE::STATE_NOT_READY) ? 0 : code;
  }
}

bool XArmAPI::_version_is_ge(int major, int minor, int revision) {
  if (major_version_number_ == 0 && minor_version_number_ == 0 && revision_version_number_ == 0) {
    unsigned char version_[40] = { 0 };
    get_version(version_);

    std::string v((const char *)version_);
    std::regex pattern_new(".*(\\d+),(\\d+),(\\S+),(\\S+),.*[vV](\\d+)\\.(\\d+)\\.(\\d+)");
    std::regex pattern(".*[vV](\\d+)\\.(\\d+)\\.(\\d+)");
    std::smatch result;
    int arm_type = 0;
    int control_type = 0;
    if (std::regex_match(v, result, pattern_new)) {
      auto it = result.begin();
      sscanf(std::string(*++it).data(), "%d", &axis);
      sscanf(std::string(*++it).data(), "%d", &device_type);
      sscanf(std::string(*++it).substr(2, 4).data(), "%d", &arm_type);
      sscanf(std::string(*++it).substr(2, 4).data(), "%d", &control_type);

      arm_type_is_1300_ = arm_type >= 1300;
      control_box_type_is_1300_ = control_type >= 1300;

      sscanf(std::string(*++it).data(), "%d", &major_version_number_);
      sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
      sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
    }
    else if (std::regex_match(v, result, pattern)) {
      auto it = result.begin();
      sscanf(std::string(*++it).data(), "%d", &major_version_number_);
      sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
      sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
    }
    else {
      std::vector<std::string> tmpList = split(v, "-");
      size_t size = tmpList.size();
      if (size >= 3) {
        int year = atoi(tmpList[size - 3].c_str());
        int month = atoi(tmpList[size - 2].c_str());
        if (year < 2019) is_old_protocol_ = true;
        else if (year == 2019) {
          is_old_protocol_ = month >= 2 ? false : true;
        }
        else {
          is_old_protocol_ = false;
        }
      }
      if (is_old_protocol_) {
        major_version_number_ = 0;
        minor_version_number_ = 0;
        revision_version_number_ = 1;
      }
      else {
        major_version_number_ = 0;
        minor_version_number_ = 1;
        revision_version_number_ = 0;
      }
    }
    version_number[0] = major_version_number_;
    version_number[1] = minor_version_number_;
    version_number[2] = revision_version_number_;
  }
  return major_version_number_ > major || (major_version_number_ == major && minor_version_number_ > minor) || (major_version_number_ == major && minor_version_number_ == minor && revision_version_number_ >= revision);
}

int XArmAPI::connect(const std::string &port) {
  if (is_connected()) return 0;
  if (port != "" && port != port_) {
    port_ = port;
  }
  if (port_ == "") {
    fprintf(stderr, "can not connect to port/ip: %s\n", port_.data());
    return API_CODE::NOT_CONNECTED;
  }
  // std::regex pattern("(\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})");
  std::regex pattern("(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)[.]){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)");
  is_ready_ = true;
  if (port_ == "localhost" || std::regex_match(port_, pattern)) {
    is_tcp_ = true;
    stream_tcp_ = new SocketPort((char *)port_.data(), XARM_CONF::TCP_PORT_CONTROL, 3, 320, 0, FEEDBACK_QUE_SIZE, FEEDBACK_DATA_MAX_LEN);
    if (stream_tcp_->is_ok() != 0) {
      fprintf(stderr, "Error: Tcp control connection failed\n");
      return -2;
    }
    feedback_thread_ = std::thread(feedback_thread_handle, this);
    feedback_thread_.detach();

    core = new UxbusCmdTcp((SocketPort *)stream_tcp_, std::bind(&XArmAPI::_set_feedback_key_transid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    printf("Tcp control connection successful\n");
    core->set_protocol_identifier(2);

    sleep_milliseconds(200);
    _check_version();

    support_feedback_ = _version_is_ge(2, 0, 102);

    stream_tcp_rich_report_ = connect_tcp_report((char *)port_.data(), "rich");
    if (report_type_ == "rich") {
      stream_tcp_report_ = stream_tcp_rich_report_;
      _report_connect_changed_callback();
    }
    if (!_is_rich_reported()) { return -3; }
    report_rich_thread_ = std::thread(report_rich_thread_handle, this);
    report_rich_thread_.detach();

    if (report_type_ != "rich") {
      stream_tcp_report_ = connect_tcp_report((char *)port_.data(), report_type_);
      _report_connect_changed_callback();
      if (!is_reported()) { return -3; }
      report_thread_ = std::thread(report_thread_handle, this);
      report_thread_.detach();
    }
  }
  else {
    is_tcp_ = false;
    stream_ser_ = new SerialPort((const char *)port_.data(), XARM_CONF::SERIAL_BAUD, 3, 320);
    core = new UxbusCmdSer((SerialPort *)stream_ser_);
    _report_connect_changed_callback();
    sleep_milliseconds(200);
    _check_version();
  }
  if (cmd_timeout_ > 0)
    set_timeout(cmd_timeout_);

  return 0;
}

int XArmAPI::_connect_503(void)
{
  stream_tcp503_ = new SocketPort((char *)port_.data(), XARM_CONF::TCP_PORT_CONTROL + 1, 3, 320);
  if (stream_tcp503_->is_ok() != 0) {
    return -2;
  }
  core503_ = new UxbusCmdTcp((SocketPort *)stream_tcp503_, std::bind(&XArmAPI::_set_feedback_key_transid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  return 0;
}

void XArmAPI::disconnect(void) {
  if (stream_tcp_ != NULL) {
    stream_tcp_->close_port();
    stream_tcp_ = NULL;
  }
  if (stream_tcp503_ != NULL) {
    stream_tcp503_->close_port();
    stream_tcp503_ = NULL;
  }
  if (stream_ser_ != NULL) {
    stream_ser_->close_port();
    stream_ser_ = NULL;
  }
  if (stream_tcp_rich_report_ != NULL) {
    stream_tcp_rich_report_->close_port();
    stream_tcp_rich_report_ = NULL;
  }
  if (stream_tcp_report_ != NULL && report_type_ != "rich") {
    stream_tcp_report_->close_port();
    stream_tcp_report_ = NULL;
  }
  _report_connect_changed_callback();
  is_ready_ = false;
}

int XArmAPI::set_timeout(fp32 timeout) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (timeout <= 0) return API_CODE::PARAM_ERROR;
  cmd_timeout_ = timeout;
  return core->set_timeout(timeout);
}

int XArmAPI::get_version(unsigned char version_[40]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_version(version_);
  return _check_code(ret);
}

int XArmAPI::get_robot_sn(unsigned char robot_sn[40]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  char str[40] = { 0 };
  int ret = core->get_robot_sn((unsigned char*)str);
  ret = _check_code(ret);
  if (ret == 0) {
    int arm_type = 0;
    int control_type = 0;
    char *control_box_sn = strchr(str, '\0') + 1;
    if (strlen(str) >= 6)
      sscanf(std::string(str).substr(2, 4).data(), "%d", &arm_type);
    if (strlen(control_box_sn) >= 6)
      sscanf(std::string(control_box_sn).substr(2, 4).data(), "%d", &control_type);
    arm_type_is_1300_ = arm_type >= 1300;
    control_box_type_is_1300_ = control_type >= 1300;
    memcpy(robot_sn, str, 14);
    memcpy(sn, robot_sn, 40);
  }
  return ret;
}

int XArmAPI::system_control(int value) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->system_control(value);
  return _check_code(ret);
}

int XArmAPI::shutdown_system(int value) {
  return system_control(value);
}

int XArmAPI::get_state(int *state_) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_state(state_);
  ret = _check_code(ret);
  if (ret == 0) {
    state = *state_;
  }
  return ret;
}

int XArmAPI::get_cmdnum(int *cmdnum_) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_cmdnum(cmdnum_);
  ret = _check_code(ret);
  if (ret == 0) {
    cmd_num = *cmdnum_;
  }
  return ret;
}

int XArmAPI::get_err_warn_code(int err_warn[2]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_err_code(err_warn);
  ret = _check_code(ret);
  if (ret == 0) {
    error_code = err_warn[0];
    warn_code = err_warn[1];
  }
  return ret;
}

int XArmAPI::get_position(fp32 pose[6]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_tcp_pose(pose);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 6; i++) {
      if (!default_is_radian && i > 2) {
        pose[i] = to_degree(pose[i]);
      }
      position[i] = pose[i];
    }
  }
  return ret;
}

int XArmAPI::get_servo_angle(fp32 angs[7], bool is_real) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = 0;
  if (is_real && _version_is_ge(1, 9, 100)) {
    ret = core->get_joint_states(angs, NULL, NULL, 1);
  }
  else {
    ret = core->get_joint_pose(angs);
  }
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 7; i++) {
      if (!default_is_radian) {
        angs[i] = to_degree(angs[i]);
      }
      angles[i] = angs[i];
    }
  }
  return ret;
}

int XArmAPI::get_joint_states(fp32 jposition[7], fp32 velocity[7], fp32 effort[7], int num) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (num < 1 || num > 3) return API_CODE::PARAM_ERROR;
  int ret = core->get_joint_states(jposition, velocity, effort, num);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 7; i++) {
      if (!default_is_radian) {
        jposition[i] = to_degree(jposition[i]);
        if (num >= 2)
          velocity[i] = to_degree(velocity[i]);
      }
    }
  }
  return ret;
}

int XArmAPI::motion_enable(bool enable, int servo_id) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->motion_en(servo_id, int(enable));
  ret = _check_code(ret);
  get_state(&state);
  if (state == 4 || state == 5) {
    sleep_finish_time_ = 0;
    if (debug_ && is_ready_) {
      printf("[motion_enable], xArm is not ready to move\n");
    }
    is_ready_ = false;
  }
  else {
    if (debug_ && !is_ready_) {
      printf("[motion_enable], xArm is ready to move\n");
    }
    is_ready_ = true;
  }
  return ret;
}

int XArmAPI::set_state(int state_) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_state(state_);
  ret = _check_code(ret);
  get_state(&state);
  if (state == 4 || state == 5) {
    // is_sync_ = false;
    sleep_finish_time_ = 0;
    if (debug_ && is_ready_) {
      printf("[set_state], xArm is not ready to move\n");
    }
    is_ready_ = false;
  }
  else {
    if (debug_ && !is_ready_) {
      printf("[set_state], xArm is ready to move\n");
    }
    is_ready_ = true;
  }
  return ret;
}

int XArmAPI::set_mode(int mode_, int detection_param) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  
  int detection_param_ = -1;
  if (_version_is_ge(1, 10, 0))
    detection_param_ = detection_param >= 0 ? detection_param : 0;
  int ret = core->set_mode(mode_, detection_param_);
  return _check_code(ret);
}

int XArmAPI::set_servo_attach(int servo_id) {
  // if (!is_connected()) return API_CODE::NOT_CONNECTED;
  // return core->set_brake(servo_id, 0);
  return motion_enable(true, servo_id);
}

int XArmAPI::set_servo_detach(int servo_id) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->set_brake(servo_id, 1);
}

int XArmAPI::clean_error(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->clean_err();
  get_state(&state);
  if (state == 4 || state == 5) {
    sleep_finish_time_ = 0;
    if (debug_ && is_ready_) {
      printf("[clean_error], xArm is not ready to move\n");
    }
    is_ready_ = false;
  }
  else {
    if (debug_ && !is_ready_) {
      printf("[clean_error], xArm is ready to move\n");
    }
    is_ready_ = true;
  }
  return ret;
}

int XArmAPI::clean_warn(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  return core->clean_war();
}

int XArmAPI::set_pause_time(fp32 sltime) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  int ret = core->sleep_instruction(sltime);
  if (get_system_time() >= sleep_finish_time_) {
    sleep_finish_time_ = get_system_time() + (long long)(sltime * 1000);
  }
  else {
    sleep_finish_time_ = sleep_finish_time_ + (long long)(sltime * 1000);
  }
  return ret;
}

std::string XArmAPI::_gen_feedback_key(bool wait)
{
  std::string feedback_key = (wait && support_feedback_) ? (std::to_string(get_us()) + std::to_string(engine())) : "";
  fb_key_transid_map_[feedback_key != "" ? feedback_key : "no_use"] = -1;
  return feedback_key;
}

int XArmAPI::_get_feedback_transid(std::string feedback_key)
{
  int trans_id = -1;
  if (feedback_key != "" && fb_key_transid_map_.count(feedback_key)) {
    trans_id = fb_key_transid_map_[feedback_key];
    fb_key_transid_map_.erase(feedback_key);
  }
  return trans_id;
}

void XArmAPI::_set_feedback_key_transid(std::string feedback_key, int trans_id, unsigned char feedback_type) {
  fb_key_transid_map_[feedback_key] = trans_id;
  fb_transid_type_map_[trans_id] = feedback_type;
  if (fb_transid_result_map_.count(trans_id)) {
    fb_transid_result_map_.erase(trans_id);
  }
}

int XArmAPI::_wait_feedback(fp32 timeout, int trans_id, int *feedback_code) {
  long long start_time = get_system_time();
  long long expired = timeout <= 0 ? 0 : (get_system_time() + (long long)(timeout * 1000) + (sleep_finish_time_ > start_time ? sleep_finish_time_ : 0));
  int state_ = state;
  int ret = get_state(&state_);
  int state5_cnt = 0;
  while (timeout <= 0 || get_system_time() < expired) {
    if (!is_connected()) {
      fb_transid_result_map_.clear();
      return API_CODE::NOT_CONNECTED;
    }
    if (error_code != 0) {
      fb_transid_result_map_.clear();
      return API_CODE::HAS_ERROR;
    }

    ret = get_state(&state_);
    if (ret != 0) return ret;

    if (state_ >= 4) {
      sleep_finish_time_ = 0;
      if (state_ == 5) state5_cnt++;
      if (state_ != 5 || state5_cnt >= 20) {
        fb_transid_result_map_.clear();
        return API_CODE::EMERGENCY_STOP;
      }
    }
    else {
      state5_cnt = 0;
    }

    if (fb_transid_result_map_.count(trans_id)) {
      if (feedback_code != NULL) *feedback_code = fb_transid_result_map_[trans_id];
      fb_transid_result_map_.erase(trans_id);
      return 0;
    }
    sleep_milliseconds(50);
  }
  return API_CODE::WAIT_FINISH_TIMEOUT;
}

int XArmAPI::_wait_move(fp32 timeout, int trans_id) {
  if (support_feedback_ && trans_id > 0) {
    return _wait_feedback(timeout, trans_id);
  }
  long long start_time = get_system_time();
  long long expired = timeout <= 0 ? 0 : (get_system_time() + (long long)(timeout * 1000) + (sleep_finish_time_ > start_time ? sleep_finish_time_ : 0));
  int cnt = 0;
  int state5_cnt = 0;
  int state_ = state;
  int ret = get_state(&state_);
  int max_cnt = (ret == 0 && state_ == 1) ? 2 : 10;
  while (timeout <= 0 || get_system_time() < expired) {
    if (!is_connected()) return API_CODE::NOT_CONNECTED;
    if (error_code != 0) return API_CODE::HAS_ERROR;
    // only wait in position mode or traj playback mode
    if (mode != 0 && mode != 11) return 0;
    ret = get_state(&state_);
    if (ret != 0) return ret;

    if (state_ >= 4) {
      sleep_finish_time_ = 0;
      if (state_ == 5) state5_cnt++;
      if (state_ != 5 || state5_cnt >= 20) {
        return API_CODE::EMERGENCY_STOP;
      }
    }
    else {
      state5_cnt = 0;
    }
    if (get_system_time() < sleep_finish_time_ || state_ == 3) {
      cnt = 0;
      max_cnt = state_ == 3 ? 2 : max_cnt;
      sleep_milliseconds(50);
      continue;
    }
    if (state_ == 1) {
      cnt = 0;
      max_cnt = 2;
      sleep_milliseconds(50);
      continue;
    }
    else {
      cnt += 1;
      if (cnt >= max_cnt)
        return 0;
      sleep_milliseconds(50);
    }
  }
  return API_CODE::WAIT_FINISH_TIMEOUT;
}

void XArmAPI::emergency_stop(void) {
  long long start_time = get_system_time();
  while (state != 4 && state != 5 && get_system_time() - start_time < 3000) {
    set_state(4);
    sleep_milliseconds(100);
  }
  sleep_finish_time_ = 0;
  // motion_enable(true, 8);
  // while ((state == 0 || state == 3 || state == 4) && get_system_time() - start_time < 3000) {
  //     set_state(0);
  //     sleep_milliseconds(100);
  // }
}

int XArmAPI::get_inverse_kinematics(fp32 source_pose[6], fp32 target_angles[7]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 pose[6];
  for (int i = 0; i < 6; i++) {
    pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : to_radian(source_pose[i]));
  }
  fp32 angs[7] = { 0 };
  int ret = core->get_ik(pose, angs);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 7; i++) {
      target_angles[i] = (float)(default_is_radian ? angs[i] : to_degree(angs[i]));
    }
  }
  return ret;
}

int XArmAPI::get_forward_kinematics(fp32 source_angles[7], fp32 target_pose[6]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 angs[7];
  for (int i = 0; i < 7; i++) {
    angs[i] = (float)(default_is_radian ? source_angles[i] : to_radian(source_angles[i]));
  }
  fp32 pose[6] = { 0 };
  int ret = core->get_fk(angs, pose);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 6; i++) {
      target_pose[i] = (float)(default_is_radian || i < 3 ? pose[i] : to_degree(pose[i]));
    }
  }
  return ret;
}

int XArmAPI::is_tcp_limit(fp32 source_pose[6], int *limit) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 pose[6];
  for (int i = 0; i < 6; i++) {
    pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : to_radian(source_pose[i]));
  }
  int ret = core->is_tcp_limit(pose, limit);
  return _check_code(ret);
}

int XArmAPI::is_joint_limit(fp32 source_angles[7], int *limit) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 angs[7];
  for (int i = 0; i < 7; i++) {
    angs[i] = (float)(default_is_radian ? source_angles[i] : to_radian(source_angles[i]));
  }
  int ret = core->is_joint_limit(angs, limit);
  return _check_code(ret);
}

int XArmAPI::reload_dynamics(void) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->reload_dynamics();
  return _check_code(ret);
}

int XArmAPI::set_counter_reset(void) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  int ret = core->cnter_reset();
  return _check_code(ret);
}

int XArmAPI::set_counter_increase(void) {
  _wait_until_not_pause();
  _wait_until_cmdnum_lt_max();
  int code = _xarm_is_ready();
  if (code != 0) return code;
  int ret = core->cnter_plus();
  return _check_code(ret);
}

int XArmAPI::get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in, int orient_type_out) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 p1[6], p2[6];
  for (int i = 0; i < 6; i++) {
    p1[i] = (float)(default_is_radian || i < 3 ? pose1[i] : to_radian(pose1[i]));
    p2[i] = (float)(default_is_radian || i < 3 ? pose2[i] : to_radian(pose2[i]));
  }
  int ret = core->get_pose_offset(p1, p2, offset, orient_type_in, orient_type_out);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 6; i++) {
      offset[i] = (float)(default_is_radian || i < 3 ? offset[i] : to_degree(offset[i]));
    }
  }
  return ret;
}

int XArmAPI::get_position_aa(fp32 pose[6]) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_position_aa(pose);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 6; i++) {
      pose[i] = (!default_is_radian && i > 2) ? to_degree(pose[i]) : pose[i];
      position_aa[i] = pose[i];
    }
  }
  return ret;
}

int XArmAPI::set_simulation_robot(bool on) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_simulation_robot((int)on);
  return _check_code(ret);
}

int XArmAPI::calibrate_tcp_coordinate_offset(float four_points[4][6], float ret_xyz[3])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 points[4][6];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      points[i][j] = (float)((j < 3 || default_is_radian) ? four_points[i][j] : to_radian(four_points[i][j]));
    }
  }
  int ret = core->cali_tcp_pose(points, ret_xyz);
  return _check_code(ret);
}

int XArmAPI::calibrate_tcp_orientation_offset(float rpy_be[3], float rpy_bt[3], float ret_rpy[3])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 rpy_be_[3];
  fp32 rpy_bt_[3];
  for (int i = 0; i < 3; i++) {
    rpy_be_[i] = (float)(default_is_radian ? rpy_be[i] : to_radian(rpy_be[i]));
    rpy_bt_[i] = (float)(default_is_radian ? rpy_bt[i] : to_radian(rpy_bt[i]));
  }
  int ret = core->cali_tcp_orient(rpy_be_, rpy_bt_, ret_rpy);
  for (int i = 0; i < 3; i++) {
    ret_rpy[i] = (float)(default_is_radian ? ret_rpy[i] : to_degree(ret_rpy[i]));
  }
  return _check_code(ret);
}

int XArmAPI::calibrate_user_orientation_offset(float three_points[3][6], float ret_rpy[3], int mode, int trust_ind)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 points[3][6];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      points[i][j] = (float)((j < 3 || default_is_radian) ? three_points[i][j] : to_radian(three_points[i][j]));
    }
  }
  int ret = core->cali_user_orient(points, ret_rpy, mode, trust_ind);
  for (int i = 0; i < 3; i++) {
    ret_rpy[i] = (float)(default_is_radian ? ret_rpy[i] : to_degree(ret_rpy[i]));
  }
  return _check_code(ret);
}

int XArmAPI::calibrate_user_coordinate_offset(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 rpy_ub_[3];
  for (int i = 0; i < 3; i++) {
    rpy_ub_[i] = (float)(default_is_radian ? rpy_ub[i] : to_radian(rpy_ub[i]));
  }
  int ret = core->cali_user_pos(rpy_ub_, pos_b_uorg, ret_xyz);
  return _check_code(ret);
}

int XArmAPI::set_cartesian_velo_continuous(bool on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_cartesian_velo_continuous((int)on_off);
  return _check_code(ret);
}

int XArmAPI::set_allow_approx_motion(bool on_off)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_allow_approx_motion((int)on_off);
  return _check_code(ret);
}

int XArmAPI::set_only_check_type(unsigned char only_check_type)
{
  only_check_type_ = (only_check_type >= 0 && only_check_type <= 3) ? only_check_type : 0;
  return 0;
}

int XArmAPI::get_dh_params(fp32 dh_params[28])
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->get_dh_params(dh_params);
  return _check_code(ret);
}

int XArmAPI::set_dh_params(fp32 dh_params[28], unsigned char flag)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int ret = core->set_dh_params(dh_params, flag);
  return _check_code(ret);
}

int XArmAPI::set_feedback_type(unsigned char feedback_type)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  if (!support_feedback_) return API_CODE::CMD_NOT_EXIST;
  int ret = core->set_feedback_type(feedback_type);
  return _check_code(ret);
}

void XArmAPI::_handle_feedback_data(void)
{
  int ret;
  int feedback_maxcount = std::max(max_callback_thread_count_ + 1, 1);
  unsigned char (*feedback_datas)[FEEDBACK_DATA_MAX_LEN] = new unsigned char[feedback_maxcount][FEEDBACK_DATA_MAX_LEN];
  int cnt = 0;
  while (is_connected()) {
    ret = stream_tcp_->read_feedback_frame(feedback_datas[cnt]);
    if (ret == 0) {
      _feedback_callback(feedback_datas[cnt]);
      cnt = (cnt + 1) % feedback_maxcount;
      memset(feedback_datas[cnt], 0, FEEDBACK_DATA_MAX_LEN);
    }
    sleep_ms(5);
  }
  delete[] feedback_datas;
}


