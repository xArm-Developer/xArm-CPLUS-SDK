/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
// #pragma warning(disable:4996)
#endif

#include <random>
#include <algorithm>
#include "xarm/wrapper/xarm_api.h"

const int FEEDBACK_QUE_SIZE = 256;
const int FEEDBACK_DATA_MAX_LEN = 256;

fp32 to_radian(fp32 val) {
  return (fp32)(val / RAD_DEGREE);
}

fp32 to_degree(fp32 val) {
  return (fp32)(val * RAD_DEGREE);
}

thread_local std::default_random_engine engine;

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
  callback_in_thread_ = max_callback_thread_count_ != 0;
  pool_ = std::make_shared<ThreadPool>(max_callback_thread_count_);
  pool_->set_max_thread_count(max_callback_thread_count_);
  max_cmdnum_ = max_cmdnum > 0 ? max_cmdnum : 256;
  axis = init_axis;
  report_type_ = report_type;
  debug_ = debug;
  _init();
  XARM_LOG_INFO("SDK_VERSION: %s\n", SDK_VERSION);
  if (!do_not_open) {
    connect();
  }
}

XArmAPI::~XArmAPI() {
  disconnect();
  _destroy();
}

void XArmAPI::_destroy(void) {}

void XArmAPI::_init(void) {
  core = nullptr;
  core503_ = nullptr;
  stream_tcp_ = nullptr;
  stream_tcp503_ = nullptr;
  stream_tcp_report_ = nullptr;
  stream_tcp_rich_report_ = nullptr;
  stream_ser_ = nullptr;
  is_ready_ = true;
  is_tcp_ = true;
  is_old_protocol_ = false;
  is_first_report_ = true;
  is_sync_ = false;
  arm_type_is_1300_ = false;
  control_box_type_is_1300_ = false;

  is_shutdown_ = false;

  major_version_number_ = 0;
  minor_version_number_ = 0;
  revision_version_number_ = 0;
  version_number[0] = major_version_number_;
  version_number[1] = minor_version_number_;
  version_number[2] = revision_version_number_;
  mt_brake_ = 0;
  mt_able_ = 0;
  min_tcp_speed_ = 0.1f;    // mm/s
  max_tcp_speed_ = 1000;   // mm/s
  min_tcp_acc_ = 1.0f;      // mm/s^2
  max_tcp_acc_ = 50000;    // mm/s^2
  min_joint_speed_ = 0.01f; // rad/s
  max_joint_speed_ = 4.0;  // rad/s
  min_joint_acc_ = 0.01f;   // rad/s^2
  max_joint_acc_ = 20.0;   // rad/s^2
  count = -1;
  iden_progress = 0;
  keep_heart_ = true;

  sleep_finish_time_ = get_system_time();

  std::fill(angles, angles + 7, 0);
  std::fill(last_used_angles, last_used_angles + 7, 0);
  std::fill(tcp_offset, tcp_offset + 6, 0);
  if (default_is_radian) {
    joint_speed_limit[0] = min_joint_speed_;
    joint_speed_limit[1] = max_joint_speed_;
    joint_acc_limit[0] = min_joint_acc_;
    joint_acc_limit[1] = max_joint_acc_;
    last_used_joint_speed = 0.3490658503988659f; // rad/s (20°/s);
    last_used_joint_acc = 8.726646259971648f;    // rad/s^2 (500°/s^2);
    fp32 pos[] = { 201.5f, 0.0f, 140.5f, 3.1415926f, 0.0f, 0.0f };
    std::copy(std::begin(pos), std::end(pos), position);
    std::copy(std::begin(pos), std::end(pos), position_aa);
    std::copy(std::begin(pos), std::end(pos), last_used_position);
  }
  else {
    joint_speed_limit[0] = to_degree(min_joint_speed_);
    joint_speed_limit[1] = to_degree(max_joint_speed_);
    joint_acc_limit[0] = to_degree(min_joint_acc_);
    joint_acc_limit[1] = to_degree(max_joint_acc_);
    last_used_joint_speed = to_degree(0.3490658503988659f); // rad/s (20°/s);
    last_used_joint_acc = to_degree(8.726646259971648f);    // rad/s^2 (500°/s^2);
    fp32 pos[] = { 201.5f, 0.0f, 140.5f, to_degree(3.1415926f), 0.0f, 0.0f };
    std::copy(std::begin(pos), std::end(pos), position);
    std::copy(std::begin(pos), std::end(pos), position_aa);
    std::copy(std::begin(pos), std::end(pos), last_used_position);
  }

  state = 4;
  mode = 0;
  cmd_num = 0;
  std::fill(joints_torque, joints_torque + 7, 0);
  std::fill(motor_brake_states, motor_brake_states + 8, 0);
  std::fill(motor_enable_states, motor_enable_states + 8, 0);
  std::fill(tcp_load, tcp_load + 4, 0);
  error_code = 0;
  warn_code = 0;
  collision_sensitivity = 0;
  teach_sensitivity = 0;
  device_type = 7;
  axis = 7;
  master_id = 0;
  slave_id = 0;
  motor_tid = 0;
  motor_fid = 0;
  tcp_jerk = 1000;        // mm/s^3
  joint_jerk = default_is_radian ? 20.0f : to_degree(20.0f); // 20 rad/s^3
  rot_jerk = 2.3f;
  max_rot_acc = 2.7f;
  tcp_speed_limit[0] = min_tcp_speed_;
  tcp_speed_limit[1] = max_tcp_speed_;
  tcp_acc_limit[0] = min_tcp_acc_;
  tcp_acc_limit[1] = max_tcp_acc_;
  last_used_tcp_speed = 100;  // mm/s
  last_used_tcp_acc = 2000;   // mm/s^2
  std::fill(gravity_direction, gravity_direction + 3, 0);
  gravity_direction[2] = -1;
  realtime_tcp_speed = 0;
  std::fill(realtime_joint_speeds, realtime_joint_speeds + 7, 0);
  std::fill(world_offset, world_offset + 6, 0);
  std::fill(temperatures, temperatures + 7, 0);
  std::fill(ft_ext_force, ft_ext_force + 6, 0);
  std::fill(ft_raw_force, ft_raw_force + 6, 0);
  gpio_reset_config[0] = 0;
  gpio_reset_config[1] = 0;
  tgpio_modbus_baud_ = -1;
  ignore_error_ = false;
  ignore_state_ = false;

  xarm_gripper_is_enabled_ = false;
  bio_gripper_is_enabled_ = false;
  robotiq_is_activated_ = false;
  last_report_time_ = get_system_time();
  max_report_interval_ = 0;
  std::fill(voltages, voltages + 7, 0);
  std::fill(currents, currents + 7, 0);
  std::fill(collision_model_params, collision_model_params + 6, 0);
  is_simulation_robot = 0;
  is_collision_detection = 0;
  collision_tool_type = 0;
  cgpio_state = 0;
  cgpio_code = 0;
  std::fill(cgpio_input_digitals, cgpio_input_digitals + 2, 0);
  std::fill(cgpio_output_digitals, cgpio_output_digitals + 2, 0);
  std::fill(cgpio_input_analogs, cgpio_input_analogs + 2, 0);
  std::fill(cgpio_output_analogs, cgpio_output_analogs + 2, 0);
  std::fill(cgpio_input_conf, cgpio_input_conf + 16, 0);
  std::fill(cgpio_output_conf, cgpio_output_conf + 16, 0);
  cmd_timeout_ = -1;

  xarm_gripper_error_ = 0;
  bio_gripper_error_ = 0;
  robotiq_error_code_ = 0;
  std::fill(xarm_gripper_versions_, xarm_gripper_versions_ + 3, -1);

  control_box_modbus_baud_ = -1;
  linear_motor_speed_ = 0;

  bio_gripper_version_ = 0;
  bio_gripper_mode_ = -1;

  memset(&linear_motor_status, 0, sizeof(linear_motor_status));
  linear_motor_status.sci = 1;

  default_bio_baud_ = 2000000;
  default_gripper_baud_ = 2000000;
  default_robotiq_baud_ = 115200;
  default_linear_motor_baud_ = 2000000;

  only_check_type_ = 0;
  only_check_result = 0;
  support_feedback_ = false;

  reduced_max_tcp_speed = 0.0f;
  reduced_max_joint_spped = 0.0f;
  std::fill(reduced_tcp_boundary, reduced_tcp_boundary + 6, 0);
  std::fill(reduced_joint_limits, reduced_joint_limits + 14, 0);

  is_reduced_mode = false;
  is_fence_mode = false;
  is_report_current = false;
  is_approx_motion = false;
  is_cart_continuous = false;
  is_collision_rebound = false;
  ft_sensor_is_enable = false;

  cgpio_alarm_code = 0;
  monitor_device_type = 0;
  monitor_device_state = 0;
  monitor_device_pos = 0;
  monitor_device_speed = 0;
  monitor_device_current = 0;

  report_rich_data_ptr_ = std::make_shared<XArmReportData>("rich");
  if (report_type_ != "rich") {
    report_data_ptr_ = std::make_shared<XArmReportData>(report_type_);
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
      default_linear_motor_baud_ = baud;
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
      *baud = default_linear_motor_baud_;
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
  return is_tcp_ ? (stream_tcp_ == nullptr ? false : stream_tcp_->is_connected()) : (stream_ser_ == nullptr ? false : stream_ser_->is_connected());
}

bool XArmAPI::_is_connected_503(void) {
  return stream_tcp503_ == nullptr ? false : stream_tcp503_->is_connected();
}

bool XArmAPI::is_lite6(void) {
  return axis == 6 && device_type == 9;
}

bool XArmAPI::is_850(void) {
  return axis == 6 && device_type == 12;
}

bool XArmAPI::is_reported(void) {
  return is_tcp_ ? (stream_tcp_report_ == nullptr ? false : stream_tcp_report_->is_connected()) : false;
}

bool XArmAPI::_is_rich_reported(void) {
  return is_tcp_ ? (stream_tcp_rich_report_ == nullptr ? false : stream_tcp_rich_report_->is_connected()) : false;
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

    sscanf(str_axis.c_str(), "%d", &axis);
    sscanf(str_device_type.c_str(), "%d", &device_type);
    if (str_arm_type.size() >= 6) {
      sscanf(str_arm_type.substr(2, 4).c_str(), "%d", &arm_type);
    }
    if (str_control_type.size() >= 6) {
      sscanf(str_control_type.substr(2, 4).c_str(), "%d", &control_type);
    }

    arm_type_is_1300_ = arm_type >= 1300;
    control_box_type_is_1300_ = control_type >= 1300;

    sscanf(std::string(*++it).c_str(), "%d", &major_version_number_);
    sscanf(std::string(*++it).c_str(), "%d", &minor_version_number_);
    sscanf(std::string(*++it).c_str(), "%d", &revision_version_number_);
  }
  else if (std::regex_match(v, result, pattern)) {
    auto it = result.begin();
    sscanf(std::string(*++it).c_str(), "%d", &major_version_number_);
    sscanf(std::string(*++it).c_str(), "%d", &minor_version_number_);
    sscanf(std::string(*++it).c_str(), "%d", &revision_version_number_);
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
    XARM_LOG_INFO("ROBOT_SN: %s\n", sn);
  }
  XARM_LOG_INFO("ROBOT_IP: %s, VERSION: v%d.%d.%d, PROTOCOL: V%d, DETAIL: %s, TYPE1300: [%d, %d]\n", port_.c_str(), major_version_number_, minor_version_number_, revision_version_number_, is_old_protocol_ ? 0 : 1, version_, control_box_type_is_1300_, arm_type_is_1300_);
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
          XARM_LOG_ERROR("The mode may be incorrect, just as a reminder, mode: %d (%d)\n", mode_, mode);
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
      sscanf(std::string(*++it).c_str(), "%d", &axis);
      sscanf(std::string(*++it).c_str(), "%d", &device_type);
      sscanf(std::string(*++it).substr(2, 4).c_str(), "%d", &arm_type);
      sscanf(std::string(*++it).substr(2, 4).c_str(), "%d", &control_type);

      arm_type_is_1300_ = arm_type >= 1300;
      control_box_type_is_1300_ = control_type >= 1300;

      sscanf(std::string(*++it).c_str(), "%d", &major_version_number_);
      sscanf(std::string(*++it).c_str(), "%d", &minor_version_number_);
      sscanf(std::string(*++it).c_str(), "%d", &revision_version_number_);
    }
    else if (std::regex_match(v, result, pattern)) {
      auto it = result.begin();
      sscanf(std::string(*++it).c_str(), "%d", &major_version_number_);
      sscanf(std::string(*++it).c_str(), "%d", &minor_version_number_);
      sscanf(std::string(*++it).c_str(), "%d", &revision_version_number_);
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

int XArmAPI::_tcp_connect()
{
  is_tcp_ = true;
  auto new_tcp = std::make_shared<SocketPort>(port_.c_str(), XARM_CONF::TCP_PORT_CONTROL, 3, 320, 0, FEEDBACK_QUE_SIZE, FEEDBACK_DATA_MAX_LEN);
  if (!new_tcp->is_connected()) {
    XARM_LOG_ERROR("Error: Tcp control connection failed\n");
    return -2;
  }
  auto new_tcp_rich_report = connect_tcp_report2(port_.c_str(), "rich");
  if (new_tcp_rich_report == nullptr) {
    new_tcp->disconnect();
    return -3;
  }
  if (report_type_ != "rich") {
    auto new_tcp_report = connect_tcp_report2(port_.c_str(), report_type_);
    if (new_tcp_report == nullptr) {
      new_tcp->disconnect();
      new_tcp_rich_report->disconnect();
      return -4;
    }
    stream_tcp_ = std::move(new_tcp);
    stream_tcp_rich_report_ = std::move(new_tcp_rich_report);
    stream_tcp_report_ = std::move(new_tcp_report);
  }
  else {
    stream_tcp_ = std::move(new_tcp);
    stream_tcp_rich_report_ = std::move(new_tcp_rich_report);
    stream_tcp_report_ = stream_tcp_rich_report_;
  }
  core = std::make_shared<UxbusCmdTcp>(stream_tcp_, std::bind(&XArmAPI::_set_feedback_key_transid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  XARM_LOG_INFO("Tcp control connection successful\n");
  core->set_protocol_identifier(2);
  sleep_milliseconds(200);
  _check_version();
  support_feedback_ = _version_is_ge(2, 0, 102);

  return 0;
}

void XArmAPI::_init_threads()
{
  feedback_thread_ = std::thread(feedback_thread_handle, this);
  // feedback_thread_.detach();
  report_rich_thread_ = std::thread(report_rich_thread_handle, this);
  // report_rich_thread_.detach();
  if (report_type_ != "rich") {
    report_thread_ = std::thread(report_thread_handle, this);
    // report_thread_.detach();
  }
}

int XArmAPI::connect(const std::string &port) {
  if (is_connected()) return 0;
  if (port != "" && port != port_) {
    port_ = port;
  }
  if (port_ == "") {
    XARM_LOG_ERROR("can not connect to port/ip: %s\n", port_.c_str());
    return API_CODE::NOT_CONNECTED;
  }
  auto new_pool = std::make_shared<ThreadPool>(max_callback_thread_count_);
  new_pool->set_max_thread_count(max_callback_thread_count_);
  // std::regex pattern("(\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})");
  std::regex pattern("(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)[.]){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)");
  if (port_ == "localhost" || std::regex_match(port_, pattern)) {
    int ret = _tcp_connect();
    if (ret != 0) return ret;
    is_ready_ = true;
    pool_ = std::move(new_pool);
    _report_connect_changed_callback();
    _init_threads();
  }
  else {
    is_tcp_ = false;
    auto new_ser = std::make_shared<SerialPort>(port_.c_str(), XARM_CONF::SERIAL_BAUD, 3, 320);
    if (!new_ser->is_connected()) {
      XARM_LOG_ERROR("Error: Serial control connection failed\n");
      return -2;
    }
    is_ready_ = true;
    pool_ = std::move(new_pool);
    stream_ser_ = std::move(new_ser);
    core = std::make_shared<UxbusCmdSer>(stream_ser_);
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
  stream_tcp503_ = std::make_shared<SocketPort>(port_.c_str(), XARM_CONF::TCP_PORT_CONTROL + 1, 3, 320);
  if (!stream_tcp503_->is_connected()) {
    return -2;
  }
  core503_ = std::make_shared<UxbusCmdTcp>(stream_tcp503_, std::bind(&XArmAPI::_set_feedback_key_transid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  return 0;
}

void XArmAPI::disconnect(void) {
  is_shutdown_ = true;

  auto tcp = stream_tcp_;
  auto tcp503 = stream_tcp503_;
  auto ser = stream_ser_;
  auto report = stream_tcp_report_;
  auto rich = stream_tcp_rich_report_;
  
  if (tcp) tcp->disconnect();
  if (tcp503) tcp503->disconnect();
  if (ser) ser->disconnect();
  if (rich) rich->disconnect();
  if (report && report != rich) report->disconnect();

  if (feedback_thread_.joinable()) feedback_thread_.join();
  if (report_thread_.joinable()) report_thread_.join();
  if (report_rich_thread_.joinable()) report_rich_thread_.join();

  if (stream_tcp_) stream_tcp_.reset();
  if (stream_tcp503_) stream_tcp503_.reset();
  if (stream_ser_) stream_ser_.reset();
  if (stream_tcp_report_) stream_tcp_report_.reset();
  if (stream_tcp_rich_report_) stream_tcp_rich_report_.reset();

  _report_connect_changed_callback();
  is_ready_ = false;

  if (pool_) pool_->stop();
}

void XArmAPI::_request_shutdown_from_report_thread() {
  is_shutdown_ = true;
  auto tcp = stream_tcp_;
  if (tcp) tcp->disconnect();
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
      sscanf(std::string(str).substr(2, 4).c_str(), "%d", &arm_type);
    if (strlen(control_box_sn) >= 6)
      sscanf(std::string(control_box_sn).substr(2, 4).c_str(), "%d", &control_type);
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
    ret = core->get_joint_states(angs, nullptr, nullptr, 1);
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
  int count = num & 0x0F;
  if (count < 1 || count > 3) return API_CODE::PARAM_ERROR;
  num = _version_is_ge(2, 6, 107) ? num : count;
  int ret = core->get_joint_states(jposition, velocity, effort, num);
  ret = _check_code(ret);
  if (ret == 0) {
    for (int i = 0; i < 7; i++) {
      if (!default_is_radian) {
        jposition[i] = to_degree(jposition[i]);
        if (count >= 2)
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
      XARM_LOG_INFO("[motion_enable], xArm is not ready to move\n");
    }
    is_ready_ = false;
  }
  else {
    if (debug_ && !is_ready_) {
      XARM_LOG_INFO("[motion_enable], xArm is ready to move\n");
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
      XARM_LOG_INFO("[set_state], xArm is not ready to move\n");
    }
    is_ready_ = false;
  }
  else {
    if (debug_ && !is_ready_) {
      XARM_LOG_INFO("[set_state], xArm is ready to move\n");
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
      XARM_LOG_INFO("[clean_error], xArm is not ready to move\n");
    }
    is_ready_ = false;
  }
  else {
    if (debug_ && !is_ready_) {
      XARM_LOG_INFO("[clean_error], xArm is ready to move\n");
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
  std::lock_guard<std::mutex> locker(fb_mutex_);
  std::string feedback_key = (wait && support_feedback_) ? (std::to_string(get_us()) + std::to_string(engine())) : "";
  fb_key_transid_map_[feedback_key != "" ? feedback_key : "no_use"] = -1;
  return feedback_key;
}

int XArmAPI::_get_feedback_transid(std::string feedback_key)
{
  std::lock_guard<std::mutex> locker(fb_mutex_);
  int trans_id = -1;
  if (feedback_key != "" && fb_key_transid_map_.count(feedback_key)) {
    trans_id = fb_key_transid_map_[feedback_key];
    fb_key_transid_map_.erase(feedback_key);
  }
  return trans_id;
}

void XArmAPI::_set_feedback_key_transid(std::string feedback_key, int trans_id, unsigned char feedback_type) {
  std::lock_guard<std::mutex> locker(fb_mutex_);
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
      std::unique_lock<std::mutex> locker(fb_mutex_);
      fb_transid_result_map_.clear();
      locker.unlock();
      return API_CODE::NOT_CONNECTED;
    }
    if (error_code != 0) {
      std::unique_lock<std::mutex> locker(fb_mutex_);
      fb_transid_result_map_.clear();
      locker.unlock();
      return API_CODE::HAS_ERROR;
    }

    ret = get_state(&state_);
    if (ret != 0) return ret;

    if (state_ >= 4) {
      sleep_finish_time_ = 0;
      if (state_ == 5) state5_cnt++;
      if (state_ != 5 || state5_cnt >= 20) {
        std::unique_lock<std::mutex> locker(fb_mutex_);
        fb_transid_result_map_.clear();
        locker.unlock();
        return API_CODE::EMERGENCY_STOP;
      }
    }
    else {
      state5_cnt = 0;
    }
    std::unique_lock<std::mutex> locker(fb_mutex_);
    if (fb_transid_result_map_.count(trans_id)) {
      if (feedback_code != nullptr) *feedback_code = fb_transid_result_map_[trans_id];
      fb_transid_result_map_.erase(trans_id);
      locker.unlock();
      return 0;
    }
    locker.unlock();
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

int XArmAPI::get_inverse_kinematics(fp32 source_pose[6], fp32 target_angles[7], bool limited, fp32 *ref_angles) {
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  fp32 pose[6];
  for (int i = 0; i < 6; i++) {
    pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : to_radian(source_pose[i]));
  }
  fp32 angs[7] = { 0 };
  int ret;
  if (_version_is_ge(2, 7, 103)) {
    if (ref_angles != nullptr) {
      fp32 ref_joints[7] = {0};
      for (int i = 0; i < 7; i++) {
        ref_joints[i] = (float)(default_is_radian ? ref_angles[i] : to_degree(ref_angles[i]));
      }
      ret = core->get_ik(pose, angs, limited, ref_joints);
    }
    else {
      ret = core->get_ik(pose, angs, limited);
    }
  }
  else {
    ret = core->get_ik(pose, angs);
  }  
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
  while (!is_shutdown_ && is_connected()) {
    ret = stream_tcp_->read_feedback_frame(feedback_datas[cnt]);
    if (ret == 0) {
      _feedback_callback(feedback_datas[cnt]);
      cnt = (cnt + 1) % feedback_maxcount;
      memset(feedback_datas[cnt], 0, FEEDBACK_DATA_MAX_LEN);
    }
    sleep_ms(5);
  }
  delete[] feedback_datas;
  XARM_LOG_INFO("xarm feedback thread is quit.\n");
}

int XArmAPI::iden_tcp_load(float result[4], float estimated_mass)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  int protocol_identifier = core->get_protocol_identifier();
  core->set_protocol_identifier(2);
  keep_heart_ = false;
  float mass = estimated_mass;
  if (_version_is_ge(1, 9, 100) && estimated_mass <= 0) {
    mass = 0.5;
  }
  int ret = core->iden_tcp_load(result, mass);
  core->set_protocol_identifier(protocol_identifier);
  keep_heart_ = true;
  return _check_code(ret);
}

int XArmAPI::iden_joint_friction(int *result, unsigned char *sn)
{
  if (!is_connected()) return API_CODE::NOT_CONNECTED;
  
  unsigned char r_sn[14];
  if (sn == nullptr) {
    unsigned char tmp_sn[40] = {0};
    int code = get_robot_sn(tmp_sn);
    if (code != 0) {
      XARM_LOG_ERROR("iden_joint_friction -> get_robot_sn failed, code=%d\n", code);
      return API_CODE::API_EXCEPTION;
    }
    memcpy(r_sn, tmp_sn, 14);
  }
  else {
    memcpy(r_sn, sn, 14);
  }

  bool valid_850 = is_850() && r_sn[0] == 'F' && r_sn[1] == 'X';
  bool valid_lite = is_lite6() && r_sn[0] == 'L' && r_sn[1] == 'I';
  bool valid_xarm = !is_850() && !is_lite6() && r_sn[0] == 'X' && r_sn[1] == (axis == 5 ? 'F' : axis == 6 ? 'I' : axis == 7 ? 'S' : ' ') ;

  if (!(valid_850 || valid_lite || valid_xarm)) {
    XARM_LOG_ERROR("iden_joint_friction -> get_robot_sn failed, sn=%s\n", r_sn);
    return API_CODE::API_EXCEPTION;
  }
  
  int protocol_identifier = core->get_protocol_identifier();
  core->set_protocol_identifier(2);
  keep_heart_ = false;
  float tmp = 0;
  int ret = core->iden_joint_friction(r_sn, &tmp);
  *result = ((int)tmp) == 0 ? 0 : -1;
  core->set_protocol_identifier(protocol_identifier);
  keep_heart_ = true;
  return _check_code(ret);
}
