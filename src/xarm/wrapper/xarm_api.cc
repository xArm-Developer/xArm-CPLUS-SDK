/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#define _CRT_SECURE_NO_WARNINGS
#include <regex>
#include <iostream>
#include <string>
// #include <unistd.h>
#include <string.h>
#include "xarm/wrapper/xarm_api.h"

using namespace std;

static bool compare_version(int v1[3], int v2[3]) {
	for (int i = 0; i < 3; i++) {
		if (v1[i] > v2[i]) {
			return true;
		}
		else if (v1[i] < v2[i]) {
			return false;
		}
	}
	return false;
}

XArmAPI::XArmAPI(
	const std::string &port,
	bool is_radian,
	bool do_not_open,
	bool check_tcp_limit,
	bool check_joint_limit,
	bool check_cmdnum_limit,
	bool check_robot_sn,
	bool check_is_ready,
	bool check_is_pause)
	:port_(port), check_joint_limit_(check_joint_limit),
	check_cmdnum_limit_(check_cmdnum_limit), check_robot_sn_(check_robot_sn),
	check_is_ready_(check_is_ready), check_is_pause_(check_is_pause) {
	default_is_radian = is_radian;
	check_tcp_limit_ = check_tcp_limit;
	_init();
	printf("SDK_VERSION: %s\n", SDK_VERSION);
	if (!do_not_open) {
		connect();
	}
}

XArmAPI::~XArmAPI() {
	disconnect();
}

void XArmAPI::_init(void) {
	cmd_tcp_ = NULL;
	cmd_ser_ = NULL;
	stream_tcp_ = NULL;
	stream_tcp_report_ = NULL;
	stream_ser_ = NULL;
	is_ready_ = true;
	is_stop_ = false;
	is_tcp_ = true;
	is_old_protocol_ = false;
	is_first_report_ = true;
	is_sync_ = false;

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
	count_ = -1;

	sleep_finish_time_ = get_system_time();

	angles = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	last_used_angles = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	tcp_offset = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
	if (default_is_radian) {
		joint_speed_limit = new fp32[2]{ min_joint_speed_, max_joint_speed_ };
		joint_acc_limit = new fp32[2]{ min_joint_acc_, max_joint_acc_ };
		last_used_joint_speed = (float)0.3490658503988659; // rad/s (20°/s);
		last_used_joint_acc = (float)8.726646259971648;    // rad/s^2 (500°/s^2);
		position = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
		last_used_position = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
	}
	else {
		joint_speed_limit = new fp32[2]{ (fp32)(min_joint_speed_ * RAD_DEGREE), (fp32)(max_joint_speed_ * RAD_DEGREE) };
		joint_acc_limit = new fp32[2]{ (fp32)(min_joint_acc_ * RAD_DEGREE), (fp32)(max_joint_acc_ * RAD_DEGREE) };
		last_used_joint_speed = (fp32)(0.3490658503988659 * RAD_DEGREE); // rad/s (20°/s);
		last_used_joint_acc = (fp32)(8.726646259971648 * RAD_DEGREE);    // rad/s^2 (500°/s^2);
		position = new fp32[6]{ 201.5, 0, 140.5, (fp32)(3.1415926 * RAD_DEGREE), 0, 0 };
		last_used_position = new fp32[6]{ 201.5, 0, 140.5, (fp32)(3.1415926 * RAD_DEGREE), 0, 0 };
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
	joint_jerk = default_is_radian ? 20 : (fp32)(20 * RAD_DEGREE); // 20 rad/s^3
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
	if (is_tcp_) {
		return stream_tcp_ == NULL ? false : stream_tcp_->is_ok() == 0;
	}
	else {
		return stream_ser_ == NULL ? false : stream_ser_->is_ok() == 0;
	}
}

bool XArmAPI::is_reported(void) {
	if (is_tcp_) {
		return stream_tcp_report_ == NULL ? false : stream_tcp_report_->is_ok() == 0;
	}
	else {
		return false;
	}
}

inline void XArmAPI::_report_location_callback(void) {
	for (u32 i = 0; i < report_location_callbacks_.size(); i++) {
		timer.AsyncWait(0, report_location_callbacks_[i], position, angles);
	}
}

inline void XArmAPI::_report_connect_changed_callback(void) {
	bool connected = stream_tcp_ == NULL ? false : stream_tcp_->is_ok() == 0;
	bool reported = stream_tcp_report_ == NULL ? false : stream_tcp_report_->is_ok() == 0;
	for (u32 i = 0; i < connect_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, connect_changed_callbacks_[i], connected, reported);
	}
}

inline void XArmAPI::_report_state_changed_callback(void) {
	for (u32 i = 0; i < state_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, state_changed_callbacks_[i], state);
	}
}

inline void XArmAPI::_report_mode_changed_callback(void) {
	for (u32 i = 0; i < mode_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, mode_changed_callbacks_[i], mode);
	}
}

inline void XArmAPI::_report_mtable_mtbrake_changed_callback(void) {
	for (u32 i = 0; i < mtable_mtbrake_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, mtable_mtbrake_changed_callbacks_[i], mt_able_, mt_brake_);
	}
}

inline void XArmAPI::_report_error_warn_changed_callback(void) {
	for (u32 i = 0; i < error_warn_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, error_warn_changed_callbacks_[i], error_code, warn_code);
	}
}

inline void XArmAPI::_report_cmdnum_changed_callback(void) {
	for (u32 i = 0; i < cmdnum_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, cmdnum_changed_callbacks_[i], cmd_num);
	}
}

inline void XArmAPI::_report_temperature_changed_callback(void) {
	for (u32 i = 0; i < temperature_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, temperature_changed_callbacks_[i], temperatures);
	}
}

inline void XArmAPI::_report_count_changed_callback(void) {
	for (u32 i = 0; i < count_changed_callbacks_.size(); i++) {
		timer.AsyncWait(0, count_changed_callbacks_[i], count_);
	}
}

void XArmAPI::_update_old(unsigned char *rx_data) {
	unsigned char *data_fp = &rx_data[4];
	int sizeof_data = bin8_to_32(rx_data);
	if (sizeof_data >= 87) {
		int state_ = state;
		state = data_fp[4];
		if (state != 3) {
			std::unique_lock<std::mutex> locker(mutex_);
			cond_.notify_all();
			locker.unlock();
		}
		if (state != state_) _report_state_changed_callback();
		if (state == 4) {
			// if (is_ready_ && sizeof_data < 187) {
			//     printf("[report], xArm is not ready to move");
			// }
			if (sizeof_data < 187) { is_ready_ = false; }
		}
		else {
			// if (!is_ready_ && sizeof_data < 187) {
			//     printf("[report], xArm is ready to move");
			// }
			if (sizeof_data < 187) { is_ready_ = true; }
		}

		int brake = mt_brake_;
		int able = mt_able_;
		mt_brake_ = data_fp[5];
		mt_able_ = data_fp[6];
		if (brake != mt_brake_ || able != mt_able_) _report_mtable_mtbrake_changed_callback();

		if (!is_first_report_) {
			bool ready = true;
			for (int i = 0; i < 8; i++) {
				motor_brake_states[i] = mt_brake_ >> i & 0x01;
				if (i < axis && !motor_brake_states[i]) {
					ready = false;
				}
			}
			for (int i = 0; i < 8; i++) {
				motor_enable_states[i] = mt_able_ >> i & 0x01;
				if (i < axis && !motor_enable_states[i]) {
					ready = false;
				}
			}
			if (state == 4 || !ready) {
				// if (is_ready_) {
				//     printf("[report], xArm is not ready to move\n");
				// }
				is_ready_ = false;
			}
			else {
				// if (!is_ready_) {
				//     printf("[report], xArm is ready to move\n");
				// }
				is_ready_ = true;
			}
		}
		else {
			is_ready_ = false;
		}
		is_first_report_ = false;

		int err = error_code;
		int warn = warn_code;
		error_code = data_fp[7];
		warn_code = data_fp[8];
		if (error_code != err || warn_code != warn) _report_error_warn_changed_callback();


		hex_to_nfp32(&data_fp[9], angles, 7);
		for (u32 i = 0; i < 7; i++) {
			angles[i] = (float)(default_is_radian ? angles[i] : angles[i] * RAD_DEGREE);
		}
		hex_to_nfp32(&data_fp[37], position, 6);
		for (u32 i = 0; i < 6; i++) {
			position[i] = (float)(default_is_radian || i < 3 ? position[i] : position[i] * RAD_DEGREE);
		}
		_report_location_callback();

		int cmdnum_ = cmd_num;
		cmd_num = bin8_to_16(&data_fp[61]);
		if (cmd_num != cmdnum_) _report_cmdnum_changed_callback();

		hex_to_nfp32(&data_fp[63], tcp_offset, 6);
		for (u32 i = 0; i < 6; i++) {
			tcp_offset[i] = (float)(default_is_radian || i < 3 ? tcp_offset[i] : tcp_offset[i] * RAD_DEGREE);
		}
	}
	if (sizeof_data >= 187) {
		device_type = data_fp[87];
		int _axis = data_fp[88];
		master_id = data_fp[89];
		slave_id = data_fp[90];
		motor_tid = data_fp[91];
		motor_fid = data_fp[92];

		if (_axis >= 5 && _axis <= 7) {
			axis = _axis;
		}
		if (device_type == 5) {
			axis = 5;
		}
		else if (device_type == 6) {
			axis = 6;
		}
		else if (device_type == 3) {
			axis = 7;
		}

		memcpy(version, &data_fp[93], 30);

		hex_to_nfp32(&data_fp[123], trs_msg_, 5);
		tcp_jerk = trs_msg_[0];
		min_tcp_acc_ = trs_msg_[1];
		max_tcp_acc_ = trs_msg_[2];
		min_tcp_speed_ = trs_msg_[3];
		max_tcp_speed_ = trs_msg_[4];
		tcp_speed_limit[0] = min_tcp_speed_;
		tcp_speed_limit[1] = max_tcp_speed_;
		tcp_acc_limit[0] = min_tcp_acc_;
		tcp_acc_limit[1] = max_tcp_acc_;

		hex_to_nfp32(&data_fp[143], p2p_msg_, 5);
		joint_jerk = default_is_radian ? p2p_msg_[0] : (fp32)(p2p_msg_[0] * RAD_DEGREE);
		min_joint_acc_ = p2p_msg_[1];
		max_joint_acc_ = p2p_msg_[2];
		min_joint_speed_ = p2p_msg_[3];
		max_joint_speed_ = p2p_msg_[4];
		if (default_is_radian) {
			joint_speed_limit[0] = min_joint_acc_;
			joint_speed_limit[1] = max_joint_acc_;
			joint_acc_limit[0] = min_joint_speed_;
			joint_acc_limit[1] = max_joint_speed_;
		}
		else {
			joint_speed_limit[0] = (float)(min_joint_acc_ * RAD_DEGREE);
			joint_speed_limit[1] = (float)(max_joint_acc_ * RAD_DEGREE);
			joint_acc_limit[0] = (float)(min_joint_speed_ * RAD_DEGREE);
			joint_acc_limit[1] = (float)(max_joint_speed_ * RAD_DEGREE);
		}

		hex_to_nfp32(&data_fp[163], rot_msg_, 2);
		rot_jerk = rot_msg_[0];
		max_rot_acc = rot_msg_[1];

		for (u32 i = 0; i < 17; i++) sv3msg_[i] = data_fp[171 + i];
	}
}

void XArmAPI::_update(unsigned char *rx_data) {
	if (is_old_protocol_) {
		_update_old(rx_data);
		return;
	}
	unsigned char *data_fp = &rx_data[4];
	int sizeof_data = bin8_to_32(rx_data);
	if (sizeof_data >= 87) {
		int state_ = state;
		state = data_fp[4] & 0x0F;
		if (state != 3) {
			std::unique_lock<std::mutex> locker(mutex_);
			cond_.notify_all();
			locker.unlock();
		}
		if (state != state_) _report_state_changed_callback();
		if (state == 4) {
			// if (is_ready_ && sizeof_data < 133) {
			//     printf("[report], xArm is not ready to move");
			// }
			if (sizeof_data < 133) { is_ready_ = false; }
		}
		else {
			// if (!is_ready_ && sizeof_data < 133) {
			//     printf("[report], xArm is ready to move");
			// }
			if (sizeof_data < 133) { is_ready_ = true; }
		}
		int mode_ = mode;
		mode = data_fp[4] >> 4;
		if (mode != mode_) _report_mode_changed_callback();
		int cmdnum_ = cmd_num;
		cmd_num = bin8_to_16(&data_fp[5]);
		if (cmd_num != cmdnum_) _report_cmdnum_changed_callback();

		hex_to_nfp32(&data_fp[7], angles, 7);
		for (u32 i = 0; i < 7; i++) {
			angles[i] = (float)(default_is_radian ? angles[i] : angles[i] * RAD_DEGREE);
		}
		hex_to_nfp32(&data_fp[35], position, 6);
		for (u32 i = 0; i < 6; i++) {
			position[i] = (float)(default_is_radian || i < 3 ? position[i] : position[i] * RAD_DEGREE);
		}
		_report_location_callback();
		hex_to_nfp32(&data_fp[59], joints_torque, 7);
	}
	if (sizeof_data >= 133) {
		int brake = mt_brake_;
		int able = mt_able_;
		mt_brake_ = data_fp[87];
		mt_able_ = data_fp[88];
		if (brake != mt_brake_ || able != mt_able_) _report_mtable_mtbrake_changed_callback();
		if (!is_first_report_) {
			bool ready = true;
			for (int i = 0; i < 8; i++) {
				motor_brake_states[i] = mt_brake_ >> i & 0x01;
				if (i < axis && !motor_brake_states[i]) {
					ready = false;
				}
			}
			for (int i = 0; i < 8; i++) {
				motor_enable_states[i] = mt_able_ >> i & 0x01;
				if (i < axis && !motor_enable_states[i]) {
					ready = false;
				}
			}
			if (state == 4 || !ready) {
				// if (is_ready_) {
				//     printf("[report], xArm is not ready to move\n");
				// }
				is_ready_ = false;
			}
			else {
				// if (!is_ready_) {
				//     printf("[report], xArm is ready to move\n");
				// }
				is_ready_ = true;
			}
		}
		else {
			is_ready_ = false;
		}
		is_first_report_ = false;

		int err = error_code;
		int warn = warn_code;
		error_code = data_fp[89];
		warn_code = data_fp[90];
		if (error_code != err || warn_code != warn) _report_error_warn_changed_callback();

		hex_to_nfp32(&data_fp[91], tcp_offset, 6);
		for (u32 i = 0; i < 6; i++) {
			tcp_offset[i] = (float)(default_is_radian || i < 3 ? tcp_offset[i] : tcp_offset[i] * RAD_DEGREE);
		}
		hex_to_nfp32(&data_fp[115], tcp_load, 4);

		if (!compare_version(version_number, new int[3]{ 0, 2, 0 })) {
			tcp_load[1] = tcp_load[1] * 1000;
			tcp_load[2] = tcp_load[2] * 1000;
			tcp_load[3] = tcp_load[3] * 1000;
		}

		collision_sensitivity = data_fp[131];
		teach_sensitivity = data_fp[132];
		hex_to_nfp32(&data_fp[133], gravity_direction, 3);
	}
	if (sizeof_data >= 245) {
		device_type = data_fp[145];
		int _axis = data_fp[146];
		master_id = data_fp[147];
		slave_id = data_fp[148];
		motor_tid = data_fp[149];
		motor_fid = data_fp[150];

		if (_axis >= 5 && _axis <= 7) {
			axis = _axis;
		}

		// if ((device_type == 5 || device_type == 6) && axis == 7) {
		//     axis = device_type;
		// }

		memcpy(version, &data_fp[151], 30);

		hex_to_nfp32(&data_fp[181], trs_msg_, 5);
		tcp_jerk = trs_msg_[0];
		min_tcp_acc_ = trs_msg_[1];
		max_tcp_acc_ = trs_msg_[2];
		min_tcp_speed_ = trs_msg_[3];
		max_tcp_speed_ = trs_msg_[4];
		tcp_speed_limit[0] = min_tcp_speed_;
		tcp_speed_limit[1] = max_tcp_speed_;
		tcp_acc_limit[0] = min_tcp_acc_;
		tcp_acc_limit[1] = max_tcp_acc_;

		hex_to_nfp32(&data_fp[201], p2p_msg_, 5);
		joint_jerk = default_is_radian ? p2p_msg_[0] : (fp32)(p2p_msg_[0] * RAD_DEGREE);
		min_joint_acc_ = p2p_msg_[1];
		max_joint_acc_ = p2p_msg_[2];
		min_joint_speed_ = p2p_msg_[3];
		max_joint_speed_ = p2p_msg_[4];
		if (default_is_radian) {
			joint_speed_limit[0] = min_joint_acc_;
			joint_speed_limit[1] = max_joint_acc_;
			joint_acc_limit[0] = min_joint_speed_;
			joint_acc_limit[1] = max_joint_speed_;
		}
		else {
			joint_speed_limit[0] = (float)(min_joint_acc_ * RAD_DEGREE);
			joint_speed_limit[1] = (float)(max_joint_acc_ * RAD_DEGREE);
			joint_acc_limit[0] = (float)(min_joint_speed_ * RAD_DEGREE);
			joint_acc_limit[1] = (float)(max_joint_speed_ * RAD_DEGREE);
		}

		hex_to_nfp32(&data_fp[221], rot_msg_, 2);
		rot_jerk = rot_msg_[0];
		max_rot_acc = rot_msg_[1];

		for (u32 i = 0; i < 17; i++) sv3msg_[i] = data_fp[229 + i];

		if (sizeof_data >= 252) {
			bool isChange = false;
			for (u32 i = 0; i < 7; i++) {
				if (temperatures[i] != data_fp[245 + i]) {
					isChange = true;
				}
			}
			if (isChange) {
				_report_temperature_changed_callback();
			}
		}
		if (sizeof_data >= 284) {
			fp32 speeds[8];
			hex_to_nfp32(&data_fp[252], speeds, 8);
			realtime_tcp_speed = speeds[0];
			realtime_joint_speeds = &speeds[1];
		}
		if (sizeof_data >= 288) {
			int cnt = bin8_to_32(&data_fp[284]);
			if (count_ != -1 && count_ != cnt) {
				count_ = cnt;
				_report_count_changed_callback();
			}
			count_ = cnt;
		}
		if (sizeof_data >= 312) {
			hex_to_nfp32(&data_fp[288], world_offset, 6);
			for (u32 i = 0; i < 6; i++) {
				world_offset[i] = (float)(default_is_radian || i < 3 ? world_offset[i] : world_offset[i] * RAD_DEGREE);
			}
		}
	}
}

void XArmAPI::_recv_report_data(void) {
	unsigned char rx_data[1280];
	int ret;
	int fail_count = 0;
	while (is_connected()) {
		sleep_milliseconds(1);
		if (fail_count > 5) break;
		if (stream_tcp_report_->is_ok() != 0) {
			fail_count += 1;
			stream_tcp_report_->close_port();
			stream_tcp_report_ = new SocketPort((char *)port_.data(), XARM_CONF::TCP_PORT_REPORT_RICH, 3, 512);
			sleep_milliseconds(10);
			continue;
		}
		ret = stream_tcp_report_->read_frame(rx_data);
		fail_count = 0;
		if (ret != 0) continue;
		_update(rx_data);
	}
	try {
		report_thread_.join();
	} catch (...) {
	}
}

static void report_thread_handle_(void *arg) {
	XArmAPI *my_this = (XArmAPI *)arg;
	my_this->_recv_report_data();
	// pthread_exit(0);
}

void XArmAPI::_check_version(void) {
	int count = 5;
	unsigned char version_[40];
	int ret = -1;
	while ((ret < 0 || ret > 2) && count > 0) {
		ret = get_version(version_);
		sleep_milliseconds(100);
		count -= 1;
	}
	std::string v((const char *)version_);
	std::regex pattern(".*[vV](\\d+)[.](\\d+)[.](\\d+).*");
	std::smatch result;
	if (std::regex_match(v, result, pattern)) {
		auto it = result.begin();
		sscanf(std::string(*++it).data(), "%d", &major_version_number_);
		sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
		sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
	}
	else {
		std::vector<std::string> tmpList = split(v, "-");
		int size = tmpList.size();
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
	printf("is_old_protocol: %d\n", is_old_protocol_);
	printf("version_number: %d.%d.%d\n", major_version_number_, minor_version_number_, revision_version_number_);
	if (check_robot_sn_) {
		count = 5;
		int err_warn[2];
		ret = -1;
		while ((ret < 0 || ret > 2) && count > 0 && warn_code == 0) {
			ret = get_robot_sn(version_);
			get_err_warn_code(err_warn);
			sleep_milliseconds(100);
			count -= 1;
		}
		printf("robot_sn: %s\n", sn);
	}
}

void XArmAPI::_check_is_pause(void) {
	if (check_is_pause_ && state == 3) {
		std::unique_lock<std::mutex> locker(mutex_);
		cond_.wait(locker, [this] { return state != 3 || !is_connected(); });
		locker.unlock();
	}
}

bool XArmAPI::version_is_ge(int major, int minor, int revision) {
	if (major_version_number_ == 0 && minor_version_number_ == 0 && revision_version_number_ == 0) {
		unsigned char version_[40];
		get_version(version_);
		std::string v((const char *)version_);
		std::regex pattern(".*[vV](\\d+)[.](\\d+)[.](\\d+)");
		std::smatch result;
		if (std::regex_match(v, result, pattern)) {
			auto it = result.begin();
			sscanf(std::string(*++it).data(), "%d", &major_version_number_);
			sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
			sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
		}
		else {
			std::vector<std::string> tmpList = split(v, "-");
			int size = tmpList.size();
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
		printf("can not connect to port/ip: %s\n", port_.data());
		return -1;
	}
	// std::regex pattern("(\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})");
	std::regex pattern("(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)[.]){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)");
	is_ready_ = true;
	if (port_ == "localhost" || std::regex_match(port_, pattern)) {
		is_tcp_ = true;
		stream_tcp_ = new SocketPort((char *)port_.data(), XARM_CONF::TCP_PORT_CONTROL, 3, 128);
		if (stream_tcp_->is_ok() != 0) {
			printf("Error: Tcp control connection failed\n");
			return -2;
		}
		cmd_tcp_ = new UxbusCmdTcp((SocketPort *)stream_tcp_);
		printf("Tcp control connection successful\n");

		sleep_milliseconds(200);
		_check_version();

		stream_tcp_report_ = new SocketPort((char *)port_.data(), XARM_CONF::TCP_PORT_REPORT_RICH, 3, 512);
		if (stream_tcp_report_->is_ok() != 0) {
			_report_connect_changed_callback();
			printf("Error: Tcp report connection failed\n");
			return -3;
		}
		_report_connect_changed_callback();
		printf("Tcp report connection successful\n");
		// report_thread_ = thread_init(report_thread_handle_, this);
		report_thread_ = std::thread(report_thread_handle_, this);
		report_thread_.detach();

		// stream_tcp_report_ = new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_NORM, 3, 512);
		// stream_tcp_report_ = new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_RICH, 3, 512);
		// stream_tcp_report_ = new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_DEVL, 3, 512);
	}
	else {
		is_tcp_ = false;
		stream_ser_ = new SerialPort((const char *)port_.data(), XARM_CONF::SERIAL_BAUD, 3, 128);
		cmd_ser_ = new UxbusCmdSer((SerialPort *)stream_ser_);
		sleep_milliseconds(200);
		_check_version();
	}

	return 0;
}

void XArmAPI::disconnect(void) {
	if (stream_tcp_ != NULL) {
		stream_tcp_->close_port();
	}
	if (stream_ser_ != NULL) {
		stream_ser_->close_port();
	}
	if (stream_tcp_report_ != NULL) {
		stream_tcp_report_->close_port();
	}
	is_ready_ = false;
}

int XArmAPI::get_version(unsigned char version_[40]) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_version(version_);
	}
	else {
		ret = cmd_ser_->get_version(version_);
	}
	return ret;
}

int XArmAPI::get_robot_sn(unsigned char robot_sn[40]) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_robot_sn(robot_sn);
	}
	else {
		ret = cmd_ser_->get_robot_sn(robot_sn);
	}
	if (ret == 0 || ret == 1 || ret == 2) {
		memcpy(sn, robot_sn, 40);
	}
	return ret;
}

int XArmAPI::shutdown_system(int value) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->shutdown_system(value);
	}
	else {
		ret = cmd_ser_->shutdown_system(value);
	}
	return ret;
}

int XArmAPI::get_state(int *state_) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_state(state_);
	}
	else {
		ret = cmd_ser_->get_state(state_);
	}
	if (ret == 0 || ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
		state = *state_;
	}
	return ret;
}

int XArmAPI::get_cmdnum(int *cmdnum_) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_cmdnum(cmdnum_);
	}
	else {
		ret = cmd_ser_->get_cmdnum(cmdnum_);
	}
	if (ret == 0 || ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
		cmd_num = *cmdnum_;
	}
	return ret;
}

int XArmAPI::get_err_warn_code(int err_warn[2]) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_err_code(err_warn);
	}
	else {
		ret = cmd_ser_->get_err_code(err_warn);
	}
	if (ret == 0 || ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
		error_code = err_warn[0];
		warn_code = err_warn[1];
	}
	return ret;
}

int XArmAPI::get_position(fp32 pose[6]) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_tcp_pose(pose);
	}
	else {
		ret = cmd_ser_->get_tcp_pose(pose);
	}
	if (ret >= 0) {
		for (u32 i = 0; i < 6; i++) {
			if (!default_is_radian && i > 2) {
				pose[i] = (float)(pose[i] * RAD_DEGREE);
			}
			position[i] = pose[i];
		}
	}
	return ret;
}

int XArmAPI::get_servo_angle(fp32 angs[7]) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_joint_pose(angs);
	}
	else {
		ret = cmd_ser_->get_joint_pose(angs);
	}
	if (ret >= 0) {
		for (u32 i = 0; i < 7; i++) {
			if (!default_is_radian) {
				angs[i] = (float)(angs[i] * RAD_DEGREE);
			}
			angles[i] = angs[i];
		}
	}
	return ret;
}

int XArmAPI::motion_enable(bool enable, int servo_id) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->motion_en(servo_id, int(enable));
	}
	else {
		ret = cmd_ser_->motion_en(servo_id, int(enable));
	}
	get_state(&state);
	if (state == 4) {
		if (is_ready_) {
			printf("[motion_enable], xArm is not ready to move\n");
		}
		is_ready_ = false;
	}
	else {
		if (!is_ready_) {
			printf("[motion_enable], xArm is ready to move\n");
		}
		is_ready_ = true;
	}
	return ret;
}

int XArmAPI::set_state(int state_) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_state(state_);
	}
	else {
		ret = cmd_ser_->set_state(state_);
	}
	get_state(&state);
	if (state == 4) {
		// is_sync_ = false;
		if (is_ready_) {
			printf("[set_state], xArm is not ready to move\n");
		}
		is_ready_ = false;
	}
	else {
		if (!is_ready_) {
			printf("[set_state], xArm is ready to move\n");
		}
		is_ready_ = true;
	}
	return ret;
}

int XArmAPI::set_mode(int mode_) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_mode(mode_);
	}
	else {
		ret = cmd_ser_->set_mode(mode_);
	}
	return ret;
}

int XArmAPI::set_servo_attach(int servo_id) {
	// int ret = 0;
	// if (is_tcp_) {
	//     ret = cmd_tcp_->set_brake(servo_id, 0);
	// } else {
	//     ret = cmd_ser_->set_brake(servo_id, 0);
	// }
	return motion_enable(true, servo_id);
}

int XArmAPI::set_servo_detach(int servo_id) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_brake(servo_id, 1);
	}
	else {
		ret = cmd_ser_->set_brake(servo_id, 1);
	}
	return ret;
}

int XArmAPI::clean_error(void) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->clean_err();
	}
	else {
		ret = cmd_ser_->clean_err();
	}
	get_state(&state);
	if (state == 4) {
		if (is_ready_) {
			printf("[clean_error], xArm is not ready to move\n");
		}
		is_ready_ = false;
	}
	else {
		if (!is_ready_) {
			printf("[clean_error], xArm is ready to move\n");
		}
		is_ready_ = true;
	}
	return ret;
}

int XArmAPI::clean_warn(void) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->clean_war();
	}
	else {
		ret = cmd_ser_->clean_war();
	}
	return ret;
}

int XArmAPI::set_pause_time(fp32 sltime) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->sleep_instruction(sltime);
	}
	else {
		ret = cmd_ser_->sleep_instruction(sltime);
	}
	if (get_system_time() >= sleep_finish_time_) {
		sleep_finish_time_ = get_system_time() + (long long)(sltime * 1000);
	}
	else {
		sleep_finish_time_ = sleep_finish_time_ + (long long)(sltime * 1000);
	}
	return ret;
}

void XArmAPI::_wait_stop(fp32 timeout) {
	is_stop_ = false;
	// fp32 base_angles[7];
	// memcpy(base_angles, angles, 7 * sizeof(fp32));
	long long start_time = get_system_time();
	int count = 0;
	while ((timeout <= 0 || (get_system_time() - start_time < timeout * 1000)) && !is_stop_ && is_connected() && !has_error()) {
		if (state == 4) {
			break;
		}
		if (get_system_time() < sleep_finish_time_) {
			sleep_milliseconds(20);
			count = 0;
			continue;
		}
		if (state == 3) {
			sleep_milliseconds(20);
			continue;
		}
		if (state != 1) {
			count += 1;
		}
		else {
			count = 0;
		}
		if (count >= 10)
			break;
		sleep_milliseconds(50);
	}
	sleep_finish_time_ = 0;
	is_stop_ = true;
}

int XArmAPI::set_position(fp32 pose[6], fp32 radius, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 mvpose[6];
	for (u32 i = 0; i < 6; i++) {
		last_used_position[i] = pose[i];
		mvpose[i] = (float)(default_is_radian || i < 3 ? last_used_position[i] : last_used_position[i] / RAD_DEGREE);
	}

	if (radius >= 0) {
		if (is_tcp_) {
			ret = cmd_tcp_->move_lineb(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius);
		}
		else {
			ret = cmd_ser_->move_lineb(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius);
		}
	}
	else {
		if (is_tcp_) {
			ret = cmd_tcp_->move_line(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
		}
		else {
			ret = cmd_ser_->move_line(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
		}
	}
	if (wait && (ret == 0 || ret == UXBUS_STATE::WAR_CODE)) {
		_wait_stop(timeout);
	}

	return ret;
}

int XArmAPI::set_position(fp32 pose[6], fp32 radius, bool wait, fp32 timeout) {
	return set_position(pose, radius, 0, 0, 0, wait, timeout);
}

int XArmAPI::set_position(fp32 pose[6], bool wait, fp32 timeout) {
	return set_position(pose, -1, 0, 0, 0, wait, timeout);
}

int XArmAPI::set_tool_position(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 mvpose[6];
	for (u32 i = 0; i < 6; i++) {
		mvpose[i] = (float)(default_is_radian || i < 3 ? pose[i] : pose[i] / RAD_DEGREE);
	}
	if (is_tcp_) {
		ret = cmd_tcp_->move_line_tool(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
	}
	else {
		ret = cmd_ser_->move_line_tool(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
	}
	if (wait && (ret == 0 || ret == UXBUS_STATE::WAR_CODE)) {
		_wait_stop(timeout);
	}

	return ret;
}

int XArmAPI::set_tool_position(fp32 pose[6], bool wait, fp32 timeout) {
	return set_tool_position(pose, 0, 0, 0, wait, timeout);
}


int XArmAPI::set_servo_angle(fp32 angs[7], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	last_used_joint_speed = speed > 0 ? speed : last_used_joint_speed;
	last_used_joint_acc = acc > 0 ? acc : last_used_joint_acc;
	fp32 mvjoint[7];
	for (u32 i = 0; i < 7; i++) {
		last_used_angles[i] = angs[i];
		mvjoint[i] = (float)(default_is_radian ? last_used_angles[i] : last_used_angles[i] / RAD_DEGREE);
	}
	fp32 speed_ = (float)(default_is_radian ? last_used_joint_speed : last_used_joint_speed / RAD_DEGREE);
	fp32 acc_ = (float)(default_is_radian ? last_used_joint_acc : last_used_joint_acc / RAD_DEGREE);


	if (is_tcp_) {
		ret = cmd_tcp_->move_joint(mvjoint, speed_, acc_, mvtime);
	}
	else {
		ret = cmd_ser_->move_joint(mvjoint, speed_, acc_, mvtime);
	}
	if (wait && (ret == 0 || ret == UXBUS_STATE::WAR_CODE)) {
		_wait_stop(timeout);
	}

	return ret;
}

int XArmAPI::set_servo_angle(fp32 angs[7], bool wait, fp32 timeout) {
	return set_servo_angle(angs, 0, 0, 0, wait, timeout);
}

int XArmAPI::set_servo_angle(int servo_id, fp32 angle, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	assert(servo_id > 0 && servo_id <= 7);
	last_used_angles[servo_id - 1] = angle;
	return set_servo_angle(last_used_angles, speed, acc, mvtime, wait, timeout);
}

int XArmAPI::set_servo_angle(int servo_id, fp32 angle, bool wait, fp32 timeout) {
	return set_servo_angle(servo_id, angle, 0, 0, 0, wait, timeout);
}

int XArmAPI::set_servo_angle_j(fp32 angs[7], fp32 speed, fp32 acc, fp32 mvtime) {
	if (!is_connected()) return -1;
	int ret = 0;
	fp32 mvjoint[7];
	for (u32 i = 0; i < 7; i++) {
		mvjoint[i] = (float)(default_is_radian ? angs[i] : angs[i] / RAD_DEGREE);
	}

	if (is_tcp_) {
		ret = cmd_tcp_->move_servoj(mvjoint, last_used_joint_speed, last_used_joint_acc, mvtime);
	}
	else {
		ret = cmd_ser_->move_servoj(mvjoint, last_used_joint_speed, last_used_joint_acc, mvtime);
	}

	return ret;
}

int XArmAPI::set_servo_cartesian(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime) {
	if (!is_connected()) return -1;
	int ret = 0;
	fp32 mvpose[6];
	for (u32 i = 0; i < 6; i++) {
		mvpose[i] = (float)(i < 3 || default_is_radian ? pose[i] : pose[i] / RAD_DEGREE);
	}

	if (is_tcp_) {
		ret = cmd_tcp_->move_servo_cartesian(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
	}
	else {
		ret = cmd_ser_->move_servo_cartesian(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
	}

	return ret;
}

int XArmAPI::move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 pose_1[6];
	fp32 pose_2[6];
	for (u32 i = 0; i < 6; i++) {
		pose_1[i] = (float)(default_is_radian || i < 3 ? pose1[i] : pose1[i] / RAD_DEGREE);
		pose_2[i] = (float)(default_is_radian || i < 3 ? pose2[i] : pose2[i] / RAD_DEGREE);
	}
	if (is_tcp_) {
		ret = cmd_tcp_->move_circle(pose_1, pose_2, last_used_tcp_speed, last_used_tcp_acc, mvtime, percent);
	}
	else {
		ret = cmd_ser_->move_circle(pose_1, pose_2, last_used_tcp_speed, last_used_tcp_acc, mvtime, percent);
	}

	if (wait && (ret == 0 || ret == UXBUS_STATE::WAR_CODE)) {
		_wait_stop(timeout);
	}

	return ret;
}

int XArmAPI::move_gohome(fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	fp32 speed_ = (float)(default_is_radian ? speed : speed / RAD_DEGREE);
	fp32 acc_ = (float)(default_is_radian ? acc : acc / RAD_DEGREE);
	speed_ = speed_ > 0 ? speed_ : (float)0.8726646259971648; // 50 °/s
	acc_ = acc_ > 0 ? acc_ : (float)17.453292519943297; // 1000 °/s^2
	if (is_tcp_) {
		ret = cmd_tcp_->move_gohome(speed_, acc_, mvtime);
	}
	else {
		ret = cmd_ser_->move_gohome(speed_, acc_, mvtime);
	}
	if (wait && (ret == 0 || ret == UXBUS_STATE::WAR_CODE)) {
		_wait_stop(timeout);
	}

	return ret;
}

int XArmAPI::move_gohome(bool wait, fp32 timeout) {
	return move_gohome(0, 0, 0, wait, timeout);
}

void XArmAPI::reset(bool wait, fp32 timeout) {
	int err_warn[2];
	int state_;
	if (!is_tcp_) {
		get_err_warn_code(err_warn);
		get_state(&state_);
	}
	if (warn_code != 0) {
		clean_warn();
	}
	if (error_code != 0) {
		clean_error();
		motion_enable(true, 8);
		set_mode(0);
		set_state(0);
	}
	if (!is_ready_) {
		motion_enable(true, 8);
		set_mode(0);
		set_state(0);
	}
	move_gohome(wait, timeout);
}

void XArmAPI::emergency_stop(void) {
	long long start_time = get_system_time();
	while (state != 4 && get_system_time() - start_time < 3000) {
		set_state(4);
		sleep_milliseconds(100);
	}
	is_stop_ = true;
	sleep_finish_time_ = 0;
	// motion_enable(true, 8);
	// while ((state == 0 || state == 3 || state == 4) && get_system_time() - start_time < 3000) {
	//     set_state(0);
	//     sleep_milliseconds(100);
	// }
}

int XArmAPI::get_inverse_kinematics(fp32 source_pose[6], fp32 target_angles[7]) {
	if (!is_connected()) return -1;
	fp32 pose[6];
	for (u32 i = 0; i < 6; i++) {
		pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : source_pose[i] / RAD_DEGREE);
	}
	int ret = 0;
	fp32 angs[7];
	if (is_tcp_) {
		ret = cmd_tcp_->get_ik(pose, angs);
	}
	else {
		ret = cmd_ser_->get_ik(pose, angs);
	}
	for (u32 i = 0; i < 7; i++) {
		target_angles[i] = (float)(default_is_radian ? angs[i] : angs[i] * RAD_DEGREE);
	}
	return ret;
}

int XArmAPI::get_forward_kinematics(fp32 source_angles[7], fp32 target_pose[6]) {
	if (!is_connected()) return -1;
	fp32 angs[7];
	for (u32 i = 0; i < 7; i++) {
		angs[i] = (float)(default_is_radian ? source_angles[i] : source_angles[i] / RAD_DEGREE);
	}
	int ret = 0;
	fp32 pose[6];
	if (is_tcp_) {
		ret = cmd_tcp_->get_fk(angs, pose);
	}
	else {
		ret = cmd_ser_->get_fk(angs, pose);
	}
	for (u32 i = 0; i < 6; i++) {
		target_pose[i] = (float)(default_is_radian || i < 3 ? pose[i] : pose[i] * RAD_DEGREE);
	}
	return ret;
}

int XArmAPI::is_tcp_limit(fp32 source_pose[6], int *limit) {
	if (!is_connected()) return -1;
	fp32 pose[6];
	for (u32 i = 0; i < 6; i++) {
		pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : source_pose[i] / RAD_DEGREE);
	}
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->is_tcp_limit(pose, limit);
	}
	else {
		ret = cmd_ser_->is_tcp_limit(pose, limit);
	}
	return ret;
}

int XArmAPI::is_joint_limit(fp32 source_angles[7], int *limit) {
	if (!is_connected()) return -1;
	fp32 angs[7];
	for (u32 i = 0; i < 7; i++) {
		angs[i] = (float)(default_is_radian ? source_angles[i] : source_angles[i] / RAD_DEGREE);
	}
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->is_joint_limit(angs, limit);
	}
	else {
		ret = cmd_ser_->is_joint_limit(angs, limit);
	}
	return ret;
}

int XArmAPI::set_collision_sensitivity(int sensitivity) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_collis_sens(sensitivity);
	}
	else {
		ret = cmd_ser_->set_collis_sens(sensitivity);
	}
	return ret;
}

int XArmAPI::set_teach_sensitivity(int sensitivity) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_teach_sens(sensitivity);
	}
	else {
		ret = cmd_ser_->set_teach_sens(sensitivity);
	}
	return ret;
}

int XArmAPI::set_gravity_direction(fp32 gravity_dir[3]) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_gravity_dir(gravity_dir);
	}
	else {
		ret = cmd_ser_->set_gravity_dir(gravity_dir);
	}
	return ret;
}

int XArmAPI::clean_conf(void) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->clean_conf();
	}
	else {
		ret = cmd_ser_->clean_conf();
	}
	return ret;
}

int XArmAPI::save_conf(void) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->save_conf();
	}
	else {
		ret = cmd_ser_->save_conf();
	}
	return ret;
}

int XArmAPI::set_tcp_offset(fp32 pose_offset[6]) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	fp32 offset[6];
	for (u32 i = 0; i < 6; i++) {
		offset[i] = (float)(default_is_radian || i < 3 ? pose_offset[i] : pose_offset[i] / RAD_DEGREE);
	}
	if (is_tcp_) {
		ret = cmd_tcp_->set_tcp_offset(offset);
	}
	else {
		ret = cmd_ser_->set_tcp_offset(offset);
	}
	return ret;
}

int XArmAPI::set_tcp_load(fp32 weight, fp32 center_of_gravity[3]) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	float _gravity[3];
	if (compare_version(version_number, new int[3]{ 0, 2, 0 })) {
		_gravity[0] = center_of_gravity[0];
		_gravity[1] = center_of_gravity[1];
		_gravity[2] = center_of_gravity[2];
	}
	else {
		_gravity[0] = (float)(center_of_gravity[0] / 1000.0);
		_gravity[1] = (float)(center_of_gravity[1] / 1000.0);
		_gravity[2] = (float)(center_of_gravity[2] / 1000.0);
	}
	if (is_tcp_) {
		ret = cmd_tcp_->set_tcp_load(weight, _gravity);
	}
	else {
		ret = cmd_ser_->set_tcp_load(weight, _gravity);
	}
	return ret;
}

int XArmAPI::set_tcp_jerk(fp32 jerk) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_tcp_jerk(jerk);
	}
	else {
		ret = cmd_ser_->set_tcp_jerk(jerk);
	}
	return ret;
}

int XArmAPI::set_tcp_maxacc(fp32 acc) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_tcp_maxacc(acc);
	}
	else {
		ret = cmd_ser_->set_tcp_maxacc(acc);
	}
	return ret;
}

int XArmAPI::set_joint_jerk(fp32 jerk) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_joint_jerk(default_is_radian ? jerk : (float)(jerk / RAD_DEGREE));
	}
	else {
		ret = cmd_ser_->set_joint_jerk(default_is_radian ? jerk : (float)(jerk / RAD_DEGREE));
	}
	return ret;
}

int XArmAPI::set_joint_maxacc(fp32 acc) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_joint_maxacc(default_is_radian ? acc : (float)(acc / RAD_DEGREE));
	}
	else {
		ret = cmd_ser_->set_joint_maxacc(default_is_radian ? acc : (float)(acc / RAD_DEGREE));
	}
	return ret;
}

int XArmAPI::set_gripper_enable(bool enable) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_set_en(int(enable));
	}
	else {
		ret = cmd_ser_->gripper_modbus_set_en(int(enable));
	}
	int err;
	int ret2 = get_gripper_err_code(&err);
	return (ret2 == 0 && err != 0) ? err : ret;
}

int XArmAPI::set_gripper_mode(int mode) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_set_mode(mode);
	}
	else {
		ret = cmd_ser_->gripper_modbus_set_mode(mode);
	}
	int err;
	int ret2 = get_gripper_err_code(&err);
	return (ret2 == 0 && err != 0) ? err : ret;
}

int XArmAPI::set_gripper_speed(fp32 speed) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_set_posspd(speed);
	}
	else {
		ret = cmd_ser_->gripper_modbus_set_posspd(speed);
	}
	int err;
	int ret2 = get_gripper_err_code(&err);
	return (ret2 == 0 && err != 0) ? err : ret;
}

int XArmAPI::get_gripper_position(fp32 *pos) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_get_pos(pos);
	}
	else {
		ret = cmd_ser_->gripper_modbus_get_pos(pos);
	}
	int err;
	int ret2 = get_gripper_err_code(&err);
	return (ret2 == 0 && err != 0) ? err : ret;
}

int XArmAPI::get_gripper_err_code(int *err) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_get_errcode(err);
	}
	else {
		ret = cmd_ser_->gripper_modbus_get_errcode(err);
	}
	return ret;
}

int XArmAPI::set_gripper_position(fp32 pos, bool wait, fp32 timeout) {
	if (!is_connected()) return -1;
	int ret = 0;
	float last_pos = 0, pos_tmp, cur_pos;;
	bool is_add = true;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_set_pos(pos);
	}
	else {
		ret = cmd_ser_->gripper_modbus_set_pos(pos);
	}
	if (wait) {
		int ret2 = 0;
		ret2 = get_gripper_position(&pos_tmp);
		if (ret2 == 0) {
			last_pos = pos_tmp;
			if (int(last_pos) == int(pos))
				return 0;
			is_add = pos > last_pos ? true : false;
		}

		long long start_time = get_system_time();
		int count = 0;
		int count2 = 0;

		while (get_system_time() - start_time < timeout * 1000) {
			ret2 = get_gripper_position(&pos_tmp);
			if (ret2 == 0) {
				cur_pos = pos_tmp;
				if (fabs(pos - cur_pos) < 1) {
					last_pos = cur_pos;
					break;
				}
				if (is_add) {
					if (cur_pos <= last_pos) {
						count += 1;
					}
					else if (cur_pos <= pos) {
						last_pos = cur_pos;
						count = 0;
						count2 = 0;
					}
					else {
						count2 += 1;
						if (count2 >= 10) {
							break;
						}
					}
				}
				else {
					if (cur_pos >= last_pos) {
						count += 1;
					}
					else if (cur_pos >= pos) {
						last_pos = cur_pos;
						count = 0;
						count2 = 0;
					}
					else {
						count2 += 1;
						if (count2 >= 10) {
							break;
						}
					}

				}
				if (count >= 5) {
					printf("gripper target: %f, current: %f\n", pos, cur_pos);
					break;
				}
			}
			else {
				ret = ret2;
				break;
			}
			sleep_milliseconds(200);
		}
		return ret;
	}
	else {
		int err;
		int ret2 = get_gripper_err_code(&err);
		return (ret2 == 0 && err != 0) ? err : ret;
	}
}

int XArmAPI::clean_gripper_error(void) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->gripper_modbus_clean_err();
	}
	else {
		ret = cmd_ser_->gripper_modbus_clean_err();
	}
	int err;
	int ret2 = get_gripper_err_code(&err);
	return (ret2 == 0 && err != 0) ? err : ret;
}

int XArmAPI::get_tgpio_digital(int *io0, int *io1) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->tgpio_get_digital(io0, io1);
	}
	else {
		ret = cmd_ser_->tgpio_get_digital(io0, io1);
	}
	return ret;
}

int XArmAPI::set_tgpio_digital(int ionum, int value) {
	if (!is_connected()) return -1;
	assert(ionum == 0 || ionum == 1);
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->tgpio_set_digital(ionum + 1, value);
	}
	else {
		ret = cmd_ser_->tgpio_set_digital(ionum + 1, value);
	}
	return ret;
}

int XArmAPI::get_tgpio_analog(int ionum, float *value) {
	if (!is_connected()) return -1;
	assert(ionum == 0 || ionum == 1);
	int ret = 0;
	if (is_tcp_) {
		if (ionum == 0)
			ret = cmd_tcp_->tgpio_get_analog1(value);
		else
			ret = cmd_tcp_->tgpio_get_analog2(value);
	}
	else {
		if (ionum == 0)
			ret = cmd_ser_->tgpio_get_analog1(value);
		else
			ret = cmd_ser_->tgpio_get_analog2(value);
	}
	return ret;
}

int XArmAPI::get_cgpio_digital(int *digitals) {
	if (!is_connected()) return -1;
	int ret = 0;
	int tmp;
	if (is_tcp_) {
		ret = cmd_tcp_->cgpio_get_auxdigit(&tmp);
	}
	else {
		ret = cmd_ser_->cgpio_get_auxdigit(&tmp);
	}
	for (int i = 0; i < 8; i++) {
		digitals[i] = tmp >> i & 0x0001;
	}
	return ret;
}

int XArmAPI::get_cgpio_analog(int ionum, fp32 *value) {
	if (!is_connected()) return -1;
	assert(ionum == 0 || ionum == 1);
	int ret = 0;
	if (is_tcp_) {
		if (ionum == 0)
			ret = cmd_tcp_->cgpio_get_analog1(value);
		else
			ret = cmd_tcp_->cgpio_get_analog2(value);
	}
	else {
		if (ionum == 0)
			ret = cmd_ser_->cgpio_get_analog1(value);
		else
			ret = cmd_ser_->cgpio_get_analog2(value);
	}
	return ret;
}

int XArmAPI::set_cgpio_digital(int ionum, int value) {
	if (!is_connected()) return -1;
	assert(ionum >= 0 && ionum <= 7);
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->cgpio_set_auxdigit(ionum, value);
	}
	else {
		ret = cmd_ser_->cgpio_set_auxdigit(ionum, value);
	}
	return ret;
}

int XArmAPI::set_cgpio_analog(int ionum, int value) {
	if (!is_connected()) return -1;
	assert(ionum == 0 || ionum == 1);
	int ret = 0;
	if (is_tcp_) {
		if (ionum == 0)
			ret = cmd_tcp_->cgpio_set_analog1(value);
		else
			ret = cmd_tcp_->cgpio_set_analog2(value);
	}
	else {
		if (ionum == 0)
			ret = cmd_ser_->cgpio_set_analog1(value);
		else
			ret = cmd_ser_->cgpio_set_analog2(value);
	}
	return ret;
}

int XArmAPI::set_cgpio_digital_input_function(int ionum, int fun) {
	if (!is_connected()) return -1;
	assert(ionum >= 0 && ionum <= 7);
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->cgpio_set_infun(ionum, fun);
	}
	else {
		ret = cmd_ser_->cgpio_set_infun(ionum, fun);
	}
	return ret;
}

int XArmAPI::set_cgpio_digital_output_function(int ionum, int fun) {
	if (!is_connected()) return -1;
	assert(ionum >= 0 && ionum <= 7);
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->cgpio_set_outfun(ionum, fun);
	}
	else {
		ret = cmd_ser_->cgpio_set_outfun(ionum, fun);
	}
	return ret;
}

int XArmAPI::get_cgpio_state(int *state_, int *digit_io, fp32 *analog, int *input_conf, int *output_conf) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->cgpio_get_state(state_, digit_io, analog, input_conf, output_conf);
	}
	else {
		ret = cmd_ser_->cgpio_get_state(state_, digit_io, analog, input_conf, output_conf);
	}
	return ret;
}

int XArmAPI::set_reduced_mode(bool on) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_reduced_mode(int(on));
	}
	else {
		ret = cmd_ser_->set_reduced_mode(int(on));
	}
	return ret;
}

int XArmAPI::set_reduced_max_tcp_speed(float speed) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_reduced_linespeed(speed);
	}
	else {
		ret = cmd_ser_->set_reduced_linespeed(speed);
	}
	return ret;
}

int XArmAPI::set_reduced_max_joint_speed(float speed) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_reduced_jointspeed(default_is_radian ? speed : (float)(speed / RAD_DEGREE));
	}
	else {
		ret = cmd_ser_->set_reduced_jointspeed(default_is_radian ? speed : (float)(speed / RAD_DEGREE));
	}
	return ret;
}

int XArmAPI::get_reduced_mode(int *mode) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_reduced_mode(mode);
	}
	else {
		ret = cmd_ser_->get_reduced_mode(mode);
	}
	return ret;
}

int XArmAPI::get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14], int *fense_is_on, int *collision_rebound_is_on) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_reduced_states(on, xyz_list, tcp_speed, joint_speed, jrange, fense_is_on, collision_rebound_is_on, version_is_ge() ? 79 : 21);
	}
	else {
		ret = cmd_ser_->get_reduced_states(on, xyz_list, tcp_speed, joint_speed, jrange, fense_is_on, collision_rebound_is_on, version_is_ge() ? 79 : 21);
	}
	if (!default_is_radian) {
		*joint_speed = (float)(*joint_speed * RAD_DEGREE);
	}
	if (version_is_ge()) {
		if (jrange != NULL && !default_is_radian) {
			for (u32 i = 0; i < 14; i++) {
				jrange[i] = (float)(jrange[i] * RAD_DEGREE);
			}
		}
	}
	return ret;
}

int XArmAPI::set_reduced_tcp_boundary(int boundary[6]) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_xyz_limits(boundary);
	}
	else {
		ret = cmd_ser_->set_xyz_limits(boundary);
	}
	return ret;
}

int XArmAPI::set_reduced_joint_range(float jrange[14]) {
	if (!is_connected()) return -1;
	int ret = 0;
	float joint_range[14];
	for (u32 i = 0; i < 14; i++) {
		joint_range[i] = default_is_radian ? jrange[i] : (float)(jrange[i] / RAD_DEGREE);
	}
	if (is_tcp_) {
		ret = cmd_tcp_->set_reduced_jrange(joint_range);
	}
	else {
		ret = cmd_ser_->set_reduced_jrange(joint_range);
	}
	return ret;
}

int XArmAPI::set_fense_mode(bool on) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_fense_on(int(on));
	}
	else {
		ret = cmd_ser_->set_fense_on(int(on));
	}
	return ret;
}

int XArmAPI::set_collision_rebound(bool on) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_collis_reb(int(on));
	}
	else {
		ret = cmd_ser_->set_collis_reb(int(on));
	}
	return ret;
}

int XArmAPI::set_world_offset(float pose_offset[6]) {
	_check_is_pause();
	if (!is_connected()) return -1;
	int ret = 0;
	fp32 offset[6];
	for (u32 i = 0; i < 6; i++) {
		offset[i] = default_is_radian || i < 3 ? pose_offset[i] : (float)(pose_offset[i] / RAD_DEGREE);
	}
	if (is_tcp_) {
		ret = cmd_tcp_->set_world_offset(offset);
	}
	else {
		ret = cmd_ser_->set_world_offset(offset);
	}
	return ret;
}

int XArmAPI::start_record_trajectory(void) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_record_traj(1);
	}
	else {
		ret = cmd_ser_->set_record_traj(1);
	}
	return ret;
}

int XArmAPI::stop_record_trajectory(char* filename) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->set_record_traj(0);
	}
	else {
		ret = cmd_ser_->set_record_traj(0);
	}
	if (ret == 0 && filename != NULL) {
		int ret2 = save_record_trajectory(filename, 10);
		return ret2;
	}
	return ret;
}

int XArmAPI::save_record_trajectory(char* filename, float timeout) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->save_traj(filename);
	}
	else {
		ret = cmd_ser_->save_traj(filename);
	}
	if (ret == 0) {
		int ret2 = 0;
		int status = 0;
		long long start_time = get_system_time();
		while (get_system_time() - start_time < timeout * 1000) {
			ret2 = get_trajectory_rw_status(&status);
			if (ret2 == 0) {
				if (status == TRAJ_STATE::IDLE) {
					return UXBUS_STATE::TRAJ_RW_FAILED;
				}
				else if (status == TRAJ_STATE::SAVE_SUCCESS) {
					return 0;
				}
				else if (status == TRAJ_STATE::SAVE_FAIL) {
					return UXBUS_STATE::TRAJ_RW_FAILED;
				}
			}
			sleep_milliseconds(100);
		}
		return UXBUS_STATE::TRAJ_RW_TOUT;
	}
	return ret;
}

int XArmAPI::load_trajectory(char* filename, float timeout) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->load_traj(filename);
	}
	else {
		ret = cmd_ser_->load_traj(filename);
	}
	if (ret == 0) {
		int ret2 = 0;
		int status = 0;
		long long start_time = get_system_time();
		while (get_system_time() - start_time < timeout * 1000) {
			ret2 = get_trajectory_rw_status(&status);
			if (ret2 == 0) {
				if (status == TRAJ_STATE::IDLE) {
					return UXBUS_STATE::TRAJ_RW_FAILED;
				}
				else if (status == TRAJ_STATE::LOAD_SUCCESS) {
					return 0;
				}
				else if (status == TRAJ_STATE::LOAD_FAIL) {
					return UXBUS_STATE::TRAJ_RW_FAILED;
				}
			}
			sleep_milliseconds(100);
		}
		return UXBUS_STATE::TRAJ_RW_TOUT;
	}
	return ret;
}

int XArmAPI::playback_trajectory(int times, char* filename, bool wait, int double_speed) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (filename != NULL) {
		ret = load_trajectory(filename, 10);
		if (ret != 0) {
			return ret;
		}
	}
	if (state == 4) return UXBUS_STATE::NOT_READY;
	if (is_tcp_) {
		ret = cmd_tcp_->playback_traj(times, double_speed);
	}
	else {
		ret = cmd_ser_->playback_traj(times, double_speed);
	}
	if (ret == 0 && wait) {
		long long start_time = get_system_time();
		while (state != 1) {
			if (state == 4) return UXBUS_STATE::NOT_READY;
			if (get_system_time() - start_time > 5000) return UXBUS_STATE::TRAJ_PLAYBACK_TOUT;
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
				return UXBUS_STATE::NOT_READY;
			}
			if (get_system_time() - start_time > 5000) {
				return UXBUS_STATE::TRAJ_PLAYBACK_TOUT;
			}
			sleep_milliseconds(100);
		}
		sleep_milliseconds(100);
		int count = 0;
		while (state != 4) {
			if (state == 2) {
				if (times == 1) break;
				count += 1;
			}
			else {
				count = 0;
			}
			if (count > max_count) break;
			sleep_milliseconds(100);
		}
		if (state != 4) {
			set_mode(0);
			set_state(0);
		}
	}
	return ret;
}

int XArmAPI::get_trajectory_rw_status(int *status) {
	if (!is_connected()) return -1;
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->get_traj_rw_status(status);
	}
	else {
		ret = cmd_ser_->get_traj_rw_status(status);
	}
	return ret;
}

template<typename callable_vector, typename callable>
inline int XArmAPI::_register_event_callback(callable_vector&& callbacks, callable&& callback) {
	if (callback == NULL) return -1;
	for (u32 i = 0; i < callbacks.size(); i++) {
		if (callbacks[i] == callback) return 1;
	}
	callbacks.push_back(callback);
	return 0;
}

template<typename callable_vector, typename callable>
inline int XArmAPI::_release_event_callback(callable_vector&& callbacks, callable&& callback) {
	if (callback == NULL) {
		callbacks.clear();
		return 0;
	}
	for (u32 i = 0; i < callbacks.size(); i++) {
		if (callbacks[i] == callback) {
			callbacks.erase(callbacks.begin() + i);
			return 0;
		}
	}
	return -1;
}

int XArmAPI::register_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)) {
	return _register_event_callback(report_location_callbacks_, callback);
}

int XArmAPI::register_connect_changed_callback(void(*callback)(bool, bool)) {
	return _register_event_callback(connect_changed_callbacks_, callback);
}

int XArmAPI::register_state_changed_callback(void(*callback)(int)) {
	return _register_event_callback(state_changed_callbacks_, callback);
}

int XArmAPI::register_mode_changed_callback(void(*callback)(int)) {
	return _register_event_callback(mode_changed_callbacks_, callback);
}

int XArmAPI::register_mtable_mtbrake_changed_callback(void(*callback)(int, int)) {
	return _register_event_callback(mtable_mtbrake_changed_callbacks_, callback);
}

int XArmAPI::register_error_warn_changed_callback(void(*callback)(int, int)) {
	return _register_event_callback(error_warn_changed_callbacks_, callback);
}

int XArmAPI::register_cmdnum_changed_callback(void(*callback)(int)) {
	return _register_event_callback(cmdnum_changed_callbacks_, callback);
}

int XArmAPI::register_temperature_changed_callback(void(*callback)(const fp32 *temps)) {
	return _register_event_callback(temperature_changed_callbacks_, callback);
}

int XArmAPI::register_count_changed_callback(void(*callback)(int cmdnum)) {
	return _register_event_callback(count_changed_callbacks_, callback);
}

int XArmAPI::release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)) {
	return _release_event_callback(report_location_callbacks_, callback);
}

int XArmAPI::release_connect_changed_callback(void(*callback)(bool, bool)) {
	return _release_event_callback(connect_changed_callbacks_, callback);
}

int XArmAPI::release_state_changed_callback(void(*callback)(int)) {
	return _release_event_callback(state_changed_callbacks_, callback);
}

int XArmAPI::release_mode_changed_callback(void(*callback)(int)) {
	return _release_event_callback(mode_changed_callbacks_, callback);
}

int XArmAPI::release_mtable_mtbrake_changed_callback(void(*callback)(int, int)) {
	return _release_event_callback(mtable_mtbrake_changed_callbacks_, callback);
}

int XArmAPI::release_error_warn_changed_callback(void(*callback)(int, int)) {
	return _release_event_callback(error_warn_changed_callbacks_, callback);
}

int XArmAPI::release_cmdnum_changed_callback(void(*callback)(int)) {
	return _release_event_callback(cmdnum_changed_callbacks_, callback);
}

int XArmAPI::release_temperature_changed_callback(void(*callback)(const fp32 *temps)) {
	return _release_event_callback(temperature_changed_callbacks_, callback);
}
int XArmAPI::release_count_changed_callback(void(*callback)(int)) {
	return _release_event_callback(count_changed_callbacks_, callback);
}

int XArmAPI::get_suction_cup(int *val) {
	int io1;
	return get_tgpio_digital(val, &io1);
}

int XArmAPI::set_suction_cup(bool on, bool wait, float timeout) {
	int code1, code2;
	if (on) {
		code1 = set_tgpio_digital(0, 1);
		code2 = set_tgpio_digital(1, 0);
	}
	else {
		code1 = set_tgpio_digital(0, 0);
		code2 = set_tgpio_digital(1, 1);
	}
	int code = code1 == 0 ? code2 : code1;
	if (code == 0 && wait) {
		long long start_time = get_system_time();
		int val, ret;
		code = UXBUS_STATE::SUCTION_CUP_TOUT;
		while (get_system_time() - start_time < timeout * 1000) {
			ret = get_suction_cup(&val);
			if (ret == UXBUS_STATE::ERR_CODE) {
				code = UXBUS_STATE::ERR_CODE;
				break;
			}
			if (ret == 0) {
				if (on && val == 1) {
					code = 0;
					break;
				}
				if (!on && val == 0) {
					code = 0;
					break;
				}
			}
			sleep_milliseconds(100);
		}
	}

	return code;
}

int XArmAPI::get_gripper_version(unsigned char versions[3]) {
	int ret1, ret2, ret3;
	unsigned char val1[5], val2[5], val3[5];
	int code;
	versions[0] = 0;
	versions[1] = 0;
	versions[2] = 0;
	if (is_tcp_) {
		ret1 = cmd_tcp_->gripper_modbus_r16s(0x0801, 1, val1);
		ret2 = cmd_tcp_->gripper_modbus_r16s(0x0802, 1, val2);
		ret3 = cmd_tcp_->gripper_modbus_r16s(0x0803, 1, val3);
	}
	else {
		ret1 = cmd_ser_->gripper_modbus_r16s(0x0801, 1, val1);
		ret2 = cmd_ser_->gripper_modbus_r16s(0x0802, 1, val2);
		ret3 = cmd_ser_->gripper_modbus_r16s(0x0803, 1, val3);
	}
	if (ret1 == 0) { versions[0] = (unsigned char)bin8_to_16(&val1[4]); }
	else { code = ret1; }
	if (ret2 == 0) { versions[1] = (unsigned char)bin8_to_16(&val2[4]); }
	else { code = ret2; }
	if (ret3 == 0) { versions[2] = (unsigned char)bin8_to_16(&val3[4]); }
	else { code = ret3; }
	return code;
}

int XArmAPI::get_servo_version(unsigned char versions[3], int servo_id) {
	int ret1, ret2, ret3;
	float val1, val2, val3;
	int code;
	versions[0] = 0;
	versions[1] = 0;
	versions[2] = 0;
	if (is_tcp_) {
		ret1 = cmd_tcp_->servo_addr_r16(servo_id, 0x0801, &val1);
		ret2 = cmd_tcp_->servo_addr_r16(servo_id, 0x0802, &val2);
		ret3 = cmd_tcp_->servo_addr_r16(servo_id, 0x0803, &val3);
	}
	else {
		ret1 = cmd_ser_->servo_addr_r16(servo_id, 0x0801, &val1);
		ret2 = cmd_ser_->servo_addr_r16(servo_id, 0x0802, &val2);
		ret3 = cmd_ser_->servo_addr_r16(servo_id, 0x0803, &val3);
	}
	if (ret1 == 0) { versions[0] = (unsigned char)val1; }
	else { code = ret1; }
	if (ret2 == 0) { versions[1] = (unsigned char)val2; }
	else { code = ret2; }
	if (ret3 == 0) { versions[2] = (unsigned char)val3; }
	else { code = ret3; }
	return code;
}

int XArmAPI::get_tgpio_version(unsigned char versions[3]) {
	int ret1, ret2, ret3;
	float val1, val2, val3;
	int code;
	versions[0] = 0;
	versions[1] = 0;
	versions[2] = 0;
	if (is_tcp_) {
		ret1 = cmd_tcp_->tgpio_addr_r16(0x0801, &val1);
		ret2 = cmd_tcp_->tgpio_addr_r16(0x0802, &val2);
		ret3 = cmd_tcp_->tgpio_addr_r16(0x0803, &val3);
	}
	else {
		ret1 = cmd_ser_->tgpio_addr_r16(0x0801, &val1);
		ret2 = cmd_ser_->tgpio_addr_r16(0x0802, &val2);
		ret3 = cmd_ser_->tgpio_addr_r16(0x0803, &val3);
	}
	if (ret1 == 0) { versions[0] = (unsigned char)val1; }
	else { code = ret1; }
	if (ret2 == 0) { versions[1] = (unsigned char)val2; }
	else { code = ret2; }
	if (ret3 == 0) { versions[2] = (unsigned char)val3; }
	else { code = ret3; }
	return code;
}

int XArmAPI::reload_dynamics(void) {
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->reload_dynamics();
	}
	else {
		ret = cmd_tcp_->reload_dynamics();
	}
	return ret;
}

int XArmAPI::set_counter_reset(void) {
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->cnter_reset();
	}
	else {
		ret = cmd_tcp_->cnter_reset();
	}
	return ret;
}

int XArmAPI::set_counter_increase(void) {
	int ret = 0;
	if (is_tcp_) {
		ret = cmd_tcp_->cnter_plus();
	}
	else {
		ret = cmd_tcp_->cnter_plus();
	}
	return ret;
}
