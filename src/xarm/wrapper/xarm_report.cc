/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"

#define PRINT_HEX_DATA(hex, len, ...)     \
{                                         \
    printf(__VA_ARGS__);              \
    for (int i = 0; i < len; ++i) {   \
        printf("%02x ", hex[i]);      \
    }                                 \
    printf("\n");                     \
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
		if (sizeof_data < 187) is_ready_ = (state == 4 || state == 5) ? false : true;

		int brake = mt_brake_;
		int able = mt_able_;
		mt_brake_ = data_fp[5];
		mt_able_ = data_fp[6];
		if (brake != mt_brake_ || able != mt_able_) _report_mtable_mtbrake_changed_callback();

		if (!is_first_report_) {
			bool ready = true;
			for (int i = 0; i < 8; i++) {
				motor_brake_states[i] = mt_brake_ >> i & 0x01;
				ready = (i < axis && !motor_brake_states[i]) ? false : ready;
			}
			for (int i = 0; i < 8; i++) {
				motor_enable_states[i] = mt_able_ >> i & 0x01;
				ready = (i < axis && !motor_enable_states[i]) ? false : ready;
			}
			is_ready_ = (state == 4 || state == 5 || !ready) ? false : true;
		}
		else {
			is_ready_ = false;
		}
		is_first_report_ = false;
		if (!is_ready_) sleep_finish_time_ = 0;

		int err = error_code;
		int warn = warn_code;
		error_code = data_fp[7];
		warn_code = data_fp[8];
		if (error_code != err || warn_code != warn) _report_error_warn_changed_callback();

		bool reset_tgpio_params = false;
		bool reset_linear_track_params = false;
		if (error_code > 0 && error_code <= 17) {
			reset_tgpio_params = true;
			reset_linear_track_params = true;
		}
		else if (error_code == 19 || error_code == 28) {
			reset_tgpio_params = true;
		}
		else if (error_code == 111) {
			reset_linear_track_params = true;
		}
		if (reset_tgpio_params) {
			modbus_baud_ = -1;
			robotiq_is_activated_ = false;
			gripper_is_enabled_ = false;
			bio_gripper_is_enabled_ = false;
			bio_gripper_speed_ = -1;
			gripper_version_numbers_[0] = -1;
			gripper_version_numbers_[1] = -1;
			gripper_version_numbers_[2] = -1;
		}
		if (reset_linear_track_params) {
			linear_track_baud_ = -1;
			linear_track_speed_ = 0;
			linear_track_status.is_enabled = 0;
		}

		// if ((error_code >= 10 && error_code <= 17) || error_code ==1 || error_code == 19 || error_code == 28) {
		// 	modbus_baud_ = -1;
		// 	robotiq_is_activated_ = false;
		// 	gripper_is_enabled_ = false;
		// 	bio_gripper_is_enabled_ = false;
		// 	bio_gripper_speed_ = -1;
		// 	gripper_version_numbers_[0] = -1;
		// 	gripper_version_numbers_[1] = -1;
		// 	gripper_version_numbers_[2] = -1;
		// }

		hex_to_nfp32(&data_fp[9], angles, 7);
		for (int i = 0; i < 7; i++) {
			angles[i] = (float)(default_is_radian ? angles[i] : angles[i] * RAD_DEGREE);
		}
		hex_to_nfp32(&data_fp[37], position, 6);
		for (int i = 0; i < 6; i++) {
			position[i] = (float)(default_is_radian || i < 3 ? position[i] : position[i] * RAD_DEGREE);
		}
		_report_location_callback();

		int cmdnum_ = cmd_num;
		cmd_num = bin8_to_16(&data_fp[61]);
		if (cmd_num != cmdnum_) _report_cmdnum_changed_callback();

		hex_to_nfp32(&data_fp[63], tcp_offset, 6);
		for (int i = 0; i < 6; i++) {
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

		axis = (device_type == 5) ? 5 : (device_type == 6) ? 6 : (device_type == 3) ? 7 : (_axis >= 5 && _axis <= 7) ? _axis : axis;

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

		for (int i = 0; i < 17; i++) sv3msg_[i] = data_fp[171 + i];
	}
}

void XArmAPI::_update(unsigned char *rx_data) {
	long long report_time = get_system_time();
	if (is_first_report_) {
		last_report_time_ = report_time;
		max_report_interval_ = 0;
	}
	long long interval = report_time - last_report_time_;
	max_report_interval_ = std::max(max_report_interval_, interval);
	last_report_time_ = report_time;
	if (is_old_protocol_) {
		_update_old(rx_data);
		return;
	}
	if (report_data_ptr_->flush_data(rx_data) != 0) return;
	_report_data_callback();
	int sizeof_data = bin8_to_32(rx_data);
	if (sizeof_data >= 87) {
		int state_ = state;
		state = report_data_ptr_->state;
		if (state != 3) {
			std::unique_lock<std::mutex> locker(mutex_);
			cond_.notify_all();
			locker.unlock();
		}
		if (state != state_) _report_state_changed_callback();
		if (sizeof_data < 133) is_ready_ = (state == 4 || state == 5) ? false : true;

		int mode_ = mode;
		mode = report_data_ptr_->mode;
		if (mode != mode_) _report_mode_changed_callback();
		int cmdnum_ = cmd_num;
		cmd_num = report_data_ptr_->cmdnum;
		if (cmd_num != cmdnum_) _report_cmdnum_changed_callback();

		for (int i = 0; i < 7; i++) {
			angles[i] = (float)(default_is_radian ? report_data_ptr_->angle[i] : report_data_ptr_->angle[i] * RAD_DEGREE);
			joints_torque[i] = report_data_ptr_->tau[i];
		}
		for (int i = 0; i < 6; i++) {
			position[i] = (float)(default_is_radian || i < 3 ? report_data_ptr_->pose[i] : report_data_ptr_->pose[i] * RAD_DEGREE);
		}
		_report_location_callback();
	}
	if (sizeof_data >= 133) {
		int brake = mt_brake_;
		int able = mt_able_;
		mt_brake_ = report_data_ptr_->mt_brake;
		mt_able_ = report_data_ptr_->mt_able;
		if (brake != mt_brake_ || able != mt_able_) _report_mtable_mtbrake_changed_callback();
		if (!is_first_report_) {
			bool ready = true;
			for (int i = 0; i < 8; i++) {
				motor_brake_states[i] = mt_brake_ >> i & 0x01;
				ready = (i < axis && !motor_brake_states[i]) ? false : ready;
			}
			for (int i = 0; i < 8; i++) {
				motor_enable_states[i] = mt_able_ >> i & 0x01;
				ready = (i < axis && !motor_enable_states[i]) ? false : ready;
			}
			is_ready_ = (state == 4 || state == 5 || !ready) ? false : true;
		}
		else {
			is_ready_ = false;
		}
		is_first_report_ = false;
		if (!is_ready_) sleep_finish_time_ = 0;

		int err = error_code;
		int warn = warn_code;
		error_code = report_data_ptr_->err;
		warn_code = report_data_ptr_->war;
		if (error_code != err || warn_code != warn) _report_error_warn_changed_callback();

		bool reset_tgpio_params = false;
		bool reset_linear_track_params = false;
		if (error_code > 0 && error_code <= 17) {
			reset_tgpio_params = true;
			reset_linear_track_params = true;
		}
		else if (error_code == 18 || error_code == 19) {
			reset_tgpio_params = true;
		}
		else if (error_code == 111) {
			reset_linear_track_params = true;
		}
		if (reset_tgpio_params) {
			modbus_baud_ = -1;
			robotiq_is_activated_ = false;
			gripper_is_enabled_ = false;
			bio_gripper_is_enabled_ = false;
			bio_gripper_speed_ = -1;
			gripper_version_numbers_[0] = -1;
			gripper_version_numbers_[1] = -1;
			gripper_version_numbers_[2] = -1;
		}
		if (reset_linear_track_params) {
			linear_track_baud_ = -1;
			linear_track_speed_ = 0;
			linear_track_status.is_enabled = 0;
		}

		if (!is_sync_ && error_code != 0 && state != 4 && state != 5) {
			_sync();
			is_sync_ = true;
		}

		for (int i = 0; i < 6; i++) {
			tcp_offset[i] = (float)(default_is_radian || i < 3 ? report_data_ptr_->tcp_offset[i] : report_data_ptr_->tcp_offset[i] * RAD_DEGREE);
		}
		for (int i = 0; i < 4; i++) {
			tcp_load[i] = report_data_ptr_->tcp_load[i];
		}

		if (!compare_version(version_number, new int[3]{ 0, 2, 0 })) {
			tcp_load[1] = tcp_load[1] * 1000;
			tcp_load[2] = tcp_load[2] * 1000;
			tcp_load[3] = tcp_load[3] * 1000;
		}

		collision_sensitivity = report_data_ptr_->collis_sens;
		teach_sensitivity = report_data_ptr_->teach_sens;
		for (int i = 0; i < 3; i++) {
			gravity_direction[i] = report_data_ptr_->gravity_dir[i];
		}
	}
	if (sizeof_data >= 245) {
		device_type = report_data_ptr_->arm_type;
		int _axis = report_data_ptr_->axis_num;
		master_id = report_data_ptr_->master_id;
		slave_id = report_data_ptr_->slave_id;
		motor_tid = report_data_ptr_->motor_tid;
		motor_fid = report_data_ptr_->motor_fid;

		axis = (_axis >= 5 && _axis <= 7) ? _axis : axis;

		memcpy(version, report_data_ptr_->versions, 30);

		tcp_jerk = report_data_ptr_->trs_jerk;
		min_tcp_acc_ = report_data_ptr_->trs_accmin;
		max_tcp_acc_ = report_data_ptr_->trs_accmax;
		min_tcp_speed_ = report_data_ptr_->trs_velomin;
		max_tcp_speed_ = report_data_ptr_->trs_velomax;
		tcp_speed_limit[0] = min_tcp_speed_;
		tcp_speed_limit[1] = max_tcp_speed_;
		tcp_acc_limit[0] = min_tcp_acc_;
		tcp_acc_limit[1] = max_tcp_acc_;

		joint_jerk = default_is_radian ? report_data_ptr_->p2p_jerk : (fp32)(report_data_ptr_->p2p_jerk * RAD_DEGREE);
		min_joint_acc_ = report_data_ptr_->p2p_accmin;
		max_joint_acc_ = report_data_ptr_->p2p_accmax;
		min_joint_speed_ = report_data_ptr_->p2p_velomin;
		max_joint_speed_ = report_data_ptr_->p2p_velomax;
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

		rot_jerk = report_data_ptr_->rot_jerk;
		max_rot_acc = report_data_ptr_->rot_accmax;

		memcpy(sv3msg_, report_data_ptr_->sv3msg, 17);

		if (sizeof_data >= 252) {
			bool isChange = false;
			for (int i = 0; i < 7; i++) {
				isChange = (temperatures[i] != report_data_ptr_->temperatures[i]) ? true : isChange;
				temperatures[i] = (float)(report_data_ptr_->temperatures[i]);
			}
			if (isChange) {
				_report_temperature_changed_callback();
			}
		}
		if (sizeof_data >= 284) {
			realtime_tcp_speed = report_data_ptr_->rt_tcp_spd;
			for (int i = 0; i < 7; i++) {
				realtime_joint_speeds[i] = report_data_ptr_->rt_joint_spds[i];
			}
		}
		if (sizeof_data >= 288) {
			if (count != -1 && count != report_data_ptr_->count) {
				count = report_data_ptr_->count;
				_report_count_changed_callback();
			}
			count = report_data_ptr_->count;
		}
		if (sizeof_data >= 312) {
			for (int i = 0; i < 6; i++) {
				world_offset[i] = (float)(default_is_radian || i < 3 ? report_data_ptr_->world_offset[i] : report_data_ptr_->world_offset[i] * RAD_DEGREE);
			}
		}
		if (sizeof_data >= 314) {
			gpio_reset_config[0] = report_data_ptr_->gpio_reset_conf[0];
			gpio_reset_config[1] = report_data_ptr_->gpio_reset_conf[1];
		}
		if (sizeof_data >= 417) {
			is_simulation_robot = report_data_ptr_->simulation_mode;
			is_collision_detection = report_data_ptr_->collision_detection;
			collision_tool_type = report_data_ptr_->collision_tool_type;
			for (int i = 0; i < 6; i++) {
				collision_model_params[i] = report_data_ptr_->collision_model_params[i];
			}

			for (int i = 0; i < 7; i++) {
				voltages[i] = report_data_ptr_->voltages[i];
				currents[i] = report_data_ptr_->currents[i];
			}

			cgpio_state = report_data_ptr_->cgpio_state;
			cgpio_code = report_data_ptr_->cgpio_code;
			cgpio_input_digitals[0] = report_data_ptr_->cgpio_input_digitals[0];
			cgpio_input_digitals[1] = report_data_ptr_->cgpio_input_digitals[1];
			cgpio_output_digitals[0] = report_data_ptr_->cgpio_output_digitals[0];
			cgpio_output_digitals[1] = report_data_ptr_->cgpio_output_digitals[1];
			cgpio_intput_anglogs[0] = report_data_ptr_->cgpio_input_analogs[0];
			cgpio_intput_anglogs[1] = report_data_ptr_->cgpio_input_analogs[1];
			cgpio_output_anglogs[0] = report_data_ptr_->cgpio_output_analogs[0];
			cgpio_output_anglogs[1] = report_data_ptr_->cgpio_output_analogs[1];
			for (int i = 0; i < 16; i++) {
				cgpio_input_conf[i] = report_data_ptr_->cgpio_input_conf[i];
				cgpio_output_conf[i] = report_data_ptr_->cgpio_output_conf[i];
			}
		}
		if (sizeof_data >= 481) {
			for (int i = 0; i < 6; i++) {
				ft_ext_force[i] = report_data_ptr_->ft_ext_force[i];
				ft_raw_force[i] = report_data_ptr_->ft_raw_force[i];
			}
		}
		if (sizeof_data >= 482) {
			if (iden_progress != report_data_ptr_->iden_progress) {
				iden_progress = report_data_ptr_->iden_progress;
				_report_iden_progress_changed_callback();
			}
			iden_progress = report_data_ptr_->iden_progress;
		}
		if (sizeof_data >= 494) {
			for (int i = 0; i < 6; i++) {
				position_aa[i] = (float)(default_is_radian || i < 3 ? report_data_ptr_->pose_aa[i] : report_data_ptr_->pose_aa[i] * RAD_DEGREE);
			}
		}
	}
}

void XArmAPI::_handle_report_data(void) {
	unsigned char rx_data[REPORT_BUF_SIZE];
	
	int ret = 0;
	int size = 0;
	int connect_fail_count = 0;
	bool need_reconnect = false;

    bool reported = is_reported();

	while (is_connected()) {
		if (ret != 0)
			sleep_milliseconds(1);
		if (connect_fail_count > 5) break;
		if (need_reconnect || !is_reported()) {
			if (need_reconnect)
				printf("report data exception, try reconnect.\n");
			else
				printf("report socket had disconnected, try connect\n");
			need_reconnect = false;
			connect_fail_count += 1;
            if (reported) {
                reported = false;
                _report_connect_changed_callback();
            }
            stream_tcp_report_ = connect_tcp_report((char *)port_.data(), report_type_);
			continue;
		}
        if (!reported) {
            reported = true;
            _report_connect_changed_callback();
        }
		memset(rx_data, 0, REPORT_BUF_SIZE);
		ret = stream_tcp_report_->read_frame(rx_data);
		connect_fail_count = 0;
		if (ret != 0) continue;
		if (size == 0) size = bin8_to_32(rx_data + 4);
		if (is_old_protocol_ && size >= 256) {
			is_old_protocol_ = false;
		}
		if (is_old_protocol_) {
			_update(rx_data);
		}
		else {
			ret = report_data_ptr_->check_data(rx_data);
			if (ret == 0)  {
				_update(rx_data);
			}
			else {
				printf("flush report data failed, ret=%d\n", ret);
			}
		}
	}
	disconnect();
	printf("xarm report thread is quit.\n");
	pool_.stop();
}
