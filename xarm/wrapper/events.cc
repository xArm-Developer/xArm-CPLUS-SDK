/*
* \file xarm/wrapper/xarm_api.cc
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/

#include "xarm/wrapper/xarm_api.h"

int XArmAPI::register_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < report_location_callbacks.size(); i++) {
        if (report_location_callbacks[i] == callback) return 1;
    }
    report_location_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::register_connect_changed_callback(void(*callback)(bool, bool)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < connect_changed_callbacks.size(); i++) {
        if (connect_changed_callbacks[i] == callback) return 1;
    }
    connect_changed_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::register_state_changed_callback(void(*callback)(u8)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < state_changed_callbacks.size(); i++) {
        if (state_changed_callbacks[i] == callback) return 1;
    }
    state_changed_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::register_mode_changed_callback(void(*callback)(u8)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < mode_changed_callbacks.size(); i++) {
        if (mode_changed_callbacks[i] == callback) return 1;
    }
    mode_changed_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::register_mtable_mtbrake_changed_callback(void(*callback)(int, int)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < mtable_mtbrake_changed_callbacks.size(); i++) {
        if (mtable_mtbrake_changed_callbacks[i] == callback) return 1;
    }
    mtable_mtbrake_changed_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::register_error_warn_changed_callback(void(*callback)(u8, u8)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < error_warn_changed_callbacks.size(); i++) {
        if (error_warn_changed_callbacks[i] == callback) return 1;
    }
    error_warn_changed_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::register_cmdnum_changed_callback(void(*callback)(int)) {
    if (callback == NULL) return -1;
    for (u32 i = 0; i < cmdnum_changed_callbacks.size(); i++) {
        if (cmdnum_changed_callbacks[i] == callback) return 1;
    }
    cmdnum_changed_callbacks.push_back(callback);
    return 0;
}

int XArmAPI::release_report_location_callback(void(*callback)(const fp32 *pose, const fp32 *angles)) {
    if (callback == NULL) {
        report_location_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < report_location_callbacks.size(); i++) {
        if (report_location_callbacks[i] == callback) {
            report_location_callbacks.erase(report_location_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}


int XArmAPI::release_connect_changed_callback(void(*callback)(bool, bool)) {
    if (callback == NULL) {
        connect_changed_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < connect_changed_callbacks.size(); i++) {
        if (connect_changed_callbacks[i] == callback) {
            connect_changed_callbacks.erase(connect_changed_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}

int XArmAPI::release_state_changed_callback(void(*callback)(u8)) {
    if (callback == NULL) {
        state_changed_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < state_changed_callbacks.size(); i++) {
        if (state_changed_callbacks[i] == callback) {
            state_changed_callbacks.erase(state_changed_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}

int XArmAPI::release_mode_changed_callback(void(*callback)(u8)) {
    if (callback == NULL) {
        mode_changed_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < mode_changed_callbacks.size(); i++) {
        if (mode_changed_callbacks[i] == callback) {
            mode_changed_callbacks.erase(mode_changed_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}

int XArmAPI::release_mtable_mtbrake_changed_callback(void(*callback)(int, int)) {
    if (callback == NULL) {
        mtable_mtbrake_changed_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < mtable_mtbrake_changed_callbacks.size(); i++) {
        if (mtable_mtbrake_changed_callbacks[i] == callback) {
            mtable_mtbrake_changed_callbacks.erase(mtable_mtbrake_changed_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}

int XArmAPI::release_error_warn_changed_callback(void(*callback)(u8, u8)) {
    if (callback == NULL) {
        error_warn_changed_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < error_warn_changed_callbacks.size(); i++) {
        if (error_warn_changed_callbacks[i] == callback) {
            error_warn_changed_callbacks.erase(error_warn_changed_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}

int XArmAPI::release_cmdnum_changed_callback(void(*callback)(int)) {
    if (callback == NULL) {
        cmdnum_changed_callbacks.clear();
        return 0;
    }
    for (u32 i = 0; i < cmdnum_changed_callbacks.size(); i++) {
        if (cmdnum_changed_callbacks[i] == callback) {
            cmdnum_changed_callbacks.erase(cmdnum_changed_callbacks.begin() + i);
            return 0;
        }
    }
    return -1;
}
