/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#if __cplusplus < 201703L
#warning "The C++ standard must be C++17 or newer"
#endif

#include <stdint.h>
#include <stddef.h> // size_t
#include <stdbool.h> // bool
#include <string.h> // memset()
#include <math.h> // std::round(), std::abs()
#include <algorithm> // std::clamp()
#include <array> // std::array
#include <ds_dbw_can/SAE_J1850_crc.hpp>

namespace ds_dbw_can {

// CRC
static constexpr uint8_t MSG_NULL[7+1] = "\0\0\0\0\0\0\0"; // For static assertions
static constexpr uint8_t crc8(uint16_t id, const uint8_t *data, size_t len) {
    return j1850::crc8_can_msg(id, false, data, len);
}
static uint8_t crc8(uint16_t id, const void *ptr, size_t len) {
    return j1850::crc8_can_msg(id, false, (const uint8_t *)ptr, len);
}


#pragma pack(push, 1) // Pack structures to a single byte

enum class SystemSyncMode : uint8_t {
    None = 0, /* Overrides */
    Disengages = 1,
    AllOrNone = 2,
    AllOrNoneWithBtn = 3,
};
static constexpr const char * systemSyncModeToString(SystemSyncMode x) {
    switch (x) {
        case SystemSyncMode::None:             return "None";
        case SystemSyncMode::Disengages:       return "Disengages";
        case SystemSyncMode::AllOrNone:        return "AllOrNone";
        case SystemSyncMode::AllOrNoneWithBtn: return "AllOrNoneWithBtn";
        default:                               return "Unknown";
    }
}

enum class CmdSrc : uint8_t {
    User = 0,
    ULC = 1,
    Remote = 2,
    Button = 3,
    CommsLoss = 4,
    Lockout = 5,
};
enum class Gear : uint8_t {
    None = 0,
    Park = 1,
    Reverse = 2,
    Neutral = 3,
    Drive = 4,
    Low = 5,
    Manual = 6,
    Sport = 7,
    Calibrate = 15,
};
enum class TurnSignal : uint8_t {
    None = 0,
    Left = 1,
    Right = 2,
    Hazard = 3,
};
enum class Quality : uint8_t {
    Ok = 0,
    Partial = 1,
    NoData = 2,
    Fault = 3,
};

struct MsgSteerCmd {
    static constexpr size_t TIMEOUT_MS = 100;
    enum class CmdType : uint8_t {
        None = 0,
        Torque = 1,     // 0.0078125 Nm
        Angle = 2,      // 0.1 deg
        Curvature = 3,  // 0.0000061 1/m
        YawRate = 4,    // 0.015 deg/s
        Percent = 14,   // 0.01 %
        Calibrate = 15, // 0.1 deg
    };
    int16_t cmd; // Interpretation changes with cmd_type
    CmdType cmd_type :4;
    uint8_t enable :1;
    uint8_t clear :1;
    uint8_t ignore :1;
    uint8_t :1;
    uint8_t rate; // 4 deg/s, 0 for default, 255 for no limit
    uint8_t accel; // 100 deg/s^2, 0 for default, 255 for no limit
    uint8_t :8;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCmdTorqueNm(float nm) {
        cmd = std::clamp<float>(std::round(nm / 0.0078125f), -INT16_MAX, INT16_MAX);
    }
    void setCmdAngleDeg(float deg, float deg_s = 0, float deg_s2 = 0) {
        cmd = std::clamp<float>(std::round(deg / 0.1f), -INT16_MAX, INT16_MAX);
        if (deg_s < 0 || std::isinf(deg_s)) {
            rate = UINT8_MAX; // Unlimited
        } else if (deg_s > 0) {
            rate = std::clamp<float>(std::round(deg_s / 4), 1, UINT8_MAX - 1);
        } else {
            rate = 0; // Default
        }
        if (deg_s2 < 0 || std::isinf(deg_s2)) {
            accel = UINT8_MAX; // Unlimited
        } else if (deg_s2 > 0) {
            accel = std::clamp<float>(std::round(deg_s2 / 100), 1, UINT8_MAX - 1);
        } else {
            accel = 0; // Default
        }
    }
    void setCmdAngleRad(float rad, float rad_s = 0, float rad_s2 = 0) {
        setCmdAngleDeg(rad * (float)(180 / M_PI), rad_s * (float)(180 / M_PI), rad_s2 * (float)(180 / M_PI));
    }
    void setCmdCurvMDeg(float curv, float deg_s = 0, float deg_s2 = 0) {
        cmd = std::clamp<float>(std::round(curv / 0.0000061f), -INT16_MAX, INT16_MAX);
        if (deg_s < 0 || std::isinf(deg_s)) {
            rate = UINT8_MAX; // Unlimited
        } else if (deg_s > 0) {
            rate = std::clamp<float>(std::round(deg_s / 4), 1, UINT8_MAX - 1);
        } else {
            rate = 0; // Default
        }
        if (deg_s2 < 0 || std::isinf(deg_s2)) {
            accel = UINT8_MAX; // Unlimited
        } else if (deg_s2 > 0) {
            accel = std::clamp<float>(std::round(deg_s2 / 100), 1, UINT8_MAX - 1);
        } else {
            accel = 0; // Default
        }
    }
    void setCmdCurvMRad(float curv, float rad_s = 0, float rad_s2 = 0) {
        setCmdCurvMDeg(curv, rad_s * (float)(180 / M_PI), rad_s2 * (float)(180 / M_PI));
    }
    void setCmdYawRateDegS(float yaw_deg_s, float deg_s = 0, float deg_s2 = 0) {
        cmd = std::clamp<float>(std::round(yaw_deg_s / 0.015f), -INT16_MAX, INT16_MAX);
        if (deg_s < 0 || std::isinf(deg_s)) {
            rate = UINT8_MAX; // Unlimited
        } else if (deg_s > 0) {
            rate = std::clamp<float>(std::round(deg_s / 4), 1, UINT8_MAX - 1);
        } else {
            rate = 0; // Default
        }
        if (deg_s2 < 0 || std::isinf(deg_s2)) {
            accel = UINT8_MAX; // Unlimited
        } else if (deg_s2 > 0) {
            accel = std::clamp<float>(std::round(deg_s2 / 100), 1, UINT8_MAX - 1);
        } else {
            accel = 0; // Default
        }
    }
    void setCmdYawRateRadS(float yaw_rad_s, float rad_s = 0, float rad_s2 = 0) {
        setCmdYawRateDegS(yaw_rad_s * (float)(180 / M_PI), rad_s * (float)(180 / M_PI), rad_s2 * (float)(180 / M_PI));
    }
    void setCmdPercentDeg(float percent, float deg_s = 0, float deg_s2 = 0) {
        cmd = std::clamp<float>(std::round(percent / 0.01f), -INT16_MAX, INT16_MAX);
        if (deg_s < 0 || std::isinf(deg_s)) {
            rate = UINT8_MAX; // Unlimited
        } else if (deg_s > 0) {
            rate = std::clamp<float>(std::round(deg_s / 4), 1, UINT8_MAX - 1);
        } else {
            rate = 0; // Default
        }
        if (deg_s2 < 0 || std::isinf(deg_s2)) {
            accel = UINT8_MAX; // Unlimited
        } else if (deg_s2 > 0) {
            accel = std::clamp<float>(std::round(deg_s2 / 100), 1, UINT8_MAX - 1);
        } else {
            accel = 0; // Default
        }
    }
    void setCmdPercentRad(float percent, float rad_s = 0, float rad_s2 = 0) {
        setCmdPercentDeg(percent, rad_s * (float)(180 / M_PI), rad_s2 * (float)(180 / M_PI));
    }
    float cmdTorqueNm() const {
        return cmd * 0.0078125f;
    }
    float cmdAngleDeg() const {
        return cmd * 0.1f;
    }
    float cmdCurvM() const {
        return cmd * 0.0000061f;
    }
    float cmdYawRateDegS() const {
        return cmd * 0.015f;
    }
    float cmdPercent() const {
        return cmd * 0.01f;
    }
    float cmdAngleRateDegS() const {
        if (rate == 0) {
            return 0; // Default
        } else if (rate < UINT8_MAX) {
            return std::max<float>(rate * 4, 100); // Minimum of 100 deg/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdAngleAccelDegS2() const {
        if (accel == 0) {
            return 0; // Default
        } else if (accel < UINT8_MAX) {
            return std::max<float>(accel * 100, 300); // Minimum of 300 deg/s^2
        } else {
            return INFINITY; // Unlimited
        }
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgSteerCmd));
struct MsgSteerCmdRmt : public MsgSteerCmd {
    static constexpr uint32_t ID = 0x200;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgSteerCmdRmt) == sizeof(MsgSteerCmd));
struct MsgSteerCmdUsr : public MsgSteerCmd {
    static constexpr uint32_t ID = 0x210;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgSteerCmdUsr) == sizeof(MsgSteerCmd));
struct MsgSteerReport1 {
    static constexpr uint32_t ID = 0x100;
    static constexpr size_t PERIOD_MIN =  8;
    static constexpr size_t PERIOD_MS  = 10;
    static constexpr size_t PERIOD_MAX = 25;
    static constexpr size_t TIMEOUT_MS = 100;
    enum class CmdType : uint8_t {
        None = 0,
        Torque = 1,
        Angle = 2,
    };
    int16_t angle :14; // 0.1 deg
    uint8_t limiting_value :1;
    uint8_t limiting_rate :1;
    int16_t cmd :14; // 0.1 deg or 0.0078125 Nm
    CmdType cmd_type :2;
    int8_t torque; // 0.0625 Nm
    uint8_t :4;
    uint8_t external_control :1;
    uint8_t override_active :1;
    uint8_t override_other :1;
    uint8_t override_latched :1;
    uint8_t ready :1;
    uint8_t enabled :1;
    uint8_t fault :1;
    uint8_t timeout :1;
    uint8_t bad_crc :1;
    uint8_t bad_rc :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setAngleDeg(float deg) {
        if (std::isfinite(deg)) {
            angle = std::clamp<float>(std::round(deg * 10), -(INT16_MAX >> 2), INT16_MAX >> 2);
        } else {
            angle = INT16_MIN >> 2;
        }
    }
    bool angleValid() const {
        return angle != INT16_MIN >> 2;
    }
    float angleDeg() const {
        if (angleValid()) {
            return angle * 0.1f;
        }
        return NAN;
    }
    float angleRad() const {
        return angleDeg() * (float)(M_PI / 180);
    }
    bool cmdValid() const {
        return cmd != INT16_MIN >> 2;
    }
    float cmdAngleDeg() const {
        if (cmdValid()) {
            return cmd * 0.1f;
        }
        return NAN;
    }
    float cmdAngleRad() const {
        return cmdAngleDeg() * (float)(M_PI / 180);
    }
    float cmdTorqueNm() const {
        if (cmdValid()) {
            return cmd * (float)0.0078125;
        }
        return NAN;
    }
    void setCmd(MsgSteerCmd::CmdType type, float deg, float nm) {
        if (type == MsgSteerCmd::CmdType::Angle && std::isfinite(deg)) {
            cmd_type = CmdType::Angle;
            cmd = std::clamp<float>(deg * 10, -(INT16_MAX >> 2), INT16_MAX >> 2);
        } else if (type == MsgSteerCmd::CmdType::Torque && std::isfinite(nm)) {
            cmd_type = CmdType::Torque;
            cmd = std::clamp<float>(nm * 128, -(INT16_MAX >> 2), INT16_MAX >> 2);
        } else {
            cmd_type = CmdType::None;
            cmd = INT16_MIN >> 2;
        }
    }
    void setTorqueNm(float nm) {
        if (std::isfinite(nm)) {
            torque = std::clamp<float>(std::round(nm * 16), -INT8_MAX, INT8_MAX);
        } else {
            torque = INT8_MIN;
        }
    }
    bool torqueValid() const {
        return torque != INT8_MIN;
    }
    float torqueNm() const {
        if (torqueValid()) {
            return torque * (float)0.0625;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgSteerReport1));
struct MsgSteerReport2 {
    static constexpr uint32_t ID = 0x300;
    static constexpr size_t PERIOD_MS = 200;
    static constexpr size_t TIMEOUT_MS = 1000;
    uint8_t degraded :1;
    uint8_t degraded_cmd_type :1;
    uint8_t degraded_comms :1;
    uint8_t degraded_internal :1;
    uint8_t degraded_vehicle :1;
    uint8_t degraded_actuator :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t fault_power :1;
    uint8_t fault_comms :1;
    uint8_t fault_internal :1;
    uint8_t fault_vehicle :1;
    uint8_t fault_actuator :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :8;
    uint8_t :8;
    uint8_t limit_rate :8; // 4 deg/s, 255=unlimited
    uint16_t limit_value :10; // 1 deg, 1023=unlimited
    uint8_t :1;
    CmdSrc cmd_src :3;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setLimitRateDegS(float deg_s) {
        if (std::isfinite(deg_s)) {
            limit_rate = std::clamp<float>(std::round(std::abs(deg_s / 4)), 0, UINT8_MAX - 1);
        } else {
            limit_rate = UINT8_MAX; // Unlimited
        }
    }
    float getLimitRateDegS() const {
        if (limit_rate != UINT8_MAX) {
            return limit_rate * 4;
        }
        return INFINITY; // Unlimited
    }
    void setLimitValueDeg(float deg) {
        if (std::isfinite(deg)) {
            limit_value = std::clamp<float>(std::round(std::abs(deg)), 0, (UINT16_MAX >> 6) - 1);
        } else {
            limit_value = UINT16_MAX >> 6; // Unlimited
        }
    }
    float getLimitValueDeg() const {
        if (limit_value != UINT16_MAX >> 6) {
            return limit_value;
        }
        return INFINITY; // Unlimited
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgSteerReport2));
struct MsgSteerReport3 {
    static constexpr uint32_t ID = 0x310;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    uint8_t degraded_comms_dbw :1;
    uint8_t degraded_comms_dbw_gateway :1;
    uint8_t degraded_comms_dbw_brake :1;
    uint8_t degraded_comms_dbw_thrtl :1;
    uint8_t degraded_comms_dbw_gear :1;
    uint8_t :1;
    uint8_t degraded_control_performance :1;
    uint8_t degraded_param_mismatch :1;
    uint8_t degraded_comms_vehicle :1;
    uint8_t :3;
    uint8_t degraded_comms_actuator :1;
    uint8_t :3;
    uint8_t degraded_vehicle_speed :1;
    uint8_t :6;
    uint8_t degraded_calibration :1;
    uint8_t fault_comms_dbw :1;
    uint8_t fault_comms_dbw_gateway :1;
    uint8_t fault_comms_dbw_brake :1;
    uint8_t fault_comms_dbw_thrtl :1;
    uint8_t fault_comms_dbw_gear :1;
    uint8_t :3;
    uint8_t fault_comms_vehicle :1;
    uint8_t :3;
    uint8_t fault_comms_actuator :1;
    uint8_t :3;
    uint8_t fault_vehicle_speed :1;
    uint8_t :3;
    uint8_t fault_angle_sensor :1;
    uint8_t fault_torque_sensor_1 :1;
    uint8_t fault_torque_sensor_2 :1;
    uint8_t fault_torque_sensor_mismatch :1;
    uint8_t fault_actuator_torque_sensor :1;
    uint8_t fault_actuator_config :1;
    uint8_t fault_actuator_assist :1;
    uint8_t :1;
    uint8_t fault_control_performance :1;
    uint8_t fault_param_mismatch :1;
    uint8_t fault_param_limits :1;
    uint8_t fault_calibration :1;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(8 == sizeof(MsgSteerReport3));
struct MsgSteerParamHash {
    static constexpr uint32_t ID = 0x330;
    static constexpr size_t PERIOD_MS = 5000;
    static constexpr size_t TIMEOUT_MS = 17500;
    uint32_t hash;
};
static_assert(4 == sizeof(MsgSteerParamHash));

struct MsgBrakeCmd {
    static constexpr size_t TIMEOUT_MS = 100;
    enum class CmdType : uint8_t {
        None = 0,       // Command      Rate limit
        Pressure = 1,   // 0.01 bar     10 bar/s
        Torque = 2,     // 1 Nm         1000 Nm/s
        Accel = 8,      // 0.001 m/s^2  1 m/s^3     (int16_t, signed)
        AccelAcc = 9,   // 0.001 m/s^2  1 m/s^3     (int16_t, signed)
        AccelAeb = 10,  // 0.001 m/s^2  1 m/s^3     (int16_t, signed)
        PedalRaw = 13,  // 0.01 %       10 %/s
        Percent = 14,   // 0.01 %       10 %/s
        Calibrate = 15,
    };
    uint16_t cmd; // Interpretation changes with cmd_type
    CmdType cmd_type :4;
    uint8_t enable :1;
    uint8_t clear :1;
    uint8_t ignore :1;
    uint8_t :1;
    uint8_t rate_inc; // See cmd_type, 0 for default, 255 for no limit
    uint8_t rate_dec; // See cmd_type, 0 for default, 255 for no limit
    uint8_t :8;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCmdPressureBar(float bar, float bar_s_inc = 0, float bar_s_dec = 0) {
        cmd = std::clamp<float>(std::round(bar / 0.01f), 0, UINT16_MAX);
        if (bar_s_inc < 0 || std::isinf(bar_s_inc)) {
            rate_inc = UINT8_MAX; // Unlimited
        } else if (bar_s_inc > 0) {
            rate_inc = std::clamp<float>(std::round(bar_s_inc / 10), 1, UINT8_MAX - 1);
        } else {
            rate_inc = 0; // Default
        }
        if (bar_s_dec < 0 || std::isinf(bar_s_dec)) {
            rate_dec = UINT8_MAX; // Unlimited
        } else if (bar_s_dec > 0) {
            rate_dec = std::clamp<float>(std::round(bar_s_dec / 10), 1, UINT8_MAX - 1);
        } else {
            rate_dec = 0; // Default
        }
    }
    void setCmdTorqueNm(float nm, float nm_s_inc = 0, float nm_s_dec = 0) {
        cmd = std::clamp<float>(std::round(nm), 0, UINT16_MAX);
        if (nm_s_inc < 0 || std::isinf(nm_s_inc)) {
            rate_inc = UINT8_MAX; // Unlimited
        } else if (nm_s_inc > 0) {
            rate_inc = std::clamp<float>(std::round(nm_s_inc / 1000), 1, UINT8_MAX - 1);
        } else {
            rate_inc = 0; // Default
        }
        if (nm_s_dec < 0 || std::isinf(nm_s_dec)) {
            rate_dec = UINT8_MAX; // Unlimited
        } else if (nm_s_dec > 0) {
            rate_dec = std::clamp<float>(std::round(nm_s_dec / 1000), 1, UINT8_MAX - 1);
        } else {
            rate_dec = 0; // Default
        }
    }
    void setCmdAccelMpS(float accel, float jerk_inc = 0, float jerk_dec = 0) {
        cmd = (int16_t)std::clamp<float>(std::round(accel / 0.001f), -INT16_MAX, INT16_MAX);
        if (jerk_inc < 0 || std::isinf(jerk_inc)) {
            rate_inc = UINT8_MAX; // Unlimited
        } else if (jerk_inc > 0) {
            rate_inc = std::clamp<float>(std::round(jerk_inc), 1, UINT8_MAX - 1);
        } else {
            rate_inc = 0; // Default
        }
        if (jerk_dec < 0 || std::isinf(jerk_dec)) {
            rate_dec = UINT8_MAX; // Unlimited
        } else if (jerk_dec > 0) {
            rate_dec = std::clamp<float>(std::round(jerk_dec), 1, UINT8_MAX - 1);
        } else {
            rate_dec = 0; // Default
        }
    }
    void setCmdPercent(float x, float inc = 0, float dec = 0) {
        cmd = std::clamp<float>(std::round(x / 0.01f), 0, UINT16_MAX);
        if (inc < 0 || std::isinf(inc)) {
            rate_inc = UINT8_MAX; // Unlimited
        } else if (inc > 0) {
            rate_inc = std::clamp<float>(std::round(inc / 10), 1, UINT8_MAX - 1);
        } else {
            rate_inc = 0; // Default
        }
        if (dec < 0 || std::isinf(dec)) {
            rate_dec = UINT8_MAX; // Unlimited
        } else if (dec > 0) {
            rate_dec = std::clamp<float>(std::round(dec / 10), 1, UINT8_MAX - 1);
        } else {
            rate_dec = 0; // Default
        }
    }
    float cmdPressureBar() const {
        return cmd * 0.01f;
    }
    uint16_t cmdTorqueNmU16() const {
        return cmd;
    }
    float cmdTorqueNm() const {
        return cmd;
    }
    int16_t cmdAccelMpSx1000() const {
        return (int16_t)cmd;
    }
    float cmdAccelMpS() const {
        return (int16_t)cmd * 0.001f;
    }
    uint16_t cmdPercentU16() const {
        constexpr uint16_t MAX = 100 / 0.01;
        if (cmd < MAX) {
            return (cmd * UINT16_MAX) / MAX;
        }
        return UINT16_MAX;
    }
    float cmdPercent() const {
        return cmd * 0.01f;
    }
    float cmdRateIncBarS() const {
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<float>(rate_inc * 10, 20); // Minimum of 20 bar/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateDecBarS() const {
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<float>(rate_dec * 10, 20); // Minimum of 20 bar/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateIncNmS() const {
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<float>(rate_inc * 1e3f, 2e3f); // Minimum of 2 kNm/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateDecNmS() const {
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<float>(rate_dec * 1e3f, 2e3f); // Minimum of 2 kNm/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateIncMS3() const {
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max(rate_inc, (uint8_t)5); // Minimum of 5 m/s^3
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateDecMS3() const {
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max(rate_dec, (uint8_t)5); // Minimum of 5 m/s^3
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateIncPercentS() const {
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<float>(rate_inc * 10, 50); // Minimum of 50 %/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateDecPercentS() const {
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<float>(rate_dec * 10, 50); // Minimum of 50 %/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    uint16_t cmdRateIncNmSU16() const { // Nm/ms (0.001Nm/s)
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<uint16_t>(rate_inc, 2); // Minimum of 2 kNm/s
        } else {
            return UINT16_MAX; // Unlimited
        }
    }
    uint16_t cmdRateDecNmSU16() const { // Nm/ms (0.001Nm/s)
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<uint16_t>(rate_dec, 2); // Minimum of 2 kNm/s
        } else {
            return UINT16_MAX; // Unlimited
        }
    }
    int16_t cmdRateIncMS3I16() const {
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max(rate_inc, (uint8_t)5); // Minimum of 5 m/s^3
        } else {
            return INT16_MAX; // Unlimited
        }
    }
    int16_t cmdRateDecMS3I16() const {
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max(rate_dec, (uint8_t)5); // Minimum of 5 m/s^3
        } else {
            return INT16_MAX; // Unlimited
        }
    }
    uint16_t cmdRateIncPercentSU16() const { // %/ms (0.001%/s)
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<uint32_t>((rate_inc << 16) / (10u * 1000u), (uint16_t)(50e-2 * 1e-3 * UINT16_MAX)); // Minimum of 50 %/s
        } else {
            return UINT16_MAX; // Unlimited
        }
    }
    uint16_t cmdRateDecPercentSU16() const { // %/ms (0.001%/s)
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<uint32_t>((rate_dec << 16) / (10u * 1000u), (uint16_t)(50e-2 * 1e-3 * UINT16_MAX)); // Minimum of 50 %/s
        } else {
            return UINT16_MAX; // Unlimited
        }
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgBrakeCmd));
struct MsgBrakeCmdRmt : public MsgBrakeCmd {
    static constexpr uint32_t ID = 0x201;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgBrakeCmdRmt) == sizeof(MsgBrakeCmd));
struct MsgBrakeCmdUsr : public MsgBrakeCmd {
    static constexpr uint32_t ID = 0x211;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgBrakeCmdUsr) == sizeof(MsgBrakeCmd));
struct MsgBrakeCmdUlc : public MsgBrakeCmd {
    static constexpr uint32_t ID = 0x221;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgBrakeCmdUlc) == sizeof(MsgBrakeCmd));
struct MsgBrakeReport1 {
    static constexpr uint32_t ID = 0x101;
    static constexpr size_t PERIOD_MIN = 15;
    static constexpr size_t PERIOD_MS  = 20;
    static constexpr size_t PERIOD_MAX = 25;
    static constexpr size_t TIMEOUT_MS = 100;
    typedef MsgBrakeCmd::CmdType CmdType;
    /* Units for input     cmd          output
     * Pressure: 0.05 bar  0.05 bar     0.05 bar
     * Torque:   4 Nm      4 Nm         4 Nm
     * Accel:    4 Nm      0.005 m/s^2  0.005 m/s^2
     * PedalRaw  0.025 %   0.025 %      0.025 %
     * Percent   0.025 %   0.025 %      0.025 %
     */
    uint16_t input :12; // Interpretation changes with cmd_type, 4095=unknown
    uint8_t btsi :1;
    uint8_t yield_request :1; // Request throttle to yield to brakes
    uint8_t limiting_value :1;
    uint8_t limiting_rate :1;
    uint16_t cmd :12; // Interpretation changes with cmd_type, 4095=unknown
    CmdType cmd_type :4;
    uint16_t output :12; // Interpretation changes with cmd_type, 4095=unknown
    uint8_t external_control :1;
    uint8_t override_active :1;
    uint8_t override_other :1;
    uint8_t override_latched :1;
    uint8_t ready :1;
    uint8_t enabled :1;
    uint8_t fault :1;
    uint8_t timeout :1;
    uint8_t bad_crc :1;
    uint8_t bad_rc :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setPressureBar(float in_bar, float cmd_bar, float out_bar) {
        cmd_type = CmdType::Pressure;
        if (std::isfinite(in_bar)) {
            input = std::clamp<float>(in_bar / 0.05f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            input = UINT16_MAX >> 4;
        }
        if (std::isfinite(cmd_bar)) {
            cmd = std::clamp<float>(cmd_bar / 0.05f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            cmd = UINT16_MAX >> 4;
        }
        if (std::isfinite(out_bar)) {
            output = std::clamp<float>(out_bar / 0.05f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            output = UINT16_MAX >> 4;
        }
    }
    void setTorqueNm(float in_nm, float cmd_nm, float out_nm) {
        cmd_type = CmdType::Torque;
        if (std::isfinite(in_nm)) {
            input = std::clamp<float>(in_nm / 4, 0, (UINT16_MAX >> 4) - 1);
        } else {
            input = UINT16_MAX >> 4;
        }
        if (std::isfinite(cmd_nm)) {
            cmd = std::clamp<float>(cmd_nm / 4, 0, (UINT16_MAX >> 4) - 1);
        } else {
            cmd = UINT16_MAX >> 4;
        }
        if (std::isfinite(out_nm)) {
            output = std::clamp<float>(out_nm / 4, 0, (UINT16_MAX >> 4) - 1);
        } else {
            output = UINT16_MAX >> 4;
        }
    }
    void setAccel(uint16_t in_nm, int16_t cmd_ms2_x1k, int16_t out_ms2_x1k) {
        cmd_type = CmdType::Accel;
        if (in_nm != UINT16_MAX) {
            input = std::clamp<uint16_t>(in_nm / 4, 0, (UINT16_MAX >> 4) - 1);
        } else {
            input = UINT16_MAX >> 4;
        }
        if (cmd_ms2_x1k != INT16_MIN) {
            cmd = std::clamp<int16_t>(cmd_ms2_x1k / 5, -(INT16_MAX >> 4), INT16_MAX >> 4);
        } else {
            cmd = (uint16_t)INT16_MIN >> 4;
        }
        if (out_ms2_x1k != INT16_MIN) {
            output = std::clamp<int16_t>(out_ms2_x1k / 5, -(INT16_MAX >> 4), INT16_MAX >> 4);
        } else {
            output = (uint16_t)INT16_MIN >> 4;
        }
    }
    void setAccelAcc(uint16_t in_nm, int16_t cmd_ms2, int16_t out_ms2) {
        setAccel(in_nm, cmd_ms2, out_ms2);
        cmd_type = CmdType::AccelAcc;
    }
    void setAccelAeb(uint16_t in_nm, int16_t cmd_ms2, int16_t out_ms2) {
        setAccel(in_nm, cmd_ms2, out_ms2);
        cmd_type = CmdType::AccelAeb;
    }
    void setPercent(float in_pc, float cmd_pc, float out_pc) {
        cmd_type = CmdType::Percent;
        if (std::isfinite(in_pc)) {
            input = std::clamp<float>(in_pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            input = UINT16_MAX >> 4;
        }
        if (std::isfinite(cmd_pc)) {
            cmd = std::clamp<float>(cmd_pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            cmd = UINT16_MAX >> 4;
        }
        if (std::isfinite(out_pc)) {
            output = std::clamp<float>(out_pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            output = UINT16_MAX >> 4;
        }
    }
    void setPedalRaw(float in_pc, float cmd_pc, float out_pc) {
        setPercent(in_pc, cmd_pc, out_pc);
        cmd_type = CmdType::PedalRaw;
    }
    void setPercentU16(uint16_t in_pc, uint16_t cmd_pc, uint16_t out_pc) {
        cmd_type = CmdType::Percent;
        constexpr uint16_t MAX = 100 / 0.025;
        input  = (in_pc  * MAX) / UINT16_MAX;
        cmd    = (cmd_pc * MAX) / UINT16_MAX;
        output = (out_pc * MAX) / UINT16_MAX;
    }
    void setPedalRawU16(uint16_t in_pc, uint16_t cmd_pc, uint16_t out_pc) {
        setPercentU16(in_pc, cmd_pc, out_pc);
        cmd_type = CmdType::PedalRaw;
    }
    void setTorqueNmU16(uint16_t in_nm, uint16_t cmd_nm, uint16_t out_nm) {
        cmd_type = CmdType::Torque;
        input  = std::clamp<uint16_t>(in_nm  / 4, 0, (UINT16_MAX >> 4) - 1);
        cmd    = std::clamp<uint16_t>(cmd_nm / 4, 0, (UINT16_MAX >> 4) - 1);
        output = std::clamp<uint16_t>(out_nm / 4, 0, (UINT16_MAX >> 4) - 1);
    }
    int32_t cmdRawSigned() const {
        switch (cmd_type) { // No default case, explicitly specify all cases
            case MsgBrakeCmd::CmdType::Accel:
            case MsgBrakeCmd::CmdType::AccelAcc:
            case MsgBrakeCmd::CmdType::AccelAeb:
                if ((int16_t)(cmd << 4) != INT16_MIN) {
                    return (int16_t)(cmd << 4) >> 4; // Sign extend bit-field
                }
                return 0;
            case MsgBrakeCmd::CmdType::Pressure:
            case MsgBrakeCmd::CmdType::Torque:
            case MsgBrakeCmd::CmdType::PedalRaw:
            case MsgBrakeCmd::CmdType::Percent:
                if (cmd != (uint16_t)(UINT16_MAX >> 4)) {
                    return cmd;
                }
                return 0;
            case MsgBrakeCmd::CmdType::None:
            case MsgBrakeCmd::CmdType::Calibrate:
                return 0;
        }
        return 0;
    }
    bool cmdNonZero() const {
        switch (cmd_type) { // No default case, explicitly specify all cases
            case MsgBrakeCmd::CmdType::Accel:
            case MsgBrakeCmd::CmdType::AccelAcc:
            case MsgBrakeCmd::CmdType::AccelAeb:
                return cmdRawSigned() < 0;
            case MsgBrakeCmd::CmdType::Pressure:
            case MsgBrakeCmd::CmdType::Torque:
            case MsgBrakeCmd::CmdType::PedalRaw:
            case MsgBrakeCmd::CmdType::Percent:
                return cmdRawSigned() > 0;
            case MsgBrakeCmd::CmdType::None:
            case MsgBrakeCmd::CmdType::Calibrate:
                return false;
        }
        return false;
    }
    void getPressureBar(float &in_bar, float &cmd_bar, float &out_bar) const {
        if (input != UINT16_MAX >> 4) {
            in_bar = input * 0.05f;
        } else {
            in_bar = NAN;
        }
        if (cmd != UINT16_MAX >> 4) {
            cmd_bar = cmd * 0.05f;
        } else {
            cmd_bar = NAN;
        }
        if (output != UINT16_MAX >> 4) {
            out_bar = output * 0.05f;
        } else {
            out_bar = NAN;
        }
    }
    void getTorqueNm(float &in_nm, float &cmd_nm, float &out_nm) const {
        if (input != UINT16_MAX >> 4) {
            in_nm = input * 4.0f;
        } else {
            in_nm = NAN;
        }
        if (cmd != UINT16_MAX >> 4) {
            cmd_nm = cmd * 4.0f;
        } else {
            cmd_nm = NAN;
        }
        if (output != UINT16_MAX >> 4) {
            out_nm = output * 4.0f;
        } else {
            out_nm = NAN;
        }
    }
    void getAccel(float &in_nm, float &cmd_ms2, float &out_ms2) const {
        if (input != UINT16_MAX >> 4) {
            in_nm = input * 4.0f;
        } else {
            in_nm = NAN;
        }
        if ((int16_t)(cmd << 4) != INT16_MIN) {
            cmd_ms2 = ((int16_t)(cmd << 4) >> 4) * 0.005f;
        } else {
            cmd_ms2 = NAN;
        }
        if ((int16_t)(output << 4) != INT16_MIN) {
            out_ms2 = ((int16_t)(output << 4) >> 4) * 0.005f;
        } else {
            out_ms2 = NAN;
        }
    }
    void getPercent(float &in_pc, float &cmd_pc, float &out_pc) const {
        if (input != UINT16_MAX >> 4) {
            in_pc = input * 0.025f;
        } else {
            in_pc = NAN;
        }
        if (cmd != UINT16_MAX >> 4) {
            cmd_pc = cmd * 0.025f;
        } else {
            cmd_pc = NAN;
        }
        if (output != UINT16_MAX >> 4) {
            out_pc = output * 0.025f;
        } else {
            out_pc = NAN;
        }
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgBrakeReport1));
struct MsgBrakeReport2 {
    static constexpr uint32_t ID = 0x301;
    static constexpr size_t PERIOD_MS = 200;
    static constexpr size_t TIMEOUT_MS = 1000;
    enum class BrkAvlMode : uint8_t {
        Unlimited = 0,
        SecondsX2 = 1,
        MillisecondsX100 = 2,
    };
    uint8_t degraded :1;
    uint8_t degraded_cmd_type :1;
    uint8_t degraded_comms :1;
    uint8_t degraded_internal :1;
    uint8_t degraded_vehicle :1;
    uint8_t degraded_actuator :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t fault_power :1;
    uint8_t fault_comms :1;
    uint8_t fault_internal :1;
    uint8_t fault_vehicle :1;
    uint8_t fault_actuator :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :8;
    uint16_t limit_value :10; // 0.1 %, 0.2 bar, 0.02 m/s^2, 1023=unlimited
    uint8_t :2;
    uint8_t comms_loss_armed :1;
    uint8_t req_park_brake :1;
    uint8_t req_shift_park :1;
    uint8_t brake_available_full :1;
    uint8_t brake_available_duration :8; // 2 sec or 100ms
    BrkAvlMode brake_available_mux :2;
    uint8_t external_button :1;
    CmdSrc cmd_src :3;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setLimitValuePercentU16(uint16_t percent, bool valid) {
        if (valid) {
            constexpr uint16_t MAX = 100 / 0.1;
            limit_value = (percent  * MAX) / UINT16_MAX;
        } else {
            limit_value = UINT16_MAX >> 6; // Unlimited
        }
    }
    void setLimitValuePercent(float percent) {
        if (std::isfinite(percent)) {
            limit_value = std::clamp<float>(std::round(std::abs(percent) / 0.1f), 0, (UINT16_MAX >> 6) - 1);
        } else {
            limit_value = UINT16_MAX >> 6; // Unlimited
        }
    }
    float getLimitValuePercent() const {
        if (limit_value != UINT16_MAX >> 6) {
            return limit_value * 0.1f;
        }
        return INFINITY; // Unlimited
    }
    void setLimitValuePressureBar(float bar) {
        if (std::isfinite(bar)) {
            limit_value = std::clamp<float>(std::round(std::abs(bar) / 0.2f), 0, (UINT16_MAX >> 6) - 1);
        } else {
            limit_value = UINT16_MAX >> 6; // Unlimited
        }
    }
    float getLimitValuePressureBar() const {
        if (limit_value != UINT16_MAX >> 6) {
            return limit_value * 0.2f;
        }
        return INFINITY; // Unlimited
    }
    void setLimitValueDecelMps2x1k(int16_t ms2_x1k) {
        if (ms2_x1k != INT16_MIN) {
            limit_value = std::clamp<uint16_t>(std::abs(ms2_x1k) / 20, 0, (UINT16_MAX >> 6) - 1);
        } else {
            limit_value = UINT16_MAX >> 6; // Unlimited
        }
    }
    float getLimitValueDecelMps2() const {
        if (limit_value != UINT16_MAX >> 6) {
            return limit_value * 0.02f;
        }
        return INFINITY; // Unlimited
    }
    void setBrkAvailDurUnlimited() {
        brake_available_mux = BrkAvlMode::Unlimited;
    }
    void setBrkAvailDurSec(uint16_t seconds, uint16_t seconds_full, uint16_t offset = 0) {
        brake_available_mux = BrkAvlMode::SecondsX2;
        if (seconds != UINT16_MAX) {
            if (seconds > offset) {
                brake_available_duration = std::min<uint16_t>((seconds - offset) / 2, UINT8_MAX - 1);
            } else {
                brake_available_duration = 0;
            }
            brake_available_full = seconds >= seconds_full;
        } else {
            brake_available_duration = UINT8_MAX;
            brake_available_full = false;
        }
    }
    void setBrkAvailDurMs(uint32_t ms, uint32_t ms_full) {
        brake_available_mux = BrkAvlMode::MillisecondsX100;
        if (ms != UINT32_MAX) {
            brake_available_duration = std::min<uint16_t>(ms / 100, UINT8_MAX - 1);
            brake_available_full = ms >= ms_full;
        } else {
            brake_available_duration = UINT8_MAX;
            brake_available_full = false;
        }
    }
    bool brkAvailDurValid() const {
        return brake_available_duration != UINT8_MAX;
    }
    float brkAvailDurSec() const {
        if (brake_available_mux == BrkAvlMode::Unlimited) {
            return INFINITY;
        } else if (brkAvailDurValid()) {
            if (brake_available_mux == BrkAvlMode::SecondsX2) {
                return brake_available_duration * 2;
            }
            if (brake_available_mux == BrkAvlMode::MillisecondsX100) {
                return brake_available_duration * 0.1f;
            }
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgBrakeReport2));
struct MsgBrakeReport3 {
    static constexpr uint32_t ID = 0x311;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    uint8_t degraded_comms_dbw :1;
    uint8_t degraded_comms_dbw_gateway :1;
    uint8_t degraded_comms_dbw_steer :1;
    uint8_t degraded_comms_dbw_thrtl :1;
    uint8_t degraded_comms_dbw_gear :1;
    uint8_t :1;
    uint8_t degraded_control_performance :1;
    uint8_t degraded_param_mismatch :1;
    uint8_t degraded_comms_vehicle :1;
    uint8_t :3;
    uint8_t degraded_comms_actuator :1;
    uint8_t degraded_comms_actuator_1 :1;
    uint8_t degraded_comms_actuator_2 :1;
    uint8_t degraded_bped_feedback :1;
    uint8_t degraded_vehicle_speed :1;
    uint8_t degraded_btsi_stuck_low :1;
    uint8_t degraded_btsi_stuck_high :1;
    uint8_t degraded_actuator_aeb_deny :1;
    uint8_t degraded_actuator_1 :1;
    uint8_t degraded_actuator_2 :1;
    uint8_t degraded_actuator_warm :1;
    uint8_t degraded_calibration :1;
    uint8_t fault_comms_dbw :1;
    uint8_t fault_comms_dbw_gateway :1;
    uint8_t fault_comms_dbw_steer :1;
    uint8_t fault_comms_dbw_thrtl :1;
    uint8_t fault_comms_dbw_gear :1;
    uint8_t :3;
    uint8_t fault_comms_vehicle :1;
    uint8_t :3;
    uint8_t fault_comms_actuator :1;
    uint8_t fault_comms_actuator_1 :1;
    uint8_t fault_comms_actuator_2 :1;
    uint8_t :1;
    uint8_t fault_vehicle_speed :1;
    uint8_t fault_actuator_acc_deny :1;
    uint8_t fault_actuator_pedal_sensor :1;
    uint8_t fault_bped_sensor_1 :1;
    uint8_t fault_bped_sensor_2 :1;
    uint8_t fault_bped_sensor_mismatch :1;
    uint8_t fault_actuator_1 :1;
    uint8_t fault_actuator_2 :1;
    uint8_t :4;
    uint8_t fault_control_performance :1;
    uint8_t fault_param_mismatch :1;
    uint8_t fault_param_limits :1;
    uint8_t fault_calibration :1;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(8 == sizeof(MsgBrakeReport3));
struct MsgBrakeParamHash {
    static constexpr uint32_t ID = 0x331;
    static constexpr size_t PERIOD_MS = 5000;
    static constexpr size_t TIMEOUT_MS = 17500;
    uint32_t hash;
};
static_assert(4 == sizeof(MsgBrakeParamHash));

struct MsgThrtlCmd {
    static constexpr size_t TIMEOUT_MS = 100;
    enum class CmdType : uint8_t {
        None = 0,
        PedalRaw = 13,
        Percent = 14,
    };
    uint16_t cmd; // 0.025 %
    CmdType cmd_type :4;
    uint8_t enable :1;
    uint8_t clear :1;
    uint8_t ignore :1;
    uint8_t :1; // Launch
    uint8_t rate_inc; // 10 %/s, 0 for default, 255 for no limit
    uint8_t rate_dec; // 10 %/s, 0 for default, 255 for no limit
    uint8_t :8;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCmdPercent(float percent, float inc_percent_s = 0, float dec_percent_s = 0) {
        cmd = std::clamp<float>(std::round(percent / 0.025f), 0, UINT16_MAX);
        if (inc_percent_s < 0 || std::isinf(inc_percent_s)) {
            rate_inc = UINT8_MAX; // Unlimited
        } else if (inc_percent_s > 0) {
            rate_inc = std::clamp<float>(std::round(inc_percent_s / 10), 1, UINT8_MAX - 1);
        } else {
            rate_inc = 0; // Default
        }
        if (dec_percent_s < 0 || std::isinf(dec_percent_s)) {
            rate_dec = UINT8_MAX; // Unlimited
        } else if (dec_percent_s > 0) {
            rate_dec = std::clamp<float>(std::round(dec_percent_s / 10), 1, UINT8_MAX - 1);
        } else {
            rate_dec = 0; // Default
        }
    }
    float cmdPercent() const {
        return cmd * 0.025f;
    }
    float cmdRateIncPercentS() const {
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<float>(rate_inc * 10, 100); // Minimum of 100 %/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    float cmdRateDecPercentS() const {
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<float>(rate_dec * 10, 100); // Minimum of 100 %/s
        } else {
            return INFINITY; // Unlimited
        }
    }
    uint16_t cmdPercentU16() const {
        constexpr uint16_t MAX = 100 / 0.025;
        if (cmd < MAX) {
            return (cmd * UINT16_MAX) / MAX;
        }
        return UINT16_MAX;
    }
    uint16_t cmdRateIncPercentSU16() const { // %/ms (0.001%/s)
        if (rate_inc == 0) {
            return 0; // Default
        } else if (rate_inc < UINT8_MAX) {
            return std::max<uint32_t>((rate_inc << 16) / (10u * 1000u), (uint16_t)(100e-2 * 1e-3 * UINT16_MAX)); // Minimum of 100 %/s
        } else {
            return UINT16_MAX; // Unlimited
        }
    }
    uint16_t cmdRateDecPercentSU16() const { // %/ms (0.001%/s)
        if (rate_dec == 0) {
            return 0; // Default
        } else if (rate_dec < UINT8_MAX) {
            return std::max<uint32_t>((rate_dec << 16) / (10u * 1000u), (uint16_t)(100e-2 * 1e-3 * UINT16_MAX)); // Minimum of 100 %/s
        } else {
            return UINT16_MAX; // Unlimited
        }
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgThrtlCmd));
struct MsgThrtlCmdRmt : public MsgThrtlCmd {
    static constexpr uint32_t ID = 0x202;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgThrtlCmdRmt) == sizeof(MsgThrtlCmd));
struct MsgThrtlCmdUsr : public MsgThrtlCmd {
    static constexpr uint32_t ID = 0x212;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgThrtlCmdUsr) == sizeof(MsgThrtlCmd));
struct MsgThrtlCmdUlc : public MsgThrtlCmd {
    static constexpr uint32_t ID = 0x222;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgThrtlCmdUlc) == sizeof(MsgThrtlCmd));
struct MsgThrtlReport1 {
    static constexpr uint32_t ID = 0x102;
    static constexpr size_t PERIOD_MS = 20;
    static constexpr size_t TIMEOUT_MS = 100;
    typedef MsgThrtlCmd::CmdType CmdType;
    uint16_t input :12; // 0.025 %, 4095=unknown
    uint16_t :1;
    uint8_t yield_request :1; // Request brakes to yield to throttle
    uint8_t limiting_value :1;
    uint8_t limiting_rate :1;
    uint16_t cmd :12; // 0.025 %
    CmdType cmd_type :4;
    uint16_t output :12; // 0.025 %, 4095=unknown
    uint8_t external_control :1;
    uint8_t override_active :1;
    uint8_t override_other :1;
    uint8_t override_latched :1;
    uint8_t ready :1;
    uint8_t enabled :1;
    uint8_t fault :1;
    uint8_t timeout :1;
    uint8_t bad_crc :1;
    uint8_t bad_rc :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setPercent(float in_pc, float cmd_pc, float out_pc) {
        cmd_type = CmdType::Percent;
        if (std::isfinite(in_pc)) {
            input = std::clamp<float>(in_pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            input = UINT16_MAX >> 4;
        }
        if (std::isfinite(cmd_pc)) {
            cmd = std::clamp<float>(cmd_pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            cmd = UINT16_MAX >> 4;
        }
        if (std::isfinite(out_pc)) {
            output = std::clamp<float>(out_pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            output = UINT16_MAX >> 4;
        }
    }
    void setPedalRaw(float in_pc, float cmd_pc, float out_pc) {
        setPercent(in_pc, cmd_pc, out_pc);
        cmd_type = CmdType::PedalRaw;
    }
    void setPercentU16(uint16_t in_pc, uint16_t cmd_pc, uint16_t out_pc) {
        cmd_type = CmdType::Percent;
        constexpr uint16_t MAX = 100 / 0.025;
        input  = (in_pc  * MAX) / UINT16_MAX;
        cmd    = (cmd_pc * MAX) / UINT16_MAX;
        output = (out_pc * MAX) / UINT16_MAX;
    }
    void setPedalRawU16(uint16_t in_pc, uint16_t cmd_pc, uint16_t out_pc) {
        setPercentU16(in_pc, cmd_pc, out_pc);
        cmd_type = CmdType::PedalRaw;
    }
    void getPercent(float &in_pc, float &cmd_pc, float &out_pc) const {
        if (input != UINT16_MAX >> 4) {
            in_pc = input * 0.025f;
        } else {
            in_pc = NAN;
        }
        if (cmd != UINT16_MAX >> 4) {
            cmd_pc = cmd * 0.025f;
        } else {
            cmd_pc = NAN;
        }
        if (output != UINT16_MAX >> 4) {
            out_pc = output * 0.025f;
        } else {
            out_pc = NAN;
        }
    }
    float getPercentInput() const {
        if (input != UINT16_MAX >> 4) {
            return input * 0.025f;
        } else {
            return NAN;
        }
    }
    uint16_t getPercentInputU16() const {
        if (input != UINT16_MAX >> 4) {
            constexpr uint16_t MAX = 100 / 0.025;
            return std::min((input << 16) / MAX, UINT16_MAX - 1);
        } else {
            return UINT16_MAX;
        }
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgThrtlReport1));
struct MsgThrtlReport2 {
    static constexpr uint32_t ID = 0x302;
    static constexpr size_t PERIOD_MS = 200;
    static constexpr size_t TIMEOUT_MS = 1000;
    uint8_t degraded :1;
    uint8_t degraded_cmd_type :1;
    uint8_t degraded_comms :1;
    uint8_t degraded_internal :1;
    uint8_t degraded_vehicle :1;
    uint8_t degraded_sensor :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t fault_power :1;
    uint8_t fault_comms :1;
    uint8_t fault_internal :1;
    uint8_t fault_vehicle :1;
    uint8_t fault_sensor :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint16_t limit_value :10; // 0.1 %, 1023=unlimited
    uint8_t :1;
    CmdSrc cmd_src :3;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        limit_value = UINT16_MAX >> 6;
        rc = save;
    }
    void setLimitValuePcU16(uint16_t pc, bool valid) {
        if (valid) {
            constexpr uint16_t MAX = 100 / 0.1;
            limit_value = (pc  * MAX) / UINT16_MAX;
        } else {
            limit_value = UINT16_MAX >> 6; // Unlimited
        }
    }
    float getLimitValuePc() const {
        if (limit_value != UINT16_MAX >> 6) {
            return limit_value * 0.1f;
        }
        return INFINITY; // Unlimited
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgThrtlReport2));
struct MsgThrtlReport3 {
    static constexpr uint32_t ID = 0x312;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    uint8_t degraded_comms_dbw :1;
    uint8_t degraded_comms_dbw_gateway :1;
    uint8_t degraded_comms_dbw_steer :1;
    uint8_t degraded_comms_dbw_brake :1;
    uint8_t degraded_comms_dbw_gear :1;
    uint8_t :1;
    uint8_t degraded_control_performance :1;
    uint8_t degraded_param_mismatch :1;
    uint8_t degraded_vehicle_speed :1;
    uint8_t degraded_aped_feedback :1;
    uint8_t degraded_actuator_pedal_sensor :1;
    uint8_t :5;
    uint8_t :7;
    uint8_t degraded_calibration :1;
    uint8_t fault_comms_dbw :1;
    uint8_t fault_comms_dbw_gateway :1;
    uint8_t fault_comms_dbw_steer :1;
    uint8_t fault_comms_dbw_brake :1;
    uint8_t fault_comms_dbw_gear :1;
    uint8_t :3;
    uint8_t fault_vehicle_speed :1;
    uint8_t fault_aped_sensor_1 :1;
    uint8_t fault_aped_sensor_2 :1;
    uint8_t fault_aped_sensor_mismatch :1;
    uint8_t fault_actuator_pedal_sensor :1;
    uint8_t :3;
    uint8_t :8;
    uint8_t :4;
    uint8_t fault_control_performance :1;
    uint8_t fault_param_mismatch :1;
    uint8_t fault_param_limits :1;
    uint8_t fault_calibration :1;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(8 == sizeof(MsgThrtlReport3));
struct MsgThrtlParamHash {
    static constexpr uint32_t ID = 0x332;
    static constexpr size_t PERIOD_MS = 5000;
    static constexpr size_t TIMEOUT_MS = 17500;
    uint32_t hash;
};
static_assert(4 == sizeof(MsgThrtlParamHash));

struct MsgGearCmd {
    static constexpr size_t TIMEOUT_MS = 0; // Event based
    Gear cmd :4;
    uint8_t :4;
    uint8_t :8;
    uint8_t :8;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
};
static_assert(4 == sizeof(MsgGearCmd));
struct MsgGearCmdRmt : public MsgGearCmd {
    static constexpr uint32_t ID = 0x203;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgGearCmdRmt) == sizeof(MsgGearCmd));
struct MsgGearCmdUsr : public MsgGearCmd {
    static constexpr uint32_t ID = 0x213;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgGearCmdUsr) == sizeof(MsgGearCmd));
struct MsgGearCmdUlc : public MsgGearCmd {
    static constexpr uint32_t ID = 0x223;
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(sizeof(MsgGearCmdUlc) == sizeof(MsgGearCmd));
struct MsgGearReport1 {
    static constexpr uint32_t ID = 0x103;
    static constexpr size_t PERIOD_MIN =  20;
    static constexpr size_t PERIOD_MS  = 100;
    static constexpr size_t PERIOD_MAX = 100;
    static constexpr size_t TIMEOUT_MS = 350;
    enum class Reject : uint8_t {
        None = 0,            // Not rejected
        Fault = 1,           // System in fault state
        Unsupported = 2,     // Unsupported gear command
        ShiftInProgress = 3, // Shift in progress
        Override = 4,        // Override on brake, throttle, or steering
        BrakeHold = 5,       // Brake hold time depleted, stay in park
        VehicleSpeed = 6,    // Excessive vehicle speed
        Vehicle = 7,         // Rejected by vehicle (try pressing the brakes)
    };
    Gear gear :4;
    Gear cmd :4;
    Gear driver :4;
    Reject reject :3;
    uint8_t :1; // Future expansion of reject
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t power_latched :1;
    uint8_t :3;
    uint8_t external_control :1;
    uint8_t override_active :1;
    uint8_t override_other :1;
    uint8_t :1; // override_latched
    uint8_t ready :1;
    uint8_t :1; // enabled
    uint8_t fault :1;
    uint8_t :1; // timeout
    uint8_t bad_crc :1;
    uint8_t :1; // bad_rc
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgGearReport1));
struct MsgGearReport2 {
    static constexpr uint32_t ID = 0x303;
    static constexpr size_t PERIOD_MS = 200;
    static constexpr size_t TIMEOUT_MS = 1000;
    uint8_t degraded :1;
    uint8_t degraded_cmd_type :1;
    uint8_t degraded_comms :1;
    uint8_t degraded_internal :1;
    uint8_t degraded_vehicle :1;
    uint8_t degraded_actuator :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t fault_power :1;
    uint8_t fault_comms :1;
    uint8_t fault_internal :1;
    uint8_t fault_vehicle :1;
    uint8_t fault_actuator :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :3;
    CmdSrc cmd_src :3;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgGearReport2));
struct MsgGearReport3 {
    static constexpr uint32_t ID = 0x313;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    uint8_t degraded_comms_dbw :1;
    uint8_t degraded_comms_dbw_gateway :1;
    uint8_t degraded_comms_dbw_steer :1;
    uint8_t degraded_comms_dbw_brake :1;
    uint8_t degraded_comms_dbw_thrtl :1;
    uint8_t :1;
    uint8_t degraded_control_performance :1;
    uint8_t degraded_param_mismatch :1;
    uint8_t degraded_comms_vehicle :1;
    uint8_t degraded_comms_vehicle_1 :1;
    uint8_t degraded_comms_vehicle_2 :1;
    uint8_t :1;
    uint8_t degraded_comms_actuator :1;
    uint8_t degraded_comms_actuator_1 :1;
    uint8_t degraded_comms_actuator_2 :1;
    uint8_t :1;
    uint8_t degraded_vehicle_speed :1;
    uint8_t degraded_gear_mismatch :1;
    uint8_t :4;
    uint8_t degraded_power :1;
    uint8_t degraded_calibration :1;
    uint8_t fault_comms_dbw :1;
    uint8_t fault_comms_dbw_gateway :1;
    uint8_t fault_comms_dbw_steer :1;
    uint8_t fault_comms_dbw_brake :1;
    uint8_t fault_comms_dbw_thrtl :1;
    uint8_t :3;
    uint8_t fault_comms_vehicle :1;
    uint8_t fault_comms_vehicle_1 :1;
    uint8_t fault_comms_vehicle_2 :1;
    uint8_t :1;
    uint8_t fault_comms_actuator :1;
    uint8_t fault_comms_actuator_1 :1;
    uint8_t fault_comms_actuator_2 :1;
    uint8_t :1;
    uint8_t fault_vehicle_speed :1;
    uint8_t :7;
    uint8_t :1;
    uint8_t fault_actuator_config :1;
    uint8_t :3;
    uint8_t fault_param_mismatch :1;
    uint8_t :1;
    uint8_t fault_calibration :1;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(8 == sizeof(MsgGearReport3));

struct MsgMonitorCmd {
    static constexpr uint32_t ID = 0x215;
    static constexpr size_t TIMEOUT_MS = 250;
    enum class CmdType : uint8_t {
        None = 0,
        ActivateTestFault = 1,
        ClearTestFault = 2,
    };
    CmdType cmd_type :2;
    uint8_t :6;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(2 == sizeof(MsgMonitorCmd));
struct MsgMonitorReport1 {
    static constexpr uint32_t ID = 0x105;
    static constexpr size_t PERIOD_MIN = 20;
    static constexpr size_t PERIOD_MS  = 100;
    static constexpr size_t PERIOD_MAX = 100;
    static constexpr size_t TIMEOUT_MS = 250;
    enum class Fault : uint8_t {
        Unknown = 0,
        None = 1,
        Fault = 2,
    };
    static constexpr Fault mergeFaults(Fault a, Fault b) {
        if (a == Fault::Fault || b == Fault::Fault) {
            return Fault::Fault;
        }
        if (a == Fault::Unknown || b == Fault::Unknown) {
            return Fault::Unknown;
        }
        return Fault::None;
    }
    bool fault :1;
    bool shutoff :1;
    bool shutoff_on_motion :1;
    bool stationary :1;
    uint8_t :2;
    Fault fault_test :2;   // Test fault to verify shutoff action
    Fault fault_system :2; // Any fault related to system enable/disable
    Fault fault_steer :2;  // Any fault related to steer control
    Fault fault_brake :2;  // Any fault related to brake control
    Fault fault_thrtl :2;  // Any fault related to throttle control
    Fault fault_gear :2;   // Any fault related to gear control
    Fault fault_ulc :2;    // Any fault related to the ULC
    uint8_t :2;
    Fault fault_vehicle_velocity :2; // Vehicle velocity measurement mismatch with OEM
    uint8_t steer_cmd_match_oem :1;
    uint8_t steer_cmd_match_dbw :1;
    uint8_t brake_cmd_match_oem :1;
    uint8_t brake_cmd_match_dbw :1;
    uint8_t thrtl_cmd_match_oem :1;
    uint8_t thrtl_cmd_match_dbw :1;
    uint8_t gear_cmd_match_oem :1;
    uint8_t gear_cmd_match_dbw :1;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(6 == sizeof(MsgMonitorReport1));
struct MsgMonitorReport2 {
    static constexpr uint32_t ID = 0x305;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    using Fault = MsgMonitorReport1::Fault;
    Fault fault_steer_feedback :2; // Steering wheel angle measurement mismatch with OEM
    Fault fault_steer_input :2;    // Steering column torque measurement mismatch with OEM
    Fault fault_steer_param :2;    // Steering parameter mismatch with DBW
    Fault fault_steer_limit :2;    // Steering limit calculation mismatch with DBW
    Fault fault_steer_override :2; // Steering override calculation mismatch with DBW
    Fault fault_steer_cmd :2;      // Steering actuator command mismatch with OEM and DBW
    Fault fault_steer_cmd_rate :2; // Steering actuator command rate faster than DBW and limit
    Fault fault_steer_cmd_en :2;   // Steering actuator command without matching DBW command enable
    Fault fault_steer_cmd_sys :2;  // Steering actuator command with DBW system disabled
    Fault fault_steer_cmd_ovr :2;  // Steering actuator command with override
    uint8_t :4;
    Fault fault_brake_feedback :2; // Brake actuator output torque/pressure measurement mismatch with OEM
    Fault fault_brake_input :2;    // Brake pedal input torque/pressure measurement mismatch with OEM
    Fault fault_brake_param :2;    // Brake parameter mismatch with DBW
    Fault fault_brake_limit :2;    // Brake limit calculation mismatch with DBW
    Fault fault_brake_override :2; // Brake override calculation mismatch with DBW
    Fault fault_brake_cmd :2;      // Brake actuator command mismatch with OEM and DBW
    Fault fault_brake_cmd_ulc :2;  // Brake command generated by ULC without matching ULC command
    Fault fault_brake_cmd_en :2;   // Brake actuator command without matching DBW command enable
    Fault fault_brake_cmd_sys :2;  // Brake actuator command with DBW system disabled
    Fault fault_brake_cmd_ovr :2;  // Brake actuator command with override
    uint8_t :4;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgMonitorReport2));
struct MsgMonitorReport3 {
    static constexpr uint32_t ID = 0x315;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    using Fault = MsgMonitorReport1::Fault;
    Fault fault_thrtl_feedback :2; // Accelerator pedal output measurement mismatch with OEM
    Fault fault_thrtl_input :2;    // Accelerator pedal input measurement mismatch with OEM
    Fault fault_thrtl_param :2;    // Throttle parameter mismatch with DBW
    Fault fault_thrtl_limit :2;    // Throttle limit calculation mismatch with DBW
    Fault fault_thrtl_override :2; // Throttle override calculation mismatch with DBW
    Fault fault_thrtl_cmd :2;      // Throttle actuator command mismatch with OEM and DBW
    Fault fault_thrtl_cmd_ulc :2;  // Throttle command generated by ULC without matching ULC command
    Fault fault_thrtl_cmd_en :2;   // Throttle actuator command without matching DBW command enable
    Fault fault_thrtl_cmd_sys :2;  // Throttle actuator command with DBW system disabled
    Fault fault_thrtl_cmd_ovr :2;  // Throttle actuator command with override
    uint8_t :4;
    Fault fault_gear_feedback :2;  // Transmission gear measurement mismatch with OEM
    Fault fault_gear_input :2;     // Gear input selection measurement mismatch with OEM
    Fault fault_gear_param :2;     // Gear parameter mismatch with DBW
    Fault fault_gear_override :2;  // Gear override calculation mismatch with DBW
    Fault fault_gear_cmd :2;       // Gear actuator command mismatch with OEM and DBW
    Fault fault_gear_cmd_ulc :2;   // Gear command generated by ULC without matching ULC command
    uint8_t :4;
    uint8_t :8;
    uint8_t :2;
    Fault fault_system_param :2;   // System parameter mismatch with DBW
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgMonitorReport3));
struct MsgMonitorThrtl {
    static constexpr uint32_t ID = 0x2A7;
    static constexpr size_t PERIOD_MS = 20;
    static constexpr size_t TIMEOUT_MS = 100;
    uint16_t pedal_pc :12; // 0.025 %
    Quality pedal_qf :2;
    uint8_t :2;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setPercent(float percent, Quality quality) {
        pedal_pc = std::clamp<float>(percent / 0.025f, 0, UINT16_MAX >> 4);
        pedal_qf = quality;
    }
    void setPercentU16(uint16_t percent, Quality quality) {
        constexpr uint16_t MAX = 100 / 0.025;
        pedal_pc = (percent  * MAX) / UINT16_MAX;
        pedal_qf = quality;
    }
    float getPercent() const {
        return pedal_pc * 0.025f;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(4 == sizeof(MsgMonitorThrtl));

struct MsgSystemCmd {
    static constexpr uint32_t ID = 0x216;
    static constexpr size_t TIMEOUT_MS = 100;
    enum class Cmd : uint8_t {
        None = 0,
        Enable = 1,
        Disable = 2,
    };
    Cmd cmd :2;
    uint8_t :2;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(2 == sizeof(MsgSystemCmd));
struct MsgSystemReport {
    static constexpr uint32_t ID = 0x106;
    static constexpr size_t PERIOD_MIN = 20;
    static constexpr size_t PERIOD_MS  = 100;
    static constexpr size_t PERIOD_MAX = 100;
    static constexpr size_t TIMEOUT_MS = 250;
    enum class State : uint8_t {
        Manual = 0, // Not ready
        Ready  = 1,
        Active = 2,
        Fault  = 7,
    };
    enum class ReasonNotReady : uint8_t {
        None                   = 0x00,
        MissingReportSteer     = 0x10,
        MissingReportBrake     = 0x11,
        MissingReportThrtl     = 0x12,
        MissingReportGear      = 0x13,
        FaultSteer             = 0x18,
        FaultBrake             = 0x19,
        FaultThrtl             = 0x1A,
        FaultGear              = 0x1B,
        BadCrcRcCmdSteer       = 0x1C,
        BadCrcRcCmdBrake       = 0x1D,
        BadCrcRcCmdThrtl       = 0x1E,
        BadCrcCmdGear          = 0x1F,
        OverrideActiveSteer    = 0x20,
        OverrideActiveBrake    = 0x21,
        OverrideActiveThrtl    = 0x22,
        OverrideActiveGear     = 0x23,
        OverrideLatchedSteer   = 0x24,
        OverrideLatchedBrake   = 0x25,
        OverrideLatchedThrtl   = 0x26,
        OverrideOtherSteer     = 0x28,
        OverrideOtherBrake     = 0x29,
        OverrideOtherThrtl     = 0x2A,
        OverrideOtherGear      = 0x2B,
        NotReadySteer          = 0x30,
        NotReadyBrake          = 0x31,
        NotReadyThrtl          = 0x32,
        MissingCmdSteer        = 0x38,
        MissingCmdBrake        = 0x39,
        MissingCmdThrtl        = 0x3A,
        LockoutVehicleVelocity = 0xA0,
        LockoutVehicleAccel    = 0xA1,
        LockoutGearReverse     = 0xA2,
        NotEnableCmdSteer      = 0xC0,
        NotEnableCmdBrake      = 0xC1,
        NotEnableCmdThrtl      = 0xC2,
        SystemReengageDelay    = 0xF8,
        SystemLockout          = 0xFA,
        SystemDisabled         = 0xFE,
        Unknown                = 0xFF,
    };
    enum class ReasonDisengage : uint8_t {
        None                   = 0x00,
        PowerCycle             = 0x01,
        LockoutVehicleVelocity = 0x10,
        LockoutVehicleAccel    = 0x11,
        LockoutGearReverse     = 0x12,
        SteerCmdDisengage      = 0x21,
        SteerCmdInvalidCrc     = 0x22,
        SteerCmdInvalidRc      = 0x23,
        SteerCmdTimeout        = 0x24,
        SteerRptFault          = 0x25,
        SteerRptOverride       = 0x26,
        SteerRptDisengage      = 0x27,
        BrakeCmdDisengage      = 0x41,
        BrakeCmdInvalidCrc     = 0x42,
        BrakeCmdInvalidRc      = 0x43,
        BrakeCmdTimeout        = 0x44,
        BrakeRptFault          = 0x45,
        BrakeRptOverride       = 0x46,
        BrakeRptDisengage      = 0x47,
        ThrtlCmdDisengage      = 0x61,
        ThrtlCmdInvalidCrc     = 0x62,
        ThrtlCmdInvalidRc      = 0x63,
        ThrtlCmdTimeout        = 0x64,
        ThrtlRptFault          = 0x65,
        ThrtlRptOverride       = 0x66,
        ThrtlRptDisengage      = 0x67,
        GearRptFault           = 0x85,
        GearRptOverride        = 0x86,
        ExternalBrake          = 0xA0,
        SystemDisableCmd       = 0xC0,
        SystemDisableBtn       = 0xC1,
        Unknown                = 0xFF,
    };
    static constexpr const char * reasonToString(ReasonNotReady x) {
        switch (x) {
            case ReasonNotReady::None:                   return "";
            case ReasonNotReady::MissingReportSteer:     return "MissingReportSteer";
            case ReasonNotReady::MissingReportBrake:     return "MissingReportBrake";
            case ReasonNotReady::MissingReportThrtl:     return "MissingReportThrtl";
            case ReasonNotReady::MissingReportGear:      return "MissingReportGear";
            case ReasonNotReady::FaultSteer:             return "FaultSteer";
            case ReasonNotReady::FaultBrake:             return "FaultBrake";
            case ReasonNotReady::FaultThrtl:             return "FaultThrtl";
            case ReasonNotReady::FaultGear:              return "FaultGear";
            case ReasonNotReady::BadCrcRcCmdSteer:       return "BadCrcRcCmdSteer";
            case ReasonNotReady::BadCrcRcCmdBrake:       return "BadCrcRcCmdBrake";
            case ReasonNotReady::BadCrcRcCmdThrtl:       return "BadCrcRcCmdThrtl";
            case ReasonNotReady::BadCrcCmdGear:          return "BadCrcCmdGear";
            case ReasonNotReady::OverrideActiveSteer:    return "OverrideActiveSteer";
            case ReasonNotReady::OverrideActiveBrake:    return "OverrideActiveBrake";
            case ReasonNotReady::OverrideActiveThrtl:    return "OverrideActiveThrtl";
            case ReasonNotReady::OverrideActiveGear:     return "OverrideActiveGear";
            case ReasonNotReady::OverrideLatchedSteer:   return "OverrideLatchedSteer";
            case ReasonNotReady::OverrideLatchedBrake:   return "OverrideLatchedBrake";
            case ReasonNotReady::OverrideLatchedThrtl:   return "OverrideLatchedThrtl";
            case ReasonNotReady::OverrideOtherSteer:     return "OverrideOtherSteer";
            case ReasonNotReady::OverrideOtherBrake:     return "OverrideOtherBrake";
            case ReasonNotReady::OverrideOtherThrtl:     return "OverrideOtherThrtl";
            case ReasonNotReady::OverrideOtherGear:      return "OverrideOtherGear";
            case ReasonNotReady::NotReadySteer:          return "NotReadySteer";
            case ReasonNotReady::NotReadyBrake:          return "NotReadyBrake";
            case ReasonNotReady::NotReadyThrtl:          return "NotReadyThrtl";
            case ReasonNotReady::MissingCmdSteer:        return "MissingCmdSteer";
            case ReasonNotReady::MissingCmdBrake:        return "MissingCmdBrake";
            case ReasonNotReady::MissingCmdThrtl:        return "MissingCmdThrtl";
            case ReasonNotReady::LockoutVehicleVelocity: return "LockoutVehicleVelocity";
            case ReasonNotReady::LockoutVehicleAccel:    return "LockoutVehicleAccel";
            case ReasonNotReady::LockoutGearReverse:     return "LockoutGearReverse";
            case ReasonNotReady::NotEnableCmdSteer:      return "NotEnableCmdSteer";
            case ReasonNotReady::NotEnableCmdBrake:      return "NotEnableCmdBrake";
            case ReasonNotReady::NotEnableCmdThrtl:      return "NotEnableCmdThrtl";
            case ReasonNotReady::SystemReengageDelay:    return "SystemReengageDelay";
            case ReasonNotReady::SystemLockout:          return "SystemLockout";
            case ReasonNotReady::SystemDisabled:         return "SystemDisabled";
            case ReasonNotReady::Unknown:                return "Unknown";
        }
        return "Unknown";
    }
    static constexpr const char * reasonToString(ReasonDisengage x) {
        switch (x) {
            case ReasonDisengage::None:                   return "";
            case ReasonDisengage::PowerCycle:             return "PowerCycle";
            case ReasonDisengage::LockoutVehicleVelocity: return "LockoutVehicleVelocity";
            case ReasonDisengage::LockoutVehicleAccel:    return "LockoutVehicleAccel";
            case ReasonDisengage::LockoutGearReverse:     return "LockoutGearReverse";
            case ReasonDisengage::SteerCmdDisengage:      return "SteerCmdDisengage";
            case ReasonDisengage::SteerCmdInvalidCrc:     return "SteerCmdInvalidCrc";
            case ReasonDisengage::SteerCmdInvalidRc:      return "SteerCmdInvalidRc";
            case ReasonDisengage::SteerCmdTimeout:        return "SteerCmdTimeout";
            case ReasonDisengage::SteerRptFault:          return "SteerRptFault";
            case ReasonDisengage::SteerRptOverride:       return "SteerRptOverride";
            case ReasonDisengage::SteerRptDisengage:      return "SteerRptDisengage";
            case ReasonDisengage::BrakeCmdDisengage:      return "BrakeCmdDisengage";
            case ReasonDisengage::BrakeCmdInvalidCrc:     return "BrakeCmdInvalidCrc";
            case ReasonDisengage::BrakeCmdInvalidRc:      return "BrakeCmdInvalidRc";
            case ReasonDisengage::BrakeCmdTimeout:        return "BrakeCmdTimeout";
            case ReasonDisengage::BrakeRptFault:          return "BrakeRptFault";
            case ReasonDisengage::BrakeRptOverride:       return "BrakeRptOverride";
            case ReasonDisengage::BrakeRptDisengage:      return "BrakeRptDisengage";
            case ReasonDisengage::ThrtlCmdDisengage:      return "ThrtlCmdDisengage";
            case ReasonDisengage::ThrtlCmdInvalidCrc:     return "ThrtlCmdInvalidCrc";
            case ReasonDisengage::ThrtlCmdInvalidRc:      return "ThrtlCmdInvalidRc";
            case ReasonDisengage::ThrtlCmdTimeout:        return "ThrtlCmdTimeout";
            case ReasonDisengage::ThrtlRptFault:          return "ThrtlRptFault";
            case ReasonDisengage::ThrtlRptOverride:       return "ThrtlRptOverride";
            case ReasonDisengage::ThrtlRptDisengage:      return "ThrtlRptDisengage";
            case ReasonDisengage::GearRptFault:           return "GearRptFault";
            case ReasonDisengage::GearRptOverride:        return "GearRptOverride";
            case ReasonDisengage::ExternalBrake:          return "ExternalBrake";
            case ReasonDisengage::SystemDisableCmd:       return "SystemDisableCmd";
            case ReasonDisengage::SystemDisableBtn:       return "SystemDisableBtn";
            case ReasonDisengage::Unknown:                return "Unknown";
        }
        return "Unknown";
    }
    uint8_t inhibit :1; // Inhibit control for steer/brake/throttle/gear
    uint8_t validate_cmd_crc_rc :1; // Parameter ValidateCmdCrcRc value
    SystemSyncMode system_sync_mode :3; // Parameter SystemSyncMode value
    uint8_t btn_enable :1;
    uint8_t btn_disable :1;
    uint8_t :1;
    ReasonDisengage reason_disengage;
    ReasonNotReady reason_not_ready;
    uint16_t time_phase :10; // Milliseconds 0-999, 0x3FF for unknown
    State state :3;
    uint8_t :3;
    uint8_t :6;
    uint8_t lockout :1;
    uint8_t override :1; // Any steer/brake/throttle/gear override
    uint8_t ready :1; // All steer/brake/throttle ready, and gear not faulted
    uint8_t enabled :1; // All steer/brake/throttle enabled, and gear not faulted (or any steer/brake/throttle enabled for Mode>=AllOrNone)
    uint8_t fault :1; // Any steer/brake/throttle/gear fault
    uint8_t /*timeout*/ :1;
    uint8_t bad_crc :1; // Invalid CRC in MsgSystemCmd
    uint8_t bad_rc :1; // Invalid rolling counter in MsgSystemCmd
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        time_phase = UINT16_MAX >> 6;
        rc = save;
    }
    void setTimePhaseMs(size_t ms) {
        time_phase = ms % 1000;
    }
    bool timePhaseValid() const {
        return time_phase != UINT16_MAX >> 6;
    }
    size_t timePhaseMs() const {
        if (timePhaseValid()) {
            return time_phase;
        }
        return SIZE_MAX;
    }
    bool operator==(const MsgSystemReport& _other) const {
        return memcmp(this, &_other, sizeof(*this)) == 0;
    }
    bool operator!=(const MsgSystemReport& _other) const {
        return !(*this == _other);
    }
    bool needsUpdate(const MsgSystemReport& previous) const {
        // Check for changes and ignore signals that always change
        MsgSystemReport self = *this;
        self.time_phase = previous.time_phase;
        self.rc = previous.rc;
        self.crc = previous.crc;
        return self != previous;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgSystemReport));

struct MsgVehicleVelocity {
    static constexpr uint32_t ID = 0x107;
    static constexpr size_t PERIOD_MIN =  8;
    static constexpr size_t PERIOD_MS  = 20;
    static constexpr size_t PERIOD_MAX = 50;
    static constexpr size_t TIMEOUT_MS = 200;
    enum class DirSrc : uint8_t {
        None = 0,   // Direction unknown
        PRNDL = 1,  // Transmission reverse gear sets negative direction
        Sensor = 2, // Explicit wheel speed/direction sensors
    };
    int16_t veh_vel_brk;  // 0.01 kph, measured by brakes
    int16_t veh_vel_prpl; // 0.01 kph, measured by propulsion
    uint8_t :8;
    uint8_t :8;
    DirSrc dir_src :2;
    uint8_t :4;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        veh_vel_brk = INT16_MIN;
        veh_vel_prpl = INT16_MIN;
        rc = save;
    }
    void setVelocityBrkKph(float kph) {
        if (std::isfinite(kph)) {
            veh_vel_brk = std::clamp<float>(kph * 100, -INT16_MAX, INT16_MAX);
        } else {
            veh_vel_brk = INT16_MIN;
        }
    }
    bool velocityBrkValid() const {
        return veh_vel_brk != INT16_MIN;
    }
    int16_t velocityBrkKphx100() const {
        return veh_vel_brk;
    }
    float velocityBrkKph() const {
        if (velocityBrkValid()) {
            return veh_vel_brk * 0.01f;
        }
        return NAN; // Invalid
    }
    float velocityBrkMps() const {
        if (velocityBrkValid()) {
            return veh_vel_brk * (0.01f / 3.6f);
        }
        return NAN; // Invalid
    }
    void setVelocityPrplKph(float kph) {
        if (std::isfinite(kph)) {
            veh_vel_prpl = std::clamp<float>(kph * 100, -INT16_MAX, INT16_MAX);
        } else {
            veh_vel_prpl = INT16_MIN;
        }
    }
    bool velocityPrplValid() const {
        return veh_vel_prpl != INT16_MIN;
    }
    int16_t velocityPrplKphx100() const {
        return veh_vel_prpl;
    }
    float velocityPrplKph() const {
        if (velocityPrplValid()) {
            return veh_vel_prpl * 0.01f;
        }
        return NAN; // Invalid
    }
    float velocityPrplMps() const {
        if (velocityPrplValid()) {
            return veh_vel_prpl * (0.01f / 3.6f);
        }
        return NAN; // Invalid
    }
    bool velocityValid() const {
        return veh_vel_brk  != INT16_MIN
            || veh_vel_prpl != INT16_MIN;
    }
    int16_t velocityKphx100() const {
        if (veh_vel_brk != INT16_MIN) {
            return veh_vel_brk;
        }
        if (veh_vel_prpl != INT16_MIN) {
            return veh_vel_prpl;
        }
        return INT16_MIN; // Invalid
    }
    float velocityKph() const {
        if (veh_vel_brk != INT16_MIN) {
            return veh_vel_brk * 0.01f;
        }
        if (veh_vel_prpl != INT16_MIN) {
            return veh_vel_prpl * 0.01f;
        }
        return NAN; // Invalid
    }
    float velocityMps() const {
        if (veh_vel_brk != INT16_MIN) {
            return veh_vel_brk * (0.01f / 3.6f);
        }
        if (veh_vel_prpl != INT16_MIN) {
            return veh_vel_prpl * (0.01f / 3.6f);
        }
        return NAN; // Invalid
    }
    bool velocityZero() const {
        if (veh_vel_brk == INT16_MIN) {
            return veh_vel_prpl == 0;
        }
        if (veh_vel_prpl == INT16_MIN) {
            return veh_vel_brk == 0;
        }
        return veh_vel_brk  == 0
            && veh_vel_prpl == 0;
    }
    uint16_t speedKphx100() const {
        if (veh_vel_brk != INT16_MIN) {
            return std::abs(veh_vel_brk);
        }
        if (veh_vel_prpl != INT16_MIN) {
            return std::abs(veh_vel_prpl);
        }
        return UINT16_MAX; // Invalid
    }
    float speedKph() const {
        return std::abs(velocityKph());
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgVehicleVelocity));

struct MsgThrtlInfo {
    static constexpr uint32_t ID = 0x109;
    static constexpr size_t PERIOD_MIN =  8;
    static constexpr size_t PERIOD_MS  = 10;
    static constexpr size_t PERIOD_MAX = 25;
    static constexpr size_t TIMEOUT_MS = 200;
    enum class OnePedalMode : uint8_t {
        Unknown = 0,
        Off = 1,
        On = 2,
        Fault = 3,
    };
    enum class DriveMode : uint8_t {
        Unknown = 0,
        Normal = 1,
        Economy = 2,
        Comfort = 3,
        Sport = 4,
        TowHaul = 5,
        Snow = 6,
        Sand = 7,
        Mud = 8,
        Rock = 9,
        Baja = 10,
        Track = 11,
    };
    enum class GearNumber : uint8_t {
        Unknown =   0, // Unknown
        Drive01 =   1, //  1st (Drive)
        Drive02 =   2, //  2nd (Drive)
        Drive03 =   3, //  3rd (Drive)
        Drive04 =   4, //  4th (Drive)
        Drive05 =   5, //  5th (Drive)
        Drive06 =   6, //  6th (Drive)
        Drive07 =   7, //  7th (Drive)
        Drive08 =   8, //  8th (Drive)
        Drive09 =   9, //  9th (Drive)
        Drive10 =  10, // 10th (Drive)
        Neutral =  16, // Neutral (N)
        Reverse1 = 17, // 1st (Reverse)
        Reverse2 = 18, // 2nd (Reverse)
        Park     = 31, // Park (P)
    };
    uint16_t accel_pedal_pc :12; // 0.025 %, 0x3FF=unknown
    Quality accel_pedal_qf :2;
    OnePedalMode one_pedal_drive :2;
    uint16_t engine_rpm; // 0.25 RPM, 0xFFFF=unknown
    uint8_t :8;
    uint8_t :4;
    DriveMode drive_mode :4;
    GearNumber gear_num :5;
    uint8_t :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        accel_pedal_qf = Quality::NoData;
        accel_pedal_pc = UINT16_MAX >> 4;
        one_pedal_drive = OnePedalMode::Unknown;
        engine_rpm = UINT16_MAX;
        gear_num = GearNumber::Unknown;
        rc = save;
    }
    void setAccelPedalPercent(float pc) {
        if (std::isfinite(pc)) {
            accel_pedal_pc = std::clamp<float>(pc / 0.025f, 0, (UINT16_MAX >> 4) - 1);
        } else {
            accel_pedal_pc = UINT16_MAX >> 4;
        }
    }
    bool accelPedalPercentValid() const {
        return accel_pedal_pc != UINT16_MAX >> 4;
    }
    bool accelPedalZero() const {
        switch (accel_pedal_qf) {
            case Quality::Ok:
            case Quality::Partial:
                return accel_pedal_pc == 0;
            case Quality::NoData:
            case Quality::Fault:
            default:
                return false;
        }
    }
    float accelPedalPercent() const {
        if (accelPedalPercentValid()) {
            return accel_pedal_pc * 0.025f;
        }
        return NAN;
    }
    uint16_t accelPedalPercentU16() const {
        if (accelPedalPercentValid()) {
            constexpr uint16_t MAX = 100 / 0.025;
            if (accel_pedal_pc < MAX) {
                return (accel_pedal_pc * UINT16_MAX) / MAX;
            }
            return UINT16_MAX;
        }
        return 0;
    }
    void setEngineRpmx4(uint16_t rpm_x4) {
        engine_rpm = rpm_x4;
    }
    void setEngineRpm(float rpm) {
        if (std::isfinite(rpm)) {
            engine_rpm = std::clamp<float>(rpm * 4, 0, UINT16_MAX - 1);
        } else {
            engine_rpm = UINT16_MAX;
        }
    }
    bool engineRpmValid() const {
        return engine_rpm != UINT16_MAX;
    }
    bool engineRpmZero() const {
        return engine_rpm == 0;
    }
    float engineRpm() const {
        if (engineRpmValid()) {
            return engine_rpm * 0.25f;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgThrtlInfo));

struct MsgBrakeInfo {
    static constexpr uint32_t ID = 0x10A;
    static constexpr size_t PERIOD_MIN =  8;
    static constexpr size_t PERIOD_MS  = 10;
    static constexpr size_t PERIOD_MAX = 25;
    static constexpr size_t TIMEOUT_MS = 200;
    uint16_t brake_torque_pedal :13; // 4 Nm, 0x1FFF for unknown
    Quality brake_pedal_qf :2;
    uint8_t :1;
    uint16_t brake_torque_request :13; // 4 Nm, 0x1FFF for unknown
    uint8_t abs_active :1;
    uint8_t abs_enabled :1;
    uint8_t esc_active :1;
    uint16_t brake_torque_actual :13; // 4 Nm, 0x1FFF for unknown
    uint8_t esc_enabled :1;
    uint8_t trac_active :1;
    uint8_t trac_enabled :1;
    uint8_t brake_vacuum :6; // 0.016 bar, 0x3F for unknown
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        brake_pedal_qf = Quality::NoData;
        brake_torque_pedal = UINT16_MAX >> 3;
        brake_torque_request = UINT16_MAX >> 3;
        brake_torque_actual = UINT16_MAX >> 3;
        brake_vacuum = UINT8_MAX >> 2;
        rc = save;
    }
    void setBrakeTorquePedalNm(uint16_t nm)   { brake_torque_pedal   = setBrakeTorqueNm(nm); }
    void setBrakeTorquePedalNm(float nm)      { brake_torque_pedal   = setBrakeTorqueNm(nm); }
    void setBrakeTorqueRequestNm(uint16_t nm) { brake_torque_request = setBrakeTorqueNm(nm); }
    void setBrakeTorqueRequestNm(float nm)    { brake_torque_request = setBrakeTorqueNm(nm); }
    void setBrakeTorqueActualNm(uint16_t nm)  { brake_torque_actual  = setBrakeTorqueNm(nm); }
    void setBrakeTorqueActualNm(float nm)     { brake_torque_actual  = setBrakeTorqueNm(nm); }
    bool brakeTorquePedalValid() const { return brakeTorqueValid(brake_torque_pedal); }
    bool brakeTorqueRequestValid() const { return brakeTorqueValid(brake_torque_request); }
    bool brakeTorqueActualValid() const { return brakeTorqueValid(brake_torque_actual); }
    uint16_t brakeTorquePedalNmU16() const { return brakeTorqueNmU16(brake_torque_pedal); }
    uint16_t brakeTorqueRequestNmU16() const { return brakeTorqueNmU16(brake_torque_request); }
    uint16_t brakeTorqueActualNmU16() const { return brakeTorqueNmU16(brake_torque_actual); }
    float brakeTorquePedalNm() const { return brakeTorqueNm(brake_torque_pedal); }
    float brakeTorqueRequestNm() const { return brakeTorqueNm(brake_torque_request); }
    float brakeTorqueActualNm() const { return brakeTorqueNm(brake_torque_actual); }
    void setBrakeVacuumBarX1000(uint16_t mbar) {
        if (mbar != UINT16_MAX) {
            brake_vacuum = std::clamp<uint16_t>(mbar / 16, 0, (UINT8_MAX >> 2) - 1);
        } else {
            brake_vacuum = UINT8_MAX >> 2;
        }
    }
    bool brakeVacuumValid() const {
        return brake_vacuum != UINT8_MAX >> 2;
    }
    uint16_t brakeVacuumBarX1000() const {
        return brakeVacuumValid() ? brake_vacuum * 16 : UINT16_MAX;
    }
    float brakeVacuumBar() const {
        return brakeVacuumValid() ? brake_vacuum * 16e-3f : NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
private:
    static uint16_t setBrakeTorqueNm(uint16_t torque) {
        if (torque != UINT16_MAX) {
            return std::clamp<uint16_t>(torque / 4, 0, (UINT16_MAX >> 3) - 1);
        } else {
            return UINT16_MAX >> 3;
        }
    }
    static uint16_t setBrakeTorqueNm(float torque) {
        if (std::isfinite(torque)) {
            return std::clamp<float>(torque / 4, 0, (UINT16_MAX >> 3) - 1);
        } else {
            return UINT16_MAX >> 3;
        }
    }
    static bool brakeTorqueValid(uint16_t torque) {
        return torque != UINT16_MAX >> 3;
    }
    static uint16_t brakeTorqueNmU16(uint16_t torque) {
        if (brakeTorqueValid(torque)) {
            return torque * 4;
        }
        return UINT16_MAX;
    }
    static float brakeTorqueNm(uint16_t torque) {
        if (brakeTorqueValid(torque)) {
            return torque * 4;
        }
        return NAN;
    }
};
static_assert(8 == sizeof(MsgBrakeInfo));

struct MsgSteerOffset {
    static constexpr uint32_t ID = 0x10C;
    static constexpr size_t PERIOD_MIN =    8;
    static constexpr size_t PERIOD_MS  =  500;
    static constexpr size_t PERIOD_MAX =  500;
    static constexpr size_t TIMEOUT_MS = 1750;
    enum class OffsetType : uint8_t {
        Unknown = 0,
        Relative = 1,
        Absolute = 2,
    };
    int16_t angle :14; // 0.1 deg, raw + offset
    uint8_t :2;
    int16_t angle_raw; // 0.1 deg, angle - offset
    int16_t angle_offset; // 0.1 deg, angle - raw
    OffsetType offset_type :2;
    uint8_t :4;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        angle = INT16_MIN >> 2;
        angle_raw = INT16_MIN;
        angle_offset = INT16_MIN;
        rc = save;
    }
    void setAngleDeg(float deg) {
        if (std::isfinite(deg)) {
            angle = std::clamp<float>(std::round(deg * 10), -(INT16_MAX >> 2), INT16_MAX >> 2);
        } else {
            angle = INT16_MIN >> 2;
        }
    }
    bool angleValid() const {
        return angle != INT16_MIN >> 2;
    }
    float angleDeg() const {
        if (angleValid()) {
            return angle * 0.1f;
        }
        return NAN;
    }
    void setAngleRawDegX10(int32_t deg) {
        if (INT16_MAX >= deg && deg >= -INT16_MAX) {
            angle_raw = deg;
        } else {
            angle_raw = INT16_MIN;
        }
    }
    void setAngleRawDeg(float deg) {
        deg = std::round(deg * 10);
        if (INT16_MAX >= deg && deg >= -INT16_MAX) {
            angle_raw = deg;
        } else {
            angle_raw = INT16_MIN;
        }
    }
    bool angleRawValid() const {
        return angle_raw != INT16_MIN;
    }
    float angleRawDeg() const {
        if (angleRawValid()) {
            return angle_raw * 0.1f;
        }
        return NAN;
    }
    void setAngleOffsetDegX10(int32_t deg) {
        if (INT16_MAX >= deg && deg >= -INT16_MAX) {
            angle_offset = deg;
        } else {
            angle_offset = INT16_MIN;
        }
    }
    void setAngleOffsetDeg(float deg) {
        deg = std::round(deg * 10);
        if (INT16_MAX >= deg && deg >= -INT16_MAX) {
            angle_offset = deg;
        } else {
            angle_offset = INT16_MIN;
        }
    }
    bool angleOffsetValid() const {
        return angle_offset != INT16_MIN;
    }
    float angleOffsetDeg() const {
        if (angleOffsetValid()) {
            return angle_offset * 0.1f;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgSteerOffset));

struct MsgUlcCmd {
    static constexpr uint32_t ID = 0x284;
    static constexpr size_t PERIOD_MS = 20;
    static constexpr size_t TIMEOUT_MS = 100;
    enum class CmdType : uint8_t {
        None = 0,
        Velocity = 1, // 0.0025 m/s
        Accel = 2,    // 0.0005 m/s^2
    };
    enum class CoastDecel : uint8_t {
        UseBrakes = 0, // Use brakes to slow down
        NoBrakes = 1,  // Coast, don't use brakes
    };
    int16_t cmd; // Interpretation changes with cmd_type
    CmdType cmd_type :3;
    uint8_t :1;
    uint8_t enable :1;
    uint8_t clear :1;
    uint8_t :2;
    uint8_t enable_shift :1;
    uint8_t enable_shift_park :1;
    CoastDecel coast_decel :1;
    uint8_t :5;
    uint8_t :8;
    uint8_t :8;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCmdVelocityMps(float velocity_m_s) {
        cmd = std::clamp<float>(std::round(velocity_m_s / 0.0025f), -INT16_MAX, INT16_MAX);
    }
    void setCmdAccelMps(float accel_m_s) {
        cmd = std::clamp<float>(std::round(accel_m_s / 0.0005f), -INT16_MAX, INT16_MAX);
    }
    float cmdVelocityMps() const {
        return std::clamp(cmd * 0.0025f, -7.0f, 45.0f);
    }
    float cmdAccelMps() const {
        return std::clamp(cmd * 0.0005f, -6.0f, 3.0f);
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgUlcCmd));
struct MsgUlcCfg {
    static constexpr uint32_t ID = 0x285;
    static constexpr size_t PERIOD_MS = 200;
    static constexpr size_t TIMEOUT_MS = 1000;
    uint8_t limit_accel;         // 0.025 m/s^2, 0.3 to 3.0 m/s^2
    uint8_t limit_decel;         // 0.025 m/s^2, 0.3 to 6.0 m/s^2
    uint8_t limit_jerk_throttle; // 0.1 m/s^3, 1.0 to 25.5 m/s^3
    uint8_t limit_jerk_brake;    // 0.1 m/s^3, 1.0 to 25.5 m/s^3
    uint8_t :8;
    uint8_t :8;
    uint8_t :4;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    bool operator==(const MsgUlcCfg& _other) const {
        return memcmp(this, &_other, sizeof(*this)) == 0;
    }
    bool operator!=(const MsgUlcCfg& _other) const {
        return !(*this == _other);
    }
    void setLimitAccelMps(float accel_m_s2) {
        if (accel_m_s2 < 0 || std::isinf(accel_m_s2)) {
            limit_accel = UINT8_MAX; // Maximum
        } else if (accel_m_s2 > 0) {
            limit_accel = std::clamp<float>(std::round(accel_m_s2 * 40), 1, UINT8_MAX - 1);
        } else {
            limit_accel = 0; // Default
        }
    }
    void setLimitDecelMps(float decel_m_s2) {
        if (decel_m_s2 < 0 || std::isinf(decel_m_s2)) {
            limit_decel = UINT8_MAX; // Maximum
        } else if (decel_m_s2 > 0) {
            limit_decel = std::clamp<float>(std::round(decel_m_s2 * 40), 1, UINT8_MAX - 1);
        } else {
            limit_decel = 0; // Default
        }
    }
    void setLimitJerkThrottleMps(float jerk_m_s3) {
        if (jerk_m_s3 < 0 || std::isinf(jerk_m_s3)) {
            limit_jerk_throttle = UINT8_MAX; // Maximum
        } else if (jerk_m_s3 > 0) {
            limit_jerk_throttle = std::clamp<float>(std::round(jerk_m_s3 * 10), 1, UINT8_MAX - 1);
        } else {
            limit_jerk_throttle = 0; // Default
        }
    }
    void setLimitJerkBrakeMps(float jerk_m_s3) {
        if (jerk_m_s3 < 0 || std::isinf(jerk_m_s3)) {
            limit_jerk_brake = UINT8_MAX; // Maximum
        } else if (jerk_m_s3 > 0) {
            limit_jerk_brake = std::clamp<float>(std::round(jerk_m_s3 * 10), 1, UINT8_MAX - 1);
        } else {
            limit_jerk_brake = 0; // Default
        }
    }
    float limitAccelMps() const {
        if (limit_accel == 0) {
            return 0; // Default
        } else {
            return std::clamp(limit_accel * 0.025f, 0.3f, 3.0f);
        }
    }
    float limitDecelMps() const {
        if (limit_decel == 0) {
            return 0; // Default
        } else {
            return std::clamp(limit_decel * 0.025f, 0.3f, 6.0f);
        }
    }
    float limitJerkThrottleMps() const {
        if (limit_jerk_throttle == 0) {
            return 0; // Default
        } else {
            return std::clamp(limit_jerk_throttle * 0.1f, 1.0f, 25.0f);
        }
    }
    float limitJerkBrakeMps() const {
        if (limit_jerk_brake == 0) {
            return 0; // Default
        } else {
            return std::clamp(limit_jerk_brake * 0.1f, 1.0f, 25.0f);
        }
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgUlcCfg));
struct MsgUlcReport {
    static constexpr uint32_t ID = 0x280;
    static constexpr size_t PERIOD_MS = 20;
    static constexpr size_t TIMEOUT_MS = 500;
    int16_t vel_ref :13; // 0.02 m/s,
    MsgUlcCmd::CmdType cmd_type :3;
    int16_t vel_meas :13; // 0.02 m/s
    uint8_t override_active :1;
    uint8_t override_latched :1;
    uint8_t preempted :1;
    int8_t accel_ref;  // 0.05 m/s^2
    int8_t accel_meas; // 0.05 m/s^2
    uint8_t ready :1;
    uint8_t enabled :1;
    MsgUlcCmd::CoastDecel coast_decel :1;
    uint8_t timeout :1;
    uint8_t bad_crc :1;
    uint8_t bad_rc :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setVelocityRefMps(float velocity_m_s) {
        if (std::isfinite(velocity_m_s)) {
            vel_ref = std::clamp<float>(velocity_m_s * 50, -(INT16_MAX >> 3), INT16_MAX >> 3);
        } else {
            vel_ref = INT16_MIN >> 3;
        }
    }
    bool velocityRefValid() const {
        return vel_ref != INT16_MIN >> 3;
    }
    float velocityRefMps() const {
        if (velocityRefValid()) {
            return vel_ref * 0.02f;
        }
        return NAN;
    }
    void setVelocityMeasMps(float velocity_m_s) {
        if (std::isfinite(velocity_m_s)) {
            vel_meas = std::clamp<float>(velocity_m_s * 50, -(INT16_MAX >> 3), INT16_MAX >> 3);
        } else {
            vel_meas = INT16_MIN >> 3;
        }
    }
    bool velocityMeasValid() const {
        return vel_meas != INT16_MIN >> 3;
    }
    float velocityMeasMps() const {
        if (velocityMeasValid()) {
            return vel_meas * 0.02f;
        }
        return NAN;
    }
    void setAccelRefMps(float accel_m_s2) {
        if (std::isfinite(accel_m_s2)) {
            accel_ref = std::clamp<float>(accel_m_s2 * 20, -INT8_MAX, INT8_MAX);
        } else {
            accel_ref = INT8_MIN;
        }
    }
    bool accelRefValid() const {
        return accel_ref != INT8_MIN;
    }
    float accelRefMps() const {
        if (accelRefValid()) {
            return accel_ref * 0.05f;
        }
        return NAN;
    }
    void setAccelMeasMps(float accel_m_s2) {
        if (std::isfinite(accel_m_s2)) {
            accel_meas = std::clamp<float>(accel_m_s2 * 20, -INT8_MAX, INT8_MAX);
        } else {
            accel_meas = INT8_MIN;
        }
    }
    bool accelMeasValid() const {
        return accel_meas != INT8_MIN;
    }
    int8_t accelMeasMpsx20() const {
        return accel_meas;
    }
    float accelMeasMps() const {
        if (accelMeasValid()) {
            return accel_meas * 0.05f;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgUlcReport));

struct MsgAccel {
    static constexpr uint32_t ID = 0x2A0;
    static constexpr size_t PERIOD_MS = 10;
    static constexpr size_t TIMEOUT_MS = 200;
    int16_t x; // 0.01 m/s^2, forward positive, backward negative
    int16_t y; // 0.01 m/s^2, left positive, right negative
    int16_t z; // 0.01 m/s^2, up positive, down negative
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        x = UNKNOWN;
        y = UNKNOWN;
        z = UNKNOWN;
        rc = save;
    }
    void setAccelXMps2(float m_s2) { setAccelMps2(x, m_s2); }
    void setAccelYMps2(float m_s2) { setAccelMps2(y, m_s2); }
    void setAccelZMps2(float m_s2) { setAccelMps2(z, m_s2); }
    bool accelXValid() const { return accelValid(x); }
    bool accelYValid() const { return accelValid(y); }
    bool accelZValid() const { return accelValid(z); }
    float accelXMps2() const { return accelMps2(x); }
    float accelYMps2() const { return accelMps2(y); }
    float accelZMps2() const { return accelMps2(z); }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
private:
    static constexpr int16_t UNKNOWN = INT16_MIN;
    static constexpr int16_t MAX = INT16_MAX;
    static void setAccelMps2(int16_t &u, float m_s2) {
        if (std::isfinite(m_s2)) {
            u = std::clamp<float>(m_s2 * 100, -MAX, MAX);
        } else {
            u = UNKNOWN;
        }
    }
    static bool accelValid(const int16_t &u) {
        return u != UNKNOWN;
    }
    static float accelMps2(const int16_t &u) {
        if (accelValid(u)) {
            return u * 0.01f;
        }
        return NAN;
    }
};
static_assert(8 == sizeof(MsgAccel));

struct MsgGyro {
    static constexpr uint32_t ID = 0x2A1;
    static constexpr size_t PERIOD_MS = 10;
    static constexpr size_t TIMEOUT_MS = 200;
    int16_t x; // 0.0002 rad/s, roll, right positive, left negative
    int16_t y; // 0.0002 rad/s, pitch, down positive, up negative
    int16_t z; // 0.0002 rad/s, yaw, left positive, right negative
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        x = UNKNOWN;
        y = UNKNOWN;
        z = UNKNOWN;
        rc = save;
    }
    void setGyroXRadS(float rad_s) { setGyroRadS(x, rad_s); }
    void setGyroYRadS(float rad_s) { setGyroRadS(y, rad_s); }
    void setGyroZRadS(float rad_s) { setGyroRadS(z, rad_s); }
    bool gyroXValid() const { return gyroValid(x); }
    bool gyroYValid() const { return gyroValid(y); }
    bool gyroZValid() const { return gyroValid(z); }
    float gyroXRadS() const { return gyroRadS(x); }
    float gyroXDegS() const { return gyroDegS(x); }
    float gyroYRadS() const { return gyroRadS(y); }
    float gyroYDegS() const { return gyroDegS(y); }
    float gyroZRadS() const { return gyroRadS(z); }
    float gyroZDegS() const { return gyroDegS(z); }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
private:
    static constexpr int16_t UNKNOWN = INT16_MIN;
    static constexpr int16_t MAX = INT16_MAX;
    static void setGyroRadS(int16_t &u, float rad_s) {
        if (std::isfinite(rad_s)) {
            u = std::clamp<float>(rad_s * 5000, -MAX, MAX);
        } else {
            u = UNKNOWN;
        }
    }
    static bool gyroValid(const int16_t &u) {
        return u != UNKNOWN;
    }
    static float gyroRadS(const int16_t &u) {
        if (gyroValid(u)) {
            return u * 0.0002f;
        }
        return NAN;
    }
    static float gyroDegS(const int16_t &u) {
        return gyroRadS(u) * (float)(180 / M_PI);
    }
};
static_assert(8 == sizeof(MsgGyro));

struct MsgWheelSpeed {
    static constexpr uint32_t ID = 0x2A4;
    static constexpr size_t PERIOD_MS = 10;
    static constexpr size_t TIMEOUT_MS = 100;
    static constexpr int16_t UNKNOWN = INT16_MIN;
    static constexpr int16_t MAX = INT16_MAX;
    int16_t front_left;  // 0.01 rad/s
    int16_t front_right; // 0.01 rad/s
    int16_t rear_left;   // 0.01 rad/s
    int16_t rear_right;  // 0.01 rad/s
    void setFrontLeftRadS(float rad_s) { setSpeedRadS(front_left, rad_s); }
    void setFrontRightRadS(float rad_s) { setSpeedRadS(front_right, rad_s); }
    void setRearLeftRadS(float rad_s) { setSpeedRadS(rear_left, rad_s); }
    void setRearRightRadS(float rad_s) { setSpeedRadS(rear_right, rad_s); }
    bool frontLeftValid() const { return speedValid(front_left); }
    bool frontRightValid() const { return speedValid(front_right); }
    bool rearLeftValid() const { return speedValid(rear_left); }
    bool rearRightValid() const { return speedValid(rear_right); }
    float frontLeftRadS() const { return speedRadS(front_left); }
    float frontLeftDegS() const { return speedDegS(front_left); }
    float frontRightRadS() const { return speedRadS(front_right); }
    float frontRightDegS() const { return speedDegS(front_right); }
    float rearLeftRadS() const { return speedRadS(rear_left); }
    float rearLeftDegS() const { return speedDegS(rear_left); }
    float rearRightRadS() const { return speedRadS(rear_right); }
    float rearRightDegS() const { return speedDegS(rear_right); }
    void reset() {
        front_left = UNKNOWN;
        front_right = UNKNOWN;
        rear_left = UNKNOWN;
        rear_right = UNKNOWN;
    }
private:
    static void setSpeedRadS(int16_t &x, float rad_s) {
        if (std::isfinite(rad_s)) {
            x = std::clamp<float>(rad_s * 100, -MAX, MAX);
        } else {
            x = UNKNOWN;
        }
    }
    static bool speedValid(const int16_t &x) {
        return x != UNKNOWN;
    }
    static float speedRadS(const int16_t &x) {
        if (speedValid(x)) {
            return x * 0.01f;
        }
        return NAN;
    }
    static float speedDegS(const int16_t &x) {
        return speedRadS(x) * (float)(180 / M_PI);
    }
};
static_assert(8 == sizeof(MsgWheelSpeed));

struct MsgWheelPosition {
    static constexpr uint32_t ID = 0x2A5;
    static constexpr size_t PERIOD_MS = 20;
    static constexpr size_t TIMEOUT_MS = 250;
    int16_t front_left;
    int16_t front_right;
    int16_t rear_left;
    int16_t rear_right;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
};
static_assert(8 == sizeof(MsgWheelPosition));

struct MsgTurnSignalCmd {
    static constexpr uint32_t ID = 0x2C1;
    static constexpr size_t PERIOD_MS = 50;
    static constexpr size_t TIMEOUT_MS = 250;
    TurnSignal cmd :2;
    uint8_t :2;
    uint8_t rc :4;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(2 == sizeof(MsgTurnSignalCmd));

struct MsgTurnSignalReport {
    static constexpr uint32_t ID = 0x2C2;
    static constexpr size_t PERIOD_MIN =  20;
    static constexpr size_t PERIOD_MS  = 100;
    static constexpr size_t PERIOD_MAX = 125;
    static constexpr size_t TIMEOUT_MS = 400;
    TurnSignal input :2;
    TurnSignal cmd :2;
    TurnSignal output :2;
    TurnSignal feedback :2;
    uint8_t fault_comms_vehicle :1;
    uint8_t :7;
    uint8_t degraded_cmd_type :1;
    uint8_t degraded_comms_dbw_steer :1;
    uint8_t degraded_comms_dbw_brake :1;
    uint8_t degraded_comms_dbw_thrtl :1;
    uint8_t degraded_comms_vehicle :1;
    uint8_t degraded_control_performance :1;
    uint8_t :2;
    uint8_t :5;
    uint8_t override_active :1;
    uint8_t override_other :1;
    uint8_t :1;
    uint8_t ready :1;
    uint8_t degraded :1;
    uint8_t fault :1;
    uint8_t timeout :1;
    uint8_t bad_crc :1;
    uint8_t bad_rc :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(6 == sizeof(MsgTurnSignalReport));

struct MsgMiscCmd {
    static constexpr uint32_t ID = 0x2C0;
    static constexpr size_t PERIOD_MS = 50;
    static constexpr size_t TIMEOUT_MS = 250;
    enum class PrkBrkCmd : uint8_t {
        None = 0,
        On = 1,
        Off = 2,
    };
    enum class DoorSelect : uint8_t {
        None = 0,
        Left = 1,
        Right = 2,
        Trunk = 3,
    };
    enum class DoorCmd : uint8_t {
        None = 0,
        Open = 1,
        Close = 2,
    };
    uint8_t :2; // Previously turn_signal_cmd
    PrkBrkCmd parking_brake_cmd :2;
    DoorSelect door_select :2;
    DoorCmd door_cmd :2;
    uint8_t :8;
    uint8_t :8;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(4 == sizeof(MsgMiscCmd));

struct MsgMiscReport1 {
    static constexpr uint32_t ID = 0x2C4;
    static constexpr size_t PERIOD_MS = 50;
    static constexpr size_t TIMEOUT_MS = 200;
    enum class PrkBrkStat : uint8_t {
        Unknown = 0,
        On = 1,
        Off = 2,
        Transition = 3,
    };
    uint8_t :2; // Previously turn_signal
    PrkBrkStat parking_brake :2;
    uint8_t pasngr_detect :1;
    uint8_t pasngr_airbag :1;
    uint8_t buckle_driver :1;
    uint8_t buckle_pasngr :1;
    uint8_t door_driver :1;
    uint8_t door_passenger :1;
    uint8_t door_rear_left :1;
    uint8_t door_rear_right :1;
    uint8_t door_hood :1;
    uint8_t door_trunk :1;
    uint8_t :2;
    uint8_t btn_ld_ok :1;
    uint8_t btn_ld_up :1;
    uint8_t btn_ld_down :1;
    uint8_t btn_ld_left :1;
    uint8_t btn_ld_right :1;
    uint8_t :1;
    uint8_t btn_rd_ok :1;
    uint8_t btn_rd_up :1;
    uint8_t btn_rd_down :1;
    uint8_t btn_rd_left :1;
    uint8_t btn_rd_right :1;
    uint8_t :1;
    uint8_t btn_cc_mode :1;
    uint8_t btn_cc_on :1;
    uint8_t btn_cc_off :1;
    uint8_t btn_cc_res :1;
    uint8_t btn_cc_cncl :1;
    uint8_t btn_cc_on_off :1;
    uint8_t btn_cc_res_cncl :1;
    uint8_t btn_cc_res_inc :1;
    uint8_t btn_cc_res_dec :1;
    uint8_t btn_cc_set_inc :1;
    uint8_t btn_cc_set_dec :1;
    uint8_t btn_acc_gap_inc :1;
    uint8_t btn_acc_gap_dec :1;
    uint8_t btn_limit_on_off :1;
    uint8_t btn_la_on_off :1;
    uint8_t btn_apa :1;
    uint8_t btn_media :1;
    uint8_t btn_vol_inc :1;
    uint8_t btn_vol_dec :1;
    uint8_t btn_mute :1;
    uint8_t btn_speak :1;
    uint8_t btn_prev :1;
    uint8_t btn_next :1;
    uint8_t btn_call_start :1;
    uint8_t btn_call_end :1;
    uint8_t :1;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgMiscReport1));

struct MsgMiscReport2 {
    static constexpr uint32_t ID = 0x2C5;
    static constexpr size_t PERIOD_MS = 50;
    static constexpr size_t TIMEOUT_MS = 200;
    enum class HeadlightCtrlLow : uint8_t {
        Unknown = 0,
        Off = 1,
        On = 2,
        Auto = 3,
        Park = 4,
    };
    enum class HeadlightCtrlHigh : uint8_t {
        Unknown = 0,
        Off = 1,
        On = 2,
        Auto = 3,
        Flash = 4,
    };
    enum class WiperFront : uint8_t {
        Unknown = 0,
        Off = 1,
        MovingOff = 2,
        ManualOff = 3,
        ManualOn = 4,
        ManualLow = 5,
        ManualHigh = 6,
        AutoOff = 7,
        AutoLow = 8,
        AutoHigh = 9,
        AutoAdjust = 10,
        MistFlick = 11,
        Wash = 12,
        CourtesyWipe = 13,
        Stalled = 14,
    };
    enum class AmbientLight : uint8_t {
        Unknown = 0,
        Dark = 1,
        Medium = 2,
        Light = 3,
    };
    HeadlightCtrlLow headlight_low_control :3;
    uint8_t headlight_low :1;
    HeadlightCtrlHigh headlight_high_control :3;
    uint8_t headlight_high :1;
    WiperFront wiper_front :4;
    uint8_t /*wiper_rear*/ :2;
    AmbientLight ambient_light :2;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t outside_air_temp;
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        outside_air_temp = UINT8_MAX;
        rc = save;
    }
    void setOutsideAirTempDegC(float deg_c) {
        if (std::isfinite(deg_c)) {
            outside_air_temp = std::clamp<float>((deg_c + 40) * 2, 0, UINT8_MAX - 1);
        } else {
            outside_air_temp = UINT8_MAX;
        }
    }
    bool outsideAirTempValid() const {
        return outside_air_temp != UINT8_MAX;
    }
    float outsideAirTempDegC() const {
        if (outsideAirTempValid()) {
            return (outside_air_temp * 0.5f) - 40;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgMiscReport2));

struct MsgDriverAssist {
    static constexpr uint32_t ID = 0x2C8;
    static constexpr size_t PERIOD_MIN =  10;
    static constexpr size_t PERIOD_MAX = 100;
    static constexpr size_t TIMEOUT_MS = 350;
    enum class DecelSrc : uint8_t {
        None = 0,
        AEB = 1,
        ACC = 2,
    };
    uint8_t decel; // 0.05 m/s^2
    DecelSrc decel_src :3;
    uint8_t fcw_active :1;    // Forward Collision Warning
    uint8_t fcw_enabled :1;   // Forward Collision Warning
    uint8_t :3;
    uint8_t aeb_active :1;    // Automated Emergency Braking
    uint8_t aeb_precharge :1; // Automated Emergency Braking
    uint8_t aeb_enabled :1;   // Automated Emergency Braking
    uint8_t acc_braking :1;   // Adaptive Cruise Control
    uint8_t acc_enabled :1;   // Adaptive Cruise Control
    uint8_t :3;
    uint8_t blis_l_alert :1;   // Blind Spot Information System
    uint8_t blis_l_enabled :1; // Blind Spot Information System
    uint8_t blis_r_alert :1;   // Blind Spot Information System
    uint8_t blis_r_enabled :1; // Blind Spot Information System
    uint8_t cta_l_alert :1;    // Cross Traffic Alert
    uint8_t cta_l_enabled :1;  // Cross Traffic Alert
    uint8_t cta_r_alert :1;    // Cross Traffic Alert
    uint8_t cta_r_enabled :1;  // Cross Traffic Alert
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    void setDecelMps2(float m_s2, DecelSrc src) {
        if (src != DecelSrc::None && std::isfinite(m_s2)) {
            decel = std::clamp<float>(std::round(m_s2 / 0.05f), 0u, UINT8_MAX);
            decel_src = src;
        } else {
            decel = 0;
            decel_src = DecelSrc::None;
        }
    }
    float decelMps2() const {
        if (decel_src != DecelSrc::None) {
            return decel * 0.05f;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
    bool operator==(const MsgDriverAssist& _other) const {
        return memcmp(this, &_other, sizeof(*this)) == 0;
    }
    bool operator!=(const MsgDriverAssist& _other) const {
        return !(*this == _other);
    }
};
static_assert(6 == sizeof(MsgDriverAssist));

struct MsgBattery {
    static constexpr uint32_t ID = 0x2C9;
    static constexpr size_t PERIOD_MS = 100;
    static constexpr size_t TIMEOUT_MS = 350;
    enum class Ignition : uint8_t {
        Unknown = 0,
        Off = 1,
        Accessory = 2,
        Run = 3,
        Start = 4,
    };
    uint16_t soc :10; // 0.1 %
    Ignition ignition :3;
    uint16_t voltage :11; // 0.02 V
    int16_t current :14; // 0.0625 A
    uint16_t :2;
    uint8_t temperature; // degC
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        soc = UINT16_MAX >> 6;
        ignition = Ignition::Unknown;
        voltage = UINT16_MAX >> 5;
        current = INT16_MIN >> 2;
        temperature = UINT8_MAX;
        rc = save;
    }
    void setSocPercent(float pc) {
        if (std::isfinite(pc)) {
            soc = std::clamp<float>(pc / 0.1f, 0, (UINT16_MAX >> 6) - 1);
        } else {
            soc = UINT16_MAX >> 6;
        }
    }
    bool socValid() const {
        return soc != UINT16_MAX >> 6;
    }
    float socPercent() const {
        if (socValid()) {
            return soc * 0.1f;
        }
        return NAN;
    }
    void setVoltageVolts(float volts) {
        if (std::isfinite(volts)) {
            voltage = std::clamp<float>(volts / 0.02f, 0, (UINT16_MAX >> 5) - 1);
        } else {
            voltage = UINT16_MAX >> 5;
        }
    }
    bool voltageValid() const {
        return voltage != UINT16_MAX >> 5;
    }
    float voltageVolts() const {
        if (voltageValid()) {
            return voltage * 0.02f;
        }
        return NAN;
    }
    void setCurrentAmps(float amps) {
        if (std::isfinite(amps)) {
            current = std::clamp<float>(amps / 0.0625f, -(INT16_MAX >> 2), INT16_MAX >> 2);
        } else {
            current = INT16_MIN >> 2;
        }
    }
    bool currentValid() const {
        return current != INT16_MIN >> 2;
    }
    float currentAmps() const {
        if (currentValid()) {
            return current * 0.0625f;
        }
        return NAN;
    }
    void setTemperatureDegC(float deg_c) {
        if (std::isfinite(deg_c)) {
            temperature = std::clamp<float>(deg_c + 40, 0, UINT8_MAX - 1);
        } else {
            temperature = UINT8_MAX;
        }
    }
    bool temperatureValid() const {
        return temperature != UINT8_MAX;
    }
    float temperatureDegC() const {
        if (temperatureValid()) {
            return temperature - 40;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgBattery));

struct MsgBatteryTraction {
    static constexpr uint32_t ID = 0x2CA;
    static constexpr size_t PERIOD_MS = 100;
    static constexpr size_t TIMEOUT_MS = 350;
    enum class ChargeStatus : uint8_t {
        Unknown = 0,
        NotCharging = 1,
        Charging = 2,
        Complete = 3,
        Fault = 7,
    };
    uint16_t soc :10; // 0.1 %
    ChargeStatus status :3;
    uint16_t voltage :11; // 0.5 V
    int16_t current :14; // 0.1 A
    uint8_t :2;
    uint8_t temperature; // degC
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        soc = UINT16_MAX >> 6;
        voltage = UINT16_MAX >> 5;
        current = INT16_MIN >> 2;
        temperature = UINT8_MAX;
        rc = save;
    }
    void setSocPercent(float pc) {
        if (std::isfinite(pc)) {
            soc = std::clamp<float>(pc / 0.1f, 0, (UINT16_MAX >> 6) - 1);
        } else {
            soc = UINT16_MAX >> 6;
        }
    }
    bool socValid() const {
        return soc != UINT16_MAX >> 6;
    }
    float socPercent() const {
        if (socValid()) {
            return soc * 0.1f;
        }
        return NAN;
    }
    void setVoltageVolts(float volts) {
        if (std::isfinite(volts)) {
            voltage = std::clamp<float>(volts / 0.5f, 0, (UINT16_MAX >> 5) - 1);
        } else {
            voltage = UINT16_MAX >> 5;
        }
    }
    bool voltageValid() const {
        return voltage != UINT16_MAX >> 5;
    }
    float voltageVolts() const {
        if (voltageValid()) {
            return voltage * 0.5f;
        }
        return NAN;
    }
    void setCurrentAmps(float amps) {
        if (std::isfinite(amps)) {
            current = std::clamp<float>(amps / 0.1f, -(INT16_MAX >> 2), INT16_MAX >> 2);
        } else {
            current = INT16_MIN >> 2;
        }
    }
    bool currentValid() const {
        return current != INT16_MIN >> 2;
    }
    float currentAmps() const {
        if (currentValid()) {
            return current * 0.1f;
        }
        return NAN;
    }
    void setTemperatureDegC(float deg_c) {
        if (std::isfinite(deg_c)) {
            temperature = std::clamp<float>(deg_c + 40, 0, UINT8_MAX - 1);
        } else {
            temperature = UINT8_MAX;
        }
    }
    bool temperatureValid() const {
        return temperature != UINT8_MAX;
    }
    float temperatureDegC() const {
        if (temperatureValid()) {
            return temperature - 40;
        }
        return NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgBatteryTraction));

struct MsgEyeTracker {
    static constexpr uint32_t ID = 0x2CB;
    static constexpr size_t PERIOD_MS = 200;
    static constexpr size_t TIMEOUT_MS = 500;
    enum class Gaze : uint8_t {
        Other               = 0x0,
        PassengerWindshield = 0x1,
        DriverWindshield    = 0x2,
        InstrumentPanel     = 0x3,
        MediaTablet         = 0x4,
        RearMirror          = 0x6,
        DriverMirror        = 0x8,
        PassengerMirror     = 0x9,
        BelowDashboard      = 0xA,
    };
    uint8_t attention_pc; // 0.4 %
    Gaze gaze :4;
    uint8_t :3;
    uint8_t eyes_present :1;
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void setAttentionPercent(float pc) {
        if (std::isfinite(pc)) {
            attention_pc = std::clamp<float>(pc / 0.4f, 0, UINT8_MAX - 1);
        } else {
            attention_pc = UINT8_MAX;
        }
    }
    float attentionValid() const {
        return attention_pc != UINT8_MAX;
    }
    float attentionPercent() const {
        if (attentionValid()) {
            return attention_pc * 0.4f;
        } else {
            return NAN;
        }
    }
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        attention_pc = UINT8_MAX;
        rc = save;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(4 == sizeof(MsgEyeTracker));

struct MsgReserved1 {
    static constexpr uint32_t ID = 0x360;
    static constexpr size_t TIMEOUT_MS = 500;
    uint8_t reserved[8];
};
static_assert(8 == sizeof(MsgReserved1));
struct MsgReserved2 {
    static constexpr uint32_t ID = 0x361;
    static constexpr size_t TIMEOUT_MS = 5000;
    uint8_t reserved[8];
};
static_assert(8 == sizeof(MsgReserved2));
struct MsgReservedDebug {
    static constexpr uint32_t ID = 0x36F;
    static constexpr size_t TIMEOUT_MS = 1000;
    int16_t nom;
    int16_t min;
    int16_t max;
    uint16_t ms :15;
    uint16_t fault :1;
};
static_assert(8 == sizeof(MsgReservedDebug));

struct MsgTirePressure {
    static constexpr uint32_t ID = 0x380;
    static constexpr size_t PERIOD_MS = 500;
    static constexpr size_t TIMEOUT_MS = 2500;
    static constexpr uint16_t UNKNOWN = UINT16_MAX >> 4;
    static constexpr uint16_t MAX = UNKNOWN - 1;
    uint16_t front_left  :12; // kPa
    uint16_t front_right :12; // kPa
    uint16_t rear_left   :12; // kPa
    uint16_t rear_right  :12; // kPa
    uint16_t spare       :12; // kPa
    uint16_t :4;
    void setFrontLeftKPa(int kpa) { front_left = setPressureKPa(kpa); }
    void setFrontRightKPa(int kpa) { front_right = setPressureKPa(kpa); }
    void setRearLeftKPa(int kpa) { rear_left = setPressureKPa(kpa); }
    void setRearRightKPa(int kpa) { rear_right = setPressureKPa(kpa); }
    void setSpareKPa(int kpa) { spare = setPressureKPa(kpa); }
    void setFrontLeftKPa(float kpa) { front_left = setPressureKPa(kpa); }
    void setFrontRightKPa(float kpa) { front_right = setPressureKPa(kpa); }
    void setRearLeftKPa(float kpa) { rear_left = setPressureKPa(kpa); }
    void setRearRightKPa(float kpa) { rear_right = setPressureKPa(kpa); }
    void setSpareKPa(float kpa) { spare = setPressureKPa(kpa); }
    bool frontLeftValid() const { return pressureValid(front_left); }
    bool frontRightValid() const { return pressureValid(front_right); }
    bool rearLeftValid() const { return pressureValid(rear_left); }
    bool rearRightValid() const { return pressureValid(rear_right); }
    bool spareValid() const { return pressureValid(spare); }
    float frontLeftKPa() const { return pressureKPa(front_left); }
    float frontRightKPa() const { return pressureKPa(front_right); }
    float rearLeftKPa() const { return pressureKPa(rear_left); }
    float rearRightKPa() const { return pressureKPa(rear_right); }
    float spareKPa() const { return pressureKPa(spare); }
    void reset() {
        memset(this, 0x00, sizeof(*this));
        front_left = UNKNOWN;
        front_right = UNKNOWN;
        rear_left = UNKNOWN;
        rear_right = UNKNOWN;
        spare = UNKNOWN;
    }
private:
    static uint16_t setPressureKPa(int kpa) {
        if (kpa >= 0) {
            return std::clamp<int>(kpa, 0, MAX);
        } else {
            return UNKNOWN;
        }
    }
    static uint16_t setPressureKPa(float kpa) {
        if (std::isfinite(kpa)) {
            return std::clamp<float>(kpa, 0, MAX);
        } else {
            return UNKNOWN;
        }
    }
    static bool pressureValid(const uint16_t &x) {
        return x != UNKNOWN;
    }
    static float pressureKPa(const uint16_t &x) {
        if (pressureValid(x)) {
            return x;
        }
        return NAN;
    }
};
static_assert(8 == sizeof(MsgTirePressure));

struct MsgFuelLevel {
    static constexpr uint32_t ID = 0x381;
    static constexpr size_t PERIOD_MS = 500;
    static constexpr size_t TIMEOUT_MS = 2500;
    uint16_t fuel_level :10; // 0.1 %
    uint8_t :3;
    uint16_t fuel_range :11; // km
    uint32_t odometer :24; // 0.1 km
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        fuel_level = UINT16_MAX >> 6;
        fuel_range = UINT16_MAX >> 5;
        odometer = UINT32_MAX >> 8;
        rc = save;
    }
    void setFuelLevelPercent(float pc) {
        if (std::isfinite(pc)) {
            fuel_level = std::clamp<float>(pc / 0.1f, 0, (UINT16_MAX >> 6) - 1);
        } else {
            fuel_level = UINT16_MAX >> 6;
        }
    }
    float fuelLevelPercent() const {
        return fuel_level != UINT16_MAX >> 6 ? fuel_level * 0.1f : NAN;
    }
    void setFuelRangeKm(uint16_t km) {
        if (km != UINT16_MAX) {
            fuel_range = std::clamp<uint16_t>(km, 0, (UINT16_MAX >> 5) - 1);
        } else {
            fuel_range = UINT16_MAX >> 5;
        }
    }
    void setFuelRangeKm(float km) {
        if (std::isfinite(km)) {
            fuel_range = std::clamp<float>(km, 0, (UINT16_MAX >> 5) - 1);
        } else {
            fuel_range = UINT16_MAX >> 5;
        }
    }
    float fuelRangeKm() const {
        return fuel_range != UINT16_MAX >> 5 ? fuel_range : NAN;
    }
    void setOdometerKmX10(uint32_t km) {
        if (km != UINT32_MAX) {
            odometer = std::clamp<uint32_t>(km, 0, (UINT32_MAX >> 8) - 1);
        } else {
            odometer = UINT32_MAX >> 8;
        }
    }
    void setOdometerKm(float km) {
        if (std::isfinite(km)) {
            odometer = std::clamp<float>(km * 10, 0, (UINT32_MAX >> 8) - 1);
        } else {
            odometer = UINT32_MAX >> 8;
        }
    }
    bool odometerValid() const {
        return odometer != UINT32_MAX >> 8;
    }
    float odometerKm() const {
        return odometerValid() ? odometer * 0.1f : NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(8 == sizeof(MsgFuelLevel));

struct MsgTrafficSignInfo {
    static constexpr uint32_t ID = 0x382;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 2500;
    enum class Status : uint8_t {
        Unknown = 0,
        Off = 1,
        Active = 2,
        Error = 3,
    };
    enum class Unit : uint8_t {
        Unknown = 0,
        Kph = 1,
        Mph = 2,
    };
    enum class Limit : uint8_t {
        Unknown = 0x00,
        NoLimit = 0xFF,
    };
    Status status :2;
    uint8_t camera_used :1;
    uint8_t navigation_used :1;
    uint8_t :2;
    Unit speed_units :2;
    Limit speed_limit;
    uint8_t :6;
    uint8_t rc :2;
    uint8_t crc;
    void reset() {
        uint8_t save = rc;
        memset(this, 0x00, sizeof(*this));
        rc = save;
    }
    float speedLimit() const {
        switch (speed_limit) {
            case Limit::Unknown: return NAN;
            case Limit::NoLimit: return INFINITY;
            default: return (float)speed_limit;
        }
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validRc(uint8_t rc) const {
        return rc != this->rc;
    }
};
static_assert(4 == sizeof(MsgTrafficSignInfo));

struct MsgGpsLatLong {
    static constexpr uint32_t ID = 0x390;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    int32_t latitude :28;  //  8e-7 deg,  -90 to 90
    int32_t longitude :28; // 16e-7 deg, -180 to 180
    uint8_t crc;
    void reset() {
        latitude = INT32_MIN >> 4;
        longitude = INT32_MIN >> 4;
        crc = 0;
    }
    void setLatitudeDeg(double deg) {
        if (std::isfinite(deg)) {
            latitude = std::clamp<double>(std::round(deg / 8e-7), -(INT32_MAX >> 4), INT32_MAX >> 4);
        } else {
            latitude = INT32_MIN >> 4;
        }
    }
    void setLongitudeDeg(double deg) {
        if (std::isfinite(deg)) {
            longitude = std::clamp<double>(std::round(deg / 16e-7), -(INT32_MAX >> 4), INT32_MAX >> 4);
        } else {
            longitude = INT32_MIN >> 4;
        }
    }
    bool latitudeValid() const {
        return latitude != INT32_MIN >> 4;
    }
    bool longitudeValid() const {
        return longitude != INT32_MIN >> 4;
    }
    double latitudeDeg() const {
        return latitudeValid() ? latitude * 8e-7 : NAN;
    }
    double longitudeDeg() const {
        return longitudeValid() ? longitude * 16e-7 : NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(8 == sizeof(MsgGpsLatLong));

struct MsgGpsAltitude {
    static constexpr uint32_t ID = 0x391;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    int16_t altitude; // 0.25 m
    uint16_t heading; // 0.01 deg
    uint16_t speed :11; // 0.25 kph
    uint8_t num_sats :5; // Number of satellites
    uint8_t :8;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
        altitude = INT16_MIN;
        heading = UINT16_MAX;
        speed = UINT16_MAX >> 5;
        num_sats = 0;
    }
    void setAltitudeM(float m) {
        if (std::isfinite(m)) {
            altitude = std::clamp<float>(std::round(m / 0.25f), -INT16_MAX, INT16_MAX);
        } else {
            altitude = INT16_MIN;
        }
    }
    bool altitudeValid() const {
        return altitude != INT16_MIN;
    }
    float altitudeM() const {
        return altitudeValid() ? altitude * 0.25f : NAN;
    }
    void setHeadingDeg(float deg) {
        if (std::isfinite(deg)) {
            heading = std::clamp<float>(std::round(fmodPos(deg, 360) / 0.01f), 0, UINT16_MAX - 1);
        } else {
            heading = UINT16_MAX;
        }
    }
    bool headingValid() const {
        return heading != UINT16_MAX;
    }
    float headingDeg() const {
        return headingValid() ? heading * 0.01f : NAN;
    }
    void setSpeedKph(float kph) {
        if (std::isfinite(kph)) {
            speed = std::clamp<float>(std::round(kph / 0.25f), 0, (UINT16_MAX >> 5) - 1);
        } else {
            speed = UINT16_MAX >> 5;
        }
    }
    bool speedValid() const {
        return speed != UINT16_MAX >> 5;
    }
    float speedKph() const {
        return speedValid() ? speed * 0.25f : NAN;
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
private:
    static constexpr float fmodPos(float a, float b) {
        float x = std::fmod(a, b);
        return x >= 0 ? x : x + b;
    }
};
static_assert(8 == sizeof(MsgGpsAltitude));

struct MsgGpsTime {
    static constexpr uint32_t ID = 0x392;
    static constexpr size_t PERIOD_MS = 1000;
    static constexpr size_t TIMEOUT_MS = 3500;
    uint8_t utc_year_2000 :7; // Years since 2000
    uint8_t valid :1;
    uint8_t utc_month :4; // 0-11 -> 1-12
    uint8_t :4;
    uint8_t utc_day :5; // 0-30 -> 1-31
    uint8_t :3;
    uint8_t utc_hours :5; // 0-23
    uint8_t :3;
    uint8_t utc_minutes :6; // 0-59
    uint8_t :2;
    uint8_t utc_seconds :6; // 0-59
    uint8_t :2;
    uint8_t :8;
    uint8_t crc;
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
    void setCrc() {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        crc = crc8(ID, this, offsetof(typeof(*this), crc));
    }
    bool validCrc() const {
        static_assert(crc8(ID, MSG_NULL, offsetof(typeof(*this), crc)) != 0x00);
        return crc == crc8(ID, this, offsetof(typeof(*this), crc));
    }
};
static_assert(8 == sizeof(MsgGpsTime));

struct MsgEcuInfo {
    static constexpr size_t PERIOD_MS = 200;
    enum class Mux : uint8_t {
        Version      = 0x00,
        CfgHash      = 0x04,
        MacAddr      = 0x08,
        License0     = 0x10, // Feature 0
        License1     = 0x11, // Feature 1
        License2     = 0x12, // Feature 2
        License3     = 0x13, // Feature 3
        License4     = 0x14, // Feature 4
        License5     = 0x15, // Feature 5
        License6     = 0x16, // Feature 6
        License7     = 0x17, // Feature 7
        LicenseDate0 = 0x40,
        LicenseDate1 = 0x41,
        BuildDate0   = 0x48,
        BuildDate1   = 0x49,
        VIN0         = 0x50,
        VIN1         = 0x51,
        VIN2         = 0x52,
        Logging      = 0x60,
    };
    Mux mux;
    union {
        struct {
            uint8_t platform;
            uint16_t major;
            uint16_t minor;
            uint16_t build;
        } version;
        struct {
            uint8_t count_modified;
            uint8_t count_configured;
            uint8_t nvm_blank :1;
            uint8_t nvm_write_pending :1;
            uint8_t :6;
            uint32_t hash;
        } cfg;
        struct {
            uint8_t addr0;
            uint8_t addr1;
            uint8_t addr2;
            uint8_t addr3;
            uint8_t addr4;
            uint8_t addr5;
            uint8_t :8;
        } mac;
        struct {
            uint8_t ready :1;
            uint8_t enabled :1;
            uint8_t trial :1;
            uint8_t :5;
            uint8_t :8;
            uint8_t :8;
            uint16_t trials_used;
            uint16_t trials_left;
        } license;
        struct {
            uint8_t date0;
            uint8_t date1;
            uint8_t date2;
            uint8_t date3;
            uint8_t date4;
            uint8_t date5;
            uint8_t date6;
        } ldate0;
        struct {
            uint8_t date7;
            uint8_t date8;
            uint8_t date9;
            uint8_t :8;
            uint8_t :8;
            uint8_t :8;
            uint8_t :8;
        } ldate1;
        struct {
            uint8_t date0;
            uint8_t date1;
            uint8_t date2;
            uint8_t date3;
            uint8_t date4;
            uint8_t date5;
            uint8_t date6;
        } bdate0;
        struct {
            uint8_t date7;
            uint8_t date8;
            uint8_t date9;
            uint8_t :8;
            uint8_t :8;
            uint8_t :8;
            uint8_t :8;
        } bdate1;
        struct {
            uint8_t vin00;
            uint8_t vin01;
            uint8_t vin02;
            uint8_t vin03;
            uint8_t vin04;
            uint8_t vin05;
            uint8_t vin06;
        } vin0;
        struct {
            uint8_t vin07;
            uint8_t vin08;
            uint8_t vin09;
            uint8_t vin10;
            uint8_t vin11;
            uint8_t vin12;
            uint8_t vin13;
        } vin1;
        struct {
            uint8_t vin14;
            uint8_t vin15;
            uint8_t vin16;
            uint8_t :8;
            uint8_t :8;
            uint8_t :8;
            uint8_t :8;
        } vin2;
        struct {
            uint8_t fault :1;
            uint8_t filesystem :1;
            uint8_t :6;
            uint8_t :8;
            uint8_t :8;
            uint32_t :8;
            uint32_t filename :24; // "000000.dbw", 0xFFFFFF for None
            bool validFilename() const {
                return filename != 0xFFFFFFu;
            }
        } logging;
    };
    void reset() {
        memset(this, 0x00, sizeof(*this));
    }
};
static_assert(8 == sizeof(MsgEcuInfo));
struct MsgEcuInfoGateway  : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C0; };
struct MsgEcuInfoSteer    : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C1; };
struct MsgEcuInfoBrake    : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C2; };
struct MsgEcuInfoThrottle : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C3; };
struct MsgEcuInfoShift    : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C4; };
struct MsgEcuInfoBOO      : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C5; };
struct MsgEcuInfoMonitor  : public MsgEcuInfo { static constexpr uint32_t ID = 0x6C6; };


#pragma pack(pop) // Undo packing


// Verify that IDs are unique and in the desired order of priorities (unit test)
static constexpr std::array<uint32_t, 69> IDS {
    // Primary reports
    MsgSteerReport1::ID,
    MsgBrakeReport1::ID,
    MsgThrtlReport1::ID,
    MsgGearReport1::ID,
    MsgMonitorReport1::ID,
    MsgSystemReport::ID,
    // Primary sensors
    MsgVehicleVelocity::ID,
    MsgThrtlInfo::ID,
    MsgBrakeInfo::ID,
    MsgSteerOffset::ID,
    // Commands (remote control)
    MsgSteerCmdRmt::ID,
    MsgBrakeCmdRmt::ID,
    MsgThrtlCmdRmt::ID,
    MsgGearCmdRmt::ID,
    // Commands (user)
    MsgSteerCmdUsr::ID,
    MsgBrakeCmdUsr::ID,
    MsgThrtlCmdUsr::ID,
    MsgGearCmdUsr::ID,
    MsgMonitorCmd::ID,
    MsgSystemCmd::ID,
    // Commands (ULC)
    MsgBrakeCmdUlc::ID,
    MsgThrtlCmdUlc::ID,
    MsgGearCmdUlc::ID,
    // ULC
    MsgUlcReport::ID,
    MsgUlcCmd::ID,
    MsgUlcCfg::ID,
    // Secondary sensors
    MsgAccel::ID,
    MsgGyro::ID,
    MsgWheelSpeed::ID,
    MsgWheelPosition::ID,
    MsgMonitorThrtl::ID,
    // Misc
    MsgMiscCmd::ID,
    MsgTurnSignalCmd::ID,
    MsgTurnSignalReport::ID,
    MsgMiscReport1::ID,
    MsgMiscReport2::ID,
    MsgDriverAssist::ID,
    MsgBattery::ID,
    MsgBatteryTraction::ID,
    MsgEyeTracker::ID,
    // Secondary reports
    MsgSteerReport2::ID,
    MsgBrakeReport2::ID,
    MsgThrtlReport2::ID,
    MsgGearReport2::ID,
    MsgMonitorReport2::ID,
    MsgSteerReport3::ID,
    MsgBrakeReport3::ID,
    MsgThrtlReport3::ID,
    MsgGearReport3::ID,
    MsgMonitorReport3::ID,
    MsgSteerParamHash::ID,
    MsgBrakeParamHash::ID,
    MsgThrtlParamHash::ID,
    // Reserved
    MsgReserved1::ID,
    MsgReserved2::ID,
    MsgReservedDebug::ID,
    // Other sensors
    MsgTirePressure::ID,
    MsgFuelLevel::ID,
    MsgTrafficSignInfo::ID,
    MsgGpsLatLong::ID,
    MsgGpsAltitude::ID,
    MsgGpsTime::ID,
    // ECU info
    MsgEcuInfoGateway::ID,
    MsgEcuInfoSteer::ID,
    MsgEcuInfoBrake::ID,
    MsgEcuInfoThrottle::ID,
    MsgEcuInfoShift::ID,
    MsgEcuInfoBOO::ID,
    MsgEcuInfoMonitor::ID,
};
template <typename T, size_t N>
static constexpr bool _is_sorted_unique(const std::array<T, N> &arr) {
    for (size_t i = 1; i < arr.size(); i++) {
        if (arr[i-1] >= arr[i]) {
            return false;
        }
    }
    return true;
}
static_assert(_is_sorted_unique(IDS));

} // namespace ds_dbw_can
