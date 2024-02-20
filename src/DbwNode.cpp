/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Dataspeed Inc.
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

#include "DbwNode.hpp"

#include <algorithm> // std::clamp()
#include <unordered_set>

// Log once per unique identifier, similar to RCLCPP_INFO_ONCE()
#define DS_LOG_ONCE_ID(logger, log_macro, id, ...) \
  do {                                             \
    static std::unordered_set<std::remove_cv<typeof(id)>::type> __set; \
    if (RCUTILS_UNLIKELY(__set.count(id) == 0)) {  \
      __set.insert((id));                          \
      log_macro((logger), __VA_ARGS__);            \
    }                                              \
  } while (0)

#define RCLCPP_INFO_ONCE_ID(logger, id, ...) DS_LOG_ONCE_ID((logger), RCLCPP_INFO, (id), __VA_ARGS__)
#define RCLCPP_WARN_ONCE_ID(logger, id, ...) DS_LOG_ONCE_ID((logger), RCLCPP_WARN, (id), __VA_ARGS__)

namespace ds_dbw_can {

// Trim whitespace
static constexpr const char * WHITESPACE = " \n\r\t\f\v";
static std::string ltrim(const std::string &str) {
    size_t start = str.find_first_not_of(WHITESPACE);
    return (start == std::string::npos) ? "" : str.substr(start);
} 
static std::string rtrim(const std::string &str) {
    size_t end = str.find_last_not_of(WHITESPACE);
    return (end == std::string::npos) ? "" : str.substr(0, end + 1);
}
static std::string trim(const std::string &str) {
    return rtrim(ltrim(str));
}

static bool timeoutMs(rclcpp::Time stamp,
               rclcpp::Time prev,
               uint32_t timeout_ms) {
  int64_t diff_ns = (stamp - prev).nanoseconds();
  int64_t timeout_ns = timeout_ms * 1000000;
  return diff_ns > timeout_ns;
}

// Convert specific DBW message to ROS CAN message
template <typename T>
static can_msgs::msg::Frame FrameFromDbw(const T &msg) {
  can_msgs::msg::Frame frame;
  frame.id = msg.ID;
  frame.is_extended = false;
  frame.dlc = sizeof(msg);
  memcpy(frame.data.data(), &msg, sizeof(msg));
  return frame;
}

// Latest firmware versions
static const PlatformMap FIRMWARE_LATEST({
#if 0
  {PlatformVersion(Platform::FORD_CD4,        Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_CD4,        Module::Brake,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_CD4,        Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_CD4,        Module::Shift,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_CD5,        Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_CD5,        Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_CD5,        Module::BOO,      ModuleVersion(0,0,1))},
#endif
  {PlatformVersion(Platform::FORD_GE1,        Module::Gateway,  ModuleVersion(2,0,6))},
  {PlatformVersion(Platform::FORD_GE1,        Module::Throttle, ModuleVersion(2,0,6))},
  {PlatformVersion(Platform::FORD_GE1,        Module::Shift,    ModuleVersion(2,0,6))},
#if 0
  {PlatformVersion(Platform::FORD_P5,         Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_P5,         Module::Brake,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_P5,         Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_P5,         Module::Shift,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_P5,         Module::BOO,      ModuleVersion(0,0,1))},
#endif
  {PlatformVersion(Platform::FORD_P702,       Module::Gateway,  ModuleVersion(2,0,3))},
  {PlatformVersion(Platform::FORD_P702,       Module::Throttle, ModuleVersion(2,0,3))},
  {PlatformVersion(Platform::FORD_P702,       Module::Shift,    ModuleVersion(2,0,3))},
#if 0
  {PlatformVersion(Platform::FORD_T6,         Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_T6,         Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_T6,         Module::Shift,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_U6,         Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_U6,         Module::Brake,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_U6,         Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_U6,         Module::Shift,    ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::FORD_U6,         Module::BOO,      ModuleVersion(0,0,1))},
#endif
  {PlatformVersion(Platform::FORD_V3,         Module::Gateway,  ModuleVersion(0,1,7))},
  {PlatformVersion(Platform::FORD_V3,         Module::Throttle, ModuleVersion(0,1,7))},
  {PlatformVersion(Platform::FORD_V3,         Module::Shift,    ModuleVersion(0,1,7))},
  {PlatformVersion(Platform::FORD_V3,         Module::BOO,      ModuleVersion(0,1,7))},
#if 0
  {PlatformVersion(Platform::POLARIS_GEM,     Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_GEM,     Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_GEM,     Module::BOO,      ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_RZR,     Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_RZR,     Module::Throttle, ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_RZR,     Module::BOO,      ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_RANGER,  Module::Gateway,  ModuleVersion(0,0,1))},
  {PlatformVersion(Platform::POLARIS_RANGER,  Module::Throttle, ModuleVersion(0,0,1))},
#endif
});

using std::placeholders::_1;

DbwNode::DbwNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("dbw_node", options),
      sync_imu_(10, std::bind(&DbwNode::recvCanImu, this, _1), MsgAccel::ID, MsgGyro::ID),
      sync_misc_(10, std::bind(&DbwNode::recvCanMisc, this, _1), MsgMiscReport1::ID, MsgMiscReport2::ID) {
  // Reduce synchronization delay
  sync_imu_.setInterMessageLowerBound(std::chrono::milliseconds(3)); // 10ms period
  sync_misc_.setInterMessageLowerBound(std::chrono::milliseconds(9)); // 50ms period

  // Initialize timestamps
  msg_ulc_cfg_stamp_ = ros_clock_.now();

  // Frame ID
  frame_id_ = declare_parameter<std::string>("frame_id", frame_id_);

  // Warn on received CRC errors, commands, and unknown messages
  warn_crc_ = declare_parameter<bool>("warn_crc", warn_crc_);
  warn_cmds_ = declare_parameter<bool>("warn_cmds", warn_cmds_);
  warn_unknown_ = declare_parameter<bool>("warn_unknown", warn_unknown_);

  // Print time delta between time synced messages
  debug_sync_ = declare_parameter<bool>("debug_sync", debug_sync_);

  // Setup Publishers
  pub_can_ = create_publisher<can_msgs::msg::Frame>("can_tx", 10);
  pub_steer_rpt_ = create_publisher<ds_dbw_msgs::msg::SteeringReport>("steering/report", 2);
  pub_steer_diag_ = create_publisher<ds_dbw_msgs::msg::SteeringDiagnostics>("steering/diag", 2);
  pub_brake_rpt_ = create_publisher<ds_dbw_msgs::msg::BrakeReport>("brake/report", 2);
  pub_brake_diag_ = create_publisher<ds_dbw_msgs::msg::BrakeDiagnostics>("brake/diag", 2);
  pub_thrtl_rpt_ = create_publisher<ds_dbw_msgs::msg::ThrottleReport>("throttle/report", 2);
  pub_thrtl_diag_ = create_publisher<ds_dbw_msgs::msg::ThrottleDiagnostics>("throttle/diag", 2);
  pub_gear_rpt_ = create_publisher<ds_dbw_msgs::msg::GearReport>("gear/report", 2);
  pub_gear_diag_ = create_publisher<ds_dbw_msgs::msg::GearDiagnostics>("gear/diag", 2);
  pub_system_rpt_ = create_publisher<ds_dbw_msgs::msg::SystemReport>("system/report", 2);
  pub_veh_vel_ = create_publisher<ds_dbw_msgs::msg::VehicleVelocity>("vehicle_velocity", 2);
  pub_thrtl_info_ = create_publisher<ds_dbw_msgs::msg::ThrottleInfo>("throttle/info", 2);
  pub_brake_info_ = create_publisher<ds_dbw_msgs::msg::BrakeInfo>("brake/info", 2);
  pub_ulc_ = create_publisher<ds_dbw_msgs::msg::UlcReport>("ulc/report", 2);
  pub_wheel_speeds_ = create_publisher<ds_dbw_msgs::msg::WheelSpeeds>("wheel_speeds", 2);
  pub_wheel_positions_ = create_publisher<ds_dbw_msgs::msg::WheelPositions>("wheel_positions", 2);
  pub_misc_ = create_publisher<ds_dbw_msgs::msg::MiscReport>("misc/report", 2);
  pub_tire_pressures_ = create_publisher<ds_dbw_msgs::msg::TirePressures>("tire_pressures", 2);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  pub_ecu_info_ = create_publisher<ds_dbw_msgs::msg::EcuInfo>("ecu_info", 10);
  pub_vin_ = create_publisher<std_msgs::msg::String>("vin", 2);
  pub_sys_enable_ = create_publisher<std_msgs::msg::Bool>("dbw_enabled", 2);
  publishDbwEnabled();

  // Setup Subscribers
  sub_enable_ = create_subscription<std_msgs::msg::Empty>("enable", 10, std::bind(&DbwNode::recvEnable, this, _1));
  sub_disable_ = create_subscription<std_msgs::msg::Empty>("disable", 10, std::bind(&DbwNode::recvDisable, this, _1));
  sub_can_ = create_subscription<can_msgs::msg::Frame>("can_rx", 100, std::bind(&DbwNode::recvCAN, this, _1));
  sub_steer_ = create_subscription<ds_dbw_msgs::msg::SteeringCmd>("steering/cmd", 1, std::bind(&DbwNode::recvSteeringCmd, this, _1));
  sub_brake_ = create_subscription<ds_dbw_msgs::msg::BrakeCmd>("brake/cmd", 1, std::bind(&DbwNode::recvBrakeCmd, this, _1));
  sub_thrtl_ = create_subscription<ds_dbw_msgs::msg::ThrottleCmd>("throttle/cmd", 1, std::bind(&DbwNode::recvThrottleCmd, this, _1));
  sub_gear_ = create_subscription<ds_dbw_msgs::msg::GearCmd>("gear/cmd", 1, std::bind(&DbwNode::recvGearCmd, this, _1));
  sub_misc_ = create_subscription<ds_dbw_msgs::msg::MiscCmd>("misc/cmd", 1, std::bind(&DbwNode::recvMiscCmd, this, _1));
  sub_ulc_ = create_subscription<ds_dbw_msgs::msg::UlcCmd>("ulc/cmd", 1, std::bind(&DbwNode::recvUlcCmd, this, _1));
  sub_calibrate_steering_ = create_subscription<std_msgs::msg::Empty>("steering/calibrate", 1, std::bind(&DbwNode::recvSteeringCalibrate, this, _1));

  // Setup Timer
  timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&DbwNode::timerCallback, this));
}

void DbwNode::recvEnable(const std_msgs::msg::Empty::ConstSharedPtr) {
  if (modeSyncNone()) {
    enableSystem();
  } else {
    // Queue clear for steer/brake/throttle, followed by system enable
    msg_steer_cmd_clear_ = true;
    msg_brake_cmd_clear_ = true;
    msg_thrtl_cmd_clear_ = true;
    msg_system_cmd_enable_ = true;
  }
}

void DbwNode::recvDisable(const std_msgs::msg::Empty::ConstSharedPtr) {
  if (modeSyncNone()) {
    disableSystem();
  } else {
    // Reset queued clear/enable requests
    msg_steer_cmd_clear_ = false;
    msg_brake_cmd_clear_ = false;
    msg_thrtl_cmd_clear_ = false;
    msg_system_cmd_enable_ = false;

    // Request system disable
    msg_system_cmd_.cmd = MsgSystemCmd::Cmd::Disable;
    msg_system_cmd_.rc++;
    msg_system_cmd_.setCrc();
    pub_can_->publish(FrameFromDbw(msg_system_cmd_));
  }
}

void DbwNode::recvCAN(const can_msgs::msg::Frame::ConstSharedPtr msg_can) {
  if (!msg_can->header.stamp.sec && !msg_can->header.stamp.nanosec) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 60e3,
        "CAN message timestamps always zero, rolling counter validation will not function");
  }
  if (!msg_can->is_rtr && !msg_can->is_error && !msg_can->is_extended) {
    switch (msg_can->id) {
      case msg_steer_rpt_1_.ID:
        if (msg_can->dlc == sizeof(msg_steer_rpt_1_)) {
          bool fault_prev = msg_steer_rpt_1_.fault;
          memcpy(&msg_steer_rpt_1_, msg_can->data.data(), sizeof(msg_steer_rpt_1_));
          const auto &msg = msg_steer_rpt_1_;
          if (msg.validCrc()) {
            if (msg_steer_rpt_1_rc_.valid(msg, msg_can->header.stamp)) {
              warnBadCrcRc(msg.bad_crc, msg.bad_rc, "Steer");
              warn_timeout_steer_.recv(msg);
              ds_dbw_msgs::msg::SteeringReport out;
              out.header.stamp = msg_can->header.stamp;
              out.steering_wheel_angle = msg.angleDeg();
              out.steering_column_torque = msg.torqueNm();
              out.cmd_type = (uint8_t)msg.cmd_type;
              if (msg.cmd_type == MsgSteerReport1::CmdType::Torque) {
                out.cmd = msg.cmdTorqueNm();
              } else if (msg.cmd_type == MsgSteerReport1::CmdType::Angle) {
                out.cmd = msg.cmdAngleDeg();
              } else {
                out.cmd = NAN;
              }
              out.limiting_value = msg.limiting_value;
              out.limiting_rate = msg.limiting_rate;
              out.external_control = msg.external_control;
              out.ready = msg.ready;
              out.enabled = msg.enabled;
              out.override_active = msg.override_active;
              out.override_other = msg.override_other;
              out.override_latched = msg.override_latched;
              out.fault = msg.fault;
              out.timeout = msg.timeout;
              out.bad_crc = msg.bad_crc;
              out.bad_rc = msg.bad_rc;
              if (true) { ///@TODO: msg_steer_rpt_2_.valid()
                out.degraded = msg_steer_rpt_2_.degraded;
                out.limit_rate = msg_steer_rpt_2_.getLimitRateDegS();
                out.limit_value = msg_steer_rpt_2_.getLimitValueDeg();
                out.cmd_src.value = (uint8_t)msg_steer_rpt_2_.cmd_src;
              } else {
                out.limit_rate = NAN;
                out.limit_value = NAN;
                out.cmd_src.value = (uint8_t)CmdSrc::User;
              }
              pub_steer_rpt_->publish(out);
              if (msg.fault) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault");
              } else if (fault_prev) {
                RCLCPP_INFO(get_logger(), "Steering fault cleared");
              }
              if (modeSyncNone()) {
                if (publishDbwEnabled()) {
                  if (enabled()) {
                    RCLCPP_INFO(get_logger(), "DBW system enabled.");
                  } else {
                    if (msg.fault) {
                      RCLCPP_ERROR(get_logger(), "DBW system disabled. Steering fault.");
                    } else if (msg.override_active) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on steering wheel.");
                    } else if (msg.override_other) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on steering wheel (from other system)");
                    } else if (msg.override_latched) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Latched override on steering wheel.");
                    } else {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Unknown cause in steering report.");
                    }
                  }
                }
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring steer report 1 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring steer report 1 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring steer report 1 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_brake_rpt_1_.ID:
        if (msg_can->dlc == sizeof(msg_brake_rpt_1_)) {
          bool fault_prev = msg_brake_rpt_1_.fault;
          memcpy(&msg_brake_rpt_1_, msg_can->data.data(), sizeof(msg_brake_rpt_1_));
          const auto &msg = msg_brake_rpt_1_;
          if (msg.validCrc()) {
            if (msg_brake_rpt_1_rc_.valid(msg, msg_can->header.stamp)) {
              warnBadCrcRc(msg.bad_crc, msg.bad_rc, "Brake");
              warn_timeout_brake_.recv(msg);
              ds_dbw_msgs::msg::BrakeReport out;
              out.header.stamp = msg_can->header.stamp;
              out.cmd_type = (uint8_t)msg.cmd_type;
              out.pressure_input  = NAN;
              out.pressure_cmd    = NAN;
              out.pressure_output = NAN;
              out.torque_input    = NAN;
              out.torque_cmd      = NAN;
              out.torque_output   = NAN;
              out.accel_cmd       = NAN;
              out.accel_output    = NAN;
              out.percent_input   = NAN;
              out.percent_cmd     = NAN;
              out.percent_output  = NAN;
              switch (msg.cmd_type) {
                case MsgBrakeReport1::CmdType::Pressure:
                  msg.getPressureBar(out.pressure_input, out.pressure_cmd, out.pressure_output);
                  break;
                case MsgBrakeReport1::CmdType::Torque:
                  msg.getTorqueNm(out.torque_input, out.torque_cmd, out.torque_output);
                  break;
                case MsgBrakeReport1::CmdType::Accel:
                case MsgBrakeReport1::CmdType::AccelAcc:
                case MsgBrakeReport1::CmdType::AccelAeb:
                  msg.getAccel(out.torque_input, out.accel_cmd, out.accel_output);
                  break;
                case MsgBrakeReport1::CmdType::PedalRaw:
                case MsgBrakeReport1::CmdType::Percent:
                  msg.getPercent(out.percent_input, out.percent_cmd, out.percent_output);
                  break;
                case MsgBrakeReport1::CmdType::None:
                case MsgBrakeReport1::CmdType::Calibrate:
                  break;
              }
              out.btsi_cmd = msg.btsi;
              out.limiting_value = msg.limiting_value;
              out.limiting_rate = msg.limiting_rate;
              out.external_control = msg.external_control;
              out.ready = msg.ready;
              out.enabled = msg.enabled;
              out.override_active = msg.override_active;
              out.override_other = msg.override_other;
              out.override_latched = msg.override_latched;
              out.fault = msg.fault;
              out.timeout = msg.timeout;
              out.bad_crc = msg.bad_crc;
              out.bad_rc = msg.bad_rc;
              if (true) { ///@TODO: msg_brake_rpt_2_.valid()
                out.degraded = msg_brake_rpt_2_.degraded;
                using Mode = MsgBrakeReport2::BrkAvlMode;
                switch (msg_brake_rpt_2_.brake_available_mux) {
                  default:                     out.limit_value = NAN; break;
                  case Mode::Unlimited:        out.limit_value = INFINITY; break;
                  case Mode::SecondsX2:        out.limit_value = msg_brake_rpt_2_.getLimitValuePressureBar(); break;
                  case Mode::MillisecondsX100: out.limit_value = msg_brake_rpt_2_.getLimitValueDecelMps2(); break;
                }
                out.brake_available_duration = msg_brake_rpt_2_.brkAvailDurSec();
                out.brake_available_full = msg_brake_rpt_2_.brake_available_full;
                out.req_shift_park = msg_brake_rpt_2_.req_shift_park;
                out.req_park_brake = msg_brake_rpt_2_.req_park_brake;
                out.external_button = msg_brake_rpt_2_.external_button;
                out.cmd_src.value = (uint8_t)msg_brake_rpt_2_.cmd_src;
              } else {
                out.limit_value = NAN;
                out.brake_available_duration = NAN;
                out.cmd_src.value = (uint8_t)CmdSrc::User;
              }
              pub_brake_rpt_->publish(out);
              if (msg.fault) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault");
              } else if (fault_prev) {
                RCLCPP_INFO(get_logger(), "Brake fault cleared");
              }
              if (modeSyncNone()) {
                if (publishDbwEnabled()) {
                  if (enabled()) {
                    RCLCPP_INFO(get_logger(), "DBW system enabled.");
                  } else {
                    if (msg.fault) {
                      RCLCPP_ERROR(get_logger(), "DBW system disabled. Braking fault.");
                    } else if (msg.override_active) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on brake pedal.");
                    } else if (msg.override_other) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on brake pedal (from other system)");
                    } else if (msg.override_latched) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Latched override on brake pedal.");
                    } else {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Unknown cause in brake report.");
                    }
                  }
                }
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring brake report 1 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring brake report 1 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring brake report 1 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_thrtl_rpt_1_.ID:
        if (msg_can->dlc == sizeof(msg_thrtl_rpt_1_)) {
          bool fault_prev = msg_thrtl_rpt_1_.fault;
          memcpy(&msg_thrtl_rpt_1_, msg_can->data.data(), sizeof(msg_thrtl_rpt_1_));
          const auto &msg = msg_thrtl_rpt_1_;
          if (msg.validCrc()) {
            if (msg_thrtl_rpt_1_rc_.valid(msg, msg_can->header.stamp)) {
              warnBadCrcRc(msg.bad_crc, msg.bad_rc, "Throttle");
              warn_timeout_thrtl_.recv(msg);
              ds_dbw_msgs::msg::ThrottleReport out;
              out.header.stamp = msg_can->header.stamp;
              out.cmd_type = (uint8_t)msg.cmd_type;
              switch (msg.cmd_type) {
                case MsgThrtlReport1::CmdType::PedalRaw:
                case MsgThrtlReport1::CmdType::Percent:
                  msg.getPercent(out.percent_input, out.percent_cmd, out.percent_output);
                  break;
                case MsgThrtlReport1::CmdType::None:
                  out.percent_input  = NAN;
                  out.percent_cmd    = NAN;
                  out.percent_output = NAN;
                  break;
              }
              out.limiting_value = msg.limiting_value;
              out.limiting_rate = msg.limiting_rate;
              out.external_control = msg.external_control;
              out.ready = msg.ready;
              out.enabled = msg.enabled;
              out.override_active = msg.override_active;
              out.override_other = msg.override_other;
              out.override_latched = msg.override_latched;
              out.fault = msg.fault;
              out.timeout = msg.timeout;
              out.bad_crc = msg.bad_crc;
              out.bad_rc = msg.bad_rc;
              if (true) { ///@TODO: msg_thrtl_rpt_2_.valid()
                out.degraded = msg_thrtl_rpt_2_.degraded;
                out.limit_value = msg_thrtl_rpt_2_.getLimitValuePc();
                out.cmd_src.value = (uint8_t)msg_thrtl_rpt_2_.cmd_src;
              } else {
                out.limit_value = NAN;
                out.cmd_src.value = (uint8_t)CmdSrc::User;
              }
              pub_thrtl_rpt_->publish(out);
              if (msg.fault) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault");
              } else if (fault_prev) {
                RCLCPP_INFO(get_logger(), "Throttle fault cleared");
              }
              if (modeSyncNone()) {
                if (publishDbwEnabled()) {
                  if (enabled()) {
                    RCLCPP_INFO(get_logger(), "DBW system enabled.");
                  } else {
                    if (msg.fault) {
                      RCLCPP_ERROR(get_logger(), "DBW system disabled. Throttle fault.");
                    } else if (msg.override_active) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on accelerator pedal.");
                    } else if (msg.override_other) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on accelerator pedal (from other system)");
                    } else if (msg.override_latched) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Latched override on accelerator pedal.");
                    } else {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Unknown cause in throttle report.");
                    }
                  }
                }
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring throttle report 1 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring throttle report 1 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring throttle report 1 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_gear_rpt_1_.ID:
        if (msg_can->dlc >= sizeof(msg_gear_rpt_1_)) {
          bool fault_prev = msg_gear_rpt_1_.fault;
          memcpy(&msg_gear_rpt_1_, msg_can->data.data(), sizeof(msg_gear_rpt_1_));
          const auto &msg = msg_gear_rpt_1_;
          if (msg.validCrc()) {
            if (msg_gear_rpt_1_rc_.valid(msg, msg_can->header.stamp)) {
              warnBadCrcRc(msg.bad_crc, false, "Gear");
              warnRejectGear((uint8_t)msg.reject);
              ds_dbw_msgs::msg::GearReport out;
              out.header.stamp = msg_can->header.stamp;
              out.gear.value = (uint8_t)msg.gear;
              out.cmd.value = (uint8_t)msg.cmd;
              out.driver.value = (uint8_t)msg.driver;
              out.reject.value = (uint8_t)msg.reject;
              out.power_latched = msg.power_latched;
              out.external_control = msg.external_control;
              out.ready = msg.ready;
              out.override_active = msg.override_active;
              out.override_other = msg.override_other;
              out.fault = msg.fault;
              out.bad_crc = msg.bad_crc;
              if (true) { ///@TODO: msg_gear_rpt_2_.valid()
                out.degraded = msg_gear_rpt_2_.degraded;
                out.cmd_src.value = (uint8_t)msg_gear_rpt_2_.cmd_src;
              } else {
                out.cmd_src.value = (uint8_t)CmdSrc::User;
              }
              pub_gear_rpt_->publish(out);
              if (msg.fault) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault");
              } else if (fault_prev) {
                RCLCPP_INFO(get_logger(), "Gear fault cleared");
              }
              if (modeSyncNone()) {
                if (publishDbwEnabled()) {
                  if (enabled()) {
                    RCLCPP_INFO(get_logger(), "DBW system enabled.");
                  } else {
                    if (msg.fault) {
                      RCLCPP_ERROR(get_logger(), "DBW system disabled. Gear fault.");
                    } else if (msg.override_active) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on gear shifter.");
                    } else if (msg.override_other) {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Override on gear shifter (from other system)");
                    } else {
                      RCLCPP_WARN(get_logger(), "DBW system disabled. Unknown cause in gear report.");
                    }
                  }
                }
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring gear report 1 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring gear report 1 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring gear report 1 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_system_rpt_.ID:
        if (msg_can->dlc >= sizeof(msg_system_rpt_)) {
          memcpy(&msg_system_rpt_, msg_can->data.data(), sizeof(msg_system_rpt_));
          const auto &msg = msg_system_rpt_;
          if (msg.validCrc()) {
            if (msg_system_rpt_rc_.valid(msg, msg_can->header.stamp)) {
              warnBadCrcRc(msg.bad_crc, msg.bad_rc, "System");
              ds_dbw_msgs::msg::SystemReport out;
              out.header.stamp = msg_can->header.stamp;
              out.inhibit = msg.inhibit;
              out.validate_cmd_crc_rc = msg.validate_cmd_crc_rc;
              out.system_sync_mode.value = (uint8_t)msg.system_sync_mode;
              out.state.value = (uint8_t)msg.state;
              out.reason_disengage = (uint8_t)msg.reason_disengage;
              out.reason_not_ready = (uint8_t)msg.reason_not_ready;
              out.reason_disengage_str = MsgSystemReport::reasonToString(msg.reason_disengage);
              out.reason_not_ready_str = MsgSystemReport::reasonToString(msg.reason_not_ready);
              out.btn_enable = msg.btn_enable;
              out.btn_disable = msg.btn_disable;
              out.override = msg.override;
              out.ready = msg.ready;
              out.enabled = msg.enabled;
              out.fault = msg.fault;
              out.bad_crc = msg.bad_crc;
              out.bad_rc = msg.bad_rc;
              pub_system_rpt_->publish(out);
              if (msg.btn_enable) {
                // Queue clear for steer/brake/throttle
                msg_steer_cmd_clear_ = true;
                msg_brake_cmd_clear_ = true;
                msg_thrtl_cmd_clear_ = true;
              }
              if (!modeSyncNone()) {
                if (publishDbwEnabled()) {
                  if (enabled()) {
                    RCLCPP_INFO(get_logger(), "DBW system enabled");
                  } else {
                    RCLCPP_WARN(get_logger(), "DBW system disabled with reason: %s", out.reason_disengage_str.c_str());
                  }
                }
              }
              if (!msg.validate_cmd_crc_rc) {
                if (!validate_cmd_crc_rc_warned_) {
                  validate_cmd_crc_rc_warned_ = true;
                  RCLCPP_WARN(get_logger(), "Drive-By-Wire command message CRC and rolling counter validation disabled");
                }
              } else {
                validate_cmd_crc_rc_warned_ = false;
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring system report with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring system report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring system report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_veh_vel_.ID:
        if (msg_can->dlc == sizeof(msg_veh_vel_)) {
          memcpy(&msg_veh_vel_, msg_can->data.data(), sizeof(msg_veh_vel_));
          const auto &msg = msg_veh_vel_;
          if (msg.validCrc()) {
            if (msg_veh_vel_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::VehicleVelocity out;
              out.header.stamp = msg_can->header.stamp;
              out.vehicle_velocity_brake      = msg.velocityBrkKph()  / 3.6f; // kph to m/s
              out.vehicle_velocity_propulsion = msg.velocityPrplKph() / 3.6f; // kph to m/s
              out.dir_src = (uint8_t)msg.dir_src;
              pub_veh_vel_->publish(out);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring vehicle velocity report with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring vehicle velocity report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring vehicle velocity report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_thrtl_info_.ID:
        if (msg_can->dlc == sizeof(msg_thrtl_info_)) {
          memcpy(&msg_thrtl_info_, msg_can->data.data(), sizeof(msg_thrtl_info_));
          const auto &msg = msg_thrtl_info_;
          if (msg.validCrc()) {
            if (msg_thrtl_info_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::ThrottleInfo out;
              out.header.stamp = msg_can->header.stamp;
              out.accel_pedal_pc = msg.accelPedalPercent();
              out.accel_pedal_qf.value = (uint8_t)msg.accel_pedal_qf;
              out.one_pedal.value = (uint8_t)msg.one_pedal_drive;
              out.engine_rpm = msg.engineRpm();
              pub_thrtl_info_->publish(out);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring throttle info with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring throttle info with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring throttle info with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_brake_info_.ID:
        if (msg_can->dlc == sizeof(msg_brake_info_)) {
          memcpy(&msg_brake_info_, msg_can->data.data(), sizeof(msg_brake_info_));
          const auto &msg = msg_brake_info_;
          if (msg.validCrc()) {
            if (msg_brake_info_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::BrakeInfo out;
              out.header.stamp = msg_can->header.stamp;
              out.brake_torque_pedal = msg.brakeTorquePedalNm();
              out.brake_torque_request = msg.brakeTorqueRequestNm();
              out.brake_torque_actual = msg.brakeTorqueActualNm();
              out.brake_pedal_qf.value = (uint8_t)msg.brake_pedal_qf;
              out.brake_vacuum = msg.brakeVacuumBar();
              out.abs_active = msg.abs_active;
              out.abs_enabled = msg.abs_enabled;
              out.esc_active = msg.esc_active;
              out.esc_enabled = msg.esc_enabled;
              out.trac_active = msg.trac_active;
              out.trac_enabled = msg.trac_enabled;
              pub_brake_info_->publish(out);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring brake info with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring brake info with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring brake info with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_ulc_rpt_.ID:
        if (msg_can->dlc == sizeof(msg_ulc_rpt_)) {
          memcpy(&msg_ulc_rpt_, msg_can->data.data(), sizeof(msg_ulc_rpt_));
          const auto &msg = msg_ulc_rpt_;
          if (msg.validCrc()) {
            if (msg_ulc_rpt_rc_.valid(msg, msg_can->header.stamp)) {
              warnBadCrcRc(msg.bad_crc, msg.bad_rc, "ULC");
              ds_dbw_msgs::msg::UlcReport out;
              out.header.stamp = msg_can->header.stamp;
              out.cmd_type = (uint8_t)msg.cmd_type;
              out.vel_ref = msg.velocityRefMps();
              out.vel_meas = msg.velocityMeasMps();
              out.accel_ref = msg.accelRefMps();
              out.accel_meas = msg.accelMeasMps();
              out.coast_decel = msg.coast_decel != MsgUlcCmd::CoastDecel::UseBrakes;
              out.ready = msg.ready;
              out.enabled = msg.enabled;
              out.override_active = msg.override_active;
              out.override_latched = msg.override_latched;
              out.preempted = msg.preempted;
              out.timeout = msg.timeout;
              out.bad_crc = msg.bad_crc;
              out.bad_rc = msg.bad_rc;
              pub_ulc_->publish(out);
              if (msg.preempted && !msg.timeout) {
                if (!ulc_preempt_warned_) {
                  RCLCPP_WARN(get_logger(), "ULC preempted by user brake or throttle commands");
                  ulc_preempt_warned_ = true;
                }
              } else {
                ulc_preempt_warned_ = false;
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring ULC report with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring ULC report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring ULC report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_accel_.ID:
        if (msg_can->dlc == sizeof(msg_accel_)) {
          memcpy(&msg_accel_, msg_can->data.data(), sizeof(msg_accel_));
          const auto &msg = msg_accel_;
          if (msg.validCrc()) {
            if (msg_accel_rc_.valid(msg, msg_can->header.stamp)) {
              sync_imu_.processMsg(msg_can);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring accel report with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring accel report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring accel report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_gyro_.ID:
        if (msg_can->dlc == sizeof(msg_gyro_)) {
          memcpy(&msg_gyro_, msg_can->data.data(), sizeof(msg_gyro_));
          const auto &msg = msg_gyro_;
          if (msg.validCrc()) {
            if (msg_gyro_rc_.valid(msg, msg_can->header.stamp)) {
              sync_imu_.processMsg(msg_can);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring gyro report with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring gyro report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring gyro report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_wheel_speed_.ID:
        if (msg_can->dlc == sizeof(msg_wheel_speed_)) {
          memcpy(&msg_wheel_speed_, msg_can->data.data(), sizeof(msg_wheel_speed_));
          const auto &msg = msg_wheel_speed_;
          ds_dbw_msgs::msg::WheelSpeeds out;
          out.header.stamp = msg_can->header.stamp;
          out.front_left  = msg.frontLeftRadS();
          out.front_right = msg.frontRightRadS();
          out.rear_left   = msg.rearLeftRadS();
          out.rear_right  = msg.rearRightRadS();
          pub_wheel_speeds_->publish(out);
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring wheel speed report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_wheel_position_.ID:
        if (msg_can->dlc == sizeof(msg_wheel_position_)) {
          memcpy(&msg_wheel_position_, msg_can->data.data(), sizeof(msg_wheel_position_));
          const auto &msg = msg_wheel_position_;
          ds_dbw_msgs::msg::WheelPositions out;
          out.header.stamp = msg_can->header.stamp;
          out.front_left  = msg.front_left;
          out.front_right = msg.front_right;
          out.rear_left   = msg.rear_left;
          out.rear_right  = msg.rear_right;
          pub_wheel_positions_->publish(out);
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring wheel speed report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_misc_rpt_1_.ID:
        if (msg_can->dlc == sizeof(msg_misc_rpt_1_)) {
          memcpy(&msg_misc_rpt_1_, msg_can->data.data(), sizeof(msg_misc_rpt_1_));
          const auto &msg = msg_misc_rpt_1_;
          if (msg.validCrc()) {
            if (msg_misc_rpt_1_rc_.valid(msg, msg_can->header.stamp)) {
              sync_misc_.processMsg(msg_can);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring misc report 1 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring misc report 1 report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring misc report 1 report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_misc_rpt_2_.ID:
        if (msg_can->dlc == sizeof(msg_misc_rpt_2_)) {
          memcpy(&msg_misc_rpt_2_, msg_can->data.data(), sizeof(msg_misc_rpt_2_));
          const auto &msg = msg_misc_rpt_2_;
          if (msg.validCrc()) {
            if (msg_misc_rpt_2_rc_.valid(msg, msg_can->header.stamp)) {
              sync_misc_.processMsg(msg_can);
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring misc report 2 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring misc report 2 report with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring misc report 2 report with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_steer_rpt_2_.ID:
        if (msg_can->dlc == sizeof(msg_steer_rpt_2_)) {
          bool degraded_prev = msg_steer_rpt_2_.degraded;
          memcpy(&msg_steer_rpt_2_, msg_can->data.data(), sizeof(msg_steer_rpt_2_));
          const auto &msg = msg_steer_rpt_2_;
          if (msg.validCrc()) {
            if (msg_steer_rpt_2_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::SteeringDiagnostics out;
              out.degraded = msg_steer_rpt_2_.degraded;
              out.degraded_cmd_type = msg_steer_rpt_2_.degraded_cmd_type;
              out.degraded_comms = msg_steer_rpt_2_.degraded_comms;
              out.degraded_internal = msg_steer_rpt_2_.degraded_internal;
              out.degraded_vehicle = msg_steer_rpt_2_.degraded_vehicle;
              out.degraded_actuator = msg_steer_rpt_2_.degraded_actuator;
              out.degraded_comms_dbw = msg_steer_rpt_3_.degraded_comms_dbw;
              out.degraded_comms_dbw_gateway = msg_steer_rpt_3_.degraded_comms_dbw_gateway;
              out.degraded_comms_dbw_brake = msg_steer_rpt_3_.degraded_comms_dbw_brake;
              out.degraded_comms_dbw_thrtl = msg_steer_rpt_3_.degraded_comms_dbw_thrtl;
              out.degraded_comms_dbw_gear = msg_steer_rpt_3_.degraded_comms_dbw_gear;
              out.degraded_control_performance = msg_steer_rpt_3_.degraded_control_performance;
              out.degraded_param_mismatch = msg_steer_rpt_3_.degraded_param_mismatch;
              out.degraded_comms_vehicle = msg_steer_rpt_3_.degraded_comms_vehicle;
              out.degraded_comms_actuator = msg_steer_rpt_3_.degraded_comms_actuator;
              out.degraded_vehicle_speed = msg_steer_rpt_3_.degraded_vehicle_speed;
              out.degraded_calibration = msg_steer_rpt_3_.degraded_calibration;
              out.fault = msg_steer_rpt_1_.fault;
              out.fault_power = msg_steer_rpt_2_.fault_power;
              out.fault_comms = msg_steer_rpt_2_.fault_comms;
              out.fault_internal = msg_steer_rpt_2_.fault_internal;
              out.fault_vehicle = msg_steer_rpt_2_.fault_vehicle;
              out.fault_actuator = msg_steer_rpt_2_.fault_actuator;
              out.fault_comms_dbw = msg_steer_rpt_3_.fault_comms_dbw;
              out.fault_comms_dbw_gateway = msg_steer_rpt_3_.fault_comms_dbw_gateway;
              out.fault_comms_dbw_brake = msg_steer_rpt_3_.fault_comms_dbw_brake;
              out.fault_comms_dbw_thrtl = msg_steer_rpt_3_.fault_comms_dbw_thrtl;
              out.fault_comms_dbw_gear = msg_steer_rpt_3_.fault_comms_dbw_gear;
              out.fault_comms_vehicle = msg_steer_rpt_3_.fault_comms_vehicle;
              out.fault_comms_actuator = msg_steer_rpt_3_.fault_comms_actuator;
              out.fault_vehicle_speed = msg_steer_rpt_3_.fault_vehicle_speed;
              out.fault_actuator_torque_sensor = msg_steer_rpt_3_.fault_actuator_torque_sensor;
              out.fault_actuator_config = msg_steer_rpt_3_.fault_actuator_config;
              out.fault_param_mismatch = msg_steer_rpt_3_.fault_param_mismatch;
              out.fault_param_limits = msg_steer_rpt_3_.fault_param_limits;
              out.fault_calibration = msg_steer_rpt_3_.fault_calibration;
              pub_steer_diag_->publish(out);
              if (msg.degraded) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded");
              } else if (degraded_prev) {
                RCLCPP_INFO(get_logger(), "Steering degraded state cleared");
              }
              if (msg.degraded_cmd_type) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Unsupported cmd_type");
              }
              if (msg.degraded_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms");
              }
              if (msg.degraded_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Internal");
              }
              if (msg.degraded_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Vehicle");
              }
              if (msg.degraded_actuator) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Actuator");
              }
              if (msg.fault_power) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Drive-By-Wire power voltage");
              }
              if (msg.fault_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms");
              }
              if (msg.fault_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Internal fault");
              }
              if (msg.fault_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Fault in vehicle");
              }
              if (msg.fault_actuator) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Fault in actuator");
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring steer report 2 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring steer report 2 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring steer report 2 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_brake_rpt_2_.ID:
        if (msg_can->dlc == sizeof(msg_brake_rpt_2_)) {
          bool degraded_prev = msg_brake_rpt_2_.degraded;
          bool req_park_brake_prev = msg_brake_rpt_2_.req_park_brake;
          bool req_shift_park_prev = msg_brake_rpt_2_.req_shift_park;
          memcpy(&msg_brake_rpt_2_, msg_can->data.data(), sizeof(msg_brake_rpt_2_));
          const auto &msg = msg_brake_rpt_2_;
          if (msg.validCrc()) {
            if (msg_brake_rpt_2_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::BrakeDiagnostics out;
              out.degraded = msg_brake_rpt_2_.degraded;
              out.degraded_cmd_type = msg_brake_rpt_2_.degraded_cmd_type;
              out.degraded_comms = msg_brake_rpt_2_.degraded_comms;
              out.degraded_internal = msg_brake_rpt_2_.degraded_internal;
              out.degraded_vehicle = msg_brake_rpt_2_.degraded_vehicle;
              out.degraded_actuator = msg_brake_rpt_2_.degraded_actuator;
              out.degraded_comms_dbw = msg_brake_rpt_3_.degraded_comms_dbw;
              out.degraded_comms_dbw_gateway = msg_brake_rpt_3_.degraded_comms_dbw_gateway;
              out.degraded_comms_dbw_steer = msg_brake_rpt_3_.degraded_comms_dbw_steer;
              out.degraded_comms_dbw_thrtl = msg_brake_rpt_3_.degraded_comms_dbw_thrtl;
              out.degraded_comms_dbw_gear = msg_brake_rpt_3_.degraded_comms_dbw_gear;
              out.degraded_control_performance = msg_brake_rpt_3_.degraded_control_performance;
              out.degraded_param_mismatch = msg_brake_rpt_3_.degraded_param_mismatch;
              out.degraded_comms_vehicle = msg_brake_rpt_3_.degraded_comms_vehicle;
              out.degraded_comms_actuator = msg_brake_rpt_3_.degraded_comms_actuator;
              out.degraded_comms_actuator_1 = msg_brake_rpt_3_.degraded_comms_actuator_1;
              out.degraded_comms_actuator_2 = msg_brake_rpt_3_.degraded_comms_actuator_2;
              out.degraded_vehicle_speed = msg_brake_rpt_3_.degraded_vehicle_speed;
              out.degraded_btsi_stuck_low = msg_brake_rpt_3_.degraded_btsi_stuck_low;
              out.degraded_btsi_stuck_high = msg_brake_rpt_3_.degraded_btsi_stuck_high;
              out.degraded_actuator_aeb_deny = msg_brake_rpt_3_.degraded_actuator_aeb_deny;
              out.degraded_actuator_1 = msg_brake_rpt_3_.degraded_actuator_1;
              out.degraded_actuator_2 = msg_brake_rpt_3_.degraded_actuator_2;
              out.degraded_calibration = msg_brake_rpt_3_.degraded_calibration;
              out.fault = msg_brake_rpt_1_.fault;
              out.fault_power = msg_brake_rpt_2_.fault_power;
              out.fault_comms = msg_brake_rpt_2_.fault_comms;
              out.fault_internal = msg_brake_rpt_2_.fault_internal;
              out.fault_vehicle = msg_brake_rpt_2_.fault_vehicle;
              out.fault_actuator = msg_brake_rpt_2_.fault_actuator;
              out.fault_comms_dbw = msg_brake_rpt_3_.fault_comms_dbw;
              out.fault_comms_dbw_gateway = msg_brake_rpt_3_.fault_comms_dbw_gateway;
              out.fault_comms_dbw_steer = msg_brake_rpt_3_.fault_comms_dbw_steer;
              out.fault_comms_dbw_thrtl = msg_brake_rpt_3_.fault_comms_dbw_thrtl;
              out.fault_comms_dbw_gear = msg_brake_rpt_3_.fault_comms_dbw_gear;
              out.fault_comms_vehicle = msg_brake_rpt_3_.fault_comms_vehicle;
              out.fault_comms_actuator = msg_brake_rpt_3_.fault_comms_actuator;
              out.fault_comms_actuator_1 = msg_brake_rpt_3_.fault_comms_actuator_1;
              out.fault_comms_actuator_2 = msg_brake_rpt_3_.fault_comms_actuator_2;
              out.fault_vehicle_speed = msg_brake_rpt_3_.fault_vehicle_speed;
              out.fault_actuator_acc_deny = msg_brake_rpt_3_.fault_actuator_acc_deny;
              out.fault_actuator_pedal_sensor = msg_brake_rpt_3_.fault_actuator_pedal_sensor;
              out.fault_actuator_1 = msg_brake_rpt_3_.fault_actuator_1;
              out.fault_actuator_2 = msg_brake_rpt_3_.fault_actuator_2;
              out.fault_param_mismatch = msg_brake_rpt_3_.fault_param_mismatch;
              out.fault_param_limits = msg_brake_rpt_3_.fault_param_limits;
              out.fault_calibration = msg_brake_rpt_3_.fault_calibration;
              pub_brake_diag_->publish(out);
              if (msg.req_park_brake && !req_park_brake_prev) {
                RCLCPP_INFO(get_logger(), "Brake hold time low, applying parking brake");
              }
              if (msg.req_shift_park && !req_shift_park_prev) {
                RCLCPP_INFO(get_logger(), "Brake hold time depleted, shifting to park");
              }
              if (msg.degraded) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded");
              } else if (degraded_prev) {
                RCLCPP_INFO(get_logger(), "Brake degraded state cleared");
              }
              if (msg.degraded_cmd_type) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Unsupported cmd_type");
              }
              if (msg.degraded_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms");
              }
              if (msg.degraded_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Internal");
              }
              if (msg.degraded_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Vehicle");
              }
              if (msg.degraded_actuator) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Actuator");
              }
              if (msg.fault_power) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Drive-By-Wire power voltage");
              }
              if (msg.fault_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms");
              }
              if (msg.fault_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Internal fault");
              }
              if (msg.fault_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Fault in vehicle");
              }
              if (msg.fault_actuator) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Fault in actuator");
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring brake report 2 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring brake report 2 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring brake report 2 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_thrtl_rpt_2_.ID:
        if (msg_can->dlc == sizeof(msg_thrtl_rpt_2_)) {
          bool degraded_prev = msg_thrtl_rpt_2_.degraded;
          memcpy(&msg_thrtl_rpt_2_, msg_can->data.data(), sizeof(msg_thrtl_rpt_2_));
          const auto &msg = msg_thrtl_rpt_2_;
          if (msg.validCrc()) {
            if (msg_thrtl_rpt_2_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::ThrottleDiagnostics out;
              out.degraded = msg_thrtl_rpt_2_.degraded;
              out.degraded_cmd_type = msg_thrtl_rpt_2_.degraded_cmd_type;
              out.degraded_comms = msg_thrtl_rpt_2_.degraded_comms;
              out.degraded_internal = msg_thrtl_rpt_2_.degraded_internal;
              out.degraded_vehicle = msg_thrtl_rpt_2_.degraded_vehicle;
              out.degraded_sensor = msg_thrtl_rpt_2_.degraded_sensor;
              out.degraded_comms_dbw = msg_thrtl_rpt_3_.degraded_comms_dbw;
              out.degraded_comms_dbw_gateway = msg_thrtl_rpt_3_.degraded_comms_dbw_gateway;
              out.degraded_comms_dbw_steer = msg_thrtl_rpt_3_.degraded_comms_dbw_steer;
              out.degraded_comms_dbw_brake = msg_thrtl_rpt_3_.degraded_comms_dbw_brake;
              out.degraded_comms_dbw_gear = msg_thrtl_rpt_3_.degraded_comms_dbw_gear;
              out.degraded_control_performance = msg_thrtl_rpt_3_.degraded_control_performance;
              out.degraded_param_mismatch = msg_thrtl_rpt_3_.degraded_param_mismatch;
              out.degraded_vehicle_speed = msg_thrtl_rpt_3_.degraded_vehicle_speed;
              out.degraded_aped_feedback = msg_thrtl_rpt_3_.degraded_aped_feedback;
              out.degraded_calibration = msg_thrtl_rpt_3_.degraded_calibration;
              out.fault = msg_thrtl_rpt_1_.fault;
              out.fault_power = msg_thrtl_rpt_2_.fault_power;
              out.fault_comms = msg_thrtl_rpt_2_.fault_comms;
              out.fault_internal = msg_thrtl_rpt_2_.fault_internal;
              out.fault_vehicle = msg_thrtl_rpt_2_.fault_vehicle;
              out.fault_sensor = msg_thrtl_rpt_2_.fault_sensor;
              out.fault_comms_dbw = msg_thrtl_rpt_3_.fault_comms_dbw;
              out.fault_comms_dbw_gateway = msg_thrtl_rpt_3_.fault_comms_dbw_gateway;
              out.fault_comms_dbw_steer = msg_thrtl_rpt_3_.fault_comms_dbw_steer;
              out.fault_comms_dbw_brake = msg_thrtl_rpt_3_.fault_comms_dbw_brake;
              out.fault_comms_dbw_gear = msg_thrtl_rpt_3_.fault_comms_dbw_gear;
              out.fault_vehicle_speed = msg_thrtl_rpt_3_.fault_vehicle_speed;
              out.fault_aped_sensor_1 = msg_thrtl_rpt_3_.fault_aped_sensor_1;
              out.fault_aped_sensor_2 = msg_thrtl_rpt_3_.fault_aped_sensor_2;
              out.fault_aped_sensor_mismatch = msg_thrtl_rpt_3_.fault_aped_sensor_mismatch;
              out.fault_param_mismatch = msg_thrtl_rpt_3_.fault_param_mismatch;
              out.fault_param_limits = msg_thrtl_rpt_3_.fault_param_limits;
              out.fault_calibration = msg_thrtl_rpt_3_.fault_calibration;
              pub_thrtl_diag_->publish(out);
              if (msg.degraded) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded");
              } else if (degraded_prev) {
                RCLCPP_INFO(get_logger(), "Throttle degraded state cleared");
              }
              if (msg.degraded_cmd_type) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Unsupported cmd_type");
              }
              if (msg.degraded_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost comms");
              }
              if (msg.degraded_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Internal");
              }
              if (msg.degraded_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Vehicle");
              }
              if (msg.degraded_sensor) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Sensor");
              }
              if (msg.fault_power) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Drive-By-Wire power voltage");
              }
              if (msg.fault_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Lost comms");
              }
              if (msg.fault_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Internal fault");
              }
              if (msg.fault_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Fault in vehicle");
              }
              if (msg.fault_sensor) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Fault in sensor");
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring throttle report 2 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring throttle report 2 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring throttle report 2 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_gear_rpt_2_.ID:
        if (msg_can->dlc == sizeof(msg_gear_rpt_2_)) {
          bool degraded_prev = msg_gear_rpt_2_.degraded;
          memcpy(&msg_gear_rpt_2_, msg_can->data.data(), sizeof(msg_gear_rpt_2_));
          const auto &msg = msg_gear_rpt_2_;
          if (msg.validCrc()) {
            if (msg_gear_rpt_2_rc_.valid(msg, msg_can->header.stamp)) {
              ds_dbw_msgs::msg::GearDiagnostics out;
              out.degraded = msg_gear_rpt_2_.degraded;
              out.degraded_cmd_type = msg_gear_rpt_2_.degraded_cmd_type;
              out.degraded_comms = msg_gear_rpt_2_.degraded_comms;
              out.degraded_internal = msg_gear_rpt_2_.degraded_internal;
              out.degraded_vehicle = msg_gear_rpt_2_.degraded_vehicle;
              out.degraded_actuator = msg_gear_rpt_2_.degraded_actuator;
              out.degraded_comms_dbw = msg_gear_rpt_3_.degraded_comms_dbw;
              out.degraded_comms_dbw_gateway = msg_gear_rpt_3_.degraded_comms_dbw_gateway;
              out.degraded_comms_dbw_steer = msg_gear_rpt_3_.degraded_comms_dbw_steer;
              out.degraded_comms_dbw_brake = msg_gear_rpt_3_.degraded_comms_dbw_brake;
              out.degraded_comms_dbw_thrtl = msg_gear_rpt_3_.degraded_comms_dbw_thrtl;
              out.degraded_control_performance = msg_gear_rpt_3_.degraded_control_performance;
              out.degraded_param_mismatch = msg_gear_rpt_3_.degraded_param_mismatch;
              out.degraded_comms_vehicle = msg_gear_rpt_3_.degraded_comms_vehicle;
              out.degraded_comms_vehicle_1 = msg_gear_rpt_3_.degraded_comms_vehicle_1;
              out.degraded_comms_vehicle_2 = msg_gear_rpt_3_.degraded_comms_vehicle_2;
              out.degraded_comms_actuator = msg_gear_rpt_3_.degraded_comms_actuator;
              out.degraded_comms_actuator_1 = msg_gear_rpt_3_.degraded_comms_actuator_1;
              out.degraded_comms_actuator_2 = msg_gear_rpt_3_.degraded_comms_actuator_2;
              out.degraded_vehicle_speed = msg_gear_rpt_3_.degraded_vehicle_speed;
              out.degraded_gear_mismatch = msg_gear_rpt_3_.degraded_gear_mismatch;
              out.degraded_power = msg_gear_rpt_3_.degraded_power;
              out.degraded_calibration = msg_gear_rpt_3_.degraded_calibration;
              out.fault = msg_gear_rpt_1_.fault;
              out.fault_power = msg_gear_rpt_2_.fault_power;
              out.fault_comms = msg_gear_rpt_2_.fault_comms;
              out.fault_internal = msg_gear_rpt_2_.fault_internal;
              out.fault_vehicle = msg_gear_rpt_2_.fault_vehicle;
              out.fault_actuator = msg_gear_rpt_2_.fault_actuator;
              out.fault_comms_dbw = msg_gear_rpt_3_.fault_comms_dbw;
              out.fault_comms_dbw_gateway = msg_gear_rpt_3_.fault_comms_dbw_gateway;
              out.fault_comms_dbw_steer = msg_gear_rpt_3_.fault_comms_dbw_steer;
              out.fault_comms_dbw_brake = msg_gear_rpt_3_.fault_comms_dbw_brake;
              out.fault_comms_dbw_thrtl = msg_gear_rpt_3_.fault_comms_dbw_thrtl;
              out.fault_comms_vehicle = msg_gear_rpt_3_.fault_comms_vehicle;
              out.fault_comms_vehicle_1 = msg_gear_rpt_3_.fault_comms_vehicle_1;
              out.fault_comms_vehicle_2 = msg_gear_rpt_3_.fault_comms_vehicle_2;
              out.fault_comms_actuator = msg_gear_rpt_3_.fault_comms_actuator;
              out.fault_comms_actuator_1 = msg_gear_rpt_3_.fault_comms_actuator_1;
              out.fault_comms_actuator_2 = msg_gear_rpt_3_.fault_comms_actuator_2;
              out.fault_vehicle_speed = msg_gear_rpt_3_.fault_vehicle_speed;
              out.fault_param_mismatch = msg_gear_rpt_3_.fault_param_mismatch;
              out.fault_calibration = msg_gear_rpt_3_.fault_calibration;
              pub_gear_diag_->publish(out);
              if (msg.degraded) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded");
              } else if (degraded_prev) {
                RCLCPP_INFO(get_logger(), "Gear degraded state cleared");
              }
              if (msg.degraded_cmd_type) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Unsupported cmd_type");
              }
              if (msg.degraded_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms");
              }
              if (msg.degraded_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Internal");
              }
              if (msg.degraded_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Vehicle");
              }
              if (msg.degraded_actuator) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Actuator");
              }
              if (msg.fault_power) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Drive-By-Wire power voltage");
              }
              if (msg.fault_comms) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms");
              }
              if (msg.fault_internal) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Internal fault");
              }
              if (msg.fault_vehicle) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Fault in vehicle");
              }
              if (msg.fault_actuator) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Fault in actuator");
              }
            } else {
              RCLCPP_WARN(get_logger(), "Ignoring gear report 2 with repeated rolling counter value");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring gear report 2 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring gear report 2 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_steer_rpt_3_.ID:
        if (msg_can->dlc == sizeof(msg_steer_rpt_3_)) {
          memcpy(&msg_steer_rpt_3_, msg_can->data.data(), sizeof(msg_steer_rpt_3_));
          const auto &msg = msg_steer_rpt_3_;
          if (msg.validCrc()) {
            if (msg.degraded_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.degraded_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.degraded_comms_dbw_brake) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with other Drive-By-Wire brake module");
            }
            if (msg.degraded_comms_dbw_thrtl) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with other Drive-By-Wire throttle module");
            }
            if (msg.degraded_comms_dbw_gear) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with other Drive-By-Wire gear module");
            }
            if (msg.degraded_control_performance) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Control performance");
            }
            if (msg.degraded_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.degraded_comms_vehicle) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with vehicle module(s)");
            }
            if (msg.degraded_comms_actuator) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Lost comms with actuator");
            }
            if (msg.degraded_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Unknown or invalid vehicle speed");
            }
            if (msg.degraded_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering degraded: Calibration");
            }
            if (msg.fault_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.fault_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.fault_comms_dbw_brake) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with other Drive-By-Wire brake module");
            }
            if (msg.fault_comms_dbw_thrtl) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with other Drive-By-Wire throttle module");
            }
            if (msg.fault_comms_dbw_gear) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with other Drive-By-Wire gear module");
            }
            if (msg.fault_comms_vehicle) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with vehicle module(s)");
            }
            if (msg.fault_comms_actuator) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Lost comms with actuator");
            }
            if (msg.fault_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Vehicle speed");
            }
            if (msg.fault_actuator_torque_sensor) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Unknown steering column torque");
            }
            if (msg.fault_actuator_config) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Unsupported actuator configuration"
                                   ", Contact support@dataspeedinc.com if not resolved in a few minutes");
            }
            if (msg.fault_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.fault_param_limits) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Steering fault: Invalid limit parameters");
            }
            if (msg.fault_calibration) {
              const char *txt = "Steering calibration fault";
              if (firmware_.get(Platform::FORD_CD4,  Module::Gateway).valid()
               || firmware_.get(Platform::FORD_P5,   Module::Gateway).valid()
               || firmware_.get(Platform::FORD_T6,   Module::Gateway).valid()
               || firmware_.get(Platform::FORD_U6,   Module::Gateway).valid()
               || firmware_.get(Platform::FORD_CD5,  Module::Gateway).valid()
               || firmware_.get(Platform::FORD_GE1,  Module::Gateway).valid()
               || firmware_.get(Platform::FORD_P702, Module::Gateway).valid()) {
                txt = "Steering calibration fault. Drive at least 25 mph for at least 10 seconds in a straight line.";
              } else if (firmware_.get(Platform::FCA_RU,  Module::Gateway).valid()
                      || firmware_.get(Platform::FCA_WK2, Module::Gateway).valid()) {
                txt = "Steering calibration fault. Drive at least 25 mph for at least 10 seconds in a straight line.";
              } else if (firmware_.get(Platform::POLARIS_GEM,    Module::Gateway).valid()
                      || firmware_.get(Platform::POLARIS_RANGER, Module::Gateway).valid()
                      || firmware_.get(Platform::POLARIS_RZR,    Module::Gateway).valid()) {
                txt = "Steering calibration fault. Press the two steering multiplier buttons at the same "
                      "time to set the center offset when the wheel is straight. For a more exact "
                      "calibration set the SteeringCal and SteeringCal offset parameters using DbwConfig.";
              }
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "%s", txt);
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring steer report 3 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring steer report 3 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_brake_rpt_3_.ID:
        if (msg_can->dlc == sizeof(msg_brake_rpt_3_)) {
          memcpy(&msg_brake_rpt_3_, msg_can->data.data(), sizeof(msg_brake_rpt_3_));
          const auto &msg = msg_brake_rpt_3_;
          if (msg.validCrc()) {
            if (msg.degraded_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.degraded_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.degraded_comms_dbw_steer) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with other Drive-By-Wire steer module");
            }
            if (msg.degraded_comms_dbw_thrtl) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with other Drive-By-Wire throttle module");
            }
            if (msg.degraded_comms_dbw_gear) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with other Drive-By-Wire gear module");
            }
            if (msg.degraded_control_performance) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Control performance");
            }
            if (msg.degraded_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.degraded_comms_vehicle) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with vehicle module(s)");
            }
            if (msg.degraded_comms_actuator) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with actuator");
            }
            if (msg.degraded_comms_actuator_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with actuator 1");
            }
            if (msg.degraded_comms_actuator_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Lost comms with actuator 2");
            }
            if (msg.degraded_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Vehicle speed");
            }
            if (msg.degraded_btsi_stuck_low) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: BTSI stuck low");
            }
            if (msg.degraded_btsi_stuck_high) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: BTSI stuck high");
            }
            if (msg.degraded_actuator_aeb_deny) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: AEB deny from actuator");
            }
            if (msg.degraded_actuator_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Actuator 1");
            }
            if (msg.degraded_actuator_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Actuator 2");
            }
            if (msg.degraded_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake degraded: Calibration");
            }
            if (msg.fault_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.fault_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.fault_comms_dbw_steer) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with other Drive-By-Wire steer module");
            }
            if (msg.fault_comms_dbw_thrtl) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with other Drive-By-Wire throttle module");
            }
            if (msg.fault_comms_dbw_gear) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with other Drive-By-Wire gear module");
            }
            if (msg.fault_comms_vehicle) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with vehicle module(s)");
            }
            if (msg.fault_comms_actuator) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with actuator");
            }
            if (msg.fault_comms_actuator_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with actuator 1");
            }
            if (msg.fault_comms_actuator_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Lost comms with actuator 2");
            }
            if (msg.fault_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Vehicle speed");
            }
            if (msg.fault_actuator_acc_deny) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: ACC deny from actuator");
            }
            if (msg.fault_actuator_pedal_sensor) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Unknown pedal sensor value from actuator");
            }
            if (msg.fault_actuator_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Fault in actuator 1");
            }
            if (msg.fault_actuator_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Fault in actuator 2");
            }
            if (msg.fault_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.fault_param_limits) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Invalid limit parameters");
            }
            if (msg.fault_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Brake fault: Calibration");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring brake report 3 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring brake report 3 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_thrtl_rpt_3_.ID:
        if (msg_can->dlc == sizeof(msg_thrtl_rpt_3_)) {
          memcpy(&msg_thrtl_rpt_3_, msg_can->data.data(), sizeof(msg_thrtl_rpt_3_));
          const auto &msg = msg_thrtl_rpt_3_;
          if (msg.validCrc()) {
            if (msg.degraded_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.degraded_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.degraded_comms_dbw_steer) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost comms with other Drive-By-Wire steer module");
            }
            if (msg.degraded_comms_dbw_brake) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost comms with other Drive-By-Wire brake module");
            }
            if (msg.degraded_comms_dbw_gear) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost comms with other Drive-By-Wire gear module");
            }
            if (msg.degraded_control_performance) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Control performance");
            }
            if (msg.degraded_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.degraded_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Unknown or invalid vehicle speed");
            }
            if (msg.degraded_aped_feedback) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Lost accelerator pedal position feedback");
            }
            if (msg.degraded_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle degraded: Calibration");
            }
            if (msg.fault_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.fault_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.fault_comms_dbw_steer) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Lost comms with other Drive-By-Wire steer module");
            }
            if (msg.fault_comms_dbw_brake) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Lost comms with other Drive-By-Wire brake module");
            }
            if (msg.fault_comms_dbw_gear) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Lost comms with other Drive-By-Wire gear module");
            }
            if (msg.fault_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Unknown or invalid vehicle speed");
            }
            if (msg.fault_aped_sensor_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Accelerator pedal position sensor channel 1 out of range");
            }
            if (msg.fault_aped_sensor_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Accelerator pedal position sensor channel 2 out of range");
            }
            if (msg.fault_aped_sensor_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Accelerator pedal position sensor dual channel mismatch");
            }
            if (msg.fault_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.fault_param_limits) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Invalid limit parameters");
            }
            if (msg.fault_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Throttle fault: Calibration");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring throttle report 3 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring throttle report 3 with invalid size of %u", msg_can->dlc);
        }
        break;

      case msg_gear_rpt_3_.ID:
        if (msg_can->dlc == sizeof(msg_gear_rpt_3_)) {
          memcpy(&msg_gear_rpt_3_, msg_can->data.data(), sizeof(msg_gear_rpt_3_));
          const auto &msg = msg_gear_rpt_3_;
          if (msg.validCrc()) {
            if (msg.degraded_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.degraded_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.degraded_comms_dbw_steer) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with other Drive-By-Wire steer module");
            }
            if (msg.degraded_comms_dbw_brake) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with other Drive-By-Wire brake module");
            }
            if (msg.degraded_comms_dbw_thrtl) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with other Drive-By-Wire throttle module");
            }
            if (msg.degraded_control_performance) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Control performance");
            }
            if (msg.degraded_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.degraded_comms_vehicle) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with vehicle module(s)");
            }
            if (msg.degraded_comms_vehicle_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost primary comms with vehicle module(s)");
            }
            if (msg.degraded_comms_vehicle_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost secondary comms with vehicle module(s)");
            }
            if (msg.degraded_comms_actuator) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost comms with actuator");
            }
            if (msg.degraded_comms_actuator_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost primary comms with actuator");
            }
            if (msg.degraded_comms_actuator_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Lost secondary comms with actuator");
            }
            if (msg.degraded_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Unknown or invalid vehicle speed");
            }
            if (msg.degraded_gear_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Sustained gear state mismatch between primary and secondary signals");
            }
            if (msg.degraded_power) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Secondary power source unavailable");
            }
            if (msg.degraded_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear degraded: Calibration");
            }
            if (msg.fault_comms_dbw) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with other Drive-By-Wire module(s)");
            }
            if (msg.fault_comms_dbw_gateway) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with other Drive-By-Wire gateway module");
            }
            if (msg.fault_comms_dbw_steer) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with other Drive-By-Wire steer module");
            }
            if (msg.fault_comms_dbw_brake) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with other Drive-By-Wire brake module");
            }
            if (msg.fault_comms_dbw_thrtl) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with other Drive-By-Wire throttle module");
            }
            if (msg.fault_comms_vehicle) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with vehicle module(s)");
            }
            if (msg.fault_comms_vehicle_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost primary comms with vehicle module(s)");
            }
            if (msg.fault_comms_vehicle_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost secondary comms with vehicle module(s)");
            }
            if (msg.fault_comms_actuator) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost comms with actuator");
            }
            if (msg.fault_comms_actuator_1) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost primary comms with actuator");
            }
            if (msg.fault_comms_actuator_2) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Lost secondary comms with actuator");
            }
            if (msg.fault_vehicle_speed) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Unknown or invalid vehicle speed");
            }
            if (msg.fault_param_mismatch) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: System parameter mismatch with other Drive-By-Wire modules");
            }
            if (msg.fault_calibration) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10e3, "Gear fault: Calibration");
            }
          } else {
            RCLCPP_WARN(get_logger(), "Ignoring gear report 3 with invalid CRC");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring gear report 3 with invalid size of %u", msg_can->dlc);
        }
        break;
      
      case msg_tire_pressure_.ID:
        if (msg_can->dlc == sizeof(msg_tire_pressure_)) {
          memcpy(&msg_tire_pressure_, msg_can->data.data(), sizeof(msg_tire_pressure_));
          const auto &msg = msg_tire_pressure_;
          ds_dbw_msgs::msg::TirePressures out;
          out.header.stamp = msg_can->header.stamp;
          out.front_left  = msg.frontLeftKPa();
          out.front_right = msg.frontRightKPa();
          out.rear_left   = msg.rearRightKPa();
          out.rear_right  = msg.rearRightKPa();
          out.spare  = msg.spareKPa();
          pub_tire_pressures_->publish(out);
        } else {
          RCLCPP_WARN(get_logger(), "Ignoring tire pressure report with invalid size of %u", msg_can->dlc);
        }
        break;

      case MsgSteerParamHash::ID:
        if (msg_can->dlc >= sizeof(MsgSteerParamHash)) {
          MsgSteerParamHash msg;
          memcpy(&msg, msg_can->data.data(), sizeof(msg));
          if (param_hash_.steer != msg.hash) {
            param_hash_.steer = msg.hash;
            RCLCPP_INFO(get_logger(), "Steer param hash: %08X", msg.hash);
          }
        }
        break;
      case MsgBrakeParamHash::ID:
        if (msg_can->dlc >= sizeof(MsgBrakeParamHash)) {
          MsgBrakeParamHash msg;
          memcpy(&msg, msg_can->data.data(), sizeof(msg));
          if (param_hash_.brake != msg.hash) {
            param_hash_.brake = msg.hash;
            RCLCPP_INFO(get_logger(), "Brake param hash: %08X", msg.hash);
          }
        }
        break;
      case MsgThrtlParamHash::ID:
        if (msg_can->dlc >= sizeof(MsgThrtlParamHash)) {
          MsgThrtlParamHash msg;
          memcpy(&msg, msg_can->data.data(), sizeof(msg));
          if (param_hash_.thrtl != msg.hash) {
            param_hash_.thrtl = msg.hash;
            RCLCPP_INFO(get_logger(), "Throttle param hash: %08X", msg.hash);
          }
        }
        break;

      case MsgEcuInfoGateway::ID:
      case MsgEcuInfoSteer::ID:
      case MsgEcuInfoBrake::ID:
      case MsgEcuInfoThrottle::ID:
      case MsgEcuInfoShift::ID:
      case MsgEcuInfoBOO::ID:
        if (msg_can->dlc >= sizeof(MsgEcuInfo)) {
          using Mux = MsgEcuInfo::Mux;
          auto &ecu_info = ecu_info_.msg[msg_can->id];
          MsgEcuInfo msg;
          memcpy(&msg, msg_can->data.data(), sizeof(msg));
          const Module module = (Module)msg_can->id;
          const char *str_m = moduleToString(module);
          RCLCPP_DEBUG(get_logger(), "ECU_INFO(%x,%02X,%s)", (uint16_t)module, (uint8_t)msg.mux, str_m);
          switch (msg.mux) {
            case Mux::Version: {
              const PlatformVersion version((Platform)msg.version.platform, module, msg.version.major, msg.version.minor, msg.version.build);
              const ModuleVersion latest = FIRMWARE_LATEST.get(version);
              const char *str_p = platformToString(version.p);
              ecu_info.name = std::string(str_p) + "_" + trim(str_m);
              ecu_info.version = std::to_string(version.v.major())
                         + "." + std::to_string(version.v.minor())
                         + "." + std::to_string(version.v.build());
              if (firmware_.get(version) != version.v) {
                firmware_.put(version);
                if (latest.valid()) {
                  RCLCPP_INFO(get_logger(), "Detected %s %s firmware version %u.%u.%u", str_p, str_m,
                              msg.version.major, msg.version.minor, msg.version.build);
                } else {
                  RCLCPP_WARN(get_logger(), "Detected %s %s firmware version %u.%u.%u, which is unsupported. Platform: 0x%02X, Module: %u",
                              str_p, str_m, msg.version.major, msg.version.minor, msg.version.build, msg.version.platform, (uint16_t)module);
                }
                if (version < latest) {
                  RCLCPP_WARN(get_logger(), "Firmware %s %s has old  version %u.%u.%u, updating to %u.%u.%u is suggested.", str_p, str_m,
                              version.v.major(), version.v.minor(), version.v.build(),
                              latest.major(), latest.minor(), latest.build());
                }
              }
              break; }
            case Mux::CfgHash:
              ecu_info.config_hash = "XXXXXXXX";
              snprintf(ecu_info.config_hash.data(), ecu_info.config_hash.size() + 1, "%08X", msg.cfg.hash);
              if (ecu_info_.cfg_hash[module] != msg.cfg.hash) {
                ecu_info_.cfg_hash[module] = msg.cfg.hash;
                RCLCPP_INFO(get_logger(), "EcuInfo: %s config hash: %08X", str_m, msg.cfg.hash);
              }
              ecu_info.config_count_modified = msg.cfg.count_modified;
              ecu_info.config_count_configured = msg.cfg.count_configured;
              ecu_info.config_nvm_blank = msg.cfg.nvm_blank;
              ecu_info.config_nvm_write_pending = msg.cfg.nvm_write_pending;
              break;
            case Mux::MacAddr: {
              ecu_info.mac_addr = "XX:XX:XX:XX:XX:XX";
              snprintf(ecu_info.mac_addr.data(), ecu_info.mac_addr.size() + 1, "%02X:%02X:%02X:%02X:%02X:%02X",
                       msg.mac.addr0, msg.mac.addr1, msg.mac.addr2,
                       msg.mac.addr3, msg.mac.addr4, msg.mac.addr5);
              std::array<uint8_t,6> mac;
              mac[0] = msg.mac.addr0;
              mac[1] = msg.mac.addr1;
              mac[2] = msg.mac.addr2;
              mac[3] = msg.mac.addr3;
              mac[4] = msg.mac.addr4;
              mac[5] = msg.mac.addr5;
              if (ecu_info_.mac[module] != mac) {
                ecu_info_.mac[module] = mac;
                RCLCPP_INFO(get_logger(), "EcuInfo: %s MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                            str_m, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
              }
              break; }
            case Mux::License0:
            case Mux::License1:
            case Mux::License2:
            case Mux::License3:
            case Mux::License4:
            case Mux::License5:
            case Mux::License6:
            case Mux::License7: {
                constexpr std::array<const char *, 8> NAME = {"BASE", "CONTROL", "SENSORS", "REMOTE", "", "", "", ""};
                constexpr std::array<bool, 8> WARN = {true, true, true, false, true, true, true, true};
                const uint8_t i = (uint8_t)msg.mux - (uint8_t)Mux::License0;
                const int id = (uint16_t)module * NAME.size() + i;
                const std::string name = strcmp(NAME[i], "") ? NAME[i] : std::string(1, '0' + i);
                if (i == 1) { // "CONTROL"
                  ecu_info.control_licensed = msg.license.enabled;
                }
                if (msg.license.ready) {
                  RCLCPP_INFO_ONCE_ID(get_logger(), module, "Licensing: %s ready", str_m);
                  if (msg.license.trial) {
                    RCLCPP_WARN_ONCE_ID(get_logger(), id,
                        "Licensing: %s feature '%s' licensed as a counted trial. Visit "
                        "https://www.dataspeedinc.com/products/maintenance-subscription/ to request a full license.",
                        str_m, name.c_str());
                  }
                } else if (module == Module::Gateway) {
                  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10e3, "Licensing: Waiting for VIN...");
                } else {
                  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10e3, "Licensing: Waiting for required info...");
                }
                if (msg.license.enabled) {
                  RCLCPP_INFO_ONCE_ID(get_logger(), id, "Licensing: %s feature '%s' enabled%s", str_m, name.c_str(), msg.license.trial ? " as a counted trial" : "");
                } else if (msg.license.ready && !WARN[i]) {
                  RCLCPP_INFO_ONCE_ID(get_logger(), id, "Licensing: %s feature '%s' not licensed.", str_m, name.c_str());
                } else if (msg.license.ready) {
                  RCLCPP_WARN_ONCE_ID(get_logger(), id,
                                      "Licensing: %s feature '%s' not licensed. Visit "
                                      "https://www.dataspeedinc.com/products/maintenance-subscription/ to request a license.",
                                      str_m, name.c_str());
                }
                if (msg.license.ready && (module == Module::Gateway) && (msg.license.trial || (!msg.license.enabled && WARN[i]))) {
                  RCLCPP_INFO_ONCE(get_logger(), "Licensing: Feature '%s' trials used: %u, remaining: %u", name.c_str(),
                                   msg.license.trials_used, msg.license.trials_left);
                }
              break; }
            case Mux::LicenseDate0: {
              std::string &date = ecu_info_.ldate_recv[module];
              date.clear();
              date.push_back(msg.ldate0.date0);
              date.push_back(msg.ldate0.date1);
              date.push_back(msg.ldate0.date2);
              date.push_back(msg.ldate0.date3);
              date.push_back(msg.ldate0.date4);
              date.push_back(msg.ldate0.date5);
              date.push_back(msg.ldate0.date6);
              break; }
            case Mux::LicenseDate1: {
              std::string &date = ecu_info_.ldate_recv[module];
              if (date.size() == 7) {
                date.push_back(msg.ldate1.date7);
                date.push_back(msg.ldate1.date8);
                date.push_back(msg.ldate1.date9);
                ecu_info.license_date = date;
                if (ecu_info_.ldate[module] != date) {
                  ecu_info_.ldate[module] = date;
                  RCLCPP_INFO(get_logger(), "EcuInfo: %s license date: %s", str_m, date.c_str());
                }
              }
              break; }
            case Mux::BuildDate0: {
              std::string &date = ecu_info_.bdate_recv[module];
              date.clear();
              date.push_back(msg.bdate0.date0);
              date.push_back(msg.bdate0.date1);
              date.push_back(msg.bdate0.date2);
              date.push_back(msg.bdate0.date3);
              date.push_back(msg.bdate0.date4);
              date.push_back(msg.bdate0.date5);
              date.push_back(msg.bdate0.date6);
              break; }
            case Mux::BuildDate1: {
              std::string &date = ecu_info_.bdate_recv[module];
              if (date.size() == 7) {
                date.push_back(msg.bdate0.date0);
                date.push_back(msg.bdate0.date1);
                date.push_back(msg.bdate0.date2);
                date.push_back(msg.bdate0.date3);
                date.push_back(msg.bdate0.date4);
                date.push_back(msg.bdate0.date5);
                date.push_back(msg.bdate0.date6);
                ecu_info.build_date = date;
                if (!ecu_info.name.empty()
                 && !ecu_info.version.empty()
                 && !ecu_info.mac_addr.empty()
                 && !ecu_info.config_hash.empty()
                 && !ecu_info.build_date.empty()) {
                  ecu_info.header.stamp = msg_can->header.stamp;
                  pub_ecu_info_->publish(ecu_info);
                }
                if (ecu_info_.bdate[module] != date) {
                  ecu_info_.bdate[module] = date;
                  RCLCPP_INFO(get_logger(), "EcuInfo: %s firmware build date: %s", str_m, date.c_str());
                }
              }
              break; }
            case Mux::VIN0:
              ecu_info_.vin_recv.clear();
              ecu_info_.vin_recv.push_back(msg.vin0.vin00);
              ecu_info_.vin_recv.push_back(msg.vin0.vin01);
              ecu_info_.vin_recv.push_back(msg.vin0.vin02);
              ecu_info_.vin_recv.push_back(msg.vin0.vin03);
              ecu_info_.vin_recv.push_back(msg.vin0.vin04);
              ecu_info_.vin_recv.push_back(msg.vin0.vin05);
              ecu_info_.vin_recv.push_back(msg.vin0.vin06);
              break;
            case Mux::VIN1:
              if (ecu_info_.vin_recv.size() == 7) {
                ecu_info_.vin_recv.push_back(msg.vin1.vin07);
                ecu_info_.vin_recv.push_back(msg.vin1.vin08);
                ecu_info_.vin_recv.push_back(msg.vin1.vin09);
                ecu_info_.vin_recv.push_back(msg.vin1.vin10);
                ecu_info_.vin_recv.push_back(msg.vin1.vin11);
                ecu_info_.vin_recv.push_back(msg.vin1.vin12);
                ecu_info_.vin_recv.push_back(msg.vin1.vin13);
              }
              break;
            case Mux::VIN2:
              if (ecu_info_.vin_recv.size() == 14) {
                ecu_info_.vin_recv.push_back(msg.vin2.vin14);
                ecu_info_.vin_recv.push_back(msg.vin2.vin15);
                ecu_info_.vin_recv.push_back(msg.vin2.vin16);
                if (ecu_info_.vin != ecu_info_.vin_recv) {
                  ecu_info_.vin = ecu_info_.vin_recv;
                  RCLCPP_INFO(get_logger(), "VIN: %s", ecu_info_.vin.c_str());
                }
                std_msgs::msg::String msg;
                msg.data = ecu_info_.vin;
                pub_vin_->publish(msg);
              }
              break;
            default:
              RCLCPP_WARN_ONCE_ID(get_logger(), msg.mux, "EcuInfo: %s: Unknown mux: %02X", str_m, (uint8_t)msg.mux);
              break;
          }
        }
        break;

      case msg_steer_cmd_.ID: RCLCPP_WARN_EXPRESSION(get_logger(), warn_cmds_, WARN_CMD_TXT, "Steer",    msg_steer_cmd_.ID); break;
      case msg_brake_cmd_.ID: RCLCPP_WARN_EXPRESSION(get_logger(), warn_cmds_, WARN_CMD_TXT, "Brake",    msg_brake_cmd_.ID); break;
      case msg_thrtl_cmd_.ID: RCLCPP_WARN_EXPRESSION(get_logger(), warn_cmds_, WARN_CMD_TXT, "Throttle", msg_thrtl_cmd_.ID); break;
      case msg_gear_cmd_.ID:  RCLCPP_WARN_EXPRESSION(get_logger(), warn_cmds_, WARN_CMD_TXT, "Gear",     msg_gear_cmd_.ID);  break;

      case MsgSteerCmdRmt::ID:
      case MsgBrakeCmdRmt::ID:
      case MsgThrtlCmdRmt::ID:
      case MsgGearCmdRmt::ID:
        break;

      case MsgBrakeCmdUlc::ID:
      case MsgThrtlCmdUlc::ID:
      case MsgGearCmdUlc::ID:
        break;

      case MsgReserved1::ID:
      case MsgReserved2::ID:
        break;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
      case 0x400 ... 0x43F: // Power Distribution with iPDS
        break;
      case 0x060 ... 0x07F: // Legacy Drive-By-Wire (DBW1)
        if (warn_unknown_) {
          RCLCPP_WARN_ONCE_ID(get_logger(), msg_can->id, "Received unsupported CAN ID %03X from legacy drive-by-wire system (DBW1)"
                                                         "\nUse the legacy dbw_fca/dbw_ford/dbw_polaris packages instead", msg_can->id);
        }
        break;
#pragma GCC diagnostic pop

      default:
        if (warn_unknown_) {
          RCLCPP_WARN_ONCE_ID(get_logger(), msg_can->id, "Received unknown CAN ID: %03X", msg_can->id);
        }
        break;
    }
  } else if (msg_can->is_extended) {
    if (warn_unknown_) {
      RCLCPP_WARN_ONCE_ID(get_logger(), msg_can->id, "Received unknown CAN ID: %08X", msg_can->id);
    }
  }
}

void DbwNode::recvCanImu(const std::vector<can_msgs::msg::Frame::ConstSharedPtr> &msgs) {
  assert(msgs.size() == 2);
  assert(msgs[0]->id == MsgAccel::ID);
  assert(msgs[1]->id == MsgGyro::ID);
  assert(msgs[0]->dlc == sizeof(MsgAccel));
  assert(msgs[1]->dlc == sizeof(MsgGyro));
  const MsgAccel &msg_accel = *reinterpret_cast<const MsgAccel *>(msgs[0]->data.data());
  const MsgGyro &msg_gyro = *reinterpret_cast<const MsgGyro *>(msgs[1]->data.data());
  sensor_msgs::msg::Imu out;
  out.header.stamp = msgs[0]->header.stamp;
  out.header.frame_id = frame_id_;
  out.orientation_covariance[0] = -1; // Orientation not present
  out.linear_acceleration.x = msg_accel.accelXMps2();
  out.linear_acceleration.y = msg_accel.accelYMps2();
  out.linear_acceleration.z = msg_accel.accelZMps2();
  out.angular_velocity.x = msg_gyro.gyroXRadS();
  out.angular_velocity.y = msg_gyro.gyroYRadS();
  out.angular_velocity.z = msg_gyro.gyroZRadS();
  pub_imu_->publish(out);
  printSyncDelta(msgs[0], msgs[1], "imu");
}

void DbwNode::recvCanMisc(const std::vector<can_msgs::msg::Frame::ConstSharedPtr> &msgs) {
  static_assert(ds_dbw_msgs::msg::TurnSignal::NONE   == (uint8_t)TurnSignal::None);
  static_assert(ds_dbw_msgs::msg::TurnSignal::LEFT   == (uint8_t)TurnSignal::Left);
  static_assert(ds_dbw_msgs::msg::TurnSignal::RIGHT  == (uint8_t)TurnSignal::Right);
  static_assert(ds_dbw_msgs::msg::TurnSignal::HAZARD == (uint8_t)TurnSignal::Hazard);
  static_assert(ds_dbw_msgs::msg::PrkBrkStat::UNKNOWN    == (uint8_t)MsgMiscReport1::PrkBrkStat::Unknown);
  static_assert(ds_dbw_msgs::msg::PrkBrkStat::ON         == (uint8_t)MsgMiscReport1::PrkBrkStat::On);
  static_assert(ds_dbw_msgs::msg::PrkBrkStat::OFF        == (uint8_t)MsgMiscReport1::PrkBrkStat::Off);
  static_assert(ds_dbw_msgs::msg::PrkBrkStat::TRANSITION == (uint8_t)MsgMiscReport1::PrkBrkStat::Transition);
  assert(msgs.size() == 2);
  assert(msgs[0]->id == MsgMiscReport1::ID);
  assert(msgs[1]->id == MsgMiscReport2::ID);
  assert(msgs[0]->dlc == sizeof(MsgMiscReport1));
  assert(msgs[1]->dlc == sizeof(MsgMiscReport2));
  const MsgMiscReport1 &msg1 = *reinterpret_cast<const MsgMiscReport1 *>(msgs[0]->data.data());
  const MsgMiscReport2 &msg2 = *reinterpret_cast<const MsgMiscReport2 *>(msgs[1]->data.data());
  ds_dbw_msgs::msg::MiscReport out;
  out.header.stamp = msgs[0]->header.stamp;
  out.turn_signal.value = (uint8_t)msg1.turn_signal;
  out.parking_brake.value = (uint8_t)msg1.parking_brake;
  out.passenger_detect = msg1.pasngr_detect;
  out.passenger_airbag = msg1.pasngr_airbag;
  out.buckle_driver = msg1.buckle_driver;
  out.buckle_passenger = msg1.buckle_pasngr;
  out.door_driver = msg1.door_driver;
  out.door_passenger = msg1.door_passenger;
  out.door_rear_left = msg1.door_rear_left;
  out.door_rear_right = msg1.door_rear_right;
  out.door_hood = msg1.door_hood;
  out.door_trunk = msg1.door_trunk;
  out.btn_ld_ok = msg1.btn_ld_ok;
  out.btn_ld_up = msg1.btn_ld_up;
  out.btn_ld_down = msg1.btn_ld_down;
  out.btn_ld_left = msg1.btn_ld_left;
  out.btn_ld_right = msg1.btn_ld_right;
  out.btn_rd_ok = msg1.btn_rd_ok;
  out.btn_rd_up = msg1.btn_rd_up;
  out.btn_rd_down = msg1.btn_rd_down;
  out.btn_rd_left = msg1.btn_rd_left;
  out.btn_rd_right = msg1.btn_rd_right;
  out.btn_cc_mode = msg1.btn_cc_mode;
  out.btn_cc_on = msg1.btn_cc_on;
  out.btn_cc_off = msg1.btn_cc_off;
  out.btn_cc_res = msg1.btn_cc_res;
  out.btn_cc_cncl = msg1.btn_cc_cncl;
  out.btn_cc_on_off = msg1.btn_cc_on_off;
  out.btn_cc_res_cncl = msg1.btn_cc_res_cncl;
  out.btn_cc_res_inc = msg1.btn_cc_res_inc;
  out.btn_cc_res_dec = msg1.btn_cc_res_dec;
  out.btn_cc_set_inc = msg1.btn_cc_set_inc;
  out.btn_cc_set_dec = msg1.btn_cc_set_dec;
  out.btn_acc_gap_inc = msg1.btn_acc_gap_inc;
  out.btn_acc_gap_dec = msg1.btn_acc_gap_dec;
  out.btn_limit_on_off = msg1.btn_limit_on_off;
  out.btn_la_on_off = msg1.btn_la_on_off;
  out.btn_apa = msg1.btn_apa;
  out.btn_media = msg1.btn_media;
  out.btn_vol_inc = msg1.btn_vol_inc;
  out.btn_vol_dec = msg1.btn_vol_dec;
  out.btn_mute = msg1.btn_mute;
  out.btn_speak = msg1.btn_speak;
  out.btn_prev = msg1.btn_prev;
  out.btn_next = msg1.btn_next;
  out.btn_call_start = msg1.btn_call_start;
  out.btn_call_end = msg1.btn_call_end;
  (void)msg2;
  pub_misc_->publish(out);
  printSyncDelta(msgs[0], msgs[1], "misc");
}

void DbwNode::recvSteeringCmd(const ds_dbw_msgs::msg::SteeringCmd::ConstSharedPtr msg) {
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_NONE      == (uint8_t)MsgSteerCmd::CmdType::None);
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_TORQUE    == (uint8_t)MsgSteerCmd::CmdType::Torque);
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_ANGLE     == (uint8_t)MsgSteerCmd::CmdType::Angle);
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_CURVATURE == (uint8_t)MsgSteerCmd::CmdType::Curvature);
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE  == (uint8_t)MsgSteerCmd::CmdType::YawRate);
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_PERCENT   == (uint8_t)MsgSteerCmd::CmdType::Percent);
  static_assert(ds_dbw_msgs::msg::SteeringCmd::CMD_CALIBRATE == (uint8_t)MsgSteerCmd::CmdType::Calibrate);
  if (std::isnan(msg->cmd) && msg->cmd_type != ds_dbw_msgs::msg::SteeringCmd::CMD_NONE) {
    RCLCPP_WARN(get_logger(), "NaN steering command");
  }
  msg_steer_cmd_.reset();
  switch (msg->cmd_type) {
    default:
      RCLCPP_WARN(get_logger(), "Unknown steer command type: %u", msg->cmd_type);
      [[fallthrough]];
    case ds_dbw_msgs::msg::SteeringCmd::CMD_NONE:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::None;
      break;
    case ds_dbw_msgs::msg::SteeringCmd::CMD_TORQUE:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::Torque;
      msg_steer_cmd_.setCmdTorqueNm(msg->cmd);
      break;
    case ds_dbw_msgs::msg::SteeringCmd::CMD_ANGLE:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::Angle;
      msg_steer_cmd_.setCmdAngleDeg(msg->cmd, msg->cmd_rate, msg->cmd_accel);
      break;
    case ds_dbw_msgs::msg::SteeringCmd::CMD_CURVATURE:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::Curvature;
      msg_steer_cmd_.setCmdCurvMDeg(msg->cmd, msg->cmd_rate, msg->cmd_accel);
      break;
    case ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::YawRate;
      msg_steer_cmd_.setCmdYawRateDegS(msg->cmd * (float)(180 / M_PI), msg->cmd_rate, msg->cmd_accel);
      break;
    case ds_dbw_msgs::msg::SteeringCmd::CMD_PERCENT:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::Percent;
      msg_steer_cmd_.setCmdPercentDeg(msg->cmd, msg->cmd_rate, msg->cmd_accel);
      break;
    case ds_dbw_msgs::msg::SteeringCmd::CMD_CALIBRATE:
      msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::Calibrate;
      msg_steer_cmd_.setCmdAngleDeg(msg->cmd);
      break;
  }
  if (modeSyncNone()) {
    msg_steer_cmd_.enable = msg->enable && enabled();
    msg_steer_cmd_.clear = msg->clear || (enable_ && msg_steer_rpt_1_.override_latched);
  } else {
    msg_steer_cmd_.enable = msg->enable;
    msg_steer_cmd_.clear = msg->clear || (msg_steer_cmd_clear_ && msg_steer_rpt_1_.override_latched);
  }
  msg_steer_cmd_.ignore = msg->ignore;
  msg_steer_cmd_.rc++;
  msg_steer_cmd_.setCrc();
  msg_steer_cmd_clear_ = false;

  pub_can_->publish(FrameFromDbw(msg_steer_cmd_));
}

void DbwNode::recvBrakeCmd(const ds_dbw_msgs::msg::BrakeCmd::ConstSharedPtr msg) {
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_NONE      == (uint8_t)MsgBrakeCmd::CmdType::None);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_PRESSURE  == (uint8_t)MsgBrakeCmd::CmdType::Pressure);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_TORQUE    == (uint8_t)MsgBrakeCmd::CmdType::Torque);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL     == (uint8_t)MsgBrakeCmd::CmdType::Accel);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_ACC == (uint8_t)MsgBrakeCmd::CmdType::AccelAcc);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_AEB == (uint8_t)MsgBrakeCmd::CmdType::AccelAeb);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_PEDAL_RAW == (uint8_t)MsgBrakeCmd::CmdType::PedalRaw);
  static_assert(ds_dbw_msgs::msg::BrakeCmd::CMD_PERCENT   == (uint8_t)MsgBrakeCmd::CmdType::Percent);
  if (std::isnan(msg->cmd) && msg->cmd_type != ds_dbw_msgs::msg::BrakeCmd::CMD_NONE) {
    RCLCPP_WARN(get_logger(), "NaN brake command");
  }
  msg_brake_cmd_.reset();
  switch (msg->cmd_type) {
    default:
      RCLCPP_WARN(get_logger(), "Unknown brake command type: %u", msg->cmd_type);
      [[fallthrough]];
    case ds_dbw_msgs::msg::BrakeCmd::CMD_NONE:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::None;
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_PRESSURE:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::Pressure;
      msg_brake_cmd_.setCmdPressureBar(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_TORQUE:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::Torque;
      msg_brake_cmd_.setCmdTorqueNm(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::Accel;
      msg_brake_cmd_.setCmdAccelMpS(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_ACC:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::AccelAcc;
      msg_brake_cmd_.setCmdAccelMpS(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_AEB:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::AccelAeb;
      msg_brake_cmd_.setCmdAccelMpS(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_PEDAL_RAW:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::PedalRaw;
      msg_brake_cmd_.setCmdPercent(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::BrakeCmd::CMD_PERCENT:
      msg_brake_cmd_.cmd_type = MsgBrakeCmd::CmdType::Percent;
      msg_brake_cmd_.setCmdPercent(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
  }
  if (modeSyncNone()) {
    msg_brake_cmd_.enable = msg->enable && enabled();
    msg_brake_cmd_.clear = msg->clear || (enable_ && msg_brake_rpt_1_.override_latched);
  } else {
    msg_brake_cmd_.enable = msg->enable;
    msg_brake_cmd_.clear = msg->clear || (msg_brake_cmd_clear_ && msg_brake_rpt_1_.override_latched);
  }
  msg_brake_cmd_.ignore = msg->ignore;
  msg_brake_cmd_.rc++;
  msg_brake_cmd_.setCrc();
  msg_brake_cmd_clear_ = false;

  pub_can_->publish(FrameFromDbw(msg_brake_cmd_));
}

void DbwNode::recvThrottleCmd(const ds_dbw_msgs::msg::ThrottleCmd::ConstSharedPtr msg) {
  static_assert(ds_dbw_msgs::msg::ThrottleCmd::CMD_NONE      == (uint8_t)MsgThrtlCmd::CmdType::None);
  static_assert(ds_dbw_msgs::msg::ThrottleCmd::CMD_PEDAL_RAW == (uint8_t)MsgThrtlCmd::CmdType::PedalRaw);
  static_assert(ds_dbw_msgs::msg::ThrottleCmd::CMD_PERCENT   == (uint8_t)MsgThrtlCmd::CmdType::Percent);
  if (std::isnan(msg->cmd) && msg->cmd_type != ds_dbw_msgs::msg::ThrottleCmd::CMD_NONE) {
    RCLCPP_WARN(get_logger(), "NaN throttle command");
  }
  msg_thrtl_cmd_.reset();
  switch (msg->cmd_type) {
    default:
      RCLCPP_WARN(get_logger(), "Unknown throttle command type: %u", msg->cmd_type);
      [[fallthrough]];
    case ds_dbw_msgs::msg::ThrottleCmd::CMD_NONE:
      msg_thrtl_cmd_.cmd_type = MsgThrtlCmd::CmdType::None;
      break;
    case ds_dbw_msgs::msg::ThrottleCmd::CMD_PEDAL_RAW:
      msg_thrtl_cmd_.cmd_type = MsgThrtlCmd::CmdType::PedalRaw;
      msg_thrtl_cmd_.setCmdPercent(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
    case ds_dbw_msgs::msg::ThrottleCmd::CMD_PERCENT:
      msg_thrtl_cmd_.cmd_type = MsgThrtlCmd::CmdType::Percent;
      msg_thrtl_cmd_.setCmdPercent(msg->cmd, msg->rate_inc, msg->rate_dec);
      break;
  }
  if (modeSyncNone()) {
    msg_thrtl_cmd_.enable = msg->enable && enabled();
    msg_thrtl_cmd_.clear = msg->clear || (enable_ && msg_thrtl_rpt_1_.override_latched);
  } else {
    msg_thrtl_cmd_.enable = msg->enable;
    msg_thrtl_cmd_.clear = msg->clear || (msg_thrtl_cmd_clear_ && msg_thrtl_rpt_1_.override_latched);
  }
  msg_thrtl_cmd_.ignore = msg->ignore;
  msg_thrtl_cmd_.rc++;
  msg_thrtl_cmd_.setCrc();
  msg_thrtl_cmd_clear_ = false;

  pub_can_->publish(FrameFromDbw(msg_thrtl_cmd_));
}

void DbwNode::recvGearCmd(const ds_dbw_msgs::msg::GearCmd::ConstSharedPtr msg) {
  static_assert(ds_dbw_msgs::msg::Gear::NONE    == (uint8_t)Gear::None);
  static_assert(ds_dbw_msgs::msg::Gear::PARK    == (uint8_t)Gear::Park);
  static_assert(ds_dbw_msgs::msg::Gear::REVERSE == (uint8_t)Gear::Reverse);
  static_assert(ds_dbw_msgs::msg::Gear::NEUTRAL == (uint8_t)Gear::Neutral);
  static_assert(ds_dbw_msgs::msg::Gear::DRIVE   == (uint8_t)Gear::Drive);
  static_assert(ds_dbw_msgs::msg::Gear::LOW     == (uint8_t)Gear::Low);
  msg_gear_cmd_.reset();
  if (!modeSyncNone() || enabled()) {
    switch (msg->cmd.value) {
      default:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Unknown gear command: %u", msg->cmd.value);
        [[fallthrough]];
      case ds_dbw_msgs::msg::Gear::NONE:    msg_gear_cmd_.cmd = Gear::None;    break;
      case ds_dbw_msgs::msg::Gear::PARK:    msg_gear_cmd_.cmd = Gear::Park;    break;
      case ds_dbw_msgs::msg::Gear::REVERSE: msg_gear_cmd_.cmd = Gear::Reverse; break;
      case ds_dbw_msgs::msg::Gear::NEUTRAL: msg_gear_cmd_.cmd = Gear::Neutral; break;
      case ds_dbw_msgs::msg::Gear::DRIVE:   msg_gear_cmd_.cmd = Gear::Drive;   break;
      case ds_dbw_msgs::msg::Gear::LOW:     msg_gear_cmd_.cmd = Gear::Low;     break;
    }
  }
  msg_gear_cmd_.setCrc();
  pub_can_->publish(FrameFromDbw(msg_gear_cmd_));
}

void DbwNode::recvMiscCmd(const ds_dbw_msgs::msg::MiscCmd::ConstSharedPtr msg) {
  msg_misc_cmd_.reset();
  if (!modeSyncNone() || enabled()) {
    switch (msg->turn_signal.value) {
      default:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Unknown turn signal command: %u", msg->turn_signal.value);
        [[fallthrough]];
      case ds_dbw_msgs::msg::TurnSignal::NONE:  msg_misc_cmd_.turn_signal_cmd = TurnSignal::None;  break;
      case ds_dbw_msgs::msg::TurnSignal::LEFT:  msg_misc_cmd_.turn_signal_cmd = TurnSignal::Left;  break;
      case ds_dbw_msgs::msg::TurnSignal::RIGHT: msg_misc_cmd_.turn_signal_cmd = TurnSignal::Right; break;
    }
    switch (msg->parking_brake.value) {
      default:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Unknown parking brake command: %u", msg->parking_brake.value);
        [[fallthrough]];
      case ds_dbw_msgs::msg::PrkBrkCmd::NONE: msg_misc_cmd_.parking_brake_cmd = MsgMiscCmd::PrkBrkCmd::None; break;
      case ds_dbw_msgs::msg::PrkBrkCmd::ON:   msg_misc_cmd_.parking_brake_cmd = MsgMiscCmd::PrkBrkCmd::On;   break;
      case ds_dbw_msgs::msg::PrkBrkCmd::OFF:  msg_misc_cmd_.parking_brake_cmd = MsgMiscCmd::PrkBrkCmd::Off;  break;
    }
    msg_misc_cmd_.door_select = MsgMiscCmd::DoorSelect::None; ///@TODO
    msg_misc_cmd_.door_cmd = MsgMiscCmd::DoorCmd::None; ///@TODO
  }
  msg_misc_cmd_.setCrc();

  pub_can_->publish(FrameFromDbw(msg_misc_cmd_));
}

void DbwNode::recvUlcCmd(const ds_dbw_msgs::msg::UlcCmd::ConstSharedPtr msg) {
  if (std::isnan(msg->cmd) && msg->cmd_type != ds_dbw_msgs::msg::UlcCmd::CMD_NONE) {
    RCLCPP_WARN(get_logger(), "NaN ULC command");
  }
  if (std::isnan(msg->limit_accel)) {
    RCLCPP_WARN(get_logger(), "NaN ULC accel limit");
  }
  if (std::isnan(msg->limit_decel)) {
    RCLCPP_WARN(get_logger(), "NaN ULC decel limit");
  }
  if (std::isnan(msg->limit_jerk_throttle)) {
    RCLCPP_WARN(get_logger(), "NaN ULC throttle jerk limit");
  }
  if (std::isnan(msg->limit_jerk_brake)) {
    RCLCPP_WARN(get_logger(), "NaN ULC brake jerk limit");
  }

  // Transmit command
  static_assert(ds_dbw_msgs::msg::UlcCmd::CMD_NONE     == (uint8_t)MsgUlcCmd::CmdType::None);
  static_assert(ds_dbw_msgs::msg::UlcCmd::CMD_VELOCITY == (uint8_t)MsgUlcCmd::CmdType::Velocity);
  static_assert(ds_dbw_msgs::msg::UlcCmd::CMD_ACCEL    == (uint8_t)MsgUlcCmd::CmdType::Accel);
  msg_ulc_cmd_.reset();
  switch (msg->cmd_type) {
    default:
      RCLCPP_WARN(get_logger(), "Unknown ULC command type: %u", msg->cmd_type);
      [[fallthrough]];
    case ds_dbw_msgs::msg::UlcCmd::CMD_NONE:
      msg_ulc_cmd_.cmd_type = MsgUlcCmd::CmdType::None;
      break;
    case ds_dbw_msgs::msg::UlcCmd::CMD_VELOCITY:
      msg_ulc_cmd_.cmd_type = MsgUlcCmd::CmdType::Velocity;
      msg_ulc_cmd_.setCmdVelocityMps(msg->cmd);
      break;
    case ds_dbw_msgs::msg::UlcCmd::CMD_ACCEL:
      msg_ulc_cmd_.cmd_type = MsgUlcCmd::CmdType::Accel;
      msg_ulc_cmd_.setCmdAccelMps(msg->cmd);
      break;
  }
  msg_ulc_cmd_.enable_shift = msg->enable_shift;
  msg_ulc_cmd_.enable_shift_park = msg->enable_shift_park;
  msg_ulc_cmd_.coast_decel = msg->coast_decel ? MsgUlcCmd::CoastDecel::NoBrakes : MsgUlcCmd::CoastDecel::UseBrakes;
  if (modeSyncNone()) {
    msg_ulc_cmd_.enable = msg->enable || enabled();
    msg_ulc_cmd_.clear = msg->clear || (enable_ && msg_ulc_rpt_.override_latched);
  } else {
    msg_ulc_cmd_.enable = msg->enable;
    msg_ulc_cmd_.clear = msg->clear;
  }
  msg_ulc_cmd_.rc++;
  msg_ulc_cmd_.setCrc();
  pub_can_->publish(FrameFromDbw(msg_ulc_cmd_));

  // Transmit config on change and repeat at slow rate
  auto stamp = ros_clock_.now();
  bool timeout = timeoutMs(stamp, msg_ulc_cfg_stamp_, msg_ulc_cfg_.PERIOD_MS);
  auto msg_ulc_cfg = msg_ulc_cfg_;
  msg_ulc_cfg.reset();
  msg_ulc_cfg.setLimitAccelMps(msg->limit_accel);
  msg_ulc_cfg.setLimitDecelMps(msg->limit_decel);
  msg_ulc_cfg.setLimitJerkThrottleMps(msg->limit_jerk_throttle);
  msg_ulc_cfg.setLimitJerkBrakeMps(msg->limit_jerk_brake);
  msg_ulc_cfg.setCrc();
  if ((msg_ulc_cfg_ != msg_ulc_cfg) || timeout) {
    msg_ulc_cfg_ = msg_ulc_cfg;
    msg_ulc_cfg_.rc++;
    msg_ulc_cfg_.setCrc();
    msg_ulc_cfg_stamp_ = stamp;
    pub_can_->publish(FrameFromDbw(msg_ulc_cfg_));
  }
}

void DbwNode::recvSteeringCalibrate(const std_msgs::msg::Empty::ConstSharedPtr) {
  /* Send steering command to save current angle as zero.
   * The preferred method is to set the 'calibrate' field in a ROS steering
   * command so that recvSteeringCmd() saves the current angle as the
   * specified command.
   */
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3,
      "The std_msgs/Empty steering calibration topic is deprecated. "
      "Instead, send a steering command with cmd_type=CMD_CALIBRATE and specify the angle"
  );
  msg_steer_cmd_.reset();
  msg_steer_cmd_.cmd_type = MsgSteerCmd::CmdType::Calibrate;
  msg_steer_cmd_.setCmdAngleDeg(0);
  msg_steer_cmd_.rc++;
  msg_steer_cmd_.setCrc();
  pub_can_->publish(FrameFromDbw(msg_steer_cmd_));
}

bool DbwNode::publishDbwEnabled(bool force) {
  bool en = enabled();
  bool change = prev_enable_ != en;
  if (prev_enable_ && !en) {
    enable_ = false;
  }
  if (change || force) {
    std_msgs::msg::Bool msg;
    msg.data = en;
    pub_sys_enable_->publish(msg);
  }
  prev_enable_ = en;
  return change;
}

void DbwNode::timerCallback() {
  // Publish status periodically, in addition to latched and on change
  if (publishDbwEnabled(true)) {
    RCLCPP_WARN(get_logger(), "DBW system enable status changed unexpectedly");
  }

  if (modeSyncNone()) {
    // Request latched overrides to be cleared
    if (enable_) {
      if (msg_steer_rpt_1_.override_latched) {
        msg_steer_cmd_.reset();
        msg_steer_cmd_.clear = true;
        msg_steer_cmd_.rc++;
        msg_steer_cmd_.setCrc();
        pub_can_->publish(FrameFromDbw(msg_steer_cmd_));
      }
      if (msg_brake_rpt_1_.override_latched) {
        msg_brake_cmd_.reset();
        msg_brake_cmd_.clear = true;
        msg_brake_cmd_.rc++;
        msg_brake_cmd_.setCrc();
        pub_can_->publish(FrameFromDbw(msg_brake_cmd_));
      }
      if (msg_thrtl_rpt_1_.override_latched) {
        msg_thrtl_cmd_.reset();
        msg_thrtl_cmd_.clear = true;
        msg_thrtl_cmd_.rc++;
        msg_thrtl_cmd_.setCrc();
        pub_can_->publish(FrameFromDbw(msg_thrtl_cmd_));
      }
    }
  } else {
    // Request system enable
    if (msg_system_cmd_enable_
    && !msg_steer_cmd_clear_
    && !msg_brake_cmd_clear_
    && !msg_thrtl_cmd_clear_) {
      msg_system_cmd_.cmd = MsgSystemCmd::Cmd::Enable;
      msg_system_cmd_.rc++;
      msg_system_cmd_.setCrc();
      pub_can_->publish(FrameFromDbw(msg_system_cmd_));
    }
    msg_system_cmd_enable_ = false;
  }
}

void DbwNode::enableSystem() {
  if (!enable_) {
    if (fault()) {
      if (msg_steer_rpt_1_.fault) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Steering fault.");
      }
      if (msg_brake_rpt_1_.fault) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Braking fault.");
      }
      if (msg_thrtl_rpt_1_.fault) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Throttle fault.");
      }
    } else {
      enable_ = true;
      if (publishDbwEnabled()) {
        RCLCPP_INFO(get_logger(), "DBW system enabled.");
      } else {
        RCLCPP_INFO(get_logger(), "DBW system enable requested. Waiting for ready.");
      }
    }
  }
}

void DbwNode::disableSystem() {
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    RCLCPP_WARN(get_logger(), "DBW system disabled.");
  }
}

void DbwNode::warnBadCrcRc(bool bad_crc, bool bad_rc, const char *name) {
  static constexpr const char * TEXT = "%s command ignored with bad %s";
  if (bad_crc && bad_rc) {
    RCLCPP_WARN_EXPRESSION(get_logger(), warn_crc_, TEXT, name, "CRC and rolling counter");
  } else if (bad_crc) {
    RCLCPP_WARN_EXPRESSION(get_logger(), warn_crc_, TEXT, name, "CRC");
  } else if (bad_rc) {
    RCLCPP_WARN_EXPRESSION(get_logger(), warn_crc_, TEXT, name, "rolling counter");
  }
}

void DbwNode::warnRejectGear(uint8_t reject) {
  static_assert((uint8_t)MsgGearReport1::Reject::None            == ds_dbw_msgs::msg::GearReject::NONE);
  static_assert((uint8_t)MsgGearReport1::Reject::Fault           == ds_dbw_msgs::msg::GearReject::FAULT);
  static_assert((uint8_t)MsgGearReport1::Reject::Unsupported     == ds_dbw_msgs::msg::GearReject::UNSUPPORTED);
  static_assert((uint8_t)MsgGearReport1::Reject::ShiftInProgress == ds_dbw_msgs::msg::GearReject::SHIFT_IN_PROGRESS);
  static_assert((uint8_t)MsgGearReport1::Reject::Override        == ds_dbw_msgs::msg::GearReject::OVERRIDE);
  static_assert((uint8_t)MsgGearReport1::Reject::BrakeHold       == ds_dbw_msgs::msg::GearReject::BRAKE_HOLD);
  static_assert((uint8_t)MsgGearReport1::Reject::Vehicle         == ds_dbw_msgs::msg::GearReject::VEHICLE);
  if (gear_reject_ != reject) {
    gear_reject_ = reject;
    switch (reject) {
      case ds_dbw_msgs::msg::GearReject::SHIFT_IN_PROGRESS:
        RCLCPP_WARN(get_logger(), "Gear shift rejected: Shift in progress");
        break;
      case ds_dbw_msgs::msg::GearReject::OVERRIDE:
        RCLCPP_WARN(get_logger(), "Gear shift rejected: Override on brake, throttle, or steering");
        break;
      case ds_dbw_msgs::msg::GearReject::BRAKE_HOLD:
        RCLCPP_WARN(get_logger(), "Gear shift rejected: Brake hold time depleted, stay in park");
        break;
      case ds_dbw_msgs::msg::GearReject::VEHICLE:
        RCLCPP_WARN(get_logger(), "Gear shift rejected: Rejected by vehicle, try pressing the brakes");
        break;
      case ds_dbw_msgs::msg::GearReject::UNSUPPORTED:
        RCLCPP_WARN(get_logger(), "Gear shift rejected: Unsupported gear command");
        break;
      case ds_dbw_msgs::msg::GearReject::FAULT:
        RCLCPP_WARN(get_logger(), "Gear shift rejected: System in fault state");
        break;
    }
  }
}

} // namespace ds_dbw_can

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ds_dbw_can::DbwNode)
