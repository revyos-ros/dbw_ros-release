/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2021, Dataspeed Inc.
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

#include "UlcNode.hpp"

#include <chrono>
#include <dataspeed_ulc_can/dispatch.hpp>

namespace dataspeed_ulc_can {

template <class T>
T UlcNode::overflowSaturation(double input, T limit_min, T limit_max, double scale_factor,
                              const std::string &input_name, const std::string &units) const {
  if (input < (limit_min * scale_factor)) {
    RCLCPP_WARN(get_logger(), "%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input,
                units.c_str(), limit_min * scale_factor, units.c_str());
    return limit_min;
  } else if (input > (limit_max * scale_factor)) {
    RCLCPP_WARN(get_logger(), "%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input,
                units.c_str(), limit_max * scale_factor, units.c_str());
    return limit_max;
  } else {
    return input / scale_factor;
  }
}

UlcNode::UlcNode(const rclcpp::NodeOptions &options) : rclcpp::Node("ulc_node", options), enable_(false) {
  // Setup publishers
  pub_report_ = create_publisher<dataspeed_ulc_msgs::msg::UlcReport>("ulc_report", 2);
  pub_can_ = create_publisher<can_msgs::msg::Frame>("can_tx", 100);

  // Setup subscribers
  using std::placeholders::_1;
  {
    auto bind = std::bind(&UlcNode::recvCan, this, _1);
    sub_can_ = create_subscription<can_msgs::msg::Frame>("can_rx", 100, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvUlcCmd, this, _1);
    sub_cmd_ = create_subscription<dataspeed_ulc_msgs::msg::UlcCmd>("ulc_cmd", 2, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvTwist, this, _1);
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvTwistStamped, this, _1);
    sub_twist_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 2, bind);
  }
  {
    auto latch_like_qos = rclcpp::QoS(2).transient_local();
    auto bind = std::bind(&UlcNode::recvEnable, this, _1);
    sub_enable_ = create_subscription<std_msgs::msg::Bool>("dbw_enabled", latch_like_qos, bind);
  }

  // Initialize timestamp
  cmd_clock_ = rclcpp::Clock(RCL_ROS_TIME);
  cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Setup timer for config message retransmission
  double freq = declare_parameter<double>("config_frequency", 5.0);
  freq = std::clamp(freq, 5.0, 50.0);
  set_parameter(rclcpp::Parameter("config_frequency", freq));

  auto duration = std::chrono::microseconds(static_cast<uint32_t>(1000000.0 / freq));
  config_timer_ = create_wall_timer(duration, std::bind(&UlcNode::configTimerCb, this));
}

void UlcNode::recvEnable(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  enable_ = msg->data;
}

void UlcNode::recvUlcCmd(const dataspeed_ulc_msgs::msg::UlcCmd::ConstSharedPtr msg) {
  // Check for differences in acceleration limits
  bool diff = (msg->linear_accel != ulc_cmd_.linear_accel) || (msg->linear_decel != ulc_cmd_.linear_decel) ||
              (msg->lateral_accel != ulc_cmd_.lateral_accel) || (msg->angular_accel != ulc_cmd_.angular_accel);
  ulc_cmd_ = *msg;

  // Publish command message
  sendCmdMsg(true);

  // Publish config message on change
  if (diff) {
    sendCfgMsg();
  }
}

void UlcNode::recvTwistCmd(const geometry_msgs::msg::Twist &msg) {
  // Populate command fields
  ulc_cmd_.linear_velocity = msg.linear.x;
  ulc_cmd_.yaw_command = msg.angular.z;
  ulc_cmd_.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;

  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_pedals = true;
  ulc_cmd_.enable_shifting = true;
  ulc_cmd_.enable_steering = true;
  ulc_cmd_.shift_from_park = false;
  ulc_cmd_.linear_accel = 0;
  ulc_cmd_.linear_decel = 0;
  ulc_cmd_.angular_accel = 0;
  ulc_cmd_.lateral_accel = 0;

  // Publish command message
  sendCmdMsg(false);
}

void UlcNode::recvTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  recvTwistCmd(*msg);
}

void UlcNode::recvTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
  recvTwistCmd(msg->twist);
}

void UlcNode::recvCan(const can_msgs::msg::Frame::ConstSharedPtr msg) {
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_ULC_REPORT:
        if (msg->dlc >= sizeof(MsgUlcReport)) {
          const MsgUlcReport *ptr = reinterpret_cast<const MsgUlcReport *>(msg->data.data());
          dataspeed_ulc_msgs::msg::UlcReport report;
          report.header.stamp = msg->header.stamp;
          report.speed_ref = (float)ptr->speed_ref * 0.02f;
          report.accel_ref = (float)ptr->accel_ref * 0.05f;
          report.speed_meas = (float)ptr->speed_meas * 0.02f;
          report.accel_meas = (float)ptr->accel_meas * 0.05f;
          report.max_steering_angle = (float)ptr->max_steering_angle * 5.0f;
          report.max_steering_vel = (float)ptr->max_steering_vel * 8.0f;
          report.pedals_enabled = ptr->pedals_enabled;
          report.steering_enabled = ptr->steering_enabled;
          report.tracking_mode = ptr->tracking_mode;
          report.speed_preempted = ptr->speed_preempted;
          report.steering_preempted = ptr->steering_preempted;
          report.override_latched = ptr->override;
          report.steering_mode = ptr->steering_mode;
          report.timeout = ptr->timeout;
          pub_report_->publish(report);
        }
        break;
    }
  }
}

void UlcNode::sendCmdMsg(bool cfg) {
  // Validate input fields
  if (validInputs(ulc_cmd_)) {
    if (cfg) {
      cmd_stamp_ = cmd_clock_.now();
    }
  } else {
    cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }

  // Build CAN message
  can_msgs::msg::Frame msg;
  msg.id = ID_ULC_CMD;
  msg.is_extended = false;
  msg.dlc = sizeof(MsgUlcCmd);
  MsgUlcCmd *ptr = reinterpret_cast<MsgUlcCmd *>(msg.data.data());
  memset(ptr, 0x00, sizeof(*ptr));

  // Populate enable bits
  if (enable_) {
    ptr->enable_pedals = ulc_cmd_.enable_pedals;
    ptr->enable_steering = ulc_cmd_.enable_steering;
    ptr->enable_shifting = ulc_cmd_.enable_shifting;
    ptr->shift_from_park = ulc_cmd_.shift_from_park;
  }

  // Populate command fields
  ptr->clear = ulc_cmd_.clear;
  ptr->linear_velocity =
      overflowSaturation(ulc_cmd_.linear_velocity, INT16_MIN, INT16_MAX, 0.0025, "ULC command speed", "m/s");
  ptr->steering_mode = ulc_cmd_.steering_mode;
  if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE) {
    ptr->yaw_command =
        overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.00025, "ULC yaw rate command", "rad/s");
  } else if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE) {
    ptr->yaw_command =
        overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.0000061, "ULC curvature command", "1/m");
  } else {
    ptr->yaw_command = 0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Unsupported ULC steering control mode [%d]",
                         ulc_cmd_.steering_mode);
    cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }

  // Publish message
  pub_can_->publish(msg);
}

void UlcNode::sendCfgMsg() {
  // Build CAN message
  can_msgs::msg::Frame msg;
  msg.id = ID_ULC_CONFIG;
  msg.is_extended = false;
  msg.dlc = sizeof(MsgUlcCfg);
  MsgUlcCfg *ptr = reinterpret_cast<MsgUlcCfg *>(msg.data.data());
  memset(ptr, 0x00, sizeof(*ptr));

  // Populate acceleration limits
  ptr->linear_accel = overflowSaturation(ulc_cmd_.linear_accel, 0, UINT8_MAX, 0.025, "Linear accel limit", "m/s^2");
  ptr->linear_decel = overflowSaturation(ulc_cmd_.linear_decel, 0, UINT8_MAX, 0.025, "Linear decel limit", "m/s^2");
  ptr->lateral_accel = overflowSaturation(ulc_cmd_.lateral_accel, 0, UINT8_MAX, 0.05, "Lateral accel limit", "m/s^2");
  ptr->angular_accel = overflowSaturation(ulc_cmd_.angular_accel, 0, UINT8_MAX, 0.02, "Angular accel limit", "rad/s^2");

  // Publish message
  pub_can_->publish(msg);

  // Reset timer
  config_timer_->reset();
}

void UlcNode::configTimerCb() {
  // Retransmit config message while command is valid
  if (cmd_clock_.now() - cmd_stamp_ < std::chrono::milliseconds(100)) {
    sendCfgMsg();
  }
}

bool UlcNode::validInputs(const dataspeed_ulc_msgs::msg::UlcCmd &cmd) const {
  bool valid = true;
  if (std::isnan(cmd.linear_velocity)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on speed input");
    valid = false;
  }
  if (std::isnan(cmd.yaw_command)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on yaw command input");
    valid = false;
  }
  if (std::isnan(cmd.linear_accel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on linear accel input");
    valid = false;
  }
  if (std::isnan(cmd.linear_decel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on linear decel input");
    valid = false;
  }
  if (std::isnan(cmd.lateral_accel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on lateral accel input");
    valid = false;
  }
  if (std::isnan(cmd.angular_accel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on angular accel input");
    valid = false;
  }
  return valid;
}

} // namespace dataspeed_ulc_can

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dataspeed_ulc_can::UlcNode)
