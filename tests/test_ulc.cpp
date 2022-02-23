#include <gtest/gtest.h>

#include <can_msgs/msg/frame.hpp>
#include <dataspeed_ulc_can/UlcNode.hpp>
#include <dataspeed_ulc_can/dispatch.hpp>
#include <dataspeed_ulc_msgs/msg/ulc_cmd.hpp>
#include <dataspeed_ulc_msgs/msg/ulc_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "MsgRx.hpp"

using namespace dataspeed_ulc_can;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;
using std::chrono::time_point;

using namespace std::chrono_literals;  // for "ms" and "s" numeric time operators

MsgRx<MsgUlcCmd> g_msg_ulc_cmd(50ms);
MsgRx<MsgUlcCfg> g_msg_ulc_cfg(50ms);
MsgRx<dataspeed_ulc_msgs::msg::UlcReport> g_msg_ulc_report(50ms);

dataspeed_ulc_msgs::msg::UlcCmd g_ulc_cmd;
double g_cfg_freq;

// Command message scale factors
const double LIN_VEL_SCALE_FACTOR = 0.0025;
const double YAW_RATE_SCALE_FACTOR = 0.00025;
const double CURVATURE_SCALE_FACTOR = 0.0000061;

// Config message scale factors
const double LINEAR_ACCEL_SCALE_FACTOR = 0.025;
const double LINEAR_DECEL_SCALE_FACTOR = 0.025;
const double LATERAL_ACCEL_SCALE_FACTOR = 0.05;
const double ANGULAR_ACCEL_SCALE_FACTOR = 0.02;

// Report message scale factors
const double SPEED_REPORT_SCALE_FACTOR = 0.02;
const double ACCEL_REPORT_SCALE_FACTOR = 0.05;
const double MAX_ANGLE_SCALE_FACTOR = 5.0;
const double MAX_RATE_SCALE_FACTOR = 8.0;

rclcpp::Publisher<dataspeed_ulc_msgs::msg::UlcCmd>::SharedPtr g_pub_ulc_cmd;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr g_pub_enable;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr g_pub_twist;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr g_pub_twist_stamped;
rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr g_pub_can;
rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr g_sub_can;
rclcpp::Subscription<dataspeed_ulc_msgs::msg::UlcReport>::SharedPtr g_sub_report;

void recvCan(const can_msgs::msg::Frame::ConstSharedPtr msg) {
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_ULC_CMD: {
        auto cmd = reinterpret_cast<const MsgUlcCmd*>(msg->data.data());
        g_msg_ulc_cmd.set(*cmd);
        break;
      }
      case ID_ULC_CONFIG: {
        auto cmd = reinterpret_cast<const MsgUlcCfg*>(msg->data.data());
        g_msg_ulc_cfg.set(*cmd);
        break;
      }
    }
  }
}

void recvReport(const dataspeed_ulc_msgs::msg::UlcReport::ConstSharedPtr msg) {
  g_msg_ulc_report.set(*msg);
}

template <class T>
static bool waitForMsg(const nanoseconds& dur, const MsgRx<T>& msg_rx) {
  auto clock = high_resolution_clock();
  const auto start = clock.now();
  while (true) {
    if (msg_rx.fresh()) {
      return true;
    }
    if ((clock.now() - start) > dur) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
}

static bool waitForTopics(const nanoseconds& dur) {
  auto clock = high_resolution_clock();
  const auto start = clock.now();
  while (true) {
    if ((g_sub_can->get_publisher_count() == 1) && (g_sub_report->get_publisher_count() == 1) &&
        (g_pub_ulc_cmd->get_subscription_count() == 1) && (g_pub_enable->get_subscription_count() == 1) &&
        (g_pub_twist->get_subscription_count() == 1) && (g_pub_twist_stamped->get_subscription_count() == 1) &&
        (g_pub_can->get_subscription_count() == 1)) {
      return true;
    }
    if ((clock.now() - start) > dur) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
}

static void checkImmediateCfg() {
  auto clock = high_resolution_clock();
  const auto stamp = clock.now();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_TRUE(waitForMsg(100ms, g_msg_ulc_cfg));
  auto cfg_time = nanoseconds(g_msg_ulc_cfg.stamp().time_since_epoch()).count();
  auto stamp_time = nanoseconds(stamp.time_since_epoch()).count();
  EXPECT_NEAR(cfg_time, stamp_time, 1e7);
}

TEST(ULCNode, topics) {
  // Wait for all topics to connect before running other tests
  ASSERT_TRUE(waitForTopics(2s));
  std::this_thread::sleep_for(1s);
}

TEST(ULCNode, cfgTiming) {
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.linear_accel = 1.0;
  g_ulc_cmd.linear_decel = 1.0;
  g_ulc_cmd.lateral_accel = 1.0;
  g_ulc_cmd.angular_accel = 1.0;

  // Publish command messages with the same acceleration limits and make sure
  // config CAN messages are sent at the nominal rate
  size_t count = 0;
  const auto end_time = high_resolution_clock::now() + 1s;
  g_msg_ulc_cfg.clear();
  auto time_old = g_msg_ulc_cfg.stamp();
  while (end_time > high_resolution_clock::now()) {
    g_pub_ulc_cmd->publish(g_ulc_cmd);
    std::this_thread::sleep_for(10ms);
    const auto time_new = g_msg_ulc_cfg.stamp();
    if (time_new.time_since_epoch() != nanoseconds::zero() && time_old.time_since_epoch() != nanoseconds::zero() &&
        time_new != time_old) {
      double stamp_old = nanoseconds(time_old.time_since_epoch()).count() / 1e9;
      double stamp_new = nanoseconds(time_new.time_since_epoch()).count() / 1e9;
      double offset = 1.0 / g_cfg_freq;
      EXPECT_NEAR(stamp_old + offset, stamp_new, 0.01);
      count++;
    }
    time_old = time_new;
  }
  EXPECT_GE(count, 1u);

  // Change accel limits and make sure config CAN messages are sent immediately
  g_ulc_cmd.linear_accel = 2.0;
  checkImmediateCfg();
  g_ulc_cmd.linear_decel = 2.0;
  checkImmediateCfg();
  g_ulc_cmd.lateral_accel = 2.0;
  checkImmediateCfg();
  g_ulc_cmd.angular_accel = 2.0;
  checkImmediateCfg();
}

TEST(ULCNode, cmdRangeSaturation) {
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();

  /*** Underflow tests ******************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = (INT16_MIN * LIN_VEL_SCALE_FACTOR) - 1.0;
  g_ulc_cmd.linear_accel = -1.0;
  g_ulc_cmd.linear_decel = -1.0;
  g_ulc_cmd.lateral_accel = -1.0;
  g_ulc_cmd.angular_accel = -1.0;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = (INT16_MIN * YAW_RATE_SCALE_FACTOR) - 0.5;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = (INT16_MIN * CURVATURE_SCALE_FACTOR) - 0.05;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/

  /*** Overflow tests *******************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = (INT16_MAX * LIN_VEL_SCALE_FACTOR) + 1.0;
  g_ulc_cmd.linear_accel = 100.0;
  g_ulc_cmd.linear_decel = 100.0;
  g_ulc_cmd.lateral_accel = 100.0;
  g_ulc_cmd.angular_accel = 100.0;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = (INT16_MAX * YAW_RATE_SCALE_FACTOR) + 0.5;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = (INT16_MAX * CURVATURE_SCALE_FACTOR) + 0.05;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/

  /*** +Inf tests ***********************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = INFINITY;
  g_ulc_cmd.linear_accel = INFINITY;
  g_ulc_cmd.linear_decel = INFINITY;
  g_ulc_cmd.lateral_accel = INFINITY;
  g_ulc_cmd.angular_accel = INFINITY;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(UINT8_MAX, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MAX, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/

  /*** -Inf tests ***********************************************************/
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.linear_velocity = -INFINITY;
  g_ulc_cmd.linear_accel = -INFINITY;
  g_ulc_cmd.linear_decel = -INFINITY;
  g_ulc_cmd.lateral_accel = -INFINITY;
  g_ulc_cmd.angular_accel = -INFINITY;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = -INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cfg));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ(0, g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = -INFINITY;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_EQ(INT16_MIN, g_msg_ulc_cmd.get().yaw_command);
  /**************************************************************************/
}

TEST(ULCNode, outOfBoundsInputs) {
  g_msg_ulc_cfg.clear();

  // NaN in linear velocity field
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.linear_velocity = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // NaN in yaw command field
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.yaw_command = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // NaN in linear accel field
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.linear_accel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // NaN in linear decel field
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.linear_decel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // NaN in lateral accel field
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.lateral_accel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // NaN in angular accel field
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.angular_accel = NAN;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // Invalid steering mode
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.steering_mode = 3;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cmd));

  // Make sure no config messages were sent during this process
  EXPECT_FALSE(waitForMsg(100ms, g_msg_ulc_cfg));
}

TEST(ULCNode, scaleFactors) {
  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.linear_velocity = 22.3;
  g_ulc_cmd.linear_accel = 1.23;
  g_ulc_cmd.linear_decel = 3.45;
  g_ulc_cmd.lateral_accel = 5.43;
  g_ulc_cmd.angular_accel = 3.21;

  // Yaw rate steering
  g_ulc_cmd.yaw_command = 0.567;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cfg));
  EXPECT_EQ((int16_t)(g_ulc_cmd.linear_velocity / LIN_VEL_SCALE_FACTOR), g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ((int16_t)(g_ulc_cmd.yaw_command / YAW_RATE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ((int16_t)(g_ulc_cmd.linear_accel / LINEAR_ACCEL_SCALE_FACTOR), g_msg_ulc_cfg.get().linear_accel);
  EXPECT_EQ((int16_t)(g_ulc_cmd.linear_decel / LINEAR_DECEL_SCALE_FACTOR), g_msg_ulc_cfg.get().linear_decel);
  EXPECT_EQ((int16_t)(g_ulc_cmd.lateral_accel / LATERAL_ACCEL_SCALE_FACTOR), g_msg_ulc_cfg.get().lateral_accel);
  EXPECT_EQ((int16_t)(g_ulc_cmd.angular_accel / ANGULAR_ACCEL_SCALE_FACTOR), g_msg_ulc_cfg.get().angular_accel);

  // Curvature steering
  g_ulc_cmd.yaw_command = 0.0789;
  g_ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE;
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_EQ((int16_t)(g_ulc_cmd.yaw_command / CURVATURE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
}

TEST(ULCNode, dbwEnable) {
  std_msgs::msg::Bool dbw_enabled_msg;

  g_ulc_cmd = dataspeed_ulc_msgs::msg::UlcCmd();
  g_ulc_cmd.enable_pedals = true;
  g_ulc_cmd.enable_steering = true;
  g_ulc_cmd.enable_shifting = true;
  g_ulc_cmd.shift_from_park = true;

  // Make sure CAN enable signals are false because dbw_enabled was not sent yet
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_FALSE(g_msg_ulc_cmd.get().enable_pedals);
  EXPECT_FALSE(g_msg_ulc_cmd.get().enable_steering);
  EXPECT_FALSE(g_msg_ulc_cmd.get().enable_shifting);
  EXPECT_FALSE(g_msg_ulc_cmd.get().shift_from_park);

  // Publish dbw_enabled as true
  dbw_enabled_msg.data = true;
  g_pub_enable->publish(dbw_enabled_msg);
  std::this_thread::sleep_for(1ms);

  // Send command again and make sure CAN enable signals are true
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_TRUE(g_msg_ulc_cmd.get().enable_pedals);
  EXPECT_TRUE(g_msg_ulc_cmd.get().enable_steering);
  EXPECT_TRUE(g_msg_ulc_cmd.get().enable_shifting);
  EXPECT_TRUE(g_msg_ulc_cmd.get().shift_from_park);

  // Publish dbw_enabled as false and make sure CAN enable signals are false
  dbw_enabled_msg.data = false;
  g_pub_enable->publish(dbw_enabled_msg);
  std::this_thread::sleep_for(50ms);
  g_msg_ulc_cmd.clear();
  g_pub_ulc_cmd->publish(g_ulc_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  ASSERT_FALSE(g_msg_ulc_cmd.get().enable_pedals);
  ASSERT_FALSE(g_msg_ulc_cmd.get().enable_steering);
  ASSERT_FALSE(g_msg_ulc_cmd.get().enable_shifting);
  ASSERT_FALSE(g_msg_ulc_cmd.get().shift_from_park);
}

TEST(ULCNode, twistInputs) {
  geometry_msgs::msg::Twist twist_cmd;
  twist_cmd.linear.x = 22.0;
  twist_cmd.angular.z = 0.2;
  std::this_thread::sleep_for(1s);

  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_twist->publish(twist_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_FALSE(waitForMsg(500ms, g_msg_ulc_cfg));
  EXPECT_EQ((int16_t)(twist_cmd.linear.x / LIN_VEL_SCALE_FACTOR), g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ((int16_t)(twist_cmd.angular.z / YAW_RATE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cmd.get().steering_mode);

  geometry_msgs::msg::TwistStamped twist_stamped_cmd;
  twist_stamped_cmd.twist = twist_cmd;
  g_msg_ulc_cmd.clear();
  g_msg_ulc_cfg.clear();
  g_pub_twist_stamped->publish(twist_stamped_cmd);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_cmd));
  EXPECT_FALSE(waitForMsg(500ms, g_msg_ulc_cfg));
  EXPECT_EQ((int16_t)(twist_cmd.linear.x / LIN_VEL_SCALE_FACTOR), g_msg_ulc_cmd.get().linear_velocity);
  EXPECT_EQ((int16_t)(twist_cmd.angular.z / YAW_RATE_SCALE_FACTOR), g_msg_ulc_cmd.get().yaw_command);
  EXPECT_EQ(0, g_msg_ulc_cmd.get().steering_mode);
}

TEST(ULCNode, reportParsing) {
  can_msgs::msg::Frame report_out;
  report_out.id = ID_ULC_REPORT;
  report_out.is_extended = false;
  report_out.dlc = sizeof(MsgUlcReport);
  MsgUlcReport* msg_report_ptr = reinterpret_cast<MsgUlcReport*>(report_out.data.data());
  memset(msg_report_ptr, 0x00, sizeof(MsgUlcReport));
  msg_report_ptr->timeout = false;
  msg_report_ptr->tracking_mode = 0;
  msg_report_ptr->steering_mode = 1;
  msg_report_ptr->steering_enabled = false;
  msg_report_ptr->pedals_enabled = true;
  msg_report_ptr->speed_ref = 22.2f / SPEED_REPORT_SCALE_FACTOR;
  msg_report_ptr->accel_ref = 1.1f / ACCEL_REPORT_SCALE_FACTOR;
  msg_report_ptr->speed_meas = 21.1f / SPEED_REPORT_SCALE_FACTOR;
  msg_report_ptr->accel_meas = 0.99f / ACCEL_REPORT_SCALE_FACTOR;
  msg_report_ptr->max_steering_vel = 16.0f / MAX_RATE_SCALE_FACTOR;
  msg_report_ptr->max_steering_angle = 55.0f / MAX_ANGLE_SCALE_FACTOR;
  msg_report_ptr->speed_preempted = true;
  msg_report_ptr->steering_preempted = false;
  msg_report_ptr->override = true;

  g_pub_can->publish(report_out);
  ASSERT_TRUE(waitForMsg(100ms, g_msg_ulc_report));
  ASSERT_FALSE(g_msg_ulc_report.get().timeout);
  ASSERT_EQ(0, g_msg_ulc_report.get().tracking_mode);
  ASSERT_EQ(1, g_msg_ulc_report.get().steering_mode);
  ASSERT_FALSE(g_msg_ulc_report.get().steering_enabled);
  ASSERT_TRUE(g_msg_ulc_report.get().pedals_enabled);
  ASSERT_FLOAT_EQ(22.2f, g_msg_ulc_report.get().speed_ref);
  ASSERT_FLOAT_EQ(1.1f, g_msg_ulc_report.get().accel_ref);
  ASSERT_FLOAT_EQ(21.1f, g_msg_ulc_report.get().speed_meas);
  ASSERT_FLOAT_EQ(0.95f, g_msg_ulc_report.get().accel_meas);
  ASSERT_FLOAT_EQ(16.0f, g_msg_ulc_report.get().max_steering_vel);
  ASSERT_FLOAT_EQ(55.0f, g_msg_ulc_report.get().max_steering_angle);
  ASSERT_TRUE(g_msg_ulc_report.get().speed_preempted);
  ASSERT_FALSE(g_msg_ulc_report.get().steering_preempted);
}

int main(int argc, char** argv) {
  // It is not recommended to normally run a ROS2 node in this way.
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  std::thread executor_thread;
  int result = 0;
  {
    auto node = std::make_shared<rclcpp::Node>("ulc_node_test");

    namespace ulc_msgs = dataspeed_ulc_msgs::msg;
    namespace geom_msgs = geometry_msgs::msg;

    g_sub_can = node->create_subscription<can_msgs::msg::Frame>("can_tx", 100, recvCan);
    g_sub_report = node->create_subscription<ulc_msgs::UlcReport>("ulc_report", 10, recvReport);
    g_pub_ulc_cmd = node->create_publisher<ulc_msgs::UlcCmd>("ulc_cmd", 2);
    g_pub_enable = node->create_publisher<std_msgs::msg::Bool>("dbw_enabled", 2);
    g_pub_twist = node->create_publisher<geom_msgs::Twist>("cmd_vel", 2);
    g_pub_twist_stamped = node->create_publisher<geom_msgs::TwistStamped>("cmd_vel_stamped", 2);
    g_pub_can = node->create_publisher<can_msgs::msg::Frame>("can_rx", 100);
    g_cfg_freq = node->declare_parameter("config_frequency", 5.0);

    auto ulc = std::make_shared<dataspeed_ulc_can::UlcNode>();

    // Setup ROS2 Equivilent to AsyncSpinner
    using rclcpp::executors::MultiThreadedExecutor;
    auto spinner = MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3);
    spinner.add_node(node);
    spinner.add_node(ulc);
    executor_thread = std::thread(std::bind(&MultiThreadedExecutor::spin, &spinner));

    // let the thread startup
    std::this_thread::sleep_for(1s);

    // Run all the tests that were declared with TEST()
    result = RUN_ALL_TESTS();

    //stop the threads and clear
    spinner.cancel();
    executor_thread.join();

    // destroy the nodes by going out of scope
  }

  // Cleanup
  rclcpp::shutdown();

  // Return test result
  return result;
}
