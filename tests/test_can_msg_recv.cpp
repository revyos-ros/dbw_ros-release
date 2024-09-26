/*********************************************************************
 * C++ unit test for CanMsgRecv.hpp
 *********************************************************************/

// File under test
#include <ds_dbw_can/CanMsgRecv.hpp>

// CRC implementation
#include <ds_dbw_can/SAE_J1850_crc.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// Google testing suite
#include <gtest/gtest.h>

// Shorter names
using namespace ds_dbw_can;

// Disable warning for missing field initializers
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

// CRC
static constexpr uint8_t MSG_NULL[7+1] = "\0\0\0\0\0\0\0"; // For static assertions
static constexpr uint8_t crc8(uint16_t id, const uint8_t *data, size_t len) {
    return j1850::crc8_can_msg(id, false, data, len);
}
static uint8_t crc8(uint16_t id, const void *ptr, size_t len) {
    return j1850::crc8_can_msg(id, false, (const uint8_t *)ptr, len);
}

// Time conversion
template <typename T>
static constexpr T to_ns(T ms) { return ms * 1'000'000; }

// Mock message for testing
struct TestCanMsg {
  static constexpr uint32_t ID = 0x100;
  static constexpr size_t PERIOD_MS = 10;
  static constexpr size_t TIMEOUT_MS = 100;
  uint8_t data[6];
  uint8_t :4;
  uint8_t rc :4;
  uint8_t crc;
  void reset() {
    uint8_t save = rc;
    memset(this, 0x00, sizeof(*this));
    rc = save;
  }
  void setDataRand() {
    for (size_t i = 0; i < sizeof(data); i++) {
      data[i] = rand();
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
  bool operator==(const TestCanMsg& _other) const {
      return memcmp(this, &_other, sizeof(*this)) == 0;
  }
  bool operator!=(const TestCanMsg& _other) const {
      return !(*this == _other);
  }
};
static_assert(8 == sizeof(TestCanMsg));

// Clock and duration constants
rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);
const rclcpp::Duration TIMEOUT(0, to_ns(TestCanMsg::TIMEOUT_MS));
const rclcpp::Duration PERIOD(0, to_ns(TestCanMsg::PERIOD_MS));
const rclcpp::Duration DUR_MIN(0, to_ns(1));

TEST(CanMsgRecv, recv) {
  // Test message
  TestCanMsg msg = {0};
  msg.reset();
  msg.setDataRand();

  // Test receiver
  CanMsgRecv<TestCanMsg> recv;

  // Invalid on initialization
  auto stamp = clock_.now();
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_NE(msg, recv.msg());

  // Valid on receive
  recv.receive(msg, stamp);
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Stay valid for timeout duration
  stamp += TIMEOUT;
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Invalid on timeout
  stamp += DUR_MIN;
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());
}

TEST(CanMsgRecvCrc, recv) {
  // Test message
  TestCanMsg msg = {0};
  msg.reset();
  msg.setDataRand();

  // Test receiver
  CanMsgRecvCrc<TestCanMsg> recv;

  // Invalid on initialization
  auto stamp = clock_.now();
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_NE(msg, recv.msg());

  // Stay invalid on receive with invalid CRC
  ASSERT_FALSE(msg.validCrc());
  ASSERT_FALSE(recv.receive(msg, stamp));
  ASSERT_FALSE(recv.validCrc());
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_NE(msg, recv.msg());

  // Valid on receive with valid CRC
  msg.setCrc();
  ASSERT_TRUE(recv.receive(msg, stamp));
  ASSERT_TRUE(recv.validCrc());
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Stay valid for timeout duration
  stamp += TIMEOUT;
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Keep previous message on receive with invalid CRC
  auto msg_bad = msg;
  msg_bad.setDataRand();
  msg_bad.setCrc();
  msg_bad.crc ^= 0xA5;
  ASSERT_FALSE(msg_bad.validCrc());
  ASSERT_FALSE(recv.receive(msg_bad, stamp));
  ASSERT_FALSE(recv.validCrc());
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Invalid on timeout
  stamp += DUR_MIN;
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Valid for normal operation
  for (size_t i = 0; i < 1000; i++) {
    stamp += PERIOD;
    msg.setDataRand();
    msg.setCrc();
    ASSERT_TRUE(recv.receive(msg, stamp));
    ASSERT_TRUE(recv.validCrc());
    ASSERT_TRUE(recv.valid(stamp));
    ASSERT_EQ(msg, recv.msg());
  }
}

TEST(CanMsgRecvCrcRc, recv) {
  // Test message
  TestCanMsg msg = {0};
  msg.reset();
  msg.setDataRand();
  msg.rc = 5;

  // Test receiver
  CanMsgRecvCrcRc<TestCanMsg> recv;

  // Invalid on initialization
  auto stamp = clock_.now();
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_NE(msg, recv.msg());

  // Stay invalid on receive with invalid CRC
  ASSERT_FALSE(msg.validCrc());
  ASSERT_FALSE(recv.receive(msg, stamp));
  ASSERT_FALSE(recv.validCrc());
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_NE(msg, recv.msg());

  // Valid on receive with valid CRC/RC
  msg.setCrc();
  ASSERT_TRUE(recv.receive(msg, stamp));
  ASSERT_TRUE(recv.validCrc());
  ASSERT_TRUE(recv.validRc());
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Stay valid for timeout duration
  stamp += TIMEOUT;
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Keep previous message on receive with invalid CRC
  auto msg_bad_crc = msg;
  msg_bad_crc.setDataRand();
  msg_bad_crc.rc++;
  msg_bad_crc.setCrc();
  msg_bad_crc.crc ^= 0xA5;
  ASSERT_FALSE(msg_bad_crc.validCrc());
  ASSERT_FALSE(recv.receive(msg_bad_crc, stamp));
  ASSERT_FALSE(recv.validCrc());
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Invalid on timeout
  stamp += DUR_MIN;
  ASSERT_FALSE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Valid for normal operation
  for (size_t i = 0; i < 1000; i++) {
    stamp += PERIOD;
    msg.setDataRand();
    msg.rc++;
    msg.setCrc();
    ASSERT_TRUE(recv.receive(msg, stamp));
    ASSERT_TRUE(recv.validCrc());
    ASSERT_TRUE(recv.valid(stamp));
    ASSERT_EQ(msg, recv.msg());
  }

  // Keep previous message on receive with repeated RC
  auto msg_bad_rc = msg;
  msg_bad_rc.setDataRand();
  msg_bad_rc.setCrc();
  ASSERT_FALSE(recv.receive(msg_bad_rc, stamp));
  ASSERT_TRUE(recv.validCrc());
  ASSERT_FALSE(recv.validRc());
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Invalid on timeout
  stamp += TIMEOUT;
  stamp += TIMEOUT;
  ASSERT_FALSE(recv.valid(stamp));

  // Valid on first receive with valid CRC
  msg.rc++;
  msg.setCrc();
  ASSERT_TRUE(recv.receive(msg, stamp));
  ASSERT_TRUE(recv.validCrc());
  ASSERT_TRUE(recv.validRc());
  ASSERT_TRUE(recv.valid(stamp));
  ASSERT_EQ(msg, recv.msg());

  // Invalid for every repeated RC
  for (size_t i = 0; i < 1000; i++) {
    stamp += PERIOD;
    msg.setDataRand();
    msg.setCrc();
    ASSERT_FALSE(recv.receive(msg, stamp));
    ASSERT_TRUE(recv.validCrc());
    ASSERT_FALSE(recv.validRc());
    ASSERT_FALSE(recv.valid(stamp) && (i > 2 + (TestCanMsg::TIMEOUT_MS / TestCanMsg::PERIOD_MS)));
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
