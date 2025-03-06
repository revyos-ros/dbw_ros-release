#pragma once

#include <std_msgs/msg/header.hpp>
#include <rclcpp/time.hpp>

namespace ds_dbw_can {

template <typename T>
class CanMsgRecv {
public:
    using Stamp = std_msgs::msg::Header::_stamp_type;
    bool receive(const T& msg, Stamp stamp) {
        // Store inputs
        stamp_ = stamp;
        msg_ = msg;
        has_msg_ = true;
        return true;
    }
    bool valid(Stamp stamp) const {
        // Calculate timestamp difference
        int64_t diff_ns = (rclcpp::Time(stamp) - rclcpp::Time(stamp_)).nanoseconds();
        // Valid if the previous message hasn't timed out
        return has_msg_ && (diff_ns <= TIMEOUT_NS);
    }
    const T& msg() const {
        return msg_;
    }
protected:
    static constexpr int64_t TIMEOUT_NS = T::TIMEOUT_MS * 1'000'000;
    Stamp stamp_;
    T msg_;
    bool has_msg_ = false;
};

template <typename T>
class CanMsgRecvCrc : public CanMsgRecv<T> {
public:
    using Stamp = std_msgs::msg::Header::_stamp_type;
    bool receive(const T& msg, Stamp stamp) {
        // Require valid CRC
        crc_valid_ = msg.validCrc();
        if (crc_valid_) {
            CanMsgRecv<T>::receive(msg, stamp);
            return true;
        }
        return false;
    }
    bool validCrc() const { return crc_valid_; }
protected:
    bool crc_valid_ = false;
};

template <typename T>
class CanMsgRecvCrcRc : public CanMsgRecvCrc<T> {
public:
    using Stamp = std_msgs::msg::Header::_stamp_type;
    bool receive(const T& msg, Stamp stamp) {
        // Require valid CRC and rolling counter (RC)
        // Any rolling counter value is accepted after message timeout
        CanMsgRecvCrc<T>::crc_valid_ = msg.validCrc();
        if (CanMsgRecvCrc<T>::crc_valid_) {
            rc_valid_ = msg.validRc(rc_) || (!CanMsgRecv<T>::valid(stamp) && validRc());
            if (rc_valid_) {
                CanMsgRecv<T>::receive(msg, stamp);
                rc_ = msg.rc;
                return true;
            }
        }
        return false;
    }
    bool validRc() const { return rc_valid_; }
protected:
    bool rc_valid_ = false;
    uint8_t rc_ = -1; // Always accept first rolling counter value
};

} // namespace ds_dbw_can
