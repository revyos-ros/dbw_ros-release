#! /usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2018-2021, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#   * Neither the name of Dataspeed Inc. nor the names of its
#     contributors may be used to endorse or promote products derived from this
#     software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
import math
from rclpy.node import Node
from ds_dbw_msgs.msg import UlcReport, UlcCmd
from std_msgs.msg import Bool


class SpeedSineWave(Node):
  APPROACHING = 0
  TRACKING = 1

  def __init__(self):
    super().__init__('speed_sine_wave')
    self.create_timer(0.02, self.timer_callback)

    self.t = 0
    self.enabled = False

    self.ulc_cmd = UlcCmd()
    self.ulc_cmd.cmd_type = UlcCmd.CMD_VELOCITY
    self.ulc_cmd.enable = True
    self.ulc_cmd.enable_shift_park = False

    # Topics
    self.create_subscription(Bool, '/vehicle/dbw_enabled', self.recv_enable, 1)
    self.pub_ulc_cmd = self.create_publisher(UlcCmd, '/vehicle/ulc/cmd', 1)

    # Parameters
    self.v1 = self.declare_parameter('v1', 0.0).value  # Speed 1
    self.v2 = self.declare_parameter('v2', 5.0).value  # Speed 2
    self.period = self.declare_parameter('period', 10.0).value # Period of wave pattern
    self.ulc_cmd.enable_shift = self.declare_parameter('enable_shift', False).value  # Enable shifting between non-Park gears
    self.ulc_cmd.limit_accel = self.declare_parameter('limit_accel', 0.0).value  # Override default acceleration limit
    self.ulc_cmd.limit_decel = self.declare_parameter('limit_decel', 0.0).value  # Override default acceleration limit

    self.speed_meas = 0
    self.reached_target_stamp = -1
    self.state = self.APPROACHING
    self.create_subscription(UlcReport, '/vehicle/ulc/report', self.recv_report, 10)

  def timer_callback(self):
    current_time = self.get_clock().now().nanoseconds / 1e9

    if not self.enabled:
      self.t = 0
      self.state = self.APPROACHING
      dbw_enable_ulc_cmd = UlcCmd()
      dbw_enable_ulc_cmd.enable = True
      dbw_enable_ulc_cmd.cmd_type = UlcCmd.CMD_NONE
      self.pub_ulc_cmd.publish(dbw_enable_ulc_cmd)
      return

    if self.state == self.APPROACHING:
      self.ulc_cmd.cmd = self.v1
      self.t = 0
      if abs(self.ulc_cmd.cmd - self.speed_meas) < 0.4 and self.reached_target_stamp < 0:
        self.reached_target_stamp = current_time

      # Wait 3 seconds before starting the sine wave input
      if self.reached_target_stamp >= 0 and (current_time - self.reached_target_stamp) > 3:
        self.state = self.TRACKING
        self.reached_target_stamp = -1

    elif self.state == self.TRACKING:
      amplitude = 0.5 * (self.v2 - self.v1)
      offset = 0.5 * (self.v2 + self.v1)
      self.ulc_cmd.cmd = offset - amplitude * math.cos(2 * math.pi / self.period * self.t)
      self.t += 0.02

    self.pub_ulc_cmd.publish(self.ulc_cmd)
    if self.v1 == 0 and self.v2 == 0:
      self.get_logger().warn('both speed targets are zero', throttle_duration_sec=1, throttle_time_source_type=rclpy.system_clock)

  def recv_enable(self, msg: Bool):
    if msg.data and not self.enabled:
      self.t = 0

    self.enabled = msg.data

  def recv_report(self, msg: UlcReport):
    self.speed_meas = msg.vel_meas


if __name__ == '__main__':
    rclpy.init()
    node = SpeedSineWave()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
