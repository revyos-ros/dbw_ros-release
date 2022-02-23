/*********************************************************************
 * Software License Agreement (Proprietary and Confidential)
 *
 *  Copyright (c) 2017-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the
 *  property of Dataspeed Inc. The intellectual and technical concepts
 *  contained herein are proprietary to Dataspeed Inc. and may be
 *  covered by U.S. and Foreign Patents, patents in process, and are
 *  protected by trade secret or copyright law. Dissemination of this
 *  information or reproduction of this material is strictly forbidden
 *  unless prior written permission is obtained from Dataspeed Inc.
 *********************************************************************/
#pragma once
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::time_point;
using std::chrono::nanoseconds;

template <typename MsgT>
class MsgRx {
public:
  MsgRx(const nanoseconds& thresh) : dur_(thresh) {
    clear();
  };
  MsgRx(const nanoseconds& thresh, const MsgT& msg) : dur_(thresh) {
    set(msg);
  };
  void set(const MsgT& msg) {
    msg_ = msg;
    stamp_ = high_resolution_clock::now();
  }
  void clear() {
    stamp_ = time_point<high_resolution_clock>();
  }
  bool fresh(const nanoseconds &delta) const {
    return age() < delta;
  }
  bool fresh() const {
    return fresh(dur_);
  }
  nanoseconds age() const {
    if (stamp_.time_since_epoch() == nanoseconds::zero()) {
      return std::chrono::seconds(9999);
    }
    return high_resolution_clock::now() - stamp_;
  }
  const MsgT& get() const {
    return msg_;
  }
  const time_point<high_resolution_clock>& stamp() const {
    return stamp_;
  }

private:
  MsgT msg_;
  time_point<high_resolution_clock> stamp_;
  nanoseconds dur_;
};
