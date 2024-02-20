/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2021, Dataspeed Inc.
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

// Version classes
#include <ds_dbw_can/ModuleVersion.hpp>
#include <ds_dbw_can/PlatformVersion.hpp>

// Standard libraries
#include <map>

namespace ds_dbw_can {

class PlatformMap {
public:
  PlatformMap() {}
  PlatformMap(Platform p, Module m, ModuleVersion v) {
    put(p, m, v);
  }
  PlatformMap(const PlatformVersion& x) {
    put(x);
  }
  PlatformMap(const std::vector<PlatformVersion>& vec) {
    put(vec);
  };
  void put(const Platform& p, const Module& m, const ModuleVersion& v) {
    if (p >= Platform::MAX || m >= Module::MAX) {
      return;
    }
    map_[p][m] = v;
  }
  void put(const PlatformVersion& x) {
    put(x.p, x.m, x.v);
  }
  void put(const std::vector<PlatformVersion>& vec) {
    for (const PlatformVersion& entry : vec) {
      put(entry);
    }
  }
  ModuleVersion get(const Platform& p, const Module& m) const {
    const auto a = map_.find(p);
    if (a != map_.end()) {
      const auto b = a->second.find(m);
      if (b != a->second.end()) {
        return b->second;
      }
    }
    return ModuleVersion();
  }
  ModuleVersion get(const PlatformVersion& x) const {
    return get(x.p, x.m);
  }
  bool hasValid(const Module& m) const {
    for (auto const& [platform, map] : map_) {
      for (auto const& [module, version] : map) {
        if (module == m && version.valid()) {
          return true;
        }
      }
    }
    return false;
  }

private:
  std::map<Platform, std::map<Module, ModuleVersion>> map_;
};

bool operator<(const PlatformVersion& x, const PlatformMap& map) {
  return x < map.get(x);
}
bool operator>(const PlatformVersion& x, const PlatformMap& map) {
  return x > map.get(x);
}
bool operator<=(const PlatformVersion& x, const PlatformMap& map) {
  return x <= map.get(x);
}
bool operator>=(const PlatformVersion& x, const PlatformMap& map) {
  return x >= map.get(x);
}
bool operator==(const PlatformVersion& x, const PlatformMap& map) {
  return x == map.get(x);
}
bool operator!=(const PlatformVersion& x, const PlatformMap& map) {
  return x != map.get(x);
}

} // namespace ds_dbw_can
