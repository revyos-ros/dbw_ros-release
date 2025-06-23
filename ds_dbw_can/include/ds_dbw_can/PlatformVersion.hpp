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

// Module Version class
#include <ds_dbw_can/ModuleVersion.hpp>

// CAN messages and IDs
#include <ds_dbw_can/dispatch.hpp>

namespace ds_dbw_can {

// Vehicle platform enumeration
enum class Platform : uint8_t {
  FORD_CD4         = 0x00, // Lincoln MKZ, Ford Fusion/Mondeo
  FORD_U6          = 0x04, // Lincoln Aviator
  FORD_CD5         = 0x05, // Ford Edge, Lincoln Nautilus
  FORD_GE1         = 0x06, // Ford Mustang Mach-E
  FORD_P702        = 0x07, // Ford F150 (2021+)
  FORD_V3          = 0x08, // Ford E-Transit
  FORD_P702R       = 0x09, // Ford F150 Raptor (2024+)
  FCA_RU           = 0x10, // Chrysler Pacifica
  FCA_WK2          = 0x11, // Jeep Grand Cherokee
  POLARIS_GEM      = 0x80, // Polaris GEM
  POLARIS_RZRXP    = 0x81, // Polaris RZR-XP
  POLARIS_RANGERXP = 0x82, // Polaris Ranger-XP
  POLARIS_RZRR     = 0x83, // Polaris RZR-R
  POLARIS_RANGERXD = 0x84, // Polaris Ranger-XD
  MAX
};

// Module type enumeration
enum class Module : uint16_t {
  Gateway = MsgEcuInfoGateway::ID,
  Steer = MsgEcuInfoSteer::ID,
  Brake = MsgEcuInfoBrake::ID,
  Throttle = MsgEcuInfoThrottle::ID,
  Shift = MsgEcuInfoShift::ID,
  BOO = MsgEcuInfoBOO::ID,
  Monitor = MsgEcuInfoMonitor::ID,
  MAX
};

constexpr static const char* platformToString(Platform x) {
  switch (x) {
    case Platform::FORD_CD4:         return "FORD_CD4";
    case Platform::FORD_U6:          return "FORD_U6";
    case Platform::FORD_CD5:         return "FORD_CD5";
    case Platform::FORD_GE1:         return "FORD_GE1";
    case Platform::FORD_P702:        return "FORD_P702";
    case Platform::FORD_V3:          return "FORD_V3";
    case Platform::FORD_P702R:       return "FORD_P702R";
    case Platform::FCA_RU:           return "FCA_RU";
    case Platform::FCA_WK2:          return "FCA_WK2";
    case Platform::POLARIS_GEM:      return "POLARIS_GEM";
    case Platform::POLARIS_RZRXP:    return "POLARIS_RZRXP";
    case Platform::POLARIS_RANGERXP: return "POLARIS_RANGERXP";
    case Platform::POLARIS_RZRR:     return "POLARIS_RZRR";
    case Platform::POLARIS_RANGERXD: return "POLARIS_RANGERXD";
    default:                         return "UNKNOWN";
  }
}

constexpr static const char* moduleToString(Module x) {
  switch (x) {
    case Module::Gateway:  return "Gateway ";
    case Module::Steer:    return "Steer   ";
    case Module::Brake:    return "Brake   ";
    case Module::Throttle: return "Throttle";
    case Module::Shift:    return "Shift   ";
    case Module::BOO:      return "BOO     ";
    case Module::Monitor:  return "Monitor ";
    default:               return "UNKNOWN";
  }
}

class PlatformVersion {
public:
  constexpr PlatformVersion() {};
  constexpr PlatformVersion(Platform p, Module m, ModuleVersion v) : p(p), m(m), v(v) {};
  constexpr PlatformVersion(Platform p, Module m, uint16_t major, uint16_t minor, uint16_t build) : p(p), m(m), v(ModuleVersion(major, minor, build)) {};
  constexpr bool operator< (const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v <  other.v); }
  constexpr bool operator> (const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v >  other.v); }
  constexpr bool operator<=(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v <= other.v); }
  constexpr bool operator>=(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v >= other.v); }
  constexpr bool operator==(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v == other.v); }
  constexpr bool operator!=(const PlatformVersion& other) const { return (this->p == other.p) && (this->m == other.m) && (this->v != other.v); }
  Platform p = (Platform)0;
  Module m = (Module)0;
  ModuleVersion v;
};

static constexpr bool operator< (const PlatformVersion& x, const ModuleVersion& y) { return x.v <  y; }
static constexpr bool operator> (const PlatformVersion& x, const ModuleVersion& y) { return x.v >  y; }
static constexpr bool operator<=(const PlatformVersion& x, const ModuleVersion& y) { return x.v <= y; }
static constexpr bool operator>=(const PlatformVersion& x, const ModuleVersion& y) { return x.v >= y; }
static constexpr bool operator==(const PlatformVersion& x, const ModuleVersion& y) { return x.v == y; }
static constexpr bool operator!=(const PlatformVersion& x, const ModuleVersion& y) { return x.v != y; }

} // namespace ds_dbw_can
