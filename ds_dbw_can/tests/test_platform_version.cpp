/*********************************************************************
 * C++ unit test for dbw_common/PV.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <ds_dbw_can/PlatformVersion.hpp>
using namespace ds_dbw_can;

using PV = PlatformVersion;
using MV = ModuleVersion;

// Test empty constructor
TEST(PlatformVersion, empty) {
  EXPECT_EQ((Platform)0, PV().p);
  EXPECT_EQ(  (Module)0, PV().m);
  EXPECT_EQ(       MV(), PV().v);
}

// Test that fields are populated by each constructor
TEST(PlatformVersion, constructor) {
  EXPECT_EQ((Platform)1, PV((Platform)1, (Module)2, MV(3, 4, 5)).p);
  EXPECT_EQ(  (Module)2, PV((Platform)1, (Module)2, MV(3, 4, 5)).m);
  EXPECT_EQ(MV(3, 4, 5), PV((Platform)1, (Module)2, MV(3, 4, 5)).v);
  EXPECT_EQ((Platform)1, PV((Platform)1, (Module)2, 3, 4, 5).p);
  EXPECT_EQ(  (Module)2, PV((Platform)1, (Module)2, 3, 4, 5).m);
  EXPECT_EQ(MV(3, 4, 5), PV((Platform)1, (Module)2, 3, 4, 5).v);
}

// Test operators
TEST(PlatformVersion, operators) {
  const Platform x = (Platform)1; const Module y = (Module)2;
  const Platform X = (Platform)3; const Module Y = (Module)3;

  // Compare PV with PV
  EXPECT_FALSE(PV(x, y, 1, 1, 1) <  PV(x, y, 1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) <  PV(x, y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) <  PV(X, y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) <  PV(x, Y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 9, 9, 9) <= PV(x, y, 1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) <= PV(x, y, 1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) <= PV(x, y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) <= PV(X, y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) <= PV(x, Y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) >  PV(x, y, 9, 9, 9));
  EXPECT_TRUE (PV(x, y, 9, 9, 9) >  PV(x, y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 9, 9, 9) >  PV(X, y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 9, 9, 9) >  PV(x, Y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) >= PV(x, y, 9, 9, 9));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) >= PV(x, y, 1, 1, 1));
  EXPECT_TRUE (PV(x, y, 9, 9, 9) >= PV(x, y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 9, 9, 9) >= PV(X, y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 9, 9, 9) >= PV(x, Y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) == PV(x, y, 9, 9, 9));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) == PV(x, y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) == PV(X, y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) == PV(x, Y, 1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) != PV(x, y, 1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) != PV(x, y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) != PV(X, y, 9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) != PV(x, Y, 9, 9, 9));

  // Compare PV with MV
  EXPECT_FALSE(PV(x, y, 1, 1, 1) <  MV(1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) <  MV(9, 9, 9));
  EXPECT_FALSE(PV(x, y, 9, 9, 9) <= MV(1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) <= MV(1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) <= MV(9, 9, 9));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) >  MV(9, 9, 9));
  EXPECT_TRUE (PV(x, y, 9, 9, 9) >  MV(1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) >= MV(9, 9, 9));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) >= MV(1, 1, 1));
  EXPECT_TRUE (PV(x, y, 9, 9, 9) >= MV(1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) == MV(9, 9, 9));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) == MV(1, 1, 1));
  EXPECT_FALSE(PV(x, y, 1, 1, 1) != MV(1, 1, 1));
  EXPECT_TRUE (PV(x, y, 1, 1, 1) != MV(9, 9, 9));
}

// Test platformToString()
TEST(PlatformVersion, platformToString) {
  EXPECT_STREQ("FORD_CD4",       platformToString(Platform::FORD_CD4));
  EXPECT_STREQ("FORD_P5",        platformToString(Platform::FORD_P5));
  EXPECT_STREQ("FORD_T6",        platformToString(Platform::FORD_T6));
  EXPECT_STREQ("FORD_U6",        platformToString(Platform::FORD_U6));
  EXPECT_STREQ("FORD_CD5",       platformToString(Platform::FORD_CD5));
  EXPECT_STREQ("FORD_GE1",       platformToString(Platform::FORD_GE1));
  EXPECT_STREQ("FORD_P702",      platformToString(Platform::FORD_P702));
  EXPECT_STREQ("FORD_V3",        platformToString(Platform::FORD_V3));
  EXPECT_STREQ("FCA_RU",         platformToString(Platform::FCA_RU));
  EXPECT_STREQ("FCA_WK2",        platformToString(Platform::FCA_WK2));
  EXPECT_STREQ("POLARIS_GEM",    platformToString(Platform::POLARIS_GEM));
  EXPECT_STREQ("POLARIS_RZRXP",  platformToString(Platform::POLARIS_RZRXP));
  EXPECT_STREQ("POLARIS_RANGER", platformToString(Platform::POLARIS_RANGER));
  EXPECT_STREQ("POLARIS_RZRR",   platformToString(Platform::POLARIS_RZRR));
  for (size_t i = 0x90; i <= UINT8_MAX; i++) {
    EXPECT_STREQ("UNKNOWN", platformToString((Platform)i)) << "i = " << i;
  }
}

// Test moduleToString()
TEST(PlatformVersion, moduleToString) {
  EXPECT_STREQ("Gateway ", moduleToString(Module::Gateway));
  EXPECT_STREQ("Steer   ", moduleToString(Module::Steer));
  EXPECT_STREQ("Brake   ", moduleToString(Module::Brake));
  EXPECT_STREQ("Throttle", moduleToString(Module::Throttle));
  EXPECT_STREQ("Shift   ", moduleToString(Module::Shift));
  EXPECT_STREQ("BOO     ", moduleToString(Module::BOO));
  EXPECT_STREQ("Monitor ", moduleToString(Module::Monitor));
  EXPECT_STREQ("UNKNOWN", moduleToString((Module)0));
  for (size_t i = 0; i < (size_t)Module::Gateway; i++) {
    EXPECT_STREQ("UNKNOWN", moduleToString((Module)i)) << "i = " << i;
  }
  for (size_t i = (size_t)Module::MAX; i < UINT16_MAX; i++) {
    EXPECT_STREQ("UNKNOWN", moduleToString((Module)i)) << "i = " << i;
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
