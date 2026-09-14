/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include <gtest/gtest.h>

#include <gz/msgs/double.pb.h>

// The scaling under test lives in ThrusterPrivateData::OnCmdThrust, and
// ThrusterPrivateData is declared in the .cc with no header of its own. Rather
// than lift it into one - which would put a diff in the vendored file that has
// nothing to do with the feature - the test compiles the translation unit
// directly. See PROVENANCE.md for why that file is kept close to upstream.
#include "Thruster.cc"  // NOLINT(build/include)

namespace
{
using gz::sim::maritime::ThrusterPrivateData;

/// \brief A thruster configured the way a real one is: asymmetric limits,
/// taken from the BlueRobotics T200 at 16 V.
/// \return Private data in normalized command mode, ready for OnCmdThrust.
std::unique_ptr<ThrusterPrivateData> MakeNormalizedThruster()
{
  auto data = std::make_unique<ThrusterPrivateData>();
  data->opmode = ThrusterPrivateData::OperationMode::NormalizedCmd;
  data->cmdMax = 51.5;
  data->cmdMin = -40.2;
  // Pin the coefficient so thrust does not depend on the advance-ratio
  // update, which needs a running simulation to have a velocity to read.
  data->thrustCoefficientSet = true;
  return data;
}

/// \brief Send one command through the subscriber callback.
/// \param[in] _data The thruster to command.
/// \param[in] _cmd The command value.
void Command(ThrusterPrivateData *_data, double _cmd)
{
  gz::msgs::Double msg;
  msg.set_data(_cmd);
  _data->OnCmdThrust(msg);
}
}  // namespace

/////////////////////////////////////////////////
TEST(NormalizedCmd, EndsAndCenterMapToTheThrustLimits)
{
  auto data = MakeNormalizedThruster();

  Command(data.get(), 1.0);
  EXPECT_DOUBLE_EQ(51.5, data->thrust);

  Command(data.get(), -1.0);
  EXPECT_DOUBLE_EQ(-40.2, data->thrust);

  Command(data.get(), 0.0);
  EXPECT_DOUBLE_EQ(0.0, data->thrust);
}

/////////////////////////////////////////////////
TEST(NormalizedCmd, EachDirectionScalesOnItsOwnLimit)
{
  auto data = MakeNormalizedThruster();

  // Half command is half of that direction's limit, not half of a shared
  // envelope - a T200 makes appreciably less astern than ahead, so +0.5 and
  // -0.5 are equal command and unequal force.
  Command(data.get(), 0.5);
  EXPECT_DOUBLE_EQ(0.5 * 51.5, data->thrust);

  Command(data.get(), -0.5);
  EXPECT_DOUBLE_EQ(0.5 * -40.2, data->thrust);
}

/////////////////////////////////////////////////
TEST(NormalizedCmd, OutOfRangeCommandsClampToFullScale)
{
  auto data = MakeNormalizedThruster();

  Command(data.get(), 1.7);
  EXPECT_DOUBLE_EQ(51.5, data->thrust);

  Command(data.get(), -1e6);
  EXPECT_DOUBLE_EQ(-40.2, data->thrust);
}

/////////////////////////////////////////////////
TEST(NormalizedCmd, NanIsTreatedAsStop)
{
  auto data = MakeNormalizedThruster();

  Command(data.get(), 1.0);
  ASSERT_DOUBLE_EQ(51.5, data->thrust);

  Command(data.get(), std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(0.0, data->thrust);
}

/////////////////////////////////////////////////
TEST(NormalizedCmd, AheadAndAsternSpinThePropellerOppositeWays)
{
  auto data = MakeNormalizedThruster();

  Command(data.get(), 1.0);
  const double ahead = data->propellerAngVel;
  EXPECT_GT(ahead, 0.0);

  Command(data.get(), -1.0);
  EXPECT_LT(data->propellerAngVel, 0.0);
}

/////////////////////////////////////////////////
TEST(ForceCmd, IsUnchangedByTheNormalizedMode)
{
  auto data = MakeNormalizedThruster();
  data->opmode = ThrusterPrivateData::OperationMode::ForceCmd;

  // The same 1.0 that is full scale in normalized mode is one newton here.
  Command(data.get(), 1.0);
  EXPECT_DOUBLE_EQ(1.0, data->thrust);

  // Force mode still clamps to the limits, as it did before this feature.
  Command(data.get(), 1e6);
  EXPECT_DOUBLE_EQ(51.5, data->thrust);
}
