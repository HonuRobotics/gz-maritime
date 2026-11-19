/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <vector>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>

#include "gz/sim/acoustics/Hearing.hh"
#include "gz/sim/acoustics/Pinger.hh"
#include "gz/sim/acoustics/Random.hh"

using namespace gz;
using namespace sim;
using namespace acoustics;
using namespace std::chrono_literals;

namespace
{
/// \brief A pinger with a period and an offset.
/// \param[in] _period s.
/// \param[in] _offset s.
/// \return The pinger.
PingerData Pinger(double _period, double _offset)
{
  PingerData p;
  p.period = _period;
  p.offset = _offset;
  return p;
}
}

//////////////////////////////////////////////////
/// A pinger pings at its offset and then every period, each ping counted in
/// exactly one window, a ping on a window's edge included.
TEST(EmissionTimes, OffsetThenEveryPeriod)
{
  const PingerData p = Pinger(1.0, 0.25);
  EXPECT_TRUE(EmissionTimes(p, -1ns, 0ns).empty());
  const auto times = EmissionTimes(p, -1ns, 3s);
  ASSERT_EQ(3u, times.size());
  EXPECT_EQ(Duration(250ms), times[0]);
  EXPECT_EQ(Duration(1250ms), times[1]);
  EXPECT_EQ(Duration(2250ms), times[2]);

  // Split into steps, every ping lands once.
  std::size_t count{0};
  for (Duration t = 0ns; t < 10s; t += 4ms)
    count += EmissionTimes(p, t, t + 4ms).size();
  EXPECT_EQ(10u, count);

  // On the edge: (0.25, 1.25] holds 1.25 and not 0.25.
  const auto edge = EmissionTimes(p, 250ms, 1250ms);
  ASSERT_EQ(1u, edge.size());
  EXPECT_EQ(Duration(1250ms), edge[0]);
}

//////////////////////////////////////////////////
/// A ping at time zero is sent in the first window; an inactive pinger never
/// pings.
TEST(EmissionTimes, TimeZeroAndInactive)
{
  PingerData p = Pinger(0.5, 0.0);
  const auto first = EmissionTimes(p, -1ns, 4ms);
  ASSERT_EQ(1u, first.size());
  EXPECT_EQ(Duration::zero(), first[0]);
  p.active = false;
  EXPECT_TRUE(EmissionTimes(p, -1ns, 10s).empty());
}

//////////////////////////////////////////////////
/// Bearing counter clockwise from the receiver's x, elevation up positive,
/// both in the receiver's frame.
TEST(Measure, BearingAndElevationInTheReceiverFrame)
{
  // Straight ahead, then to port, then astern.
  const math::Pose3d origin;
  EXPECT_NEAR(0.0, Measure(origin, {10, 0, 0}).bearing, 1e-12);
  EXPECT_NEAR(GZ_PI / 2, Measure(origin, {0, 10, 0}).bearing, 1e-12);
  EXPECT_NEAR(GZ_PI, std::abs(Measure(origin, {-10, 0, 0}).bearing), 1e-12);

  // A pinger below gives a negative elevation; the range is slant.
  const Direction below = Measure(origin, {10, 0, -10});
  EXPECT_NEAR(-GZ_PI / 4, below.elevation, 1e-12);
  EXPECT_NEAR(std::sqrt(200.0), below.range, 1e-12);

  // A receiver turned 90 degrees to port sees a pinger on +x to starboard.
  const math::Pose3d turned(5, 0, 0, 0, 0, GZ_PI / 2);
  EXPECT_NEAR(-GZ_PI / 2, Measure(turned, {15, 0, 0}).bearing, 1e-12);
}

//////////////////////////////////////////////////
/// A band is a centre and a full width; a centre of zero hears everything.
TEST(InBand, CentreAndWidth)
{
  EXPECT_TRUE(InBand(30000, 0, 0));
  EXPECT_TRUE(InBand(30000, 30000, 0));
  EXPECT_TRUE(InBand(30500, 30000, 1000));
  EXPECT_FALSE(InBand(30501, 30000, 1000));
  EXPECT_FALSE(InBand(33000, 30000, 1000));
}

//////////////////////////////////////////////////
/// Nothing is lost before the dropout starts, everything at the reach, and
/// in between the chance rises in a straight line.
TEST(DropoutProbability, RisesToTheReach)
{
  EXPECT_DOUBLE_EQ(0.0, DropoutProbability(90, 100, 1.0));
  EXPECT_DOUBLE_EQ(0.0, DropoutProbability(1000, 0, 0.5));
  EXPECT_DOUBLE_EQ(0.0, DropoutProbability(50, 100, 0.5));
  EXPECT_DOUBLE_EQ(0.5, DropoutProbability(75, 100, 0.5));
  EXPECT_DOUBLE_EQ(1.0, DropoutProbability(100, 100, 0.5));
}

//////////////////////////////////////////////////
/// Spherical spreading: 6 dB for each doubling, 0 within a metre.
TEST(Amplitude, SphericalSpreading)
{
  EXPECT_DOUBLE_EQ(0.0, Amplitude(0.5));
  EXPECT_NEAR(-20.0, Amplitude(10), 1e-12);
  EXPECT_NEAR(Amplitude(10) - 20.0 * std::log10(2.0), Amplitude(20), 1e-12);
}

//////////////////////////////////////////////////
/// Sound over a distance, or at once without a speed.
TEST(TravelTime, DistanceOverSpeed)
{
  EXPECT_EQ(Duration(100ms), TravelTime(150, 1500));
  EXPECT_EQ(Duration::zero(), TravelTime(150, 0));
}

//////////////////////////////////////////////////
/// The first window is the step before, so a system added to a world that
/// has run for an hour sends one step of pings, not an hour of them; at the
/// world's first step a ping at time zero is in it.
TEST(EmissionTimes, TheFirstWindowIsOneStep)
{
  EXPECT_EQ(Duration(-1ns), FirstWindowStart(4ms, 4ms));
  EXPECT_EQ(Duration(3600s), FirstWindowStart(3600s + 4ms, 4ms));

  const PingerData p = Pinger(1.0, 0.0);
  EXPECT_EQ(1u, EmissionTimes(p, FirstWindowStart(4ms, 4ms), 4ms).size());
  const Duration late = 3600s + 4ms;
  EXPECT_TRUE(EmissionTimes(p, FirstWindowStart(late, 4ms), late).empty());
  EXPECT_EQ(1u, EmissionTimes(p, FirstWindowStart(3601s, 4ms), 3601s).size());
}

//////////////////////////////////////////////////
/// A field of view: from the lowest elevation heard to the highest.
TEST(InElevation, BetweenTheTwoAngles)
{
  EXPECT_TRUE(InElevation(0.4, -GZ_PI / 2, GZ_PI / 2));
  EXPECT_TRUE(InElevation(GZ_PI / 2, -GZ_PI / 2, GZ_PI / 2));
  EXPECT_TRUE(InElevation(-0.2, -0.5, 0.5));
  EXPECT_FALSE(InElevation(0.6, -0.5, 0.5));
  EXPECT_FALSE(InElevation(-0.6, -0.5, 0.5));
}

//////////////////////////////////////////////////
/// Two pings overlap when the later arrives while the earlier still sounds,
/// whichever is given first, and however long the later one lasts.
TEST(Overlap, TheLaterArrivesWhileTheEarlierSounds)
{
  // 0.13 ms apart, 4 ms pulses.
  EXPECT_TRUE(Overlap(10ms, 4ms, 10ms + 130us, 4ms));
  // 3.7 ms apart, 1 ms pulses.
  EXPECT_FALSE(Overlap(10ms, 1ms, 10ms + 3700us, 1ms));
  // The earlier pulse decides, not the longer.
  EXPECT_FALSE(Overlap(10ms, 1ms, 13ms, 4ms));
  EXPECT_FALSE(Overlap(13ms, 4ms, 10ms, 1ms));
  EXPECT_TRUE(Overlap(10ms, 4ms, 13ms, 1ms));
  EXPECT_TRUE(Overlap(13ms, 1ms, 10ms, 4ms));
  // Over exactly when the other arrives.
  EXPECT_FALSE(Overlap(10ms, 4ms, 14ms, 4ms));
  // Together, and a pulse of zero turns the rule off.
  EXPECT_TRUE(Overlap(10ms, 4ms, 10ms, 4ms));
  EXPECT_FALSE(Overlap(10ms, 0ms, 10ms, 0ms));
}

//////////////////////////////////////////////////
/// The draws are written out, so a seed gives the same numbers on every
/// standard library: the hash is FNV-1a, and a number in [0, 1) is one word
/// of the Mersenne Twister.
TEST(Random, TheSameEverywhere)
{
  EXPECT_EQ(0x811c9dc5u, Hash(""));
  EXPECT_EQ(0xe40c292cu, Hash("a"));
  EXPECT_EQ(0xbf9cf968u, Hash("foobar"));

  // The first word of a default seeded generator is 3499211612.
  Generator rng;
  EXPECT_DOUBLE_EQ(3499211612.0 / 4294967296.0, Uniform(rng));

  // Two generators of one seed draw the same numbers, spread evenly.
  Generator one(7), two(7);
  double sum{0.0};
  const int n = 20000;
  for (int i = 0; i < n; ++i)
  {
    const double u = Uniform(one);
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
    ASSERT_DOUBLE_EQ(u, Uniform(two));
    sum += u;
  }
  EXPECT_NEAR(0.5, sum / n, 0.01);
}

//////////////////////////////////////////////////
/// Angles wrap into (-pi, pi].
TEST(WrapAngle, IntoHalfOpenTurn)
{
  EXPECT_NEAR(GZ_PI, WrapAngle(GZ_PI), 1e-12);
  EXPECT_NEAR(GZ_PI, WrapAngle(-GZ_PI), 1e-12);
  EXPECT_NEAR(-GZ_PI / 2, WrapAngle(3 * GZ_PI / 2), 1e-12);
  EXPECT_NEAR(0.1, WrapAngle(0.1 + 4 * GZ_PI), 1e-12);
}

//////////////////////////////////////////////////
/// Spread, an unknown bias or a moving dynamic bias hide the range; a
/// fixed bias mean, readable in the SDF, does not.
TEST(HidesRange, SpreadOrABiasTheReceiverCannotRead)
{
  EXPECT_FALSE(HidesRange(0.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(HidesRange(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(HidesRange(0.0, 6.0, 0.0, 0.0));
  EXPECT_TRUE(HidesRange(0.0, 0.0, 3.0, 100.0));
  // A dynamic bias with no correlation time never moves.
  EXPECT_FALSE(HidesRange(0.0, 0.0, 3.0, 0.0));
}

//////////////////////////////////////////////////
/// Parameters are set by name, and out of range values are refused.
TEST(Pinger, SetParameter)
{
  PingerData p;
  EXPECT_TRUE(SetParameter(p, "frequency", 27000));
  EXPECT_DOUBLE_EQ(27000, p.frequency);
  EXPECT_TRUE(SetParameter(p, "active", 0));
  EXPECT_FALSE(p.active);
  EXPECT_TRUE(SetParameter(p, "reach", 0));
  EXPECT_FALSE(SetParameter(p, "period", 0));
  EXPECT_FALSE(SetParameter(p, "offset", -1));
  EXPECT_FALSE(SetParameter(p, "frequency", NAN));
  EXPECT_FALSE(SetParameter(p, "colour", 1));
  EXPECT_DOUBLE_EQ(1.0, p.period);
}

//////////////////////////////////////////////////
/// A pinger survives a round trip through its text form exactly, as it
/// must to replicate to another process.
TEST(Pinger, Serialization)
{
  PingerData p;
  p.name = "green buoy";
  p.active = false;
  p.pose = math::Pose3d(0.1, -0.2, -2.0 / 3.0, 0, 0, 0.3);
  p.frequency = 27500.5;
  p.period = 1.0 / 3.0;
  p.offset = 0.25;
  p.reach = 150;
  p.pulse_length = 0.004;
  p.generation = 7;

  std::stringstream ss;
  ss << p;
  PingerData q;
  ss >> q;
  EXPECT_EQ(p.name, q.name);
  EXPECT_EQ(p.active, q.active);
  EXPECT_EQ(p.generation, q.generation);
  EXPECT_EQ(p.pose.Pos(), q.pose.Pos());
  EXPECT_NEAR(0.3, q.pose.Rot().Yaw(), 1e-15);
  EXPECT_EQ(p.frequency, q.frequency);
  EXPECT_EQ(p.period, q.period);
  EXPECT_EQ(p.offset, q.offset);
  EXPECT_EQ(p.reach, q.reach);
  EXPECT_EQ(p.pulse_length, q.pulse_length);
}
