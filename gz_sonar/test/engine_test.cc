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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Engine.hh"
#include "gz/maritime/sonar/Rays.hh"
#include "gz/maritime/sonar/Readout.hh"
#include "gz/maritime/sonar/SyntheticScene.hh"
#include "gz/maritime/sonar/Water.hh"

using gz::math::Pose3d;
using gz::math::Vector3d;
namespace sonar = gz::maritime::sonar;

namespace
{
constexpr double kDeg = M_PI / 180.0;

sonar::Water HarborWater()
{
  sonar::Water w;
  w.temperature = 29.0;
  w.salinity = 30.0;
  w.soundSpeed = 1500.0;
  return w;
}

/// \brief The ECHO with a 10 m window, no speckle: the scenes of
/// spike/sonar_phase1/scenes.md.
sonar::SonarConfig CleanEcho()
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  c.samples = 512;
  c.speckle = false;
  c.noiseFloorDb = -300.0;
  return c;
}

sonar::AcousticProperties Lambert(double _db)
{
  sonar::AcousticProperties p;
  p.lambertDb = _db;
  return p;
}

sonar::BeamIntensity Ping(const sonar::SonarConfig &_config,
  const sonar::SyntheticScene &_scene, const Pose3d &_pose,
  std::uint64_t _ping = 0)
{
  sonar::Engine engine(_config, HarborWater());
  const auto rays = sonar::RayFan(_config);
  auto hits = _scene.Cast(_pose, rays, 1.5 * _config.rangeMax);
  sonar::AssignFootprints(hits, sonar::RayFanElevationCount(_config));
  return engine.Ping(hits, _ping);
}

double TotalDb(const sonar::BeamIntensity &_b)
{
  return 10.0 * std::log10(
    std::accumulate(_b.data.begin(), _b.data.end(), 0.0));
}

/// \brief Beam and sample of the brightest cell.
std::pair<std::size_t, std::size_t> Brightest(const sonar::BeamIntensity &_b)
{
  const auto it = std::max_element(_b.data.begin(), _b.data.end());
  const auto i = static_cast<std::size_t>(it - _b.data.begin());
  return {i / _b.samples, i % _b.samples};
}
}  // namespace

/////////////////////////////////////////////////
// Francois and Garrison at 100 kHz, 10 C, 35 PSU, pH 8, the surface: about
// 34 dB/km, worked by hand from the paper's equations.
TEST(Water, AbsorptionMatchesFrancoisGarrison)
{
  sonar::Water w;
  w.temperature = 10.0;
  w.salinity = 35.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(100e3, w), 0.0336, 0.0015);

  // It rises with frequency across the sonars in scope.
  double last = 0.0;
  for (double f : {100e3, 240e3, 450e3, 700e3, 900e3, 1.2e6})
  {
    const double a = sonar::AbsorptionDbPerMeter(f, w);
    EXPECT_GT(a, last);
    last = a;
  }
}

/////////////////////////////////////////////////
TEST(Scattering, LambertAndSpecular)
{
  sonar::AcousticProperties p = Lambert(-20.0);
  EXPECT_NEAR(sonar::Backscatter(p, 0.0), 0.01, 1e-12);
  EXPECT_NEAR(sonar::Backscatter(p, 60.0 * kDeg), 0.0025, 1e-12);

  p.specularDb = 0.0;
  p.specularWidth = 4.0 * kDeg;
  EXPECT_NEAR(sonar::Backscatter(p, 0.0), 1.01, 1e-12);
  // Well outside the lobe only Lambert remains.
  EXPECT_NEAR(sonar::Backscatter(p, 30.0 * kDeg),
    0.01 * std::pow(std::cos(30.0 * kDeg), 2), 1e-9);
}

/////////////////////////////////////////////////
TEST(Pattern, HalfPowerWidthAndSidelobe)
{
  const sonar::SonarConfig c = CleanEcho();
  EXPECT_NEAR(sonar::HorizontalPattern(c, 0.0), 1.0, 1e-12);
  EXPECT_NEAR(sonar::HorizontalPattern(c, 0.5 * c.beamWidth), 0.5, 1e-6);

  // First sidelobe peak, searched beyond the first null.
  double peak = 0.0;
  for (double x = 1.6 * c.beamWidth; x < 3.0 * c.beamWidth;
       x += 0.001 * c.beamWidth)
  {
    peak = std::max(peak, sonar::HorizontalPattern(c, x));
  }
  EXPECT_NEAR(10.0 * std::log10(peak), c.sidelobeLevelDb, 0.2);

  EXPECT_NEAR(sonar::VerticalPattern(c, 0.5 * c.verticalFov), 0.5, 1e-9);
}

/////////////////////////////////////////////////
// Scene A: the sphere's near face, on the acoustic axis.
TEST(Engine, SphereAtItsRangeAndBearing)
{
  sonar::SyntheticScene scene;
  scene.AddSphere({5, 0, -3}, 0.25, Lambert(-10.0), 1);
  scene.AddPlane({0, 0, -10}, {0, 0, 1}, Lambert(-27.0), 2);
  const sonar::SonarConfig c = CleanEcho();
  const auto b = Ping(c, scene, Pose3d(0, 0, -3, 0, 0, 0));

  const auto [beam, sample] = Brightest(b);
  EXPECT_NEAR(b.ranges[sample], 4.75, 2.0 * c.SampleSpacing());
  EXPECT_NEAR(b.azimuths[beam], 0.0, c.beamWidth);
  EXPECT_EQ(b.labels[beam * b.samples + sample], 1u);
}

/////////////////////////////////////////////////
// A sphere's echo falls as 40 log R plus two-way absorption.
TEST(Engine, SpreadingAndAbsorption)
{
  sonar::SonarConfig c = CleanEcho();
  c.elevationRays = 512;
  auto total = [&](double _range) {
    sonar::SyntheticScene scene;
    scene.AddSphere({_range, 0, 0}, 0.25, Lambert(-10.0), 1);
    return TotalDb(Ping(c, scene, Pose3d::Zero));
  };
  const sonar::Engine engine(c, HarborWater());
  const double expected = 40.0 * std::log10(2.0) +
    2.0 * engine.Absorption() * 4.0;
  EXPECT_NEAR(total(4.0) - total(8.0), expected, 0.5);
}

/////////////////////////////////////////////////
// A flat patch returns mu cos^2(theta) times its area: tilting it away by
// theta costs cos^2(theta), doubling its side gains four times.
TEST(Engine, LambertLawAndAreaDependence)
{
  sonar::SonarConfig c = CleanEcho();
  c.elevationRays = 512;
  auto total = [&](double _tilt, double _half) {
    sonar::SyntheticScene scene;
    const Vector3d n(-std::cos(_tilt), std::sin(_tilt), 0);
    scene.AddRectangle({5, 0, 0}, n, {0, 0, 1}, _half, _half,
      Lambert(-20.0), 1);
    return TotalDb(Ping(c, scene, Pose3d::Zero));
  };
  const double flat = total(0.0, 0.2);
  EXPECT_NEAR(flat - total(30.0 * kDeg, 0.2),
    -20.0 * std::log10(std::cos(30.0 * kDeg)), 0.4);
  EXPECT_NEAR(flat - total(60.0 * kDeg, 0.2),
    -20.0 * std::log10(std::cos(60.0 * kDeg)), 0.4);
  EXPECT_NEAR(total(0.0, 0.4) - flat, 10.0 * std::log10(4.0), 0.3);
}

/////////////////////////////////////////////////
// Scene B: the box hides the seabed behind it.
TEST(Engine, ShadowBehindABox)
{
  const sonar::SonarConfig c = CleanEcho();
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  sonar::SyntheticScene open;
  open.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  sonar::SyntheticScene boxed = open;
  boxed.AddBox({4.5, -0.5, 0}, {5.5, 0.5, 0.5}, Lambert(-15.0), 2);

  const auto a = Ping(c, open, pose);
  const auto b = Ping(c, boxed, pose);
  const std::size_t center = c.beams / 2;
  double before = 0.0;
  double after = 0.0;
  for (std::size_t k = 0; k < c.samples; ++k)
  {
    if (a.ranges[k] > 6.5 && a.ranges[k] < 7.5)
    {
      before += a.At(center, k);
      after += b.At(center, k);
    }
  }
  ASSERT_GT(before, 0.0);
  EXPECT_LT(after, 0.05 * before);
}

/////////////////////////////////////////////////
// A small target off to one side: the beams around it follow the beam
// pattern.
TEST(Engine, CrossBeamProfileFollowsThePattern)
{
  sonar::SonarConfig c = CleanEcho();
  c.raysPerBeam = 8;
  c.elevationRays = 512;
  const std::size_t j0 = 160;
  const double az = c.BeamAzimuth(j0);
  sonar::SyntheticScene scene;
  scene.AddSphere({5 * std::cos(az), 5 * std::sin(az), 0}, 0.01,
    Lambert(-10.0), 1);
  const auto b = Ping(c, scene, Pose3d::Zero);

  std::vector<double> profile(c.beams, 0.0);
  for (std::size_t j = 0; j < c.beams; ++j)
    for (std::size_t k = 0; k < c.samples; ++k)
      profile[j] += b.At(j, k);
  ASSERT_GT(profile[j0], 0.0);
  for (std::size_t j = j0 - 2; j <= j0 + 2; ++j)
  {
    const double expected = sonar::HorizontalPattern(c,
      c.BeamAzimuth(j) - az);
    EXPECT_NEAR(10.0 * std::log10(profile[j] / profile[j0]),
      10.0 * std::log10(expected), 1.0) << "beam " << j;
  }
}

/////////////////////////////////////////////////
// Two targets at one range, above and below the axis, land in the same
// beam and sample: the imaging sonar cannot tell elevation.
TEST(Engine, ElevationAmbiguity)
{
  sonar::SonarConfig c = CleanEcho();
  c.elevationRays = 256;
  auto brightest = [&](double _elevation) {
    sonar::SyntheticScene scene;
    scene.AddSphere({5 * std::cos(_elevation), 0, 5 * std::sin(_elevation)},
      0.1, Lambert(-10.0), 1);
    return Brightest(Ping(c, scene, Pose3d::Zero));
  };
  EXPECT_EQ(brightest(5.0 * kDeg), brightest(-5.0 * kDeg));
}

/////////////////////////////////////////////////
// Fully developed speckle: the ratio to the clean intensity is
// exponential, mean 1 and standard deviation 1.
TEST(Engine, SpeckleStatistics)
{
  sonar::SonarConfig c = CleanEcho();
  sonar::SyntheticScene scene;
  scene.AddRectangle({5, 0, 0}, {-1, 0, 0}, {0, 0, 1}, 3.0, 3.0,
    Lambert(-20.0), 1);
  const auto clean = Ping(c, scene, Pose3d::Zero);
  c.speckle = true;
  const auto speckled = Ping(c, scene, Pose3d::Zero);

  const double top = *std::max_element(clean.data.begin(), clean.data.end());
  std::vector<double> ratio;
  for (std::size_t i = 0; i < clean.data.size(); ++i)
  {
    if (clean.data[i] > 1e-3 * top)
      ratio.push_back(speckled.data[i] / clean.data[i]);
  }
  ASSERT_GT(ratio.size(), 1000u);
  const double mean = std::accumulate(ratio.begin(), ratio.end(), 0.0) /
    static_cast<double>(ratio.size());
  double var = 0.0;
  for (double r : ratio)
    var += (r - mean) * (r - mean);
  var /= static_cast<double>(ratio.size());
  EXPECT_NEAR(mean, 1.0, 0.1);
  EXPECT_NEAR(std::sqrt(var), 1.0, 0.15);
}

/////////////////////////////////////////////////
TEST(Engine, SameSeedSamePing)
{
  sonar::SonarConfig c = CleanEcho();
  c.speckle = true;
  c.noiseFloorDb = -40.0;
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  sonar::Engine engine(c, HarborWater());

  auto frame = [&](std::uint64_t _ping) {
    const auto b = Ping(c, scene, pose, _ping);
    return sonar::ReadImaging(
      sonar::ReadLevels(b, c, engine.Absorption(), _ping), c).data;
  };
  EXPECT_EQ(frame(7), frame(7));
  EXPECT_NE(frame(7), frame(8));
}

/////////////////////////////////////////////////
// A sonar that assumes the wrong sound speed puts everything at the wrong
// range, in proportion.
TEST(Engine, AssumedSoundSpeedScalesRange)
{
  sonar::SonarConfig c = CleanEcho();
  c.soundSpeed = 1450.0;
  sonar::SyntheticScene scene;
  scene.AddSphere({5, 0, 0}, 0.25, Lambert(-10.0), 1);
  const auto b = Ping(c, scene, Pose3d::Zero);
  const auto [beam, sample] = Brightest(b);
  (void)beam;
  EXPECT_NEAR(b.ranges[sample], 4.75 * 1450.0 / 1500.0,
    2.0 * c.SampleSpacing());
}

/////////////////////////////////////////////////
// The 8-bit mapping: full scale to 255, the bottom of the window to 1,
// below it to 0.
TEST(Readout, ImagingMapping)
{
  sonar::SonarConfig c = CleanEcho();
  sonar::Levels l;
  l.beams = 1;
  l.samples = 4;
  l.db = {0.0, -c.dynamicRangeDb, -c.dynamicRangeDb - 1.0, 10.0};
  const auto f = sonar::ReadImaging(l, c);
  EXPECT_EQ(f.data[0], 255);
  EXPECT_EQ(f.data[1], 1);
  EXPECT_EQ(f.data[2], 0);
  EXPECT_EQ(f.data[3], 255);
}

/////////////////////////////////////////////////
// Sidescan: taking out the documented time-varied gain and the gain gives
// back the received level.
TEST(Readout, SidescanTvgIsRemovable)
{
  sonar::SonarConfig c = sonar::OmniscanSidescan();
  c.speckle = false;
  c.noiseFloorDb = -300.0;
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  // Looking to port (+y), 20 degrees down, 3 m up.
  const Pose3d pose(0, 0, 3, 0, 20.0 * kDeg, 90.0 * kDeg);
  sonar::Engine engine(c, HarborWater());
  const auto b = Ping(c, scene, pose);
  const auto line = sonar::ReadSidescan(
    sonar::ReadLevels(b, c, engine.Absorption(), 0));

  std::size_t checked = 0;
  for (std::size_t k = 0; k < c.samples; ++k)
  {
    if (b.At(0, k) <= 0.0)
      continue;
    const double received = line.db[k] - line.tvgDb[k] - line.gainDb;
    EXPECT_NEAR(received, 10.0 * std::log10(b.At(0, k)), 1e-3);
    ++checked;
  }
  EXPECT_GT(checked, 100u);
  // Nothing comes back before the seabed: the nadir gap.
  EXPECT_LE(b.At(0, 0), 0.0);
}

/////////////////////////////////////////////////
// A profiler over a flat seabed: every sounding lies on it.
TEST(Readout, ProfilingFlatSeabed)
{
  for (auto mode : {sonar::ProfilingDetection::kBeamformed,
                    sonar::ProfilingDetection::kAngleOfArrival})
  {
    sonar::SonarConfig c = sonar::SurveyorProfiling();
    c.detection = mode;
    c.rangeMax = 30.0;
    c.angleNoise = 0.0;
    sonar::SyntheticScene scene;
    scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
    // 10 m up, looking straight down, the fan across y.
    const Pose3d pose(0, 0, 10, 0, 90.0 * kDeg, 0);
    sonar::Engine engine(c, HarborWater());
    const auto b = Ping(c, scene, pose);
    const auto soundings = sonar::ReadProfiling(
      sonar::ReadLevels(b, c, engine.Absorption(), 0), c, 1500.0, 0);

    ASSERT_FALSE(soundings.empty());
    if (mode == sonar::ProfilingDetection::kAngleOfArrival)
    {
      EXPECT_LE(soundings.size(), c.maxSoundings);
      EXPECT_GE(soundings.size(), 10u);
    }
    for (const auto &s : soundings)
    {
      if (s.flag != sonar::SoundingFlag::kOk)
        continue;
      const double expected = 10.0 / std::cos(s.angle);
      // The seabed's footprint in a beam spans range, so allow a beam
      // width's worth of it on top of the sampling.
      const double slack = 10.0 * std::tan(std::abs(s.angle) +
        c.beamWidth) - 10.0 * std::tan(std::abs(s.angle));
      EXPECT_NEAR(s.range, expected, 3.0 * c.SampleSpacing() + slack)
        << "angle " << s.angle / kDeg;
      EXPECT_NEAR(s.travelTime, 2.0 * s.range / 1500.0, 1e-12);
    }
  }
}

/////////////////////////////////////////////////
// Threads change how fast, never what: the same bits for 1 or 8 threads.
TEST(Engine, SameResultForAnyThreadCount)
{
  sonar::SonarConfig c = CleanEcho();
  c.speckle = true;
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  scene.AddBox({4.5, -0.5, 0}, {5.5, 0.5, 0.5}, Lambert(-15.0), 2);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  c.threads = 1;
  const auto one = Ping(c, scene, pose, 3);
  c.threads = 8;
  const auto eight = Ping(c, scene, pose, 3);
  EXPECT_EQ(one.data, eight.data);
  EXPECT_EQ(one.labels, eight.labels);
}

/////////////////////////////////////////////////
// Smoothed automatic gain: a sudden change moves the gain by 1 - e^(-dt/tau)
// of the way per ping, and a zero time constant follows each ping.
TEST(Readout, GainTrackerSmooths)
{
  sonar::GainTracker t;
  EXPECT_DOUBLE_EQ(t.Update(-40.0, 0.1, 1.0), -40.0);
  const double step = 1.0 - std::exp(-0.1);
  EXPECT_NEAR(t.Update(-60.0, 0.1, 1.0), -40.0 - 20.0 * step, 1e-9);

  sonar::GainTracker follow;
  follow.Update(-40.0, 0.1, 0.0);
  EXPECT_DOUBLE_EQ(follow.Update(-60.0, 0.1, 0.0), -60.0);
}
