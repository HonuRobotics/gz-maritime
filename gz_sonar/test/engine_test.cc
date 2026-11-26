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
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Engine.hh"
#include "gz/maritime/sonar/Random.hh"
#include "gz/maritime/sonar/Rays.hh"
#include "gz/maritime/sonar/Readout.hh"
#include "gz/maritime/sonar/SyntheticScene.hh"
#include "gz/maritime/sonar/Water.hh"

using gz::math::Pose3d;
using gz::math::Vector3d;
namespace sonar = gz::maritime::sonar;

// The thread-failure test makes the system refuse threads by standing in
// for pthread_create, which a sanitizer does too; with one, the test is
// skipped.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define GZ_SONAR_TEST_SANITIZED
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(memory_sanitizer)
#define GZ_SONAR_TEST_SANITIZED
#endif
#endif

#if defined(__linux__) && !defined(GZ_SONAR_TEST_SANITIZED)
#define GZ_SONAR_TEST_CAN_REFUSE_THREADS
#include <dlfcn.h>
#include <pthread.h>

namespace
{
/// \brief Threads the system still grants before it refuses with EAGAIN;
/// negative for no limit.
std::atomic<int> threadsGranted{-1};

/// \brief Threads refused so far.
std::atomic<int> threadsRefused{0};
}  // namespace

//////////////////////////////////////////////////
/// \brief Stands in for the C library's pthread_create, which
/// std::thread calls, to refuse a thread on request.
extern "C" int pthread_create(pthread_t *_thread, const pthread_attr_t *_attr,
  void *(*_start)(void *), void *_arg) noexcept
{
  using Create = int (*)(pthread_t *, const pthread_attr_t *,
    void *(*)(void *), void *);
  static const auto real =
    reinterpret_cast<Create>(dlsym(RTLD_NEXT, "pthread_create"));
  int granted = threadsGranted.load();
  while (granted > 0 &&
         !threadsGranted.compare_exchange_weak(granted, granted - 1))
  {
  }
  if (granted == 0)
  {
    ++threadsRefused;
    return EAGAIN;
  }
  return real(_thread, _attr, _start, _arg);
}
#endif

namespace
{
constexpr double kDeg = M_PI / 180.0;

//////////////////////////////////////////////////
sonar::Water HarborWater()
{
  sonar::Water w;
  w.temperature = 29.0;
  w.salinity = 30.0;
  w.soundSpeed = 1500.0;
  return w;
}

//////////////////////////////////////////////////
/// \brief The ECHO with a 10 m window, no speckle: the scenes of
/// docs/design/render-a-ping.md.
/// \return The configuration.
sonar::SonarConfig CleanEcho()
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  c.samples = 512;
  c.speckle = false;
  c.noiseFloorDb = -300.0;
  return c;
}

//////////////////////////////////////////////////
sonar::AcousticProperties Lambert(double _db)
{
  sonar::AcousticProperties p;
  p.lambertDb = _db;
  return p;
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
double TotalDb(const sonar::BeamIntensity &_b)
{
  return 10.0 * std::log10(
    std::accumulate(_b.data.begin(), _b.data.end(), 0.0));
}

//////////////////////////////////////////////////
/// \brief Beam and sample of the brightest cell.
/// \param[in] _b The intensity.
/// \return The beam and the sample.
std::pair<std::size_t, std::size_t> Brightest(const sonar::BeamIntensity &_b)
{
  const auto it = std::max_element(_b.data.begin(), _b.data.end());
  const auto i = static_cast<std::size_t>(it - _b.data.begin());
  return {i / _b.samples, i % _b.samples};
}
}  // namespace

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
TEST(Scattering, LambertAndSpecular)
{
  sonar::AcousticProperties p = Lambert(-20.0);
  EXPECT_NEAR(sonar::Backscatter(p, 0.0), 0.01, 1e-12);
  EXPECT_NEAR(sonar::Backscatter(p, 60.0 * kDeg), 0.0025, 1e-12);

  p.specularDb = 0.0;
  p.specularWidth = 4.0 * kDeg;
  const double w = 4.0 * kDeg;
  EXPECT_NEAR(sonar::Backscatter(p, 0.0), 0.01 + 1.0 / (8.0 * M_PI * w * w),
    1e-9);
  // Well outside the lobe only Lambert remains.
  EXPECT_NEAR(sonar::Backscatter(p, 30.0 * kDeg),
    0.01 * std::pow(std::cos(30.0 * kDeg), 2), 1e-9);
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
// The specular term is tied to the reflection coefficient: a sphere of
// radius a with R^2 = 1 has the target strength of a rigid sphere, a^2 / 4,
// whatever the width of its lobe. Measured against a Lambert sphere, whose
// target strength is 2 pi a^2 mu / 3, so the sonar's own scale cancels.
TEST(Engine, SpecularSphereIsARigidSphere)
{
  sonar::SonarConfig c = CleanEcho();
  c.raysPerBeam = 8;
  c.elevationRays = 512;
  const double a = 0.25;
  auto total = [&](const sonar::AcousticProperties &_p) {
    sonar::SyntheticScene scene;
    scene.AddSphere({5, 0, 0}, a, _p, 1);
    return TotalDb(Ping(c, scene, Pose3d::Zero));
  };
  const double mu = 0.1;
  const double lambert = total(Lambert(10.0 * std::log10(mu)));
  for (double w : {0.05, 0.1})
  {
    sonar::AcousticProperties p = Lambert(-200.0);
    p.specularDb = 0.0;
    p.specularWidth = w;
    EXPECT_NEAR(total(p) - lambert,
      10.0 * std::log10((a * a / 4.0) / (2.0 * M_PI * a * a * mu / 3.0)),
      0.5) << "width " << w;
  }
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

namespace
{
//////////////////////////////////////////////////
/// \brief A uniform intensity over the sonar's beams and samples, for
/// readout tests.
/// \param[in] _config The sonar.
/// \param[in] _value The intensity.
/// \return The intensity.
sonar::BeamIntensity Flat(const sonar::SonarConfig &_config, double _value)
{
  sonar::BeamIntensity b;
  b.beams = _config.beams;
  b.samples = _config.samples;
  b.data.assign(b.beams * b.samples, _value);
  b.labels.assign(b.data.size(), 0);
  for (std::size_t k = 0; k < b.samples; ++k)
    b.ranges.push_back(_config.SampleRange(k));
  for (std::size_t j = 0; j < b.beams; ++j)
    b.azimuths.push_back(_config.BeamAzimuth(j));
  return b;
}

//////////////////////////////////////////////////
/// \brief Mean and standard deviation over a vector of powers.
/// \param[in] _v The powers.
/// \return The mean and the standard deviation.
std::pair<double, double> MeanStd(const std::vector<double> &_v)
{
  const double mean = std::accumulate(_v.begin(), _v.end(), 0.0) /
    static_cast<double>(_v.size());
  double var = 0.0;
  for (double x : _v)
    var += (x - mean) * (x - mean);
  return {mean, std::sqrt(var / static_cast<double>(_v.size()))};
}

//////////////////////////////////////////////////
/// \brief The published levels as powers relative to _ref dB.
/// \param[in] _l The levels.
/// \param[in] _ref The reference, dB.
/// \return The powers.
std::vector<double> Powers(const sonar::Levels &_l, double _ref)
{
  std::vector<double> p;
  for (double d : _l.db)
    p.push_back(std::pow(10.0, (d - _ref) / 10.0));
  return p;
}
}  // namespace

//////////////////////////////////////////////////
// Fully developed speckle: the ratio to the clean intensity is
// exponential, mean 1 and standard deviation 1.
TEST(Readout, SpeckleStatistics)
{
  sonar::SonarConfig c = CleanEcho();
  c.speckle = true;
  c.tvg = sonar::Tvg::kNone;
  c.gainDb = 0.0;
  const auto l = sonar::ReadLevels(Flat(c, 1e-6), c, 0.0, 0);
  const auto [mean, std] = MeanStd(Powers(l, -60.0));
  EXPECT_NEAR(mean, 1.0, 0.05);
  EXPECT_NEAR(std, 1.0, 0.05);
}

//////////////////////////////////////////////////
// Echo and noise are one field: where they are equal, one exponential draw
// on their sum keeps the contrast at 1, where two independent draws would
// give 1 / sqrt(2).
TEST(Readout, OneDrawOnSignalPlusNoise)
{
  sonar::SonarConfig c = CleanEcho();
  c.speckle = true;
  c.tvg = sonar::Tvg::kNone;
  c.gainDb = 0.0;
  c.noiseFloorDb = -60.0;
  const auto l = sonar::ReadLevels(Flat(c, 1e-6), c, 0.0, 0);
  const auto [mean, std] = MeanStd(Powers(l, -60.0));
  EXPECT_NEAR(mean, 2.0, 0.1);
  EXPECT_NEAR(std / mean, 1.0, 0.05);
}

//////////////////////////////////////////////////
// The noise is a received level: in the published levels it rises with
// range as the time-varied gain does, and moves with the gain.
TEST(Readout, NoiseRidesWithTheGain)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.gainDb = 40.0;
  sonar::Engine engine(c, HarborWater());
  const auto l = sonar::ReadLevels(Flat(c, 0.0), c, engine.Absorption(), 0);
  for (std::size_t k : {std::size_t{10}, c.samples / 2, c.samples - 10})
  {
    std::vector<double> column;
    for (std::size_t j = 0; j < l.beams; ++j)
      column.push_back(std::pow(10.0, l.db[j * l.samples + k] / 10.0));
    const double meanDb = 10.0 * std::log10(MeanStd(column).first);
    EXPECT_NEAR(meanDb, c.noiseFloorDb + l.tvgDb[k] + 40.0, 0.6)
      << "sample " << k;
    EXPECT_NEAR(sonar::NoiseLevel(l, c, k),
      c.noiseFloorDb + l.tvgDb[k] + 40.0, 1e-9);
  }
  EXPECT_GT(l.tvgDb[c.samples - 10] - l.tvgDb[10], 40.0);
}

//////////////////////////////////////////////////
TEST(Engine, SameSeedSamePing)
{
  sonar::SonarConfig c = CleanEcho();
  c.speckle = true;
  c.noiseFloorDb = -120.0;
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
// A profiler in empty water: the default threshold passes no noise, where
// 10 dB passed a few samples a ping.
TEST(Readout, ProfilingEmptyWaterIsQuiet)
{
  for (auto mode : {sonar::ProfilingDetection::kBeamformed,
                    sonar::ProfilingDetection::kAngleOfArrival})
  {
    sonar::SonarConfig c = sonar::SurveyorProfiling();
    c.detection = mode;
    sonar::Engine engine(c, HarborWater());
    auto count = [&](double _threshold) {
      c.detectionThresholdDb = _threshold;
      std::size_t n = 0;
      for (std::uint64_t ping = 0; ping < 50; ++ping)
      {
        n += sonar::ReadProfiling(sonar::ReadLevels(Flat(c, 0.0), c,
          engine.Absorption(), ping), c, 1500.0, ping).size();
      }
      return n;
    };
    EXPECT_EQ(count(sonar::SonarConfig().detectionThresholdDb), 0u);
    EXPECT_GT(count(10.0), 50u);
  }
}

//////////////////////////////////////////////////
// Threads change how fast, never what: the same bits for 1, 2, 3, 8 or 24
// threads (as many of them as the machine gives), at the imaging preset's
// full fan.
TEST(Engine, SameResultForAnyThreadCount)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  scene.AddBox({4.5, -0.5, 0}, {5.5, 0.5, 0.5}, Lambert(-15.0), 2);
  scene.AddSphere({12, 3, 0.25}, 0.25, Lambert(-10.0), 3);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  c.threads = 1;
  const auto one = Ping(c, scene, pose, 3);
  for (std::size_t threads : {0u, 2u, 3u, 5u, 8u, 24u})
  {
    c.threads = threads;
    const auto many = Ping(c, scene, pose, 3);
    EXPECT_EQ(one.data, many.data) << threads << " threads";
    EXPECT_EQ(one.labels, many.labels) << threads << " threads";
  }
}

//////////////////////////////////////////////////
// An engine reuses its buffers from ping to ping: a ping is the same as on
// a fresh engine, whatever came before it.
TEST(Engine, PingDoesNotDependOnThePrevious)
{
  sonar::SonarConfig c = CleanEcho();
  c.threads = 4;
  sonar::SyntheticScene near;
  near.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  near.AddBox({2.5, -0.5, 0}, {3.5, 0.5, 1.5}, Lambert(-10.0), 2);
  sonar::SyntheticScene far;
  far.AddSphere({8, -1, 2}, 0.5, Lambert(-15.0), 3);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  const auto rays = sonar::RayFan(c);
  auto nearHits = near.Cast(pose, rays, 1.5 * c.rangeMax);
  sonar::AssignFootprints(nearHits, sonar::RayFanElevationCount(c));
  auto farHits = far.Cast(pose, rays, 1.5 * c.rangeMax);
  sonar::AssignFootprints(farHits, sonar::RayFanElevationCount(c));

  const sonar::Engine fresh(c, HarborWater());
  const auto expected = fresh.Ping(farHits, 5);
  const sonar::Engine reused(c, HarborWater());
  reused.Ping(nearHits, 4);
  const auto after = reused.Ping(farHits, 5);
  EXPECT_EQ(expected.data, after.data);
  EXPECT_EQ(expected.labels, after.labels);
  EXPECT_EQ(expected.ranges, after.ranges);
  EXPECT_EQ(expected.azimuths, after.azimuths);
}

//////////////////////////////////////////////////
// One per core by default, but no more than kDefaultMaxEngineThreads; a
// smaller count is taken as given, and a larger one gets what there is.
TEST(Engine, ThreadCount)
{
  sonar::SonarConfig c = CleanEcho();
  c.threads = 0;
  const std::size_t all = std::min<std::size_t>(
    std::max(1u, std::thread::hardware_concurrency()),
    sonar::kDefaultMaxEngineThreads);
  EXPECT_EQ(sonar::Engine(c, HarborWater()).Threads(), all);
  for (std::size_t threads : {1u, 2u, 3u, 8u, 9u, 24u, 300u})
  {
    c.threads = threads;
    EXPECT_EQ(sonar::Engine(c, HarborWater()).Threads(),
      std::min(threads, all)) << threads;
  }
}

namespace
{
//////////////////////////////////////////////////
/// \brief Threads of this process.
/// \return The number of threads.
std::size_t ThreadsOfProcess()
{
  std::size_t n = 0;
  for (const auto &e : std::filesystem::directory_iterator("/proc/self/task"))
  {
    (void)e;
    ++n;
  }
  return n;
}

//////////////////////////////////////////////////
/// \brief Worker threads the engines of this process share.
/// \return The number of threads.
std::size_t SharedWorkers()
{
  return std::min<std::size_t>(
    std::max(1u, std::thread::hardware_concurrency()),
    sonar::kDefaultMaxEngineThreads) - 1;
}
}  // namespace

#ifdef __linux__
//////////////////////////////////////////////////
// The engines share their threads, and the threads go with the last engine:
// eight sonars run no more threads than one, and making and dropping
// engines, as a world does when sonars are removed and spawned again,
// leaves no threads behind.
TEST(Engine, NoThreadsLeftBehind)
{
  sonar::SonarConfig c = CleanEcho();
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  const auto hits = scene.Cast(pose, sonar::RayFan(c), 1.5 * c.rangeMax);

  const std::size_t before = ThreadsOfProcess();
  for (int round = 0; round < 5; ++round)
  {
    // An engine on one thread starts none.
    c.threads = 1;
    auto single = std::make_unique<sonar::Engine>(c, HarborWater());
    EXPECT_EQ(ThreadsOfProcess(), before);

    std::vector<std::unique_ptr<sonar::Engine>> engines;
    for (std::size_t threads : {0u, 8u, 3u, 0u, 2u, 24u, 0u, 0u})
    {
      c.threads = threads;
      engines.push_back(std::make_unique<sonar::Engine>(c, HarborWater()));
      EXPECT_EQ(ThreadsOfProcess(), before + SharedWorkers());
    }
    for (auto &engine : engines)
      engine->Ping(hits, static_cast<std::uint64_t>(round));
    EXPECT_EQ(ThreadsOfProcess(), before + SharedWorkers());

    // The threads stay while any engine uses them, whichever it is.
    std::swap(engines[round], engines.back());
    while (engines.size() > 1)
    {
      engines.pop_back();
      EXPECT_EQ(ThreadsOfProcess(), before + SharedWorkers());
    }
    engines.clear();
    EXPECT_EQ(ThreadsOfProcess(), before);
  }
}
#endif

//////////////////////////////////////////////////
// Engines that ping at the same moment take turns at the shared threads
// and share nothing else: every ping is the same bits as alone on one
// thread, for sonars of each kind and thread limit, while other engines
// come and go, and none waits forever.
TEST(Engine, EnginesPingingAtOnce)
{
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  scene.AddBox({4.5, -0.5, 0}, {5.5, 0.5, 0.5}, Lambert(-15.0), 2);
  scene.AddSphere({12, 3, 0.25}, 0.25, Lambert(-10.0), 3);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);

  struct Sonar
  {
    sonar::SonarConfig config;
    std::vector<sonar::RayHit> hits;
    sonar::BeamIntensity expected;
    std::unique_ptr<sonar::Engine> engine;
  };
  std::vector<Sonar> sonars;
  auto add = [&](sonar::SonarConfig _config, std::size_t _threads) {
    Sonar s;
    s.config = _config;
    s.hits = scene.Cast(pose, sonar::RayFan(_config),
      1.5 * _config.rangeMax);
    sonar::AssignFootprints(s.hits, sonar::RayFanElevationCount(_config));
    s.config.threads = 1;
    s.expected = sonar::Engine(s.config, HarborWater()).Ping(s.hits, 0);
    s.config.threads = _threads;
    s.engine = std::make_unique<sonar::Engine>(s.config, HarborWater());
    sonars.push_back(std::move(s));
  };
  sonar::SonarConfig oneCell = CleanEcho();
  oneCell.beams = 1;
  oneCell.samples = 1;
  add(CleanEcho(), 0);
  add(CleanEcho(), 0);
  add(CleanEcho(), 3);
  add(CleanEcho(), 1);
  add(sonar::EchoImaging(), 0);
  add(sonar::OmniscanSidescan(), 0);
  add(sonar::SurveyorProfiling(), 2);
  add(oneCell, 300);

  for (int repetition = 0; repetition < 3; ++repetition)
  {
    std::atomic<int> wrong{0};
    std::atomic<bool> go{false};
    std::atomic<bool> stop{false};
    std::vector<std::thread> threads;
    for (Sonar &s : sonars)
    {
      threads.emplace_back([&]() {
        while (!go)
          std::this_thread::yield();
        for (std::uint64_t ping = 0; ping < 12; ++ping)
        {
          const auto out = s.engine->Ping(s.hits, ping);
          if (out.data != s.expected.data || out.labels != s.expected.labels)
            ++wrong;
        }
      });
    }
    // Sonars spawned and removed meanwhile, as in a running world.
    std::thread churn([&]() {
      sonar::SonarConfig c = CleanEcho();
      while (!go)
        std::this_thread::yield();
      for (std::size_t n = 0; !stop; ++n)
      {
        c.threads = n % 4;
        const sonar::Engine engine(c, HarborWater());
        const auto out = engine.Ping(sonars[0].hits, n);
        if (out.data != sonars[0].expected.data)
          ++wrong;
      }
    });
    go = true;
    for (auto &t : threads)
      t.join();
    stop = true;
    churn.join();
    EXPECT_EQ(wrong, 0) << "repetition " << repetition;
  }
}

//////////////////////////////////////////////////
// Pings from several threads on one engine wait for each other, with the
// other engines' pings going on around them.
TEST(Engine, OneEngineFromSeveralThreads)
{
  const sonar::SonarConfig c = CleanEcho();
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  scene.AddSphere({5, 1, 1}, 0.5, Lambert(-10.0), 2);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  auto hits = scene.Cast(pose, sonar::RayFan(c), 1.5 * c.rangeMax);
  sonar::AssignFootprints(hits, sonar::RayFanElevationCount(c));
  const std::vector<sonar::RayHit> none;

  const sonar::Engine engine(c, HarborWater());
  const sonar::Engine other(c, HarborWater());
  const auto expected = engine.Ping(hits, 0);
  const auto empty = engine.Ping(none, 0);
  EXPECT_EQ(std::count(empty.data.begin(), empty.data.end(), 0.0),
    static_cast<std::ptrdiff_t>(empty.data.size()));

  std::atomic<int> wrong{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 6; ++t)
  {
    threads.emplace_back([&, t]() {
      for (std::uint64_t ping = 0; ping < 10; ++ping)
      {
        // Two threads on the other engine, four on this one, and every
        // other ping of one of them with no hit at all.
        const sonar::Engine &e = t < 2 ? other : engine;
        const bool blank = t == 5 && ping % 2 == 1;
        const auto out = e.Ping(blank ? none : hits, ping);
        if (out.data != (blank ? empty.data : expected.data) ||
            out.labels != (blank ? empty.labels : expected.labels))
        {
          ++wrong;
        }
      }
    });
  }
  for (auto &t : threads)
    t.join();
  EXPECT_EQ(wrong, 0);
}

//////////////////////////////////////////////////
// A system that refuses a thread after granting others leaves an engine
// that works on those it got, and the next engine to start the threads
// gets them all. The old pool never returned from its constructor.
TEST(Engine, ThreadThatFailsToStart)
{
#ifdef GZ_SONAR_TEST_CAN_REFUSE_THREADS
  if (SharedWorkers() == 0)
    GTEST_SKIP() << "one core: the engines start no threads";
  sonar::SonarConfig c = CleanEcho();
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  scene.AddBox({2.5, -0.5, 0}, {3.5, 0.5, 1.5}, Lambert(-10.0), 2);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);
  auto hits = scene.Cast(pose, sonar::RayFan(c), 1.5 * c.rangeMax);
  sonar::AssignFootprints(hits, sonar::RayFanElevationCount(c));
  c.threads = 1;
  const auto expected = sonar::Engine(c, HarborWater()).Ping(hits, 0);
  c.threads = 0;

  // Refused at the first thread, after some, and at the last.
  for (std::size_t granted : {std::size_t(0), SharedWorkers() / 2,
       SharedWorkers() - 1})
  {
    const std::size_t before = ThreadsOfProcess();
    threadsRefused = 0;
    threadsGranted = static_cast<int>(granted);
    {
      const sonar::Engine engine(c, HarborWater());
      threadsGranted = -1;
      EXPECT_EQ(threadsRefused, 1) << granted;
      EXPECT_EQ(engine.Threads(), granted + 1);
      EXPECT_EQ(ThreadsOfProcess(), before + granted);
      for (std::uint64_t ping = 0; ping < 3; ++ping)
      {
        const auto out = engine.Ping(hits, ping);
        EXPECT_EQ(out.data, expected.data) << granted;
        EXPECT_EQ(out.labels, expected.labels) << granted;
      }
      // A second engine shares the short pool while the first lives.
      const sonar::Engine second(c, HarborWater());
      EXPECT_EQ(second.Threads(), granted + 1);
      EXPECT_EQ(second.Ping(hits, 0).data, expected.data);
    }
    // With both gone the threads are joined, and the next engine starts a
    // full set.
    const sonar::Engine next(c, HarborWater());
    EXPECT_EQ(next.Threads(), SharedWorkers() + 1);
    EXPECT_EQ(next.Ping(hits, 0).data, expected.data);
  }
  threadsGranted = -1;
#else
  GTEST_SKIP() << "cannot refuse a thread on this build";
#endif
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
// A plate at 4 m in front of a wall at 8 m. Where a ray on the plate's
// edge has a neighbor on the wall, the jump is not a slanted surface, so
// the footprint must not spread the plate's echo across it, whether or not
// the two share a label (parts of one model do). The water between them
// stays empty.
TEST(Engine, FootprintDoesNotSpanAJump)
{
  const sonar::SonarConfig c = CleanEcho();
  for (std::uint32_t wallLabel : {3u, 2u})
  {
    sonar::SyntheticScene scene;
    scene.AddRectangle({4, 0, 2}, {-1, 0, 0}, {0, 1, 0}, 0.5, 0.5,
      Lambert(-15.0), 2);
    scene.AddPlane({8, 0, 0}, {-1, 0, 0}, Lambert(-20.0), wallLabel);
    const auto b = Ping(c, scene, Pose3d(0, 0, 2, 0, 0, 0));

    double plate = 0.0, gap = 0.0;
    for (std::size_t i = 0; i < b.beams; ++i)
    {
      if (std::abs(b.azimuths[i]) > std::atan(0.4 / 4.0))
        continue;
      for (std::size_t k = 0; k < b.samples; ++k)
      {
        const double r = b.ranges[k];
        const double v = b.data[i * b.samples + k];
        if (r > 3.5 && r < 4.5)
          plate = std::max(plate, v);
        else if (r > 4.5 && r < 7.5)
          gap = std::max(gap, v);
      }
    }
    ASSERT_GT(plate, 0.0);
    std::printf("wall label %u: gap %.1f dB relative to the plate\n",
      wallLabel, gap > 0.0 ? 10.0 * std::log10(gap / plate) : -999.0);
    EXPECT_EQ(gap, 0.0) << "wall label " << wallLabel;
  }
}

//////////////////////////////////////////////////
// The bound on a jump between neighbors is loose enough for a flat seabed
// seen at a grazing angle: from 1 m above it, out to the end of the ECHO's
// 30 m window, every pair of neighboring rays on the seabed is one surface.
TEST(Rays, SeabedAtGrazingAnglesIsOneSurface)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  const auto rays = sonar::RayFan(c);
  const auto hits = scene.Cast(Pose3d(0, 0, 1, 0, 0, 0), rays, 1.5 * c.rangeMax);
  const std::size_t ne = sonar::RayFanElevationCount(c);
  std::size_t pairs = 0, split = 0;
  for (std::size_t n = 0; n + 1 < hits.size(); ++n)
  {
    for (std::size_t m : {n + 1, n + ne})
    {
      if (m >= hits.size() || (m == n + 1 && m % ne == 0))
        continue;
      if (!hits[n].hit || !hits[m].hit || hits[n].range > c.rangeMax ||
          hits[m].range > c.rangeMax)
      {
        continue;
      }
      ++pairs;
      if (!sonar::SameSurface(hits[n], hits[m]))
        ++split;
    }
  }
  EXPECT_GT(pairs, 10000u);
  EXPECT_EQ(split, 0u);
}

namespace
{
//////////////////////////////////////////////////
/// \brief One ray straight ahead, hitting a Lambert patch head on: the
/// sonar equation can be worked by hand for it.
/// \param[in] _range The range, m.
/// \return The hit.
sonar::RayHit OneHit(double _range)
{
  sonar::RayHit h;
  h.hit = true;
  h.range = _range;
  h.ray.solidAngle = 1e-4;
  h.ray.spacing = 0.01;
  h.properties = Lambert(-20.0);
  h.label = 1;
  return h;
}

//////////////////////////////////////////////////
double Total(const sonar::BeamIntensity &_b)
{
  return std::accumulate(_b.data.begin(), _b.data.end(), 0.0);
}

//////////////////////////////////////////////////
/// \brief The samples a beam's echo covers, first to last.
/// \param[in] _b The intensity.
/// \param[in] _beam The beam's index.
/// \return The first and last samples.
std::pair<std::size_t, std::size_t> Extent(const sonar::BeamIntensity &_b,
  std::size_t _beam)
{
  std::size_t first = _b.samples, last = 0;
  for (std::size_t k = 0; k < _b.samples; ++k)
  {
    if (_b.At(_beam, k) > 0.0)
    {
      first = std::min(first, k);
      last = k;
    }
  }
  return {first, last};
}
}  // namespace

//////////////////////////////////////////////////
// The sonar equation for one ray, term by term against the hand value:
// the echo of a patch filling a fixed solid angle falls as 1 / R^2 and
// two-way absorption, rises with the solid angle and the Lambert strength,
// goes as mu cos^2 over cos with incidence (the patch's area grows as
// 1 / cos), and the vertical pattern takes 3 dB off at the edge of the
// vertical field of view and 12 dB at twice that.
TEST(Engine, SonarEquationForOneRay)
{
  sonar::SonarConfig c = CleanEcho();
  const sonar::Engine engine(c, HarborWater());
  const double alpha = engine.Absorption();
  auto total = [&](const sonar::RayHit &_h) {
    return Total(engine.Ping({_h}, 0));
  };
  const double ref = total(OneHit(2.0));
  ASSERT_GT(ref, 0.0);

  EXPECT_NEAR(10.0 * std::log10(ref / total(OneHit(6.0))),
    20.0 * std::log10(3.0) + 2.0 * alpha * 4.0, 1e-6);

  sonar::RayHit h = OneHit(2.0);
  h.ray.solidAngle = 2e-4;
  EXPECT_NEAR(total(h) / ref, 2.0, 1e-9);

  h = OneHit(2.0);
  h.properties = Lambert(-10.0);
  EXPECT_NEAR(total(h) / ref, 10.0, 1e-9);

  h = OneHit(2.0);
  h.incidence = 75.0 * kDeg;
  EXPECT_NEAR(total(h) / ref, std::cos(75.0 * kDeg), 1e-9);

  h = OneHit(2.0);
  h.ray.elevation = 0.5 * c.verticalFov;
  EXPECT_NEAR(total(h) / ref, 0.5, 1e-9);
  h.ray.elevation = -c.verticalFov;
  EXPECT_NEAR(total(h) / ref, 1.0 / 16.0, 1e-9);
  // Five half-widths out, 75 dB down: below the -50 dB floor, dropped.
  h.ray.elevation = 2.5 * c.verticalFov;
  EXPECT_EQ(total(h), 0.0);
}

//////////////////////////////////////////////////
// Sample k covers [rangeMin + k dr, rangeMin + (k + 1) dr) and is
// published at its center: an echo 0.3 samples past a center is brightest
// in that sample, and its published range is 0.3 samples short.
TEST(Engine, EchoInTheSampleThatHoldsItsRange)
{
  const sonar::SonarConfig c = CleanEcho();
  const double dr = c.SampleSpacing();
  const sonar::Engine engine(c, HarborWater());
  for (std::size_t k : {std::size_t{0}, std::size_t{100}, c.samples - 1})
  {
    const double r = c.rangeMin + (static_cast<double>(k) + 0.8) * dr;
    const auto b = engine.Ping({OneHit(r)}, 0);
    const auto [beam, sample] = Brightest(b);
    (void)beam;
    EXPECT_EQ(sample, k);
    EXPECT_NEAR(b.ranges[sample], r - 0.3 * dr, 1e-9);
  }
}

//////////////////////////////////////////////////
// The range window is [rangeMin, rangeMax) in apparent range: an echo
// outside it is dropped whole, even where a long pulse would smear it
// across the edge, and an echo just inside is kept. With a mis-set sound
// speed the window is applied to the range the sonar thinks it sees.
TEST(Engine, RangeWindowIsHalfOpen)
{
  sonar::SonarConfig c = CleanEcho();
  c.pulseLength = 1e-4;
  auto total = [&](double _range) {
    const sonar::Engine engine(c, HarborWater());
    return Total(engine.Ping({OneHit(_range)}, 0));
  };
  EXPECT_EQ(total(c.rangeMax), 0.0);
  EXPECT_EQ(total(c.rangeMax + 0.01), 0.0);
  EXPECT_EQ(total(c.rangeMin - 0.01), 0.0);
  EXPECT_GT(total(c.rangeMax - 0.05), 0.0);
  EXPECT_GT(total(c.rangeMin + 0.05), 0.0);

  // Assumed 1450 m/s in 1500 m/s water: 10.2 m reads as 9.86 m.
  c.soundSpeed = 1450.0;
  EXPECT_GT(total(10.2), 0.0);
  EXPECT_EQ(total(10.4), 0.0);
}

//////////////////////////////////////////////////
// An echo spreads uniformly over the wider of the pulse (c tau / 2) and its
// footprint in range, given by the sampler or estimated from the incidence
// and capped at 8 samples; spreading moves energy between samples but
// never changes the total.
TEST(Engine, PulseAndFootprintSpreadTheEcho)
{
  sonar::SonarConfig c = CleanEcho();
  const double dr = c.SampleSpacing();
  const std::size_t beam = c.beams / 2;
  auto ping = [&](const sonar::RayHit &_h) {
    const sonar::Engine engine(c, HarborWater());
    return engine.Ping({_h}, 0);
  };
  auto span = [&](const sonar::BeamIntensity &_b) {
    const auto [first, last] = Extent(_b, beam);
    return static_cast<double>(last - first + 1) * dr;
  };

  // A 7.5 mm pulse is under a sample: one or two samples.
  const auto shortPulse = ping(OneHit(5.0));
  EXPECT_LE(span(shortPulse), 2.0 * dr);

  // A 75 mm pulse.
  c.pulseLength = 1e-4;
  const auto longPulse = ping(OneHit(5.0));
  EXPECT_GE(span(longPulse), 0.075);
  EXPECT_LE(span(longPulse), 0.075 + 2.0 * dr);
  EXPECT_NEAR(Total(longPulse) / Total(shortPulse), 1.0, 1e-9);

  // A 0.3 m footprint from the sampler, wider than the pulse.
  sonar::RayHit h = OneHit(5.0);
  h.footprint = 0.3;
  const auto wide = ping(h);
  EXPECT_GE(span(wide), 0.3);
  EXPECT_LE(span(wide), 0.3 + 2.0 * dr);
  EXPECT_NEAR(Total(wide) / Total(shortPulse), 1.0, 1e-9);

  // No footprint given, near grazing: estimated, and capped at 8 samples.
  c.pulseLength = 1e-5;
  h = OneHit(5.0);
  h.incidence = 85.0 * kDeg;
  h.ray.spacing = 0.05;
  const auto capped = ping(h);
  EXPECT_GE(span(capped), 8.0 * dr);
  EXPECT_LE(span(capped), 10.0 * dr);
}

//////////////////////////////////////////////////
// One ray off to the side: across every beam, the echo follows the
// horizontal pattern down to its -50 dB floor, sidelobes included, and is
// zero beyond it.
TEST(Engine, BeamProfileFollowsThePatternToTheFloor)
{
  const sonar::SonarConfig c = CleanEcho();
  // A ray of the fan, so its azimuth is the center of one of the engine's
  // azimuth columns.
  const auto rays = sonar::RayFan(c);
  double az = rays[0].azimuth;
  for (const auto &r : rays)
    if (std::abs(r.azimuth - 0.2) < std::abs(az - 0.2))
      az = r.azimuth;
  sonar::RayHit h = OneHit(5.0);
  h.ray.azimuth = az;
  const sonar::Engine engine(c, HarborWater());
  const auto b = engine.Ping({h}, 0);

  std::vector<double> profile(c.beams, 0.0);
  for (std::size_t j = 0; j < c.beams; ++j)
    for (std::size_t k = 0; k < c.samples; ++k)
      profile[j] += b.At(j, k);
  std::size_t j0 = 0;
  for (std::size_t j = 0; j < c.beams; ++j)
    if (profile[j] > profile[j0])
      j0 = j;
  const double w0 = sonar::HorizontalPattern(c, az - c.BeamAzimuth(j0));
  std::size_t sidelobes = 0;
  for (std::size_t j = 0; j < c.beams; ++j)
  {
    const double w = sonar::HorizontalPattern(c, az - c.BeamAzimuth(j));
    if (w > 2e-5)
    {
      EXPECT_NEAR(profile[j] / profile[j0], w / w0, 1e-9 * w / w0)
        << "beam " << j;
      if (w < 1e-3)
        ++sidelobes;
    }
    else if (w < 0.5e-5)
    {
      EXPECT_EQ(profile[j], 0.0) << "beam " << j;
    }
  }
  EXPECT_GT(sidelobes, 10u);
}

//////////////////////////////////////////////////
// Without speckle the noise is still drawn: exponential, mean and standard
// deviation both the noise floor.
TEST(Readout, NoiseWithoutSpeckleIsExponential)
{
  sonar::SonarConfig c = CleanEcho();
  c.tvg = sonar::Tvg::kNone;
  c.gainDb = 0.0;
  c.noiseFloorDb = -100.0;
  const auto l = sonar::ReadLevels(Flat(c, 0.0), c, 0.0, 0);
  const auto [mean, std] = MeanStd(Powers(l, -100.0));
  EXPECT_NEAR(mean, 1.0, 0.02);
  EXPECT_NEAR(std, 1.0, 0.03);
}

//////////////////////////////////////////////////
// The time-varied gain by hand: the spreading law in dB per decade, plus
// two-way absorption when asked for.
TEST(Readout, TimeVariedGainByHand)
{
  sonar::SonarConfig c = CleanEcho();
  c.tvg = sonar::Tvg::kNone;
  EXPECT_DOUBLE_EQ(sonar::TimeVariedGain(c, 0.2, 10.0), 0.0);
  c.tvg = sonar::Tvg::kSpreading;
  EXPECT_NEAR(sonar::TimeVariedGain(c, 0.2, 10.0), 30.0, 1e-12);
  c.tvg = sonar::Tvg::kSpreadingAbsorption;
  EXPECT_NEAR(sonar::TimeVariedGain(c, 0.2, 10.0), 34.0, 1e-12);
  c.tvgSpreading = 40.0;
  EXPECT_NEAR(sonar::TimeVariedGain(c, 0.05, 100.0), 90.0, 1e-12);
}

//////////////////////////////////////////////////
// A ray's echo falls as 20 log R plus two-way absorption, so a time-varied
// gain of 20 log R + 2 alpha R brings it to the same published level at
// any range. One-way absorption in the gain would leave about 1 dB between
// 3 m and 8 m at 700 kHz.
TEST(Readout, TvgFlattensOneRay)
{
  sonar::SonarConfig c = CleanEcho();
  c.tvg = sonar::Tvg::kSpreadingAbsorption;
  c.tvgSpreading = 20.0;
  c.gainDb = 0.0;
  const sonar::Engine engine(c, HarborWater());
  ASSERT_GT(engine.Absorption() * 5.0, 1.0);
  auto published = [&](double _range) {
    const auto l = sonar::ReadLevels(engine.Ping({OneHit(_range)}, 0), c,
      engine.Absorption(), 0);
    double sum = 0.0;
    for (double d : l.db)
      sum += std::pow(10.0, d / 10.0);
    return 10.0 * std::log10(sum);
  };
  EXPECT_NEAR(published(3.0), published(8.0), 0.1);
}

//////////////////////////////////////////////////
// Automatic gain by hand. A uniform frame: the median goes to -24 dB.
// Unlit cells do not count. A sparse frame, where more than 0.1 % of the
// lit cells are a strong target: the cap puts the target at full scale.
// With a tracker, the gain moves part of the way per ping.
TEST(Readout, AutomaticGainByHand)
{
  sonar::SonarConfig c = CleanEcho();
  c.tvg = sonar::Tvg::kNone;
  c.gainDb.reset();
  auto flat = Flat(c, 1e-6);
  EXPECT_NEAR(sonar::ReadLevels(flat, c, 0.0, 0).gainDb, 36.0, 1e-9);

  auto half = flat;
  for (std::size_t i = 0; i < half.data.size(); i += 2)
    half.data[i] = 0.0;
  EXPECT_NEAR(sonar::ReadLevels(half, c, 0.0, 0).gainDb, 36.0, 1e-9);

  auto sparse = flat;
  for (std::size_t i = 0; i < sparse.data.size(); i += 100)
    sparse.data[i] = 1e-2;
  EXPECT_NEAR(sonar::ReadLevels(sparse, c, 0.0, 0).gainDb, 20.0, 1e-9);

  // Speckled, the median of the echoes is that of an exponential, ln 2:
  // 1.6 dB below the mean. Unlit cells still do not count.
  sonar::SonarConfig speckled = c;
  speckled.speckle = true;
  const double lnTwoDb = 10.0 * std::log10(std::log(2.0));
  EXPECT_NEAR(sonar::ReadLevels(flat, speckled, 0.0, 0).gainDb,
    36.0 - lnTwoDb, 0.05);
  EXPECT_NEAR(sonar::ReadLevels(half, speckled, 0.0, 0).gainDb,
    36.0 - lnTwoDb, 0.05);

  c.gainTimeConstant = 1.0;
  sonar::GainTracker t;
  EXPECT_NEAR(sonar::ReadLevels(flat, c, 0.0, 0, &t, 0.1).gainDb, 36.0, 1e-9);
  EXPECT_NEAR(sonar::ReadLevels(Flat(c, 1e-4), c, 0.0, 1, &t, 0.1).gainDb,
    36.0 - 20.0 * (1.0 - std::exp(-0.1)), 1e-9);
}

namespace
{
//////////////////////////////////////////////////
/// \brief Profiler levels by hand: noise everywhere, no gain, and in each
/// beam one sample _snr dB above the noise at the given range.
/// \param[in] _config The sonar.
/// \param[in] _range The echo's range in each beam, m.
/// \param[in] _snr The echo's level over the noise in each beam, dB.
/// \return The levels.
sonar::Levels ProfilerLevels(const sonar::SonarConfig &_config,
  const std::vector<double> &_range, const std::vector<double> &_snr)
{
  sonar::Levels l;
  l.beams = _range.size();
  l.samples = _config.samples;
  for (std::size_t k = 0; k < l.samples; ++k)
    l.ranges.push_back(_config.SampleRange(k));
  for (std::size_t j = 0; j < l.beams; ++j)
    l.azimuths.push_back(-0.5 + static_cast<double>(j) /
      static_cast<double>(l.beams));
  l.tvgDb.assign(l.samples, 0.0);
  l.gainDb = 0.0;
  l.db.assign(l.beams * l.samples, _config.noiseFloorDb);
  const double dr = _config.SampleSpacing();
  for (std::size_t j = 0; j < l.beams; ++j)
  {
    const auto k = static_cast<std::size_t>(
      (_range[j] - _config.rangeMin) / dr);
    l.db[j * l.samples + k] = _config.noiseFloorDb + _snr[j];
  }
  return l;
}

//////////////////////////////////////////////////
std::pair<double, double> RangeStats(const sonar::SonarConfig &_config,
  double _snr)
{
  const std::size_t beams = 100;
  const sonar::Levels l = ProfilerLevels(_config,
    std::vector<double>(beams, 10.0), std::vector<double>(beams, _snr));
  const std::size_t k = static_cast<std::size_t>(
    (10.0 - _config.rangeMin) / _config.SampleSpacing());
  std::vector<double> err;
  for (std::uint64_t ping = 0; ping < 50; ++ping)
    for (const auto &s : sonar::ReadProfiling(l, _config, 1500.0, ping))
      err.push_back(s.range - l.ranges[k]);
  double mean = 0.0;
  for (double e : err)
    mean += e / static_cast<double>(err.size());
  double var = 0.0;
  for (double e : err)
    var += (e - mean) * (e - mean) / static_cast<double>(err.size());
  return {mean, std::sqrt(var)};
}
}  // namespace

//////////////////////////////////////////////////
// The detection threshold is a level above the noise: 13 dB does not pass
// 15, 16 does.
TEST(Readout, ProfilingThreshold)
{
  sonar::SonarConfig c = sonar::SurveyorProfiling();
  c.detection = sonar::ProfilingDetection::kBeamformed;
  ASSERT_DOUBLE_EQ(c.detectionThresholdDb, 15.0);
  const std::vector<double> ranges(40, 10.0);
  EXPECT_TRUE(sonar::ReadProfiling(ProfilerLevels(c, ranges,
    std::vector<double>(40, 13.0)), c, 1500.0, 0).empty());
  EXPECT_EQ(sonar::ReadProfiling(ProfilerLevels(c, ranges,
    std::vector<double>(40, 16.0)), c, 1500.0, 0).size(), 40u);
}

//////////////////////////////////////////////////
// Range noise: unbiased, rangeNoise at high signal-to-noise and twice that
// at the threshold, sigma = rangeNoise (1 + 10^(-margin / 20)); half a
// sample by default.
TEST(Readout, ProfilingRangeNoise)
{
  sonar::SonarConfig c = sonar::SurveyorProfiling();
  c.detection = sonar::ProfilingDetection::kBeamformed;
  c.rangeNoise = 0.02;
  const auto [mean, high] = RangeStats(c, 60.0);
  EXPECT_NEAR(mean, 0.0, 0.002);
  EXPECT_NEAR(high, 0.02 * (1.0 + std::pow(10.0, -45.0 / 20.0)), 0.001);
  const auto [mean2, low] = RangeStats(c, 15.5);
  (void)mean2;
  EXPECT_NEAR(low, 0.02 * (1.0 + std::pow(10.0, -0.5 / 20.0)), 0.002);

  c.rangeNoise.reset();
  const auto [mean3, half] = RangeStats(c, 60.0);
  (void)mean3;
  EXPECT_NEAR(half, 0.5 * c.SampleSpacing(), 0.05 * c.SampleSpacing());
}

//////////////////////////////////////////////////
// Beamformed soundings far from the median of their neighbors are flagged,
// at the edge of the fan as in the middle, and no others.
TEST(Readout, OutlierFilterFlagsTheOddSounding)
{
  sonar::SonarConfig c = sonar::SurveyorProfiling();
  c.detection = sonar::ProfilingDetection::kBeamformed;
  c.rangeNoise = 1e-6;
  std::vector<double> ranges(20, 10.0);
  ranges[0] = 11.0;
  ranges[10] = 11.0;
  const auto s = sonar::ReadProfiling(ProfilerLevels(c, ranges,
    std::vector<double>(20, 40.0)), c, 1500.0, 0);
  ASSERT_EQ(s.size(), 20u);
  for (std::size_t j = 0; j < s.size(); ++j)
  {
    EXPECT_EQ(s[j].flag, (j == 0 || j == 10) ? sonar::SoundingFlag::kBadFilter :
      sonar::SoundingFlag::kOk) << "beam " << j;
  }
}

//////////////////////////////////////////////////
// Angle of arrival: each sector reports its strongest echo, at that beam's
// angle plus angle noise of the configured standard deviation.
TEST(Readout, AngleOfArrivalTakesEachSectorsStrongest)
{
  sonar::SonarConfig c = sonar::SurveyorProfiling();
  c.detection = sonar::ProfilingDetection::kAngleOfArrival;
  c.maxSoundings = 2;
  c.angleNoise = 0.0;
  std::vector<double> snr(20, 0.0);
  snr[3] = 30.0;
  snr[7] = 40.0;
  snr[15] = 20.0;
  const auto l = ProfilerLevels(c, std::vector<double>(20, 10.0), snr);
  const auto s = sonar::ReadProfiling(l, c, 1500.0, 0);
  ASSERT_EQ(s.size(), 2u);
  EXPECT_DOUBLE_EQ(s[0].angle, l.azimuths[7]);
  EXPECT_DOUBLE_EQ(s[1].angle, l.azimuths[15]);

  c.angleNoise = 0.01;
  std::vector<double> err;
  for (std::uint64_t ping = 0; ping < 2000; ++ping)
    for (const auto &x : sonar::ReadProfiling(l, c, 1500.0, ping))
      err.push_back(x.angle - (x.angle < 0.0 ? l.azimuths[7] : l.azimuths[15]));
  double var = 0.0;
  for (double e : err)
    var += e * e / static_cast<double>(err.size());
  EXPECT_NEAR(std::sqrt(var), 0.01, 0.0005);
}

//////////////////////////////////////////////////
// Every draw depends on the seed, the ping and the stream; the normal draw
// is standard.
TEST(Random, SeedPingAndStreamAllMatter)
{
  const double base = sonar::Random(1, 0, 0).Uniform();
  EXPECT_NE(sonar::Random(2, 0, 0).Uniform(), base);
  EXPECT_NE(sonar::Random(1, 1, 0).Uniform(), base);
  EXPECT_NE(sonar::Random(1, 0, 1).Uniform(), base);
  EXPECT_EQ(sonar::Random(1, 0, 0).Uniform(), base);

  sonar::Random rng(5, 6, 7);
  const int n = 200000;
  double sum = 0.0, sum2 = 0.0;
  for (int i = 0; i < n; ++i)
  {
    const double x = rng.Normal();
    sum += x;
    sum2 += x * x;
  }
  EXPECT_NEAR(sum / n, 0.0, 0.01);
  EXPECT_NEAR(std::sqrt(sum2 / n), 1.0, 0.01);
}

//////////////////////////////////////////////////
// The seed changes the speckle of a frame.
TEST(Readout, SeedChangesTheSpeckle)
{
  sonar::SonarConfig c = CleanEcho();
  c.speckle = true;
  auto frame = [&](std::uint64_t _seed) {
    c.seed = _seed;
    return sonar::ReadLevels(Flat(c, 1e-6), c, 0.0, 3).db;
  };
  EXPECT_EQ(frame(1), frame(1));
  EXPECT_NE(frame(1), frame(2));
}

//////////////////////////////////////////////////
// Francois and Garrison, worked by hand from the paper's equations, where
// one term dominates. In fresh water only pure-water viscosity is left,
// A3 P3 f^2; the paper fits A3 with one polynomial up to 20 C and another
// above, which meet at 20 C, and it falls with temperature. At the worlds'
// harbor water (29 C, 30 PSU) the totals at the three presets'
// frequencies. At 1 kHz, boric acid.
TEST(Water, AbsorptionTermByTerm)
{
  sonar::Water w;
  w.salinity = 0.0;
  // dB/km at 1 MHz, fresh water: 1e6 kHz^2 times A3.
  w.temperature = 10.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(1e6, w) * 1000.0, 310.8, 0.5);
  w.temperature = 29.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(1e6, w) * 1000.0, 170.15, 0.3);
  // Proportional to f^2.
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(2e6, w) /
    sonar::AbsorptionDbPerMeter(1e6, w), 4.0, 1e-9);
  // The pressure correction P3 at 1000 m: 1 - 3.83e-2 + 4.9e-4.
  sonar::Water deep = w;
  deep.depth = 1000.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(1e6, deep) /
    sonar::AbsorptionDbPerMeter(1e6, w), 0.96219, 1e-5);
  // The two polynomials meet at 20 C.
  sonar::Water below = w, above = w;
  below.temperature = 20.0 - 1e-6;
  above.temperature = 20.0 + 1e-6;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(1e6, above) /
    sonar::AbsorptionDbPerMeter(1e6, below), 1.0, 1e-3);
  double last = std::numeric_limits<double>::infinity();
  for (double t = 0.0; t <= 35.0; t += 1.0)
  {
    w.temperature = t;
    const double a = sonar::AbsorptionDbPerMeter(1e6, w);
    EXPECT_LT(a, last) << t << " C";
    last = a;
  }

  // The harbor: magnesium sulfate and pure water, dB/km.
  const sonar::Water harbor = HarborWater();
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(240e3, harbor) * 1000.0, 95.76, 0.3);
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(450e3, harbor) * 1000.0, 159.66, 0.5);
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(700e3, harbor) * 1000.0, 223.51, 0.7);
  sonar::Water harborDeep = harbor;
  harborDeep.depth = 1000.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(700e3, harborDeep) * 1000.0, 200.75,
    0.6);

  // 1 kHz, 10 C, 35 PSU: boric acid 0.0515 of 0.0601 dB/km.
  sonar::Water sea;
  sea.temperature = 10.0;
  sea.salinity = 35.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(1e3, sea) * 1000.0, 0.06013, 0.0005);
  // 300 Hz, 30 PSU, below the boric relaxation (1.03 kHz here), so it
  // scales with its frequency: boric acid 0.00836 of 0.00902 dB/km.
  sea.salinity = 30.0;
  EXPECT_NEAR(sonar::AbsorptionDbPerMeter(300.0, sea) * 1000.0, 0.009021,
    0.00015);
}

//////////////////////////////////////////////////
// The fan reaches elevationSpan half-widths of the vertical beam, with
// elevationRays rays per two half-widths; each ray stands for its share of
// solid angle and is spaced by the coarser of its two steps.
TEST(Rays, FanGeometry)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const auto rays = sonar::RayFan(c);
  const std::size_t ne = sonar::RayFanElevationCount(c);
  EXPECT_EQ(ne, 96u);
  const double elStep = rays[1].elevation - rays[0].elevation;
  const double azStep = rays[ne].azimuth - rays[0].azimuth;
  EXPECT_NEAR(rays[ne - 1].elevation + 0.5 * elStep,
    0.5 * c.verticalFov * c.elevationSpan, 1e-12);
  EXPECT_NEAR(rays[0].elevation - 0.5 * elStep,
    -0.5 * c.verticalFov * c.elevationSpan, 1e-12);
  for (std::size_t n : {std::size_t{0}, ne / 2, ne - 1})
  {
    EXPECT_NEAR(rays[n].solidAngle,
      azStep * elStep * std::cos(rays[n].elevation), 1e-15);
    EXPECT_NEAR(rays[n].spacing, std::max(azStep, elStep), 1e-15);
  }
}

//////////////////////////////////////////////////
// Neighbors are one surface when they share a label and their ranges are
// within what a surface at kMaxSlant gives; the footprint is half the
// range between the two neighbors on the surface, the larger over the two
// directions of the grid.
TEST(Rays, SameSurfaceAndFootprint)
{
  sonar::RayHit a = OneHit(5.0);
  sonar::RayHit b = OneHit(5.0);
  EXPECT_TRUE(sonar::SameSurface(a, b));
  b.label = 2;
  EXPECT_FALSE(sonar::SameSurface(a, b));
  b.label = 1;
  const double bound = 5.0 * a.ray.spacing * sonar::kMaxSlant;
  b.range = 5.0 + 0.9 * bound;
  EXPECT_TRUE(sonar::SameSurface(a, b));
  b.range = 5.0 + 1.1 * bound;
  EXPECT_FALSE(sonar::SameSurface(a, b));

  // Three columns of three, range rising 1 cm a ray in elevation, the same
  // across azimuth.
  std::vector<sonar::RayHit> hits;
  for (int i = 0; i < 3; ++i)
    for (int k = 0; k < 3; ++k)
      hits.push_back(OneHit(5.0 + 0.01 * k));
  sonar::AssignFootprints(hits, 3);
  EXPECT_NEAR(hits[4].footprint, 0.01, 1e-12);
  EXPECT_NEAR(hits[3].footprint, 0.01, 1e-12);
}

//////////////////////////////////////////////////
// Each cell takes the label of its strongest contribution: two targets at
// one range, three beams apart, each label wins in its own beams, whichever
// way the beams are gathered.
TEST(Engine, LabelOfTheStrongestContribution)
{
  const sonar::SonarConfig c = CleanEcho();
  const auto rays = sonar::RayFan(c);
  auto nearest = [&](double _az) {
    double best = rays[0].azimuth;
    for (const auto &r : rays)
      if (std::abs(r.azimuth - _az) < std::abs(best - _az))
        best = r.azimuth;
    return best;
  };
  const std::size_t ja = 120, jb = 123;
  sonar::RayHit a = OneHit(5.0);
  a.ray.azimuth = nearest(c.BeamAzimuth(ja));
  a.label = 1;
  sonar::RayHit b = OneHit(5.0);
  b.ray.azimuth = nearest(c.BeamAzimuth(jb));
  b.label = 2;
  const sonar::Engine engine(c, HarborWater());
  for (const auto &hits : {std::vector<sonar::RayHit>{a, b},
                           std::vector<sonar::RayHit>{b, a}})
  {
    const auto out = engine.Ping(hits, 0);
    for (std::size_t j : {ja - 1, ja, ja + 1, jb - 1, jb, jb + 1})
    {
      const auto [first, last] = Extent(out, j);
      ASSERT_LE(first, last);
      for (std::size_t k = first; k <= last; ++k)
      {
        if (out.At(j, k) > 0.0)
          EXPECT_EQ(out.labels[j * out.samples + k], j < ja + 2 ? 1u : 2u)
            << "beam " << j;
      }
    }
  }
}
