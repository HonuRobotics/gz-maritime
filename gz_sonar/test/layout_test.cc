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
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "gz/maritime/sonar/CameraSampler.hh"
#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Rays.hh"

namespace sonar = gz::maritime::sonar;

namespace
{
constexpr double kDeg = M_PI / 180.0;

//////////////////////////////////////////////////
/// \brief The three exemplars.
std::vector<sonar::SonarConfig> Presets()
{
  return {sonar::EchoImaging(), sonar::OmniscanSidescan(),
    sonar::SurveyorProfiling()};
}

//////////////////////////////////////////////////
/// \brief Expect a layout inside every limit, with every ray of the fan's
/// outline inside its camera's image.
/// \param[in] _c The sonar.
/// \param[in] _l Its layout.
/// \param[in] _what What is being tried, for the failure message.
void ExpectSane(const sonar::SonarConfig &_c, const sonar::CameraLayout &_l,
  const std::string &_what)
{
  ASSERT_GE(_l.yaws.size(), 1u) << _what;
  EXPECT_LE(_l.yaws.size(), sonar::kMaxCameras) << _what;
  EXPECT_GE(_l.width, 4u) << _what;
  EXPECT_GE(_l.height, 4u) << _what;
  EXPECT_LE(_l.width, sonar::kMaxCameraSide) << _what;
  EXPECT_LE(_l.height, sonar::kMaxCameraSide) << _what;
  EXPECT_LE(static_cast<double>(_l.yaws.size()) * _l.width * _l.height,
    sonar::kMaxCameraPixels) << _what;
  EXPECT_TRUE(std::isfinite(_l.focal) && _l.focal > 0.0) << _what;
  EXPECT_TRUE(_l.hfov > 0.0 && _l.hfov < M_PI) << _what;
  EXPECT_GT(_l.farClip, _l.nearClip) << _what;
  EXPECT_TRUE(std::isfinite(static_cast<float>(_l.farClip))) << _what;

  const sonar::RayGrid g = sonar::RayFanGrid(_c);
  EXPECT_LE(g.columns, sonar::kMaxFanColumns) << _what;
  EXPECT_LE(g.columns * static_cast<double>(g.rows), sonar::kMaxFanRays)
    << _what;
  EXPECT_LE(g.columns * static_cast<double>(_c.samples), sonar::kMaxFanCells)
    << _what;
  // The four corners of the fan and its middle each fall inside the image
  // of some camera.
  const auto columns = static_cast<std::size_t>(g.columns);
  for (const std::size_t i : {std::size_t{0}, columns / 2, columns - 1})
  {
    for (const std::size_t k : {std::size_t{0}, g.rows / 2, g.rows - 1})
    {
      sonar::Ray r;
      r.azimuth = g.Azimuth(i);
      r.elevation = g.Elevation(k);
      bool inside = false;
      for (const double yaw : _l.yaws)
      {
        const double a = r.azimuth - yaw;
        const double x = std::cos(r.elevation) * std::cos(a);
        const double y = std::cos(r.elevation) * std::sin(a);
        const double z = std::sin(r.elevation);
        if (x <= 0.0)
          continue;
        const double u = 0.5 * _l.width - _l.focal * y / x;
        const double v = 0.5 * _l.height - _l.focal * z / x;
        inside = inside || (u >= 1.0 && u <= _l.width - 1.0 && v >= 1.0 &&
          v <= _l.height - 1.0);
      }
      EXPECT_TRUE(inside) << _what << ": ray " << i << ", " << k;
    }
  }
}
}  // namespace

//////////////////////////////////////////////////
// The exemplars' cameras are what they were before the layout was planned
// ahead of the rays, and the sampler takes the plan as it is.
TEST(CameraLayout, ExemplarsKeepTheirCameras)
{
  struct Expected { std::size_t cameras; unsigned int width, height; };
  const Expected expected[] = {{2, 604, 675}, {1, 45, 2200}, {2, 356, 77}};
  const auto presets = Presets();
  for (std::size_t n = 0; n < presets.size(); ++n)
  {
    std::string error;
    const auto plan = sonar::PlanCameraLayout(presets[n], error);
    ASSERT_TRUE(plan) << error;
    EXPECT_EQ(plan->yaws.size(), expected[n].cameras);
    EXPECT_EQ(plan->width, expected[n].width);
    EXPECT_EQ(plan->height, expected[n].height);
    ExpectSane(presets[n], *plan, "preset " + std::to_string(n));

    const sonar::CameraSampler sampler(presets[n]);
    const auto &l = sampler.Layout();
    EXPECT_EQ(l.yaws, plan->yaws);
    EXPECT_EQ(l.width, plan->width);
    EXPECT_EQ(l.height, plan->height);
    EXPECT_EQ(l.focal, plan->focal);
    EXPECT_EQ(l.hfov, plan->hfov);
    const sonar::RayGrid g = sonar::RayFanGrid(presets[n]);
    EXPECT_EQ(static_cast<double>(sampler.Rays().size()),
      g.columns * static_cast<double>(g.rows));
  }
}

//////////////////////////////////////////////////
// One elevation row has no step between rows. The camera is then a few
// pixels high, sized by the azimuth step, not 4 billion; and the sampler
// reads the row's ranges from it.
TEST(CameraLayout, OneElevationRow)
{
  for (sonar::SonarConfig c : Presets())
  {
    const unsigned int width = sonar::CameraSampler(c).Layout().width;
    c.elevationRays = 1;
    c.elevationSpan = 2.0;
    ASSERT_EQ(sonar::RayFanElevationCount(c), 1u);
    std::string error;
    const auto plan = sonar::PlanCameraLayout(c, error);
    ASSERT_TRUE(plan) << error;
    ExpectSane(c, *plan, "one row");
    EXPECT_EQ(plan->width, width);
    EXPECT_LE(plan->height, 8u);

    // A wall square to each camera, 5 m off: every ray reads it.
    const sonar::CameraSampler sampler(c);
    const auto &l = sampler.Layout();
    const std::vector<float> image(l.width * l.height, 5.0f);
    const std::vector<const float *> depth(l.yaws.size(), image.data());
    const auto hits = sampler.Sample(depth, {}, 1, {});
    ASSERT_EQ(hits.size(), sampler.Rays().size());
    for (const auto &h : hits)
    {
      ASSERT_TRUE(h.hit);
      // Depth 5 m along the camera's axis, at most half a camera off it.
      EXPECT_GE(h.range, 5.0 - 1e-6);
      EXPECT_LE(h.range, 5.0 / std::cos(0.5 * l.hfov) + 1e-6);
    }
  }
  // Two and three rows across a wide fan: the margin of one ray step must
  // not carry the camera past what a pinhole sees.
  for (const std::size_t rows : {2u, 3u, 5u})
  {
    sonar::SonarConfig c = sonar::EchoImaging();
    c.verticalFov = 80.0 * kDeg;
    c.elevationSpan = 2.0;
    c.elevationRays = rows;
    std::string error;
    const auto plan = sonar::PlanCameraLayout(c, error);
    if (plan)
      ExpectSane(c, *plan, std::to_string(rows) + " rows");
    else
      EXPECT_FALSE(error.empty());
  }
}

//////////////////////////////////////////////////
// A fan that cannot be built is refused with a reason before a ray is
// made, and a sampler asked for it anyway is empty rather than built.
TEST(CameraLayout, RefusesWhatCannotBeBuilt)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<sonar::SonarConfig> bad;
  auto with = [&](auto _set) {
    for (sonar::SonarConfig c : Presets())
    {
      _set(c);
      bad.push_back(c);
    }
  };
  // 1e11 azimuth columns: building them is what hung the server.
  with([](sonar::SonarConfig &_c) { _c.horizontalFov = 1e-9; });
  // Degrees for radians: nineteen turns.
  with([](sonar::SonarConfig &_c) { _c.horizontalFov = 120.0; });
  with([](sonar::SonarConfig &_c) { _c.horizontalFov = 0.0; });
  with([](sonar::SonarConfig &_c) { _c.horizontalFov = -1.0; });
  with([&](sonar::SonarConfig &_c) { _c.horizontalFov = nan; });
  with([&](sonar::SonarConfig &_c) { _c.horizontalFov = inf; });
  with([](sonar::SonarConfig &_c) { _c.verticalFov = 0.0; });
  with([&](sonar::SonarConfig &_c) { _c.verticalFov = nan; });
  with([](sonar::SonarConfig &_c) { _c.verticalFov = 3.0; });
  with([](sonar::SonarConfig &_c) { _c.verticalFov = 50.0; });
  with([](sonar::SonarConfig &_c) { _c.beamWidth = 0.0; });
  with([](sonar::SonarConfig &_c) { _c.beamWidth = 1e-12; });
  with([](sonar::SonarConfig &_c) { _c.beamWidth = 1.6; });
  with([&](sonar::SonarConfig &_c) { _c.beamWidth = nan; });
  with([](sonar::SonarConfig &_c) { _c.elevationSpan = 0.0; });
  with([](sonar::SonarConfig &_c) { _c.elevationSpan = -2.0; });
  with([](sonar::SonarConfig &_c) { _c.elevationSpan = 1e300; });
  with([&](sonar::SonarConfig &_c) { _c.elevationSpan = nan; });
  with([](sonar::SonarConfig &_c) { _c.beams = 0; });
  with([](sonar::SonarConfig &_c) { _c.raysPerBeam = 0; });
  with([](sonar::SonarConfig &_c) { _c.elevationRays = 0; });
  with([](sonar::SonarConfig &_c) { _c.samples = 0; });
  with([](sonar::SonarConfig &_c) {
    _c.elevationRays = std::numeric_limits<std::size_t>::max(); });
  with([](sonar::SonarConfig &_c) { _c.raysPerBeam = 100000; });
  with([](sonar::SonarConfig &_c) { _c.beams = 4096; _c.raysPerBeam = 64; });
  with([](sonar::SonarConfig &_c) { _c.samples = 100000000; });
  with([](sonar::SonarConfig &_c) { _c.rangeMax = 1e300; });
  with([&](sonar::SonarConfig &_c) { _c.rangeMax = nan; });
  with([](sonar::SonarConfig &_c) { _c.rangeMax = 0.01; });

  for (const sonar::SonarConfig &c : bad)
  {
    std::string error;
    EXPECT_FALSE(sonar::PlanCameraLayout(c, error));
    EXPECT_FALSE(error.empty());
    const sonar::CameraSampler sampler(c);
    EXPECT_TRUE(sampler.Layout().yaws.empty()) << error;
    EXPECT_TRUE(sampler.Rays().empty()) << error;
    EXPECT_TRUE(sampler.Sample({}, {}, 1, {}).empty()) << error;
  }
  // The reasons name what was asked for.
  std::string error;
  sonar::SonarConfig c = sonar::EchoImaging();
  c.horizontalFov = 120.0;
  EXPECT_FALSE(sonar::PlanCameraLayout(c, error));
  EXPECT_NE(error.find("120"), std::string::npos) << error;
  EXPECT_NE(error.find("radians"), std::string::npos) << error;
  c.horizontalFov = 1e-9;
  EXPECT_FALSE(sonar::PlanCameraLayout(c, error));
  EXPECT_NE(error.find("azimuth columns"), std::string::npos) << error;
}

//////////////////////////////////////////////////
// Every parameter that shapes the fan, scaled from a thousandth to a
// thousand times each exemplar's value, alone and in pairs: each
// combination is either refused with a reason or gives cameras and a fan
// inside the limits, and nothing is built to find out.
TEST(CameraLayout, SweepIsRefusedOrSane)
{
  const double scales[] = {1e-3, 1e-2, 0.1, 0.3, 0.5, 2.0, 3.0, 10.0, 100.0,
    1e3};
  using Setter = void (*)(sonar::SonarConfig &, double);
  static const auto scaled = [](std::size_t _n, double _s) {
    return static_cast<std::size_t>(std::max(1.0, std::round(
      static_cast<double>(_n) * _s)));
  };
  const Setter setters[] = {
    [](sonar::SonarConfig &_c, double _s) { _c.horizontalFov *= _s; },
    [](sonar::SonarConfig &_c, double _s) { _c.verticalFov *= _s; },
    [](sonar::SonarConfig &_c, double _s) { _c.beamWidth *= _s; },
    [](sonar::SonarConfig &_c, double _s) { _c.elevationSpan *= _s; },
    [](sonar::SonarConfig &_c, double _s) { _c.rangeMax *= _s; },
    [](sonar::SonarConfig &_c, double _s) {
      _c.beams = scaled(_c.beams, _s); },
    [](sonar::SonarConfig &_c, double _s) {
      _c.raysPerBeam = scaled(_c.raysPerBeam, _s); },
    [](sonar::SonarConfig &_c, double _s) {
      _c.elevationRays = scaled(_c.elevationRays, _s); },
    [](sonar::SonarConfig &_c, double _s) {
      _c.samples = scaled(_c.samples, _s); },
  };
  std::size_t accepted = 0, refused = 0;
  auto check = [&](const sonar::SonarConfig &_c, const std::string &_what) {
    std::string error;
    const auto plan = sonar::PlanCameraLayout(_c, error);
    if (!plan)
    {
      ++refused;
      EXPECT_FALSE(error.empty()) << _what;
      return;
    }
    ++accepted;
    ExpectSane(_c, *plan, _what);
  };
  const auto presets = Presets();
  const std::size_t n = sizeof(setters) / sizeof(setters[0]);
  for (std::size_t p = 0; p < presets.size(); ++p)
  {
    for (std::size_t a = 0; a < n; ++a)
    {
      for (const double sa : scales)
      {
        sonar::SonarConfig one = presets[p];
        setters[a](one, sa);
        const std::string what = "preset " + std::to_string(p) + ", " +
          std::to_string(a) + " x " + std::to_string(sa);
        check(one, what);
        for (std::size_t b = a + 1; b < n; ++b)
        {
          for (const double sb : scales)
          {
            sonar::SonarConfig two = one;
            setters[b](two, sb);
            check(two, what + ", " + std::to_string(b) + " x " +
              std::to_string(sb));
          }
        }
      }
    }
  }
  // Both outcomes happen, or the sweep tests nothing.
  EXPECT_GT(accepted, 1000u);
  EXPECT_GT(refused, 1000u);
  std::printf("sweep: %zu accepted, %zu refused\n", accepted, refused);
}

//////////////////////////////////////////////////
// What is accepted near the limits is really built: the sampler's rays and
// cameras are the plan's, for the largest fans the sweep lets through.
TEST(CameraLayout, LargeFansBuild)
{
  std::vector<sonar::SonarConfig> large;
  sonar::SonarConfig c = sonar::EchoImaging();
  c.beams = 2048;
  c.elevationRays = 32;
  large.push_back(c);
  c = sonar::OmniscanSidescan();
  c.raysPerBeam = 24;
  large.push_back(c);
  c = sonar::SurveyorProfiling();
  c.horizontalFov = 6.0;
  c.beams = 1024;
  large.push_back(c);
  for (const sonar::SonarConfig &config : large)
  {
    std::string error;
    const auto plan = sonar::PlanCameraLayout(config, error);
    ASSERT_TRUE(plan) << error;
    ExpectSane(config, *plan, "large");
    const sonar::CameraSampler sampler(config);
    const sonar::RayGrid g = sonar::RayFanGrid(config);
    EXPECT_EQ(static_cast<double>(sampler.Rays().size()),
      g.columns * static_cast<double>(g.rows));
    EXPECT_EQ(sampler.Layout().yaws.size(), plan->yaws.size());
    EXPECT_EQ(sampler.Layout().height, plan->height);
  }
}
