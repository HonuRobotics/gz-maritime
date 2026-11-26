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
#include <cstdio>
#include <cstdint>
#include <limits>
#include <vector>

#include <gz/math/Pose3.hh>

#include "gz/maritime/sonar/CameraSampler.hh"
#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Rays.hh"
#include "gz/maritime/sonar/SyntheticScene.hh"

using gz::math::Pose3d;
namespace sonar = gz::maritime::sonar;

namespace
{
constexpr double kDeg = M_PI / 180.0;

sonar::AcousticProperties Lambert(double _db)
{
  sonar::AcousticProperties p;
  p.lambertDb = _db;
  return p;
}

/// \brief Draw what a renderer would: per pixel, depth along the camera's
/// axis and the label of the surface hit.
void Draw(const sonar::CameraSampler &_sampler,
  const sonar::SyntheticScene &_scene, const Pose3d &_pose,
  std::vector<std::vector<float>> &_depth,
  std::vector<std::vector<std::uint8_t>> &_labels)
{
  const auto &l = _sampler.Layout();
  _depth.assign(l.yaws.size(), {});
  _labels.assign(l.yaws.size(), {});
  for (std::size_t c = 0; c < l.yaws.size(); ++c)
  {
    std::vector<sonar::Ray> rays;
    for (unsigned int v = 0; v < l.height; ++v)
      for (unsigned int u = 0; u < l.width; ++u)
        rays.push_back(_sampler.PixelRay(c, u, v));
    const auto hits = _scene.Cast(_pose, rays, l.farClip);
    _depth[c].resize(hits.size());
    _labels[c].resize(hits.size());
    for (std::size_t n = 0; n < hits.size(); ++n)
    {
      // Depth along the camera axis: range times the cosine to the axis.
      const auto d = hits[n].ray.Direction();
      const double cosAxis = d.X() * std::cos(l.yaws[c]) +
        d.Y() * std::sin(l.yaws[c]);
      _depth[c][n] = hits[n].hit ?
        static_cast<float>(hits[n].range * cosAxis) :
        std::numeric_limits<float>::infinity();
      _labels[c][n] = static_cast<std::uint8_t>(hits[n].hit ? hits[n].label : 0);
    }
  }
}
}  // namespace

/////////////////////////////////////////////////
// The fan of the ECHO splits into cameras no wider than the limit, and
// every ray lands inside its camera's image.
TEST(CameraSampler, LayoutCoversTheFan)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  const sonar::CameraSampler s(c);
  const auto &l = s.Layout();
  EXPECT_EQ(l.yaws.size(), 2u);
  EXPECT_LT(l.hfov, 75.0 * kDeg);
  EXPECT_GT(l.width, 100u);
  EXPECT_GT(l.height, 100u);

  const sonar::CameraSampler side(sonar::OmniscanSidescan());
  EXPECT_EQ(side.Layout().yaws.size(), 1u);
}

/////////////////////////////////////////////////
// Scene B through synthetic images: ranges match casting the fan directly
// to a millimeter on the seabed and the box, labels agree, and so does the
// incidence angle from neighboring hits.
TEST(CameraSampler, MatchesDirectCastOnSceneB)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  c.samples = 512;
  const sonar::CameraSampler sampler(c);

  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  scene.AddBox({4.5, -0.5, 0}, {5.5, 0.5, 0.5}, Lambert(-15.0), 2);
  const Pose3d pose(0, 0, 2, 0, 15.0 * kDeg, 0);

  std::vector<std::vector<float>> depth;
  std::vector<std::vector<std::uint8_t>> labels;
  Draw(sampler, scene, pose, depth, labels);
  std::vector<const float *> dp;
  std::vector<const std::uint8_t *> lp;
  for (std::size_t i = 0; i < depth.size(); ++i)
  {
    dp.push_back(depth[i].data());
    lp.push_back(labels[i].data());
  }
  const std::vector<sonar::AcousticProperties> table{
    Lambert(-20.0), Lambert(-27.0), Lambert(-15.0)};
  const auto sampled = sampler.Sample(dp, lp, 1, table);
  const auto truth = scene.Cast(pose, sampler.Rays(), sampler.Layout().farClip);
  ASSERT_EQ(sampled.size(), truth.size());

  std::size_t compared = 0, far = 0, labelMiss = 0, angleMiss = 0;
  for (std::size_t n = 0; n < truth.size(); ++n)
  {
    if (!truth[n].hit || !sampled[n].hit || truth[n].range > 10.0)
      continue;
    ++compared;
    if (std::abs(sampled[n].range - truth[n].range) > 0.002)
      ++far;
    if (sampled[n].label != truth[n].label)
      ++labelMiss;
    if (std::abs(sampled[n].incidence - truth[n].incidence) > 1.0 * kDeg)
      ++angleMiss;
    if (sampled[n].label == 2)
      EXPECT_DOUBLE_EQ(sampled[n].properties.lambertDb, -15.0);
  }
  ASSERT_GT(compared, 10000u);
  // Only rays on a silhouette may miss; they are a small fraction.
  EXPECT_LT(static_cast<double>(far) / compared, 0.01);
  EXPECT_LT(static_cast<double>(labelMiss) / compared, 0.01);
  EXPECT_LT(static_cast<double>(angleMiss) / compared, 0.02);
}

namespace
{
/// \brief Draw the scene and sample it, as the sonar system does.
std::vector<sonar::RayHit> DrawAndSample(const sonar::CameraSampler &_sampler,
  const sonar::SyntheticScene &_scene, const Pose3d &_pose,
  const std::vector<sonar::AcousticProperties> &_table)
{
  std::vector<std::vector<float>> depth;
  std::vector<std::vector<std::uint8_t>> labels;
  Draw(_sampler, _scene, _pose, depth, labels);
  std::vector<const float *> dp;
  std::vector<const std::uint8_t *> lp;
  for (std::size_t i = 0; i < depth.size(); ++i)
  {
    dp.push_back(depth[i].data());
    lp.push_back(labels[i].data());
  }
  return _sampler.Sample(dp, lp, 1, _table);
}

double Median(std::vector<double> _v)
{
  if (_v.empty())
    return 0.0;
  std::nth_element(_v.begin(), _v.begin() + _v.size() / 2, _v.end());
  return _v[_v.size() / 2];
}
}  // namespace

/////////////////////////////////////////////////
// A flat seabed seen at grazing angles out to the end of the ECHO's 30 m
// window: depth changes by far more than 2 % from one pixel to the next,
// yet the read is still exact, since 1/z stays planar. From 2 m and from
// 1 m above the seabed, every ray's range is well under a sample (29 mm)
// from the truth.
TEST(CameraSampler, ExactOnASeabedAtGrazingAngles)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const double dr = (c.rangeMax - c.rangeMin) / static_cast<double>(c.samples);
  const sonar::CameraSampler sampler(c);
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  const std::vector<sonar::AcousticProperties> table{
    Lambert(-20.0), Lambert(-27.0)};

  for (double height : {2.0, 1.0})
  {
    const Pose3d pose(0, 0, height, 0, 0, 0);
    const auto sampled = DrawAndSample(sampler, scene, pose, table);
    const auto truth = scene.Cast(pose, sampler.Rays(),
      sampler.Layout().farClip);
    ASSERT_EQ(sampled.size(), truth.size());
    std::vector<double> nearErr, farErr;
    for (std::size_t n = 0; n < truth.size(); ++n)
    {
      if (!truth[n].hit || truth[n].range > c.rangeMax)
        continue;
      ASSERT_TRUE(sampled[n].hit);
      const double e = std::abs(sampled[n].range - truth[n].range);
      if (truth[n].range <= 15.0)
        nearErr.push_back(e);
      else if (truth[n].range >= 22.0)
        farErr.push_back(e);
    }
    ASSERT_FALSE(farErr.empty());
    const double farMax = *std::max_element(farErr.begin(), farErr.end());
    const double nearMax = *std::max_element(nearErr.begin(), nearErr.end());
    std::printf("height %.0f m: 0-15 m median %.6f max %.6f m (%zu rays); "
      "22-30 m median %.6f max %.6f m (%zu rays)\n", height,
      Median(nearErr), nearMax, nearErr.size(), Median(farErr), farMax,
      farErr.size());
    EXPECT_LT(nearMax, 0.01 * dr);
    EXPECT_LT(farMax, 0.1 * dr);
  }
}

/////////////////////////////////////////////////
// Two walls square to the camera, at 4 m and 8 m, meeting along a row of
// pixels and sharing a label, as two parts of one model would. Across the
// row the four pixels around a ray differ in v only, so 1/z has no twist
// there; the step must still not be interpolated, or a ray would put an
// echo in the empty water between the walls.
TEST(CameraSampler, DoesNotBlendAStepAlongAPixelRow)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const sonar::CameraSampler sampler(c);
  const auto &l = sampler.Layout();
  const float zNear = 4.0f, zFar = 8.0f;
  const std::size_t size = static_cast<std::size_t>(l.width) * l.height;
  std::vector<std::vector<float>> depth(l.yaws.size());
  std::vector<std::vector<std::uint8_t>> labels(l.yaws.size(),
    std::vector<std::uint8_t>(size, 1));
  for (auto &d : depth)
  {
    d.resize(size);
    for (unsigned int v = 0; v < l.height; ++v)
      for (unsigned int u = 0; u < l.width; ++u)
        d[v * l.width + u] = v < l.height / 2 ? zFar : zNear;
  }
  std::vector<const float *> dp;
  std::vector<const std::uint8_t *> lp;
  for (std::size_t i = 0; i < depth.size(); ++i)
  {
    dp.push_back(depth[i].data());
    lp.push_back(labels[i].data());
  }
  const auto hits = sampler.Sample(dp, lp, 1, {Lambert(-20.0)});

  std::size_t between = 0, checked = 0;
  for (const auto &h : hits)
  {
    ASSERT_TRUE(h.hit);
    // Depth along the axis of the camera the ray falls in: the nearest one.
    const auto d = h.ray.Direction();
    double cosAxis = 0.0;
    for (double yaw : l.yaws)
      cosAxis = std::max(cosAxis, d.X() * std::cos(yaw) + d.Y() * std::sin(yaw));
    const double z = h.range * cosAxis;
    ++checked;
    if (z > zNear + 1e-3 && z < zFar - 1e-3)
      ++between;
  }
  EXPECT_GT(checked, 1000u);
  EXPECT_EQ(between, 0u);
}
