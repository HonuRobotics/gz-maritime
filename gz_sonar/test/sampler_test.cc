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
#include <functional>
#include <limits>
#include <string>
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

//////////////////////////////////////////////////
sonar::AcousticProperties Lambert(double _db)
{
  sonar::AcousticProperties p;
  p.lambertDb = _db;
  return p;
}

//////////////////////////////////////////////////
/// \brief Draw what a renderer would: per pixel, depth along the camera's
/// axis and the label of the surface hit.
/// \param[in] _sampler The sampler, for its cameras.
/// \param[in] _scene The scene.
/// \param[in] _pose The sensor's pose.
/// \param[out] _depth One depth image per camera.
/// \param[out] _labels One label image per camera.
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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
//////////////////////////////////////////////////
/// \brief Draw the scene and sample it, as the sonar system does.
/// \param[in] _sampler The sampler.
/// \param[in] _scene The scene.
/// \param[in] _pose The sensor's pose.
/// \param[in] _table Acoustic properties by label.
/// \param[in] _withLabels Whether label images are drawn too.
/// \return The hits.
std::vector<sonar::RayHit> DrawAndSample(const sonar::CameraSampler &_sampler,
  const sonar::SyntheticScene &_scene, const Pose3d &_pose,
  const std::vector<sonar::AcousticProperties> &_table,
  bool _withLabels = true)
{
  std::vector<std::vector<float>> depth;
  std::vector<std::vector<std::uint8_t>> labels;
  Draw(_sampler, _scene, _pose, depth, labels);
  std::vector<const float *> dp;
  std::vector<const std::uint8_t *> lp;
  for (std::size_t i = 0; i < depth.size(); ++i)
  {
    dp.push_back(depth[i].data());
    if (_withLabels)
      lp.push_back(labels[i].data());
  }
  return _sampler.Sample(dp, lp, 1, _table);
}

//////////////////////////////////////////////////
double Median(std::vector<double> _v)
{
  if (_v.empty())
    return 0.0;
  std::nth_element(_v.begin(), _v.begin() + _v.size() / 2, _v.end());
  return _v[_v.size() / 2];
}
}  // namespace

//////////////////////////////////////////////////
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

namespace
{
//////////////////////////////////////////////////
/// \brief A seabed that is not one plane: its height over the plane z = 0,
/// cast by marching along the ray.
struct Seabed
{
  /// \brief What the seabed is, for the printed table.
  std::string name;

  /// \brief Height at (x, y), m.
  std::function<double(double, double)> height;

  /// \brief The largest height either way, m.
  double amplitude{0.0};

  /// \brief Marching step, m: well under the seabed's shortest feature.
  double step{0.05};

  /// \brief Range to the seabed along a ray, m.
  /// \param[in] _origin The ray's start, above the seabed.
  /// \param[in] _dir Its unit direction.
  /// \param[in] _maxRange Farthest hit, m.
  /// \return The range, or infinity for a miss.
  double Cast(const gz::math::Vector3d &_origin,
    const gz::math::Vector3d &_dir, double _maxRange) const
  {
    const double inf = std::numeric_limits<double>::infinity();
    if (_dir.Z() >= 0.0)
      return inf;
    auto above = [&](double _t) {
      const auto p = _origin + _dir * _t;
      return p.Z() - this->height(p.X(), p.Y());
    };
    // The seabed lies between z = -amplitude and +amplitude.
    double t = std::max(0.0, (_origin.Z() - this->amplitude) / -_dir.Z());
    const double tEnd = std::min(_maxRange,
      (_origin.Z() + this->amplitude) / -_dir.Z() + this->step);
    double before = t;
    for (; t <= tEnd; before = t, t += this->step)
    {
      if (above(t) > 0.0)
        continue;
      double lo = before, hi = t;
      for (int i = 0; i < 40; ++i)
      {
        const double mid = 0.5 * (lo + hi);
        (above(mid) > 0.0 ? lo : hi) = mid;
      }
      return hi < _maxRange ? hi : inf;
    }
    return inf;
  }
};

//////////////////////////////////////////////////
/// \brief A mesh of triangles on a 1 m grid, each vertex up to 1 cm above
/// or below the plane: flat facets that meet at creases.
/// \return The seabed.
Seabed MeshSeabed()
{
  Seabed s;
  s.name = "mesh, 1 m cells, +-1 cm";
  s.amplitude = 0.01;
  s.step = 0.05;
  s.height = [](double _x, double _y) {
    auto vertex = [](long _i, long _j) {
      auto h = static_cast<std::uint32_t>(_i * 73856093L) ^
        static_cast<std::uint32_t>(_j * 19349663L);
      h = (h ^ (h >> 13)) * 0x5bd1e995u;
      h ^= h >> 15;
      return 0.01 * (2.0 * (h & 0xffffu) / 65535.0 - 1.0);
    };
    const double fi = std::floor(_x), fj = std::floor(_y);
    const auto i = static_cast<long>(fi), j = static_cast<long>(fj);
    const double a = _x - fi, b = _y - fj;
    const double z00 = vertex(i, j), z11 = vertex(i + 1, j + 1);
    // Two triangles per cell, split along the diagonal.
    if (a >= b)
    {
      const double z10 = vertex(i + 1, j);
      return z00 + a * (z10 - z00) + b * (z11 - z10);
    }
    const double z01 = vertex(i, j + 1);
    return z00 + b * (z01 - z00) + a * (z11 - z01);
  };
  return s;
}

//////////////////////////////////////////////////
/// \brief A smooth undulation, 5 cm high over 20 m.
/// \return The seabed.
Seabed UndulatingSeabed()
{
  Seabed s;
  s.name = "undulation, 5 cm over 20 m";
  s.amplitude = 0.05;
  s.step = 0.2;
  s.height = [](double _x, double _y) {
    return 0.05 * std::sin(2.0 * M_PI * _x / 20.0) *
      std::cos(2.0 * M_PI * _y / 20.0);
  };
  return s;
}

//////////////////////////////////////////////////
/// \brief Ripples 1 cm high, crests across the sonar's axis.
/// \param[in] _wavelength Crest to crest, m.
/// \return The seabed.
Seabed RippledSeabed(double _wavelength)
{
  Seabed s;
  char name[64];
  std::snprintf(name, sizeof(name), "ripples, 1 cm, every %.1f m",
    _wavelength);
  s.name = name;
  s.amplitude = 0.01;
  s.step = _wavelength / 16.0;
  s.height = [_wavelength](double _x, double) {
    return 0.01 * std::sin(2.0 * M_PI * _x / _wavelength);
  };
  return s;
}

//////////////////////////////////////////////////
/// \brief Draw a seabed as depth images, sample them, and compare each
/// ray's range with casting the ray itself.
/// \param[in] _sampler The sampler.
/// \param[in] _seabed The seabed.
/// \param[in] _pose The sensor's pose.
/// \param[in] _from Nearest true range counted, m.
/// \param[in] _to Farthest true range counted, m.
/// \param[in] _tolerance A ray is off when its range errs by more, m.
/// \param[out] _count Rays counted.
/// \return The share of the rays counted that are off.
double ShareOff(const sonar::CameraSampler &_sampler, const Seabed &_seabed,
  const Pose3d &_pose, double _from, double _to, double _tolerance,
  std::size_t &_count)
{
  const auto &l = _sampler.Layout();
  std::vector<std::vector<float>> depth(l.yaws.size());
  for (std::size_t c = 0; c < l.yaws.size(); ++c)
  {
    depth[c].resize(static_cast<std::size_t>(l.width) * l.height);
    for (unsigned int v = 0; v < l.height; ++v)
    {
      for (unsigned int u = 0; u < l.width; ++u)
      {
        const auto d = _sampler.PixelRay(c, u, v).Direction();
        const double range = _seabed.Cast(_pose.Pos(),
          _pose.Rot().RotateVector(d), l.farClip);
        const double cosAxis = d.X() * std::cos(l.yaws[c]) +
          d.Y() * std::sin(l.yaws[c]);
        depth[c][v * l.width + u] = static_cast<float>(range * cosAxis);
      }
    }
  }
  std::vector<const float *> dp;
  for (const auto &d : depth)
    dp.push_back(d.data());
  const auto hits = _sampler.Sample(dp, {}, 1, {Lambert(-27.0)});

  std::size_t off = 0;
  _count = 0;
  for (const auto &h : hits)
  {
    const double truth = _seabed.Cast(_pose.Pos(),
      _pose.Rot().RotateVector(h.ray.Direction()), l.farClip);
    if (!(truth >= _from && truth <= _to))
      continue;
    ++_count;
    if (!h.hit || std::abs(h.range - truth) > _tolerance)
      ++off;
  }
  return _count ? static_cast<double>(off) / static_cast<double>(_count) : 0.0;
}
}  // namespace

//////////////////////////////////////////////////
// Seabeds that are not one plane, seen at grazing angles at the far end of
// the ECHO's window: a mesh whose facets meet at creases of a degree or
// so, and a smooth undulation. Their depths are still read between pixels,
// so from 2 m up nearly every ray is within one range sample of casting the
// ray itself, also with the sonar pitched and rolled, which is the same as
// a sloping seabed. A test for one exact plane refuses every crease and
// leaves 79 to 96 % of these rays more than a sample off (0 % from 5 m up,
// where neighboring depths are within 2 %). From 1 m and 0.5 m up the mesh
// is seen at 1 to 2 degrees, no more than its creases' own angle, and a
// pixel covers a meter or more of range: reading between pixels halves the
// share or better, but a crease inside a pixel cannot be placed.
TEST(CameraSampler, ReadsBetweenPixelsOnASeabedThatIsNotOnePlane)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const double dr = (c.rangeMax - c.rangeMin) / static_cast<double>(c.samples);
  const sonar::CameraSampler sampler(c);
  struct Case
  {
    Seabed seabed;
    Pose3d pose;
    double from;
    double limit;
  };
  // From 0.5 m up the fan's rays are 6 m apart on the seabed at 22 m, so
  // the band starts nearer to hold enough of them.
  const std::vector<Case> cases{
    {MeshSeabed(), Pose3d(0.3, 0.2, 2, 0, 0, 0), 22.0, 0.10},
    {UndulatingSeabed(), Pose3d(0.3, 0.2, 2, 0, 0, 0), 22.0, 0.02},
    {MeshSeabed(), Pose3d(0.3, 0.2, 5, 0, 0, 0), 22.0, 0.10},
    {MeshSeabed(), Pose3d(0.3, 0.2, 1, 0, 0, 0), 22.0, 0.50},
    {MeshSeabed(), Pose3d(0.3, 0.2, 0.5, 0, 0, 0), 8.0, 0.25},
    {UndulatingSeabed(), Pose3d(0.3, 0.2, 5, 0, 0, 0), 22.0, 0.02},
    {UndulatingSeabed(), Pose3d(0.3, 0.2, 1, 0, 0, 0), 22.0, 0.02},
    {UndulatingSeabed(), Pose3d(0.3, 0.2, 0.5, 0, 0, 0), 8.0, 0.10},
    {MeshSeabed(), Pose3d(0.3, 0.2, 2, 10 * kDeg, 5 * kDeg, 0), 22.0, 0.10},
    {MeshSeabed(), Pose3d(0.3, 0.2, 2, -10 * kDeg, -3 * kDeg, 0), 22.0, 0.10},
    {UndulatingSeabed(), Pose3d(0.3, 0.2, 2, 10 * kDeg, 5 * kDeg, 0), 22.0,
      0.02},
  };
  for (const auto &k : cases)
  {
    std::size_t count = 0;
    const double share = ShareOff(sampler, k.seabed, k.pose, k.from, 30.0, dr,
      count);
    std::printf("%-28s height %.1f m roll %3.0f pitch %2.0f deg: %5.1f %% of "
      "%zu rays at %.0f to 30 m off by more than a sample\n",
      k.seabed.name.c_str(), k.pose.Pos().Z(), k.pose.Rot().Roll() / kDeg,
      k.pose.Rot().Pitch() / kDeg, 100.0 * share, count, k.from);
    EXPECT_GT(count, 1000u);
    EXPECT_LT(share, k.limit) << k.seabed.name << " from " <<
      k.pose.Pos().Z() << " m";
  }
}

//////////////////////////////////////////////////
// Ripples 1 cm high: whether they can be read depends on their length
// against a pixel's. At 22 to 30 m from 2 m up a pixel covers 0.5 to 0.9 m
// of seabed along the range. Ripples 5 m long are resolved and read between
// pixels. Ripples of 1 m or 0.2 m are not in the depth image at all, which
// holds one depth per pixel: most rays are then off by more than a sample
// however the image is read, though by less than the stretch of seabed the
// ray stands for.
TEST(CameraSampler, RipplesFinerThanAPixelAreNotResolved)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const double dr = (c.rangeMax - c.rangeMin) / static_cast<double>(c.samples);
  const sonar::CameraSampler sampler(c);
  for (const double wavelength : {5.0, 1.0, 0.2})
  {
    const Seabed seabed = RippledSeabed(wavelength);
    std::size_t count = 0;
    const double share = ShareOff(sampler, seabed,
      Pose3d(0.3, 0.2, 2, 0, 0, 0), 22.0, 30.0, dr, count);
    std::printf("%-28s height 2.0 m: %5.1f %% of %zu rays at 22 to 30 m off "
      "by more than a sample\n", seabed.name.c_str(), 100.0 * share, count);
    EXPECT_GT(count, 1000u);
    if (wavelength > 2.0)
      EXPECT_LT(share, 0.02);
  }
}

//////////////////////////////////////////////////
// Steps stay steps where the seabed around them is read between pixels: a
// box 1 m high and a low sill of 0.2 m on a seabed seen at a grazing angle,
// from 2 m and from 1 m up, with labels and without (then only the depths
// tell the box from the seabed). No ray reads a hit in the shadow behind
// the box, between its top and the seabed beyond; a ray at an edge may read
// the other side of it, as the nearest pixel does, but not the empty water.
TEST(CameraSampler, StepOnAGrazingSeabedIsNotBlended)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const double dr = (c.rangeMax - c.rangeMin) / static_cast<double>(c.samples);
  const sonar::CameraSampler sampler(c);
  const std::vector<sonar::AcousticProperties> table{
    Lambert(-20.0), Lambert(-27.0), Lambert(-15.0)};
  for (const double height : {2.0, 1.0})
  {
    for (const double top : {1.0, 0.2})
    {
      for (const bool withLabels : {true, false})
      {
        sonar::SyntheticScene scene;
        scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
        scene.AddBox({12.0, -3.0, 0}, {13.0, 3.0, top}, Lambert(-15.0), 2);
        const Pose3d pose(0, 0, height, 0, 0, 0);
        const auto sampled = DrawAndSample(sampler, scene, pose, table,
          withLabels);
        const auto truth = scene.Cast(pose, sampler.Rays(),
          sampler.Layout().farClip);
        // The shadow on the ground: from under the box's far top edge to
        // where the ray over that edge meets the seabed.
        const double shadowFrom = 13.0;
        const double shadowTo = 13.0 * height / (height - top);
        std::size_t near = 0, inShadow = 0, off = 0;
        for (std::size_t n = 0; n < truth.size(); ++n)
        {
          if (!truth[n].hit || std::abs(truth[n].ray.azimuth) > 10.0 * kDeg ||
              truth[n].range < 10.0 || truth[n].range > c.rangeMax)
          {
            continue;
          }
          ++near;
          ASSERT_TRUE(sampled[n].hit);
          if (std::abs(sampled[n].range - truth[n].range) > dr)
            ++off;
          // How far ahead the ray's hit is; the sonar is level.
          const double x = sampled[n].range * sampled[n].ray.Direction().X();
          if (x > shadowFrom + dr && x < shadowTo - dr)
            ++inShadow;
        }
        std::printf("height %.0f m, box %.1f m, labels %d: shadow %.2f to "
          "%.2f m, %zu of %zu rays in it, %zu off by more than a sample\n",
          height, top, withLabels, shadowFrom, shadowTo, inShadow, near, off);
        EXPECT_GT(near, 1000u);
        EXPECT_EQ(inShadow, 0u);
        // The fan has few rays in elevation out here, so those at the box's
        // edges, read from the nearest pixel, are several percent of them.
        EXPECT_LT(static_cast<double>(off) / near, 0.10);
      }
    }
  }
}

//////////////////////////////////////////////////
// Walls square to the camera, near and far, meeting along rows of pixels
// and sharing a label, as two parts of one model would. Across a row the
// four pixels around a ray differ in v only, so 1/z has no twist there;
// the step must still not be interpolated, or a ray would put an echo in
// the empty water between the walls. The walls alternate row by row, so
// every ray's four pixels straddle a step (a single step at the middle
// row falls between two rays of the fan and tests nothing). At 4 m and 8 m,
// and at 4 m and 4.4 m, a step of 10 %: more than the 2 % within which
// depths are blended without the planarity test.
TEST(CameraSampler, DoesNotBlendAStepAlongAPixelRow)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const sonar::CameraSampler sampler(c);
  const auto &l = sampler.Layout();
  for (const float zFar : {8.0f, 4.4f})
  {
    const float zNear = 4.0f;
    const std::size_t size = static_cast<std::size_t>(l.width) * l.height;
    std::vector<std::vector<float>> depth(l.yaws.size());
    std::vector<std::vector<std::uint8_t>> labels(l.yaws.size(),
      std::vector<std::uint8_t>(size, 1));
    for (auto &d : depth)
    {
      d.resize(size);
      for (unsigned int v = 0; v < l.height; ++v)
        for (unsigned int u = 0; u < l.width; ++u)
          d[v * l.width + u] = v % 2 ? zFar : zNear;
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
        cosAxis = std::max(cosAxis,
          d.X() * std::cos(yaw) + d.Y() * std::sin(yaw));
      const double z = h.range * cosAxis;
      ++checked;
      if (z > zNear + 1e-3 && z < zFar - 1e-3)
        ++between;
    }
    EXPECT_GT(checked, 1000u);
    EXPECT_EQ(between, 0u) << "far wall at " << zFar << " m";
  }
}

//////////////////////////////////////////////////
// A pole one ray wide, facing the sonar, in front of a wall: its neighbors
// in azimuth are on the wall, so they cannot give its normal, and it keeps
// the 45 degree fallback, as the same pole does in open water. With the
// wall's own label, and with the pole's label on the wall too (one model),
// where only the jump in range tells them apart.
TEST(CameraSampler, PoleOneRayWideBeforeAWallKeepsTheFallback)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  const sonar::CameraSampler sampler(c);
  // The azimuth step of the fan, and the ray column nearest straight ahead.
  const auto &rays = sampler.Rays();
  const std::size_t ne = sonar::RayFanElevationCount(c);
  const double azStep = rays[ne].azimuth - rays[0].azimuth;
  double az0 = rays[0].azimuth;
  for (std::size_t n = 0; n < rays.size(); n += ne)
    if (std::abs(rays[n].azimuth) < std::abs(az0))
      az0 = rays[n].azimuth;
  const double x = 4.0;
  const double yc = x * std::tan(az0);
  const double half = 0.5 * x * azStep;

  for (std::uint32_t wallLabel : {3u, 2u})
  {
    for (bool wall : {false, true})
    {
      sonar::SyntheticScene scene;
      scene.AddBox({x, yc - half, 0.0}, {x + 0.02, yc + half, 4.0},
        Lambert(-15.0), 2);
      if (wall)
        scene.AddPlane({8, 0, 0}, {-1, 0, 0}, Lambert(-20.0), wallLabel);
      const Pose3d pose(0, 0, 2, 0, 0, 0);
      const auto hits = DrawAndSample(sampler, scene, pose,
        {Lambert(-20.0), Lambert(-20.0), Lambert(-15.0), Lambert(-20.0)});
      std::size_t onPole = 0, bad = 0;
      double worst = 0.0;
      for (const auto &h : hits)
      {
        if (!h.hit || h.range > 6.0)
          continue;
        ++onPole;
        // Head on is 0; the fallback is 45 degrees. Anything else is a
        // normal taken across the jump to the wall.
        const double inc = h.incidence;
        const double err = std::min(inc, std::abs(inc - 45.0 * kDeg));
        worst = std::max(worst, inc);
        if (err > 2.0 * kDeg)
          ++bad;
      }
      std::printf("wall %d label %u: %zu rays on the pole, largest "
        "incidence %.1f deg\n", wall, wallLabel, onPole, worst / kDeg);
      EXPECT_GT(onPole, 10u);
      EXPECT_EQ(bad, 0u) << "wall " << wall << " label " << wallLabel;
    }
  }
}

//////////////////////////////////////////////////
// A box face seen square on, with the box's top seen at a grazing angle
// above it. A ray on the face just under the edge has a neighbor on the
// face and one on the top, both the same surface by the jump between them;
// the one-sided difference with the smaller jump is the one on the face,
// so the face keeps its own normal up to its edge, within the few degrees
// a millimeter's misread of range at the edge costs; the top's normal is
// over 80 degrees away.
TEST(CameraSampler, NormalOnAFaceStaysOnTheFaceAtItsEdge)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  const sonar::CameraSampler sampler(c);
  sonar::SyntheticScene scene;
  scene.AddBox({4.0, -0.5, 0.0}, {5.0, 0.5, 0.5}, Lambert(-15.0), 2);
  const Pose3d pose(0, 0, 1, 0, 0, 0);
  const auto sampled = DrawAndSample(sampler, scene, pose,
    {Lambert(-20.0), Lambert(-20.0), Lambert(-15.0)});
  const auto truth = scene.Cast(pose, sampler.Rays(),
    sampler.Layout().farClip);
  std::size_t face = 0, miss = 0;
  for (std::size_t n = 0; n < truth.size(); ++n)
  {
    // On the face, away from its side edges.
    if (!truth[n].hit || !sampled[n].hit || truth[n].incidence > 30.0 * kDeg ||
        std::abs(truth[n].ray.azimuth) > 0.1)
    {
      continue;
    }
    ++face;
    if (std::abs(sampled[n].incidence - truth[n].incidence) > 5.0 * kDeg)
      ++miss;
  }
  std::printf("%zu rays on the face, %zu with the wrong normal\n", face, miss);
  EXPECT_GT(face, 500u);
  EXPECT_EQ(miss, 0u);
}

//////////////////////////////////////////////////
// The sampler reads hits past the end of the range window, to half as far
// again: a sonar that assumes too slow a sound speed reads them inside it.
// Every hit carries the footprint its neighbors give it.
TEST(CameraSampler, HitsPastTheWindowAndFootprints)
{
  const sonar::SonarConfig c = sonar::EchoImaging();
  const sonar::CameraSampler sampler(c);
  sonar::SyntheticScene scene;
  scene.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 1);
  const Pose3d pose(0, 0, 2, 0, 0, 0);
  const auto sampled = DrawAndSample(sampler, scene, pose,
    {Lambert(-20.0), Lambert(-27.0)});
  const auto truth = scene.Cast(pose, sampler.Rays(),
    sampler.Layout().farClip);
  std::size_t past = 0, inside = 0, withFootprint = 0;
  for (std::size_t n = 0; n < truth.size(); ++n)
  {
    if (!truth[n].hit)
      continue;
    if (truth[n].range > c.rangeMax && truth[n].range < 1.4 * c.rangeMax)
    {
      ++past;
      EXPECT_TRUE(sampled[n].hit) << "range " << truth[n].range;
    }
    else if (truth[n].range < c.rangeMax)
    {
      ++inside;
      if (sampled[n].footprint > 0.0)
        ++withFootprint;
    }
  }
  EXPECT_GT(past, 1000u);
  EXPECT_GT(inside, 10000u);
  EXPECT_GT(static_cast<double>(withFootprint) / inside, 0.99);
}
