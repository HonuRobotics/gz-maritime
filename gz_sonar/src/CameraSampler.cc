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
#include "gz/maritime/sonar/CameraSampler.hh"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gz::maritime::sonar
{
namespace
{
/// \brief Incidence for a hit whose surface normal cannot be estimated.
constexpr double kUnknownIncidence = M_PI / 4.0;
}  // namespace

namespace
{
/// \brief Neighboring depths within this fraction are one surface.
constexpr double kContinuous = 0.02;

math::Vector3d Rotate(double _yaw, const math::Vector3d &_v)
{
  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  return {c * _v.X() - s * _v.Y(), s * _v.X() + c * _v.Y(), _v.Z()};
}
}  // namespace

CameraSampler::CameraSampler(const SonarConfig &_config,
  double _maxCameraWidth)
  : config(_config), rays(RayFan(_config)),
    elevationCount(RayFanElevationCount(_config))
{
  // The fan's extent and spacing, from the rays themselves.
  double azMin = 0.0, azMax = 0.0, elMax = 0.0;
  double azStep = std::numeric_limits<double>::max();
  double elStep = std::numeric_limits<double>::max();
  for (std::size_t n = 0; n < this->rays.size(); ++n)
  {
    const Ray &r = this->rays[n];
    azMin = std::min(azMin, r.azimuth);
    azMax = std::max(azMax, r.azimuth);
    elMax = std::max(elMax, std::abs(r.elevation));
  }
  if (this->rays.size() > this->elevationCount)
  {
    azStep = this->rays[this->elevationCount].azimuth - this->rays[0].azimuth;
  }
  if (this->elevationCount > 1)
    elStep = this->rays[1].elevation - this->rays[0].elevation;
  // A fan of one column (a sidescan beam sampled once across) has no
  // azimuth step of its own; take the elevation step.
  if (azStep == std::numeric_limits<double>::max())
    azStep = elStep;
  const double step = std::min(azStep, elStep);
  const double span = azMax - azMin + azStep;

  const auto count = static_cast<std::size_t>(
    std::max(1.0, std::ceil(span / _maxCameraWidth)));
  const double half = 0.5 * span / static_cast<double>(count);
  this->layout.focal = 1.0 / step;
  this->layout.width = static_cast<unsigned int>(
    std::ceil(2.0 * this->layout.focal * std::tan(half))) + 4;
  this->layout.height = static_cast<unsigned int>(std::ceil(
    2.0 * this->layout.focal * std::tan(elMax + elStep) / std::cos(half))) + 4;
  this->layout.hfov = 2.0 * std::atan(0.5 * this->layout.width /
    this->layout.focal);
  this->layout.farClip = 1.5 * _config.rangeMax;
  for (std::size_t c = 0; c < count; ++c)
  {
    this->layout.yaws.push_back(azMin - 0.5 * azStep +
      (static_cast<double>(c) + 0.5) * span / static_cast<double>(count));
  }

  this->lookup.resize(this->rays.size());
  for (std::size_t n = 0; n < this->rays.size(); ++n)
  {
    const Ray &r = this->rays[n];
    const auto c = std::min(count - 1, static_cast<std::size_t>(
      (r.azimuth - (azMin - 0.5 * azStep)) / span *
      static_cast<double>(count)));
    const math::Vector3d d = Rotate(-this->layout.yaws[c], r.Direction());
    Lookup &l = this->lookup[n];
    l.camera = c;
    l.u = 0.5 * this->layout.width - this->layout.focal * d.Y() / d.X();
    l.v = 0.5 * this->layout.height - this->layout.focal * d.Z() / d.X();
    l.rangePerDepth = d.Length() / d.X();
  }
}

const CameraLayout &CameraSampler::Layout() const
{
  return this->layout;
}

const std::vector<Ray> &CameraSampler::Rays() const
{
  return this->rays;
}

Ray CameraSampler::PixelRay(std::size_t _camera, unsigned int _u,
  unsigned int _v) const
{
  const math::Vector3d inCamera(1.0,
    -(static_cast<double>(_u) + 0.5 - 0.5 * this->layout.width) /
      this->layout.focal,
    -(static_cast<double>(_v) + 0.5 - 0.5 * this->layout.height) /
      this->layout.focal);
  const math::Vector3d d =
    Rotate(this->layout.yaws[_camera], inCamera).Normalized();
  Ray r;
  r.azimuth = std::atan2(d.Y(), d.X());
  r.elevation = std::asin(std::clamp(d.Z(), -1.0, 1.0));
  return r;
}

std::vector<RayHit> CameraSampler::Sample(
  const std::vector<const float *> &_depth,
  const std::vector<const std::uint8_t *> &_labels, unsigned int _channels,
  const std::vector<AcousticProperties> &_properties) const
{
  const auto w = static_cast<long>(this->layout.width);
  const auto h = static_cast<long>(this->layout.height);
  std::vector<RayHit> hits(this->rays.size());

  for (std::size_t n = 0; n < this->rays.size(); ++n)
  {
    RayHit &hit = hits[n];
    hit.ray = this->rays[n];
    const Lookup &l = this->lookup[n];
    if (l.camera >= _depth.size() || !_depth[l.camera])
      continue;
    const float *dd = _depth[l.camera];

    const long pu = std::clamp(static_cast<long>(std::floor(l.u)), 0L, w - 1);
    const long pv = std::clamp(static_cast<long>(std::floor(l.v)), 0L, h - 1);
    double z = dd[pv * w + pu];

    // 1/z is linear in screen space on a plane; interpolate it across the
    // four pixels around the ray, unless they straddle a silhouette.
    const double x = l.u - 0.5;
    const double y = l.v - 0.5;
    const long u0 = std::clamp(static_cast<long>(std::floor(x)), 0L, w - 2);
    const long v0 = std::clamp(static_cast<long>(std::floor(y)), 0L, h - 2);
    const double fx = std::clamp(x - static_cast<double>(u0), 0.0, 1.0);
    const double fy = std::clamp(y - static_cast<double>(v0), 0.0, 1.0);
    const double z00 = dd[v0 * w + u0];
    const double z10 = dd[v0 * w + u0 + 1];
    const double z01 = dd[(v0 + 1) * w + u0];
    const double z11 = dd[(v0 + 1) * w + u0 + 1];
    const double zMin = std::min({z00, z10, z01, z11});
    const double zMax = std::max({z00, z10, z01, z11});
    if (std::isfinite(zMax) && zMin > 0.0 && zMax - zMin < kContinuous * zMin)
    {
      z = 1.0 / ((1 - fx) * (1 - fy) / z00 + fx * (1 - fy) / z10 +
        (1 - fx) * fy / z01 + fx * fy / z11);
    }

    const double range = z * l.rangePerDepth;
    if (!std::isfinite(z) || z <= 0.0 || range >= this->layout.farClip)
      continue;
    hit.hit = true;
    hit.range = range;
    std::uint32_t label = 0;
    if (l.camera < _labels.size() && _labels[l.camera])
      label = _labels[l.camera][(pv * w + pu) * _channels];
    hit.label = label;
    if (!_properties.empty())
    {
      hit.properties = label < _properties.size() ?
        _properties[label] : _properties[0];
    }
  }

  // Normals from neighboring hits on the ray grid, in the sensor frame: for
  // each direction, the one-sided difference with the smaller range jump,
  // so a silhouette does not tilt the surface toward the background when
  // one neighbor is on the surface. With neither (an object one ray wide)
  // the normal is unknown and the hit keeps kUnknownIncidence.
  const std::size_t ne = this->elevationCount;
  const std::size_t cols = hits.size() / std::max<std::size_t>(ne, 1);
  std::vector<math::Vector3d> points(hits.size());
  for (std::size_t n = 0; n < hits.size(); ++n)
    points[n] = hits[n].ray.Direction() * hits[n].range;
  auto ok = [&](long _i, long _k) {
    return _i >= 0 && _k >= 0 && _i < static_cast<long>(cols) &&
      _k < static_cast<long>(ne) &&
      hits[static_cast<std::size_t>(_i) * ne + static_cast<std::size_t>(_k)].hit;
  };
  for (std::size_t i = 0; i < cols; ++i)
  {
    for (std::size_t k = 0; k < ne; ++k)
    {
      const std::size_t n = i * ne + k;
      if (!hits[n].hit)
        continue;
      // Without two neighbors on the same surface the normal is unknown (a
      // rope, a pole end on, a target one ray wide). Head on would give it
      // the strongest return and a full glint; 45 degrees gives a plain
      // diffuse return and no glint, unless a normal is found below.
      hits[n].incidence = kUnknownIncidence;
      auto pick = [&](long _di, long _dk, math::Vector3d &_out) {
        const long ia = static_cast<long>(i) + _di;
        const long ka = static_cast<long>(k) + _dk;
        const long ib = static_cast<long>(i) - _di;
        const long kb = static_cast<long>(k) - _dk;
        const bool fa = ok(ia, ka);
        const bool fb = ok(ib, kb);
        if (!fa && !fb)
          return false;
        const std::size_t na = fa ? static_cast<std::size_t>(ia) * ne +
          static_cast<std::size_t>(ka) : 0;
        const std::size_t nb = fb ? static_cast<std::size_t>(ib) * ne +
          static_cast<std::size_t>(kb) : 0;
        const double ea = fa ? std::abs(hits[na].range - hits[n].range) : 1e9;
        const double eb = fb ? std::abs(hits[nb].range - hits[n].range) : 1e9;
        _out = ea <= eb ? points[na] - points[n] : points[n] - points[nb];
        return true;
      };
      math::Vector3d a, b;
      if (!pick(1, 0, a) || !pick(0, 1, b))
        continue;
      math::Vector3d normal = a.Cross(b);
      if (normal.Length() < 1e-12)
        continue;
      normal.Normalize();
      const math::Vector3d dir = hits[n].ray.Direction();
      hits[n].incidence = std::acos(std::min(1.0, std::abs(normal.Dot(dir))));
    }
  }

  AssignFootprints(hits, ne);
  return hits;
}
}  // namespace gz::maritime::sonar
