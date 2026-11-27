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
#include "gz/maritime/sonar/Rays.hh"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace gz::maritime::sonar
{
//////////////////////////////////////////////////
math::Vector3d Ray::Direction() const
{
  const double ce = std::cos(this->elevation);
  return {ce * std::cos(this->azimuth), ce * std::sin(this->azimuth),
    std::sin(this->elevation)};
}

//////////////////////////////////////////////////
double RayGrid::Azimuth(std::size_t _i) const
{
  return -this->azHalf + (static_cast<double>(_i) + 0.5) * this->azStep;
}

//////////////////////////////////////////////////
double RayGrid::Elevation(std::size_t _k) const
{
  return -this->elHalf + (static_cast<double>(_k) + 0.5) * this->elStep;
}

//////////////////////////////////////////////////
RayGrid RayFanGrid(const SonarConfig &_config)
{
  RayGrid g;
  const double beamSpacing = _config.horizontalFov /
    static_cast<double>(std::max<std::size_t>(_config.beams, 1));
  g.azStep = std::min(beamSpacing, _config.beamWidth) /
    static_cast<double>(std::max<std::size_t>(_config.raysPerBeam, 1));
  g.azHalf = 0.5 * _config.horizontalFov + 2.0 * _config.beamWidth;
  g.columns = std::ceil(2.0 * g.azHalf / g.azStep);

  g.elHalf = 0.5 * _config.verticalFov * _config.elevationSpan;
  g.rows = RayFanElevationCount(_config);
  g.elStep = 2.0 * g.elHalf / static_cast<double>(g.rows);
  return g;
}

//////////////////////////////////////////////////
bool CheckRayFan(const SonarConfig &_config, std::string &_error)
{
  auto number = [](double _v) {
    std::ostringstream out;
    out << std::setprecision(6) << _v;
    return out.str();
  };
  // Written so that a NaN fails each test.
  auto positive = [](double _v) { return std::isfinite(_v) && _v > 0.0; };
  struct Named { const char *name; double value; };
  for (const Named &n : {Named{"horizontal field of view", _config.horizontalFov},
                         Named{"vertical field of view", _config.verticalFov},
                         Named{"beam width", _config.beamWidth},
                         Named{"elevation span", _config.elevationSpan}})
  {
    if (!positive(n.value))
    {
      _error = std::string("the ") + n.name + " must be positive and finite, "
        "not " + number(n.value);
      return false;
    }
  }
  if (_config.beams < 1 || _config.raysPerBeam < 1 ||
      _config.elevationRays < 1 || _config.samples < 1)
  {
    _error = "beams, rays per beam, elevation rays and samples must be at "
      "least 1";
    return false;
  }
  // elevationRays * elevationSpan is rounded to the row count; keep it
  // where the rounding is defined.
  if (static_cast<double>(_config.elevationRays) * _config.elevationSpan >
      2.0 * kMaxFanRays)
  {
    _error = "elevation rays " + std::to_string(_config.elevationRays) +
      " times elevation span " + number(_config.elevationSpan) +
      " asks for more rows than a fan may have rays";
    return false;
  }

  const RayGrid g = RayFanGrid(_config);
  // The cameras that draw the fan are pinholes: they cannot look as far as
  // 90 degrees up or down, and their image grows as the tangent on the way.
  if (!(g.elHalf < kMaxFanElevation))
  {
    _error = "the rays reach " + number(g.elHalf * 180.0 / M_PI) +
      " degrees in elevation (vertical field of view " +
      number(_config.verticalFov) + " rad times elevation span " +
      number(_config.elevationSpan) + ", halved); they must stay under " +
      number(kMaxFanElevation * 180.0 / M_PI);
    return false;
  }
  // Past a full circle the fan would sample the same directions twice.
  if (!(2.0 * g.azHalf <= 2.0 * M_PI))
  {
    _error = "the fan spans " + number(2.0 * g.azHalf * 180.0 / M_PI) +
      " degrees in azimuth (horizontal field of view " +
      number(_config.horizontalFov) + " rad plus four beam widths of " +
      number(_config.beamWidth) + " rad); it must not pass 360. Angles are "
      "in radians";
    return false;
  }
  if (!positive(g.azStep) || !positive(g.elStep) || !positive(g.columns))
  {
    _error = "the fan's ray spacing is not a positive number";
    return false;
  }
  if (g.columns > kMaxFanColumns)
  {
    _error = "the fan needs " + number(g.columns) + " azimuth columns of "
      "rays, " + number(g.azStep) + " rad apart (the smaller of horizontal "
      "field of view over beams and beam width, over rays per beam) across " +
      number(2.0 * g.azHalf) + " rad; at most " + number(kMaxFanColumns);
    return false;
  }
  const double rays = g.columns * static_cast<double>(g.rows);
  if (rays > kMaxFanRays)
  {
    _error = "the fan needs " + number(rays) + " rays (" +
      number(g.columns) + " azimuth columns of " + std::to_string(g.rows) +
      " elevation rays); at most " + number(kMaxFanRays);
    return false;
  }
  const double cells = g.columns * static_cast<double>(_config.samples);
  if (cells > kMaxFanCells)
  {
    _error = "the engine would keep " + number(cells) + " column samples (" +
      number(g.columns) + " azimuth columns of " +
      std::to_string(_config.samples) + " samples); at most " +
      number(kMaxFanCells);
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
std::vector<Ray> RayFan(const SonarConfig &_config)
{
  const RayGrid g = RayFanGrid(_config);
  const auto azCount = static_cast<std::size_t>(g.columns);

  std::vector<Ray> rays;
  rays.reserve(azCount * g.rows);
  for (std::size_t i = 0; i < azCount; ++i)
  {
    const double az = g.Azimuth(i);
    for (std::size_t k = 0; k < g.rows; ++k)
    {
      Ray r;
      r.azimuth = az;
      r.elevation = g.Elevation(k);
      r.solidAngle = g.azStep * g.elStep * std::cos(r.elevation);
      r.spacing = std::max(g.azStep, g.elStep);
      rays.push_back(r);
    }
  }
  return rays;
}

//////////////////////////////////////////////////
std::size_t RayFanElevationCount(const SonarConfig &_config)
{
  const double n = static_cast<double>(_config.elevationRays) *
    _config.elevationSpan / 2.0;
  return std::max<std::size_t>(static_cast<std::size_t>(std::lround(n)), 1);
}

//////////////////////////////////////////////////
double AngleBetween(const Ray &_a, const Ray &_b)
{
  return std::acos(std::clamp(_a.Direction().Dot(_b.Direction()), -1.0, 1.0));
}

//////////////////////////////////////////////////
bool SameSurface(const RayHit &_hit, const RayHit &_neighbor, double _angle,
  const RayHit *_before, const RayHit *_after)
{
  if (!_hit.hit || !_neighbor.hit || _hit.label != _neighbor.label)
    return false;
  // From the nearer of the two, so the test is the same both ways.
  const double r = std::min(_hit.range, _neighbor.range);
  const double jump = std::abs(_hit.range - _neighbor.range);
  if (jump <= r * _angle * kTwoRaySlant)
    return true;
  // A larger jump is one surface seen at a slant, or two surfaces; the two
  // ranges alone fit either. A third ray tells: on a plane 1/r is linear
  // in the ray's direction, so it changes by as much again to the next ray
  // along the line, at any grazing angle, while past the edge of a plate
  // the wall behind it does not carry on at the jump's rate.
  const double q0 = 1.0 / _hit.range;
  const double q1 = 1.0 / _neighbor.range;
  const double d = q1 - q0;
  auto witness = [&](const RayHit *_third) {
    return _third && _third->hit && _third->label == _hit.label;
  };
  auto carriesOn = [&](double _step) {
    return std::abs(_step - d) <=
      kCarryOn * std::max(std::abs(_step), std::abs(d));
  };
  const bool before = witness(_before);
  const bool after = witness(_after);
  if (before || after)
  {
    return (before && carriesOn(q0 - 1.0 / _before->range)) ||
      (after && carriesOn(1.0 / _after->range - q1));
  }
  // No third ray on either side is on a surface with this label. A plane
  // seen at a grazing angle goes on toward the sonar, so the next ray on
  // the nearer hit's side would have hit it: if that ray found open water,
  // the two are one surface only up to the steepest slant. If there is no
  // such ray (the grid's edge) or it hit something else, nothing speaks
  // against a plane, and a plane is not split.
  const RayHit *nearer = _hit.range <= _neighbor.range ? _before : _after;
  return !nearer || nearer->hit || jump <= r * _angle * kMaxSlant;
}

//////////////////////////////////////////////////
std::vector<std::uint8_t> SurfaceLinks(const std::vector<RayHit> &_hits,
  std::size_t _elevationCount)
{
  std::vector<std::uint8_t> links(_hits.size(), 0);
  const std::size_t ne = _elevationCount;
  if (ne == 0 || _hits.empty() || _hits.size() % ne != 0)
    return links;
  const std::size_t cols = _hits.size() / ne;
  // Rays of one row share an elevation, and an azimuth step spans less
  // angle the farther the row is from the fan's plane.
  std::vector<double> cosElevation(ne);
  for (std::size_t k = 0; k < ne; ++k)
    cosElevation[k] = std::cos(_hits[k].ray.elevation);
  for (std::size_t i = 0; i < cols; ++i)
  {
    for (std::size_t k = 0; k < ne; ++k)
    {
      const std::size_t n = i * ne + k;
      const RayHit &h = _hits[n];
      if (!h.hit)
        continue;
      if (k + 1 < ne && _hits[n + 1].hit)
      {
        const double angle =
          std::abs(_hits[n + 1].ray.elevation - h.ray.elevation);
        if (SameSurface(h, _hits[n + 1], angle, k > 0 ? &_hits[n - 1] : nullptr,
            k + 2 < ne ? &_hits[n + 2] : nullptr))
        {
          links[n] |= kLinkElevation;
        }
      }
      if (i + 1 < cols && _hits[n + ne].hit)
      {
        const double angle = cosElevation[k] *
          std::abs(_hits[n + ne].ray.azimuth - h.ray.azimuth);
        if (SameSurface(h, _hits[n + ne], angle,
            i > 0 ? &_hits[n - ne] : nullptr,
            i + 2 < cols ? &_hits[n + 2 * ne] : nullptr))
        {
          links[n] |= kLinkAzimuth;
        }
      }
    }
  }
  return links;
}

//////////////////////////////////////////////////
void AssignFootprints(std::vector<RayHit> &_hits, std::size_t _elevationCount)
{
  AssignFootprints(_hits, _elevationCount,
    SurfaceLinks(_hits, _elevationCount));
}

//////////////////////////////////////////////////
void AssignFootprints(std::vector<RayHit> &_hits, std::size_t _elevationCount,
  const std::vector<std::uint8_t> &_links)
{
  const std::size_t ne = _elevationCount;
  if (ne == 0 || _hits.size() % ne != 0 || _links.size() != _hits.size())
    return;
  const std::size_t cols = _hits.size() / ne;
  // The spread toward one pair of neighbors: half the distance between
  // their ranges, or the distance to the one that shares the surface.
  auto spread = [&](std::size_t _n, bool _okA, std::size_t _a, bool _okB,
      std::size_t _b) {
    if (_okA && _okB)
      return 0.5 * std::abs(_hits[_a].range - _hits[_b].range);
    if (_okA)
      return std::abs(_hits[_a].range - _hits[_n].range);
    if (_okB)
      return std::abs(_hits[_b].range - _hits[_n].range);
    return 0.0;
  };
  std::vector<double> footprint(_hits.size(), 0.0);
  for (std::size_t i = 0; i < cols; ++i)
  {
    for (std::size_t k = 0; k < ne; ++k)
    {
      const std::size_t n = i * ne + k;
      if (!_hits[n].hit)
        continue;
      footprint[n] = std::max(
        spread(n, k > 0 && (_links[n - 1] & kLinkElevation), n - 1,
          _links[n] & kLinkElevation, n + 1),
        spread(n, i > 0 && (_links[n - ne] & kLinkAzimuth), n - ne,
          _links[n] & kLinkAzimuth, n + ne));
    }
  }
  for (std::size_t n = 0; n < _hits.size(); ++n)
    _hits[n].footprint = footprint[n];
}
}  // namespace gz::maritime::sonar
