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
bool SameSurface(const RayHit &_hit, const RayHit &_neighbor)
{
  if (!_hit.hit || !_neighbor.hit || _hit.label != _neighbor.label)
    return false;
  // From the nearer of the two, so the test is the same both ways.
  const double r = std::min(_hit.range, _neighbor.range);
  const double spacing = std::max(_hit.ray.spacing, _neighbor.ray.spacing);
  return std::abs(_hit.range - _neighbor.range) <= r * spacing * kMaxSlant;
}

//////////////////////////////////////////////////
void AssignFootprints(std::vector<RayHit> &_hits, std::size_t _elevationCount)
{
  if (_elevationCount == 0 || _hits.size() % _elevationCount != 0)
    return;
  const std::size_t cols = _hits.size() / _elevationCount;
  auto at = [&](long _i, long _k) -> const RayHit * {
    if (_i < 0 || _k < 0 || _i >= static_cast<long>(cols) ||
        _k >= static_cast<long>(_elevationCount))
    {
      return nullptr;
    }
    return &_hits[static_cast<std::size_t>(_i) * _elevationCount +
      static_cast<std::size_t>(_k)];
  };
  // The spread toward one pair of neighbors: half the distance between
  // their ranges, or the distance to the one that shares the surface.
  auto spread = [](const RayHit &_h, const RayHit *_a, const RayHit *_b) {
    const bool okA = _a && SameSurface(_h, *_a);
    const bool okB = _b && SameSurface(_h, *_b);
    if (okA && okB)
      return 0.5 * std::abs(_a->range - _b->range);
    if (okA)
      return std::abs(_a->range - _h.range);
    if (okB)
      return std::abs(_b->range - _h.range);
    return 0.0;
  };
  std::vector<double> footprint(_hits.size(), 0.0);
  for (std::size_t i = 0; i < cols; ++i)
  {
    for (std::size_t k = 0; k < _elevationCount; ++k)
    {
      const RayHit &h = _hits[i * _elevationCount + k];
      if (!h.hit)
        continue;
      const auto li = static_cast<long>(i);
      const auto lk = static_cast<long>(k);
      footprint[i * _elevationCount + k] = std::max(
        spread(h, at(li, lk - 1), at(li, lk + 1)),
        spread(h, at(li - 1, lk), at(li + 1, lk)));
    }
  }
  for (std::size_t n = 0; n < _hits.size(); ++n)
    _hits[n].footprint = footprint[n];
}
}  // namespace gz::maritime::sonar
