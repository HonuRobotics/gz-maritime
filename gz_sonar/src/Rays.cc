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

namespace gz::maritime::sonar
{
math::Vector3d Ray::Direction() const
{
  const double ce = std::cos(this->elevation);
  return {ce * std::cos(this->azimuth), ce * std::sin(this->azimuth),
    std::sin(this->elevation)};
}

std::vector<Ray> RayFan(const SonarConfig &_config)
{
  const double beamSpacing = _config.horizontalFov /
    static_cast<double>(std::max<std::size_t>(_config.beams, 1));
  const double azStep = std::min(beamSpacing, _config.beamWidth) /
    static_cast<double>(std::max<std::size_t>(_config.raysPerBeam, 1));
  const double azHalf = 0.5 * _config.horizontalFov + 2.0 * _config.beamWidth;
  const auto azCount = static_cast<std::size_t>(
    std::ceil(2.0 * azHalf / azStep));

  const double elHalf = 0.5 * _config.verticalFov * _config.elevationSpan;
  const std::size_t elCount = RayFanElevationCount(_config);
  const double elStep = 2.0 * elHalf / static_cast<double>(elCount);

  std::vector<Ray> rays;
  rays.reserve(azCount * elCount);
  for (std::size_t i = 0; i < azCount; ++i)
  {
    const double az = -azHalf + (static_cast<double>(i) + 0.5) * azStep;
    for (std::size_t k = 0; k < elCount; ++k)
    {
      Ray r;
      r.azimuth = az;
      r.elevation = -elHalf + (static_cast<double>(k) + 0.5) * elStep;
      r.solidAngle = azStep * elStep * std::cos(r.elevation);
      r.spacing = std::max(azStep, elStep);
      rays.push_back(r);
    }
  }
  return rays;
}

std::size_t RayFanElevationCount(const SonarConfig &_config)
{
  const double n = static_cast<double>(_config.elevationRays) *
    _config.elevationSpan / 2.0;
  return std::max<std::size_t>(static_cast<std::size_t>(std::lround(n)), 1);
}

bool SameSurface(const RayHit &_hit, const RayHit &_neighbor)
{
  if (!_hit.hit || !_neighbor.hit || _hit.label != _neighbor.label)
    return false;
  // From the nearer of the two, so the test is the same both ways.
  const double r = std::min(_hit.range, _neighbor.range);
  const double spacing = std::max(_hit.ray.spacing, _neighbor.ray.spacing);
  return std::abs(_hit.range - _neighbor.range) <= r * spacing * kMaxSlant;
}

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
