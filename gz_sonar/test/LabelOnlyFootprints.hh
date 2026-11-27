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
#ifndef GZ_SONAR_TEST_LABELONLYFOOTPRINTS_HH_
#define GZ_SONAR_TEST_LABELONLYFOOTPRINTS_HH_

#include <cmath>
#include <vector>

#include "gz/maritime/sonar/Engine.hh"
#include "gz/maritime/sonar/Rays.hh"

namespace gz::maritime::sonar::test
{
//////////////////////////////////////////////////
/// \brief Footprints as they were before neighbors had to be on the same
/// surface: every neighbor that hit, with the same label, counts, whatever
/// the jump to it. It fills a seabed at any grazing angle (and smears a
/// target into the water behind it), so it is the reference for how well
/// a seabed must stay filled.
/// \param[in,out] _hits Hits in RayFan order.
/// \param[in] _elevationCount Rays per azimuth column.
inline void LabelOnlyFootprints(std::vector<RayHit> &_hits,
  std::size_t _elevationCount)
{
  const std::size_t ne = _elevationCount;
  const std::size_t cols = _hits.size() / ne;
  auto spread = [&](const RayHit &_h, const RayHit *_a, const RayHit *_b) {
    const bool okA = _a && _a->hit && _a->label == _h.label;
    const bool okB = _b && _b->hit && _b->label == _h.label;
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
    for (std::size_t k = 0; k < ne; ++k)
    {
      const std::size_t n = i * ne + k;
      if (!_hits[n].hit)
        continue;
      footprint[n] = std::max(
        spread(_hits[n], k > 0 ? &_hits[n - 1] : nullptr,
          k + 1 < ne ? &_hits[n + 1] : nullptr),
        spread(_hits[n], i > 0 ? &_hits[n - ne] : nullptr,
          i + 1 < cols ? &_hits[n + ne] : nullptr));
    }
  }
  for (std::size_t n = 0; n < _hits.size(); ++n)
    _hits[n].footprint = footprint[n];
}

//////////////////////////////////////////////////
/// \brief The share of cells between two ranges, over every beam, that
/// hold no echo at all.
/// \param[in] _b The intensity.
/// \param[in] _from Nearest range counted, m.
/// \param[in] _to Farthest range counted, m.
/// \return The share, 0 to 1.
inline double EmptyShare(const BeamIntensity &_b, double _from, double _to)
{
  std::size_t cells = 0, empty = 0;
  for (std::size_t i = 0; i < _b.beams; ++i)
  {
    for (std::size_t k = 0; k < _b.samples; ++k)
    {
      if (_b.ranges[k] < _from || _b.ranges[k] > _to)
        continue;
      ++cells;
      if (!(_b.data[i * _b.samples + k] > 0.0))
        ++empty;
    }
  }
  return cells ? static_cast<double>(empty) / static_cast<double>(cells) : 0.0;
}
}  // namespace gz::maritime::sonar::test

#endif  // GZ_SONAR_TEST_LABELONLYFOOTPRINTS_HH_
