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
#include "gz/sim/maritime_gui/WindSock.hh"

#include <algorithm>
#include <cmath>

#include <gz/math/Color.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include "gz/sim/maritime_gui/Flow.hh"

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief Bands along the cone, alternating orange and white.
  constexpr int kBands{5};

  /// \brief Facets around the cone.
  constexpr int kFacets{14};

  /// \brief The tail's radius against the mouth's.
  constexpr double kTaper{0.45};
}

//////////////////////////////////////////////////
math::Vector3d SockAxis(const math::Vector3d &_wind,
                        const math::Vector3d &_fallback)
{
  math::Vector3d flat(_wind.X(), _wind.Y(), 0.0);
  const double speed = flat.Length();
  if (speed > 1e-6)
  {
    flat /= speed;
  }
  else
  {
    flat = math::Vector3d(_fallback.X(), _fallback.Y(), 0.0);
    flat = flat.Length() > 1e-6 ? flat.Normalized() : math::Vector3d::UnitX;
  }
  const double droop = GZ_DTOR(kSockMaxDroop) *
      (1.0 - std::min(1.0, speed / kSockFullSpeed));
  return flat * std::cos(droop) - math::Vector3d::UnitZ * std::sin(droop);
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> WindSockMarkers(const std::string &_ns,
    std::uint64_t _firstId, const math::Vector3d &_base, double _height,
    const math::Vector3d &_axis, double _length)
{
  const math::Vector3d top = _base + math::Vector3d(0, 0, _height);
  const math::Vector3d a = _axis.Length() > 1e-6 ? _axis.Normalized() :
      math::Vector3d::UnitX;

  // A frame around the axis.
  math::Vector3d e1 = a.Cross(math::Vector3d::UnitZ);
  if (e1.Length() < 1e-6)
    e1 = a.Cross(math::Vector3d::UnitX);
  e1.Normalize();
  const math::Vector3d e2 = a.Cross(e1).Normalized();

  const double r0 = 0.1 * _length;
  auto ring = [&](double _t, int _i)
  {
    const double r = r0 * (1.0 - (1.0 - kTaper) * _t);
    const double phi = 2.0 * GZ_PI * _i / kFacets;
    return top + a * (_t * _length) +
        (e1 * std::cos(phi) + e2 * std::sin(phi)) * r;
  };

  std::vector<math::Vector3d> orange;
  std::vector<math::Vector3d> white;
  for (int b = 0; b < kBands; ++b)
  {
    auto &tris = (b % 2 == 0) ? orange : white;
    const double t0 = static_cast<double>(b) / kBands;
    const double t1 = static_cast<double>(b + 1) / kBands;
    for (int i = 0; i < kFacets; ++i)
    {
      const auto p00 = ring(t0, i);
      const auto p01 = ring(t0, i + 1);
      const auto p10 = ring(t1, i);
      const auto p11 = ring(t1, i + 1);
      // Both windings, so the renderer's back face culling cannot hide a
      // facet, outside or in.
      tris.insert(tris.end(), {p00, p10, p11, p00, p11, p10,
                               p00, p11, p01, p00, p01, p11});
    }
  }

  // The pole: a thin six sided prism, solid rather than a hairline.
  std::vector<math::Vector3d> pole;
  const double pr = 0.025 * _length;
  for (int i = 0; i < 6; ++i)
  {
    const double p0 = 2.0 * GZ_PI * i / 6.0;
    const double p1 = 2.0 * GZ_PI * (i + 1) / 6.0;
    const math::Vector3d d0(std::cos(p0) * pr, std::sin(p0) * pr, 0.0);
    const math::Vector3d d1(std::cos(p1) * pr, std::sin(p1) * pr, 0.0);
    const auto b0 = _base + d0, b1 = _base + d1, t0 = top + d0, t1 = top + d1;
    pole.insert(pole.end(), {b0, b1, t1, b0, t1, b1, b0, t1, t0, b0, t0, t1});
  }

  return {
    SolidMarker(_ns, _firstId, msgs::Marker::TRIANGLE_LIST,
                math::Color(0.75f, 0.75f, 0.75f, 1.0f), pole),
    SolidMarker(_ns, _firstId + 1, msgs::Marker::TRIANGLE_LIST,
                math::Color(1.0f, 0.35f, 0.0f, 1.0f), orange),
    SolidMarker(_ns, _firstId + 2, msgs::Marker::TRIANGLE_LIST,
                math::Color(0.95f, 0.95f, 0.95f, 1.0f), white)};
}
}  // namespace gz::sim::maritime_gui
