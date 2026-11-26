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
#ifndef GZ_MARITIME_SONAR_RAYS_HH_
#define GZ_MARITIME_SONAR_RAYS_HH_

#include <cstdint>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/maritime/sonar/Config.hh"

namespace gz::maritime::sonar
{
/// \brief One ray of the fan, in the sensor frame.
struct Ray
{
  /// \brief Angle about the sensor's z axis, rad.
  double azimuth{0.0};

  /// \brief Angle out of the sensor's x-y plane, rad.
  double elevation{0.0};

  /// \brief Solid angle the ray stands for, sr.
  double solidAngle{0.0};

  /// \brief Angular spacing of the fan around this ray, rad: the larger of
  /// its azimuth and elevation steps. Sets how far the ray's footprint
  /// spreads in range on a surface seen at a slant.
  double spacing{0.0};

  /// \brief Unit direction in the sensor frame.
  math::Vector3d Direction() const;
};

/// \brief What one ray found: the first surface along it.
struct RayHit
{
  /// \brief The ray.
  Ray ray;

  /// \brief Whether it hit anything.
  bool hit{false};

  /// \brief Distance to the hit, m.
  double range{0.0};

  /// \brief Angle between the ray and the surface normal, rad.
  double incidence{0.0};

  /// \brief How the surface scatters.
  AcousticProperties properties;

  /// \brief Which entity was hit, for ground truth; 0 for none.
  std::uint32_t label{0};

  /// \brief The range interval this ray's patch covers, m: half the
  /// spread of the ranges its neighbors on the ray grid hit on the same
  /// surface. 0 when unknown, and the engine estimates it.
  double footprint{0.0};
};

/// \brief The fan of rays that samples a sonar's field of view: finer than
/// the beams in azimuth (raysPerBeam per beam width or beam spacing,
/// whichever is smaller), elevationRays across elevationSpan half-widths
/// of the vertical beam (by default the vertical field of view either
/// side), and two beam widths beyond each edge of the fan, so the beam
/// pattern's tapers and near sidelobes see the scene too.
std::vector<Ray> RayFan(const SonarConfig &_config);

/// \brief Number of elevation rays in each azimuth column of RayFan's grid.
std::size_t RayFanElevationCount(const SonarConfig &_config);

/// \brief The steepest slant a surface is taken to have: the tangent of its
/// largest incidence angle, 50 (88.9 degrees). Past it a lone ray's
/// footprint stops growing, and a jump in range between neighboring rays is
/// taken for two surfaces rather than one seen at a grazing angle.
constexpr double kMaxSlant = 50.0;

/// \brief Whether two neighboring rays on the grid hit the same surface:
/// both hit, with the same label, and their ranges differ by no more than a
/// surface at kMaxSlant would give across the ray spacing. The bound tells
/// apart parts of one model (a plate and a wall behind it) and, when no
/// labels are drawn, everything; a flat seabed seen at a grazing angle out
/// to a sonar's range stays within it.
bool SameSurface(const RayHit &_hit, const RayHit &_neighbor);

/// \brief Set each hit's footprint from its neighbors on the ray grid:
/// RayFan's order, azimuth columns of _elevationCount rays each. Only
/// neighbors on the same surface (SameSurface) count, so a ray at the edge
/// of a target does not take the jump to the background behind it, and the
/// footprint is no larger than one on a surface at kMaxSlant.
/// \param[in,out] _hits Hits in RayFan order.
/// \param[in] _elevationCount Rays per azimuth column.
void AssignFootprints(std::vector<RayHit> &_hits, std::size_t _elevationCount);
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_RAYS_HH_
