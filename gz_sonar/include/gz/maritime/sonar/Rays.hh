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

#include <cmath>
#include <cstdint>
#include <string>
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
  /// \return The direction.
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
/// \param[in] _config The sonar.
/// \return The rays, azimuth columns of RayFanElevationCount rays each.
std::vector<Ray> RayFan(const SonarConfig &_config);

/// \brief Number of elevation rays in each azimuth column of RayFan's grid.
/// \param[in] _config The sonar.
/// \return The number of elevation rays, at least 1.
std::size_t RayFanElevationCount(const SonarConfig &_config);

/// \brief The shape of RayFan's grid, worked out without building a ray, so
/// a fan can be sized up (and refused) before anything is allocated for it.
struct RayGrid
{
  /// \brief Azimuth columns. A double: a careless configuration can ask for
  /// more than any integer type holds.
  double columns{0.0};

  /// \brief Elevation rays in each column, RayFanElevationCount.
  std::size_t rows{0};

  /// \brief Angular steps between columns and between rows, rad.
  double azStep{0.0};
  double elStep{0.0};

  /// \brief Half the fan's extent in azimuth and in elevation, rad.
  double azHalf{0.0};
  double elHalf{0.0};

  /// \brief Azimuth of column _i, rad.
  /// \param[in] _i The column's index.
  /// \return The azimuth.
  double Azimuth(std::size_t _i) const;

  /// \brief Elevation of row _k, rad.
  /// \param[in] _k The row's index.
  /// \return The elevation.
  double Elevation(std::size_t _k) const;
};

/// \brief The shape of the fan RayFan would build.
/// \param[in] _config The sonar.
/// \return The grid's shape.
RayGrid RayFanGrid(const SonarConfig &_config);

/// \brief How far up or down a fan's rays may reach, rad: 85 degrees. The
/// cameras that draw the fan are pinholes, whose image grows as the tangent
/// of the angle and cannot reach 90.
inline constexpr double kMaxFanElevation = 85.0 * M_PI / 180.0;

/// \brief Most azimuth columns a fan may have: thirty times the ECHO's
/// 1 075, and room for 2 048 beams of 8 rays each with their margins. The
/// engine keeps a row of samples and a set of beam weights per column.
inline constexpr double kMaxFanColumns = 32768.0;

/// \brief Most rays a fan may have: twenty times the ECHO's 103 296. Each
/// ray is a hit of about 100 bytes every ping, and a depth lookup.
inline constexpr double kMaxFanRays = 2097152.0;

/// \brief Most column samples (columns times range samples) the engine may
/// keep: it holds three arrays of that size, 20 bytes a cell, so this is
/// about 670 MB. The ECHO's is 1.1 million; the ECHO at 20 000 samples
/// still fits.
inline constexpr double kMaxFanCells = 33554432.0;

/// \brief Whether a sonar's fan is one RayFan and the engine can build:
/// angles that are finite, positive and inside the limits the cameras have,
/// and a grid within kMaxFanColumns, kMaxFanRays and kMaxFanCells. A fan
/// that fails would take minutes or all the memory to build, so check
/// before RayFan, Engine or CameraSampler.
/// \param[in] _config The sonar.
/// \param[out] _error Why not, naming the values, when it is not.
/// \return True if the fan can be built.
bool CheckRayFan(const SonarConfig &_config, std::string &_error);

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
/// \param[in] _hit One ray's hit.
/// \param[in] _neighbor Its neighbor's hit.
/// \return True if they hit the same surface.
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
