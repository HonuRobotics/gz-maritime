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

/// \brief The steepest slant a surface is taken to have when nothing else
/// can be said: the tangent of its incidence angle, 50 (88.9 degrees). Past
/// it a lone ray's footprint stops growing, and two neighboring rays with
/// open water beside them are taken for two surfaces.
constexpr double kMaxSlant = 50.0;

/// \brief The steepest slant two neighboring rays alone can vouch for when
/// a third ray is there to ask: the tangent of the incidence angle, 1 (45
/// degrees). A jump in range between them up to what such a surface gives
/// is one surface; a larger jump is one surface only if the third ray
/// carries it on (kCarryOn). 45 degrees is also the incidence a hit gets
/// when its normal is unknown, so taking a near and a far surface for one
/// within this bound costs no more than not knowing the normal.
constexpr double kTwoRaySlant = 1.0;

/// \brief How evenly 1/range must carry on to a third ray for a larger jump
/// between two rays to be one surface seen at a slant: the two steps of
/// 1/range differ by no more than half the larger.
constexpr double kCarryOn = 0.5;

/// \brief Bit of SurfaceLinks: the ray and the next in its azimuth column
/// (one elevation step on) hit the same surface.
constexpr std::uint8_t kLinkElevation = 1;

/// \brief Bit of SurfaceLinks: the ray and the one beside it in the next
/// azimuth column hit the same surface.
constexpr std::uint8_t kLinkAzimuth = 2;

/// \brief The angle between two rays.
/// \param[in] _a One ray.
/// \param[in] _b The other.
/// \return The angle, rad.
double AngleBetween(const Ray &_a, const Ray &_b);

/// \brief Whether two neighboring rays on the grid hit the same surface.
/// Both must hit, with the same label. Then either their ranges differ by
/// no more than a surface at kTwoRaySlant would give across the angle
/// between them, or 1/range carries on evenly (kCarryOn) to the next ray
/// along the grid line, on either side. Two ranges alone fit a plate's edge
/// and the wall behind it as well as one surface seen at a slant; the third
/// ray tells them apart, since 1/range is linear in direction on a plane,
/// at any grazing angle, and level on each side of a step. So the test
/// tells apart parts of one model and, when no labels are drawn,
/// everything, and it does not split a plane that three rays in a line
/// hit. What it cannot tell is a gap no wider than the far surface's own
/// step from ray to ray. Where neither next ray hit anything with the same
/// label, the two rays are all there is. A plane seen at a grazing angle
/// goes on toward the sonar, so if the next ray on the nearer hit's side
/// found open water they are one surface only up to kMaxSlant; at the
/// grid's edge, or beside something with another label, nothing speaks
/// against a plane and they are one surface. A fan with two rays in
/// elevation therefore cannot tell a plate from the wall behind it there.
/// \param[in] _hit One ray's hit.
/// \param[in] _neighbor Its neighbor's hit.
/// \param[in] _angle The angle between the two rays, rad.
/// \param[in] _before The hit of the ray before _hit on the grid line
/// through the two, or null at the grid's edge.
/// \param[in] _after The hit of the ray after _neighbor on that line, or
/// null.
/// \return True if they hit the same surface.
bool SameSurface(const RayHit &_hit, const RayHit &_neighbor, double _angle,
  const RayHit *_before, const RayHit *_after);

/// \brief For each ray of the grid, which of its next neighbors hit the
/// same surface (SameSurface): kLinkElevation, kLinkAzimuth, or both.
/// \param[in] _hits Hits in RayFan order.
/// \param[in] _elevationCount Rays per azimuth column.
/// \return One byte of bits per hit.
std::vector<std::uint8_t> SurfaceLinks(const std::vector<RayHit> &_hits,
  std::size_t _elevationCount);

/// \brief Set each hit's footprint from its neighbors on the ray grid:
/// RayFan's order, azimuth columns of _elevationCount rays each. Only
/// neighbors on the same surface (SameSurface) count, so a ray at the edge
/// of a target does not take the jump to the background behind it.
/// \param[in,out] _hits Hits in RayFan order.
/// \param[in] _elevationCount Rays per azimuth column.
void AssignFootprints(std::vector<RayHit> &_hits, std::size_t _elevationCount);

/// \brief The same, with the links already found.
/// \param[in,out] _hits Hits in RayFan order.
/// \param[in] _elevationCount Rays per azimuth column.
/// \param[in] _links SurfaceLinks of the hits.
void AssignFootprints(std::vector<RayHit> &_hits, std::size_t _elevationCount,
  const std::vector<std::uint8_t> &_links);
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_RAYS_HH_
