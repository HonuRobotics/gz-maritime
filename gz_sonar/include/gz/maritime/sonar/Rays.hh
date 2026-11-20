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
};

/// \brief The fan of rays that samples a sonar's field of view: finer than
/// the beams in azimuth (raysPerBeam per beam width or beam spacing,
/// whichever is smaller), elevationRays across twice the vertical field of
/// view, and two beam widths beyond each edge of the fan, so the beam
/// pattern's tapers and near sidelobes see the scene too.
std::vector<Ray> RayFan(const SonarConfig &_config);
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_RAYS_HH_
