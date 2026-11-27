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
#ifndef GZ_WAVES_BUOYANCY_DISPLACEMENT_HH_
#define GZ_WAVES_BUOYANCY_DISPLACEMENT_HH_

#include <functional>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Geometry.hh>

#include "gz/sim/marked_shapes/MarkedShapes.hh"

/// \file
/// \brief How much of a marked shape is under a sloping water surface, and
/// where, and the plane that stands in for the surface around it: the
/// geometry of wave buoyancy, apart from any world, so it can be tested
/// alone. Internal to the wave buoyancy system.

namespace gz::sim::waves_buoyancy
{
/// \brief Whether the buoyancy floats a shape of this type: a box, a
/// cylinder, a sphere, a capsule, an ellipsoid or a cone, as the flat
/// buoyancy does. A mesh does not.
/// \param[in] _type The collision's geometry type.
/// \return True if it floats.
bool Floats(sdf::GeometryType _type);

/// \brief A marked shape's whole volume.
/// \param[in] _shape The shape.
/// \return m^3; 0 for a shape that does not float.
double Volume(const marked_shapes::Shape &_shape);

/// \brief A plane standing in for the water surface: a point on it and its
/// upward normal, world frame.
struct Surface
{
  /// \brief A point of the plane, m.
  public: math::Vector3d point;

  /// \brief Its upward unit normal.
  public: math::Vector3d normal{math::Vector3d::UnitZ};
};

/// \brief The height of the water surface over a world point (x, y), world
/// z.
using HeightAt = std::function<double(double _x, double _y)>;

/// \brief The plane fitted through the water surface around a shape: the
/// height at its centre and at each end of its horizontal footprint, along
/// its heading and across it, fitted by least squares. Over a shape longer
/// than the chop the plane averages it, where the surface at the centre
/// alone would pitch the shape with every ripple.
/// \param[in] _shape The shape.
/// \param[in] _worldPose The shape's world pose.
/// \param[in] _height The water surface's height.
/// \return The plane.
Surface FitSurface(const marked_shapes::Shape &_shape,
                   const math::Pose3d &_worldPose, const HeightAt &_height);

/// \brief A shape cut by a water surface: the part under it and the part
/// over it.
struct Displacement
{
  /// \brief m^3 under the water.
  public: double volume{0.0};

  /// \brief World position of the centre of that volume, the centre of
  /// buoyancy; the shape's centre when there is none.
  public: math::Vector3d centre;

  /// \brief m^3 over the water, in the air.
  public: double airVolume{0.0};

  /// \brief World position of the centre of the part in the air.
  public: math::Vector3d airCentre;
};

/// \brief The parts of a shape under and over the water, the surface taken
/// as a plane.
/// \param[in] _shape The shape.
/// \param[in] _worldPose The shape's world pose.
/// \param[in] _surface The water surface's plane.
/// \return What is under the water.
Displacement Below(const marked_shapes::Shape &_shape,
                   const math::Pose3d &_worldPose, const Surface &_surface);
}  // namespace gz::sim::waves_buoyancy

#endif  // GZ_WAVES_BUOYANCY_DISPLACEMENT_HH_
