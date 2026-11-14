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
#ifndef GZ_SIM_WAVES_BUOYANCY_DISPLACEMENT_HH_
#define GZ_SIM_WAVES_BUOYANCY_DISPLACEMENT_HH_

#include <optional>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Geometry.hh>

/// \file
/// \brief How much of a shape is under a sloping water surface, and where:
/// the geometry of wave buoyancy, apart from any world, so it can be tested
/// alone.

namespace gz::sim::waves_buoyancy
{
/// \brief A shape a collision marks `gz:buoyancy="true"`: a box, a cylinder
/// or a sphere, where it sits on its link.
struct BuoyantShape
{
  /// \brief What it is.
  enum class Kind
  {
    /// \brief A box of `size`.
    kBox,

    /// \brief A cylinder of `radius` and `length` along its z.
    kCylinder,

    /// \brief A sphere of `radius`.
    kSphere
  };

  /// \brief What it is.
  public: Kind kind{Kind::kBox};

  /// \brief m, a box's sides.
  public: math::Vector3d size;

  /// \brief m, a cylinder's or a sphere's radius.
  public: double radius{0.0};

  /// \brief m, a cylinder's length.
  public: double length{0.0};

  /// \brief Its pose in its link's frame.
  public: math::Pose3d pose;

  /// \brief Its whole volume.
  /// \return m^3.
  public: double Volume() const;
};

/// \brief A shape from a collision's geometry.
/// \param[in] _geometry The geometry.
/// \param[in] _pose The collision's pose in its link's frame.
/// \return The shape, or nothing for a geometry other than a box, a
/// cylinder or a sphere.
std::optional<BuoyantShape> ShapeFromGeometry(const sdf::Geometry &_geometry,
                                              const math::Pose3d &_pose);

/// \brief The part of a shape under a water surface.
struct Displacement
{
  /// \brief m^3 under the water.
  public: double volume{0.0};

  /// \brief World position of the centre of that volume, the centre of
  /// buoyancy; the shape's centre when there is none.
  public: math::Vector3d centre;
};

/// \brief The part of a shape under the water, the surface taken as the
/// plane tangent to it above the shape: a point on it and its normal. For a
/// shape short against the wavelength the tangent plane is the surface
/// there; a long hull is better split into several shapes.
/// \param[in] _shape The shape.
/// \param[in] _worldPose The shape's world pose.
/// \param[in] _surface A point of the water surface, world frame.
/// \param[in] _normal The surface's upward normal there, world frame.
/// \return What is under the water.
Displacement Below(const BuoyantShape &_shape,
                   const math::Pose3d &_worldPose,
                   const math::Vector3d &_surface,
                   const math::Vector3d &_normal);
}  // namespace gz::sim::waves_buoyancy

#endif  // GZ_SIM_WAVES_BUOYANCY_DISPLACEMENT_HH_
