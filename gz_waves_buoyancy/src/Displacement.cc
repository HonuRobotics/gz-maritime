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
#include "gz/sim/waves_buoyancy/Displacement.hh"

#include <cmath>
#include <optional>

#include <gz/math/Box.hh>
#include <gz/math/Cylinder.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Sphere.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>

namespace gz::sim::waves_buoyancy
{
namespace
{
  /// \brief Below a plane in a gz-math shape's own frame.
  /// \tparam ShapeT A gz-math shape with VolumeBelow and CenterOfVolumeBelow.
  /// \param[in] _shape The shape.
  /// \param[in] _plane The plane, in the shape's frame.
  /// \param[out] _volume m^3 below it.
  /// \return The centre of that volume, in the shape's frame, if any.
  template <typename ShapeT>
  std::optional<math::Vector3d> Under(const ShapeT &_shape,
                                      const math::Planed &_plane,
                                      double &_volume)
  {
    _volume = _shape.VolumeBelow(_plane);
    if (_volume <= 0.0)
      return std::nullopt;
    return _shape.CenterOfVolumeBelow(_plane);
  }
}

//////////////////////////////////////////////////
double BuoyantShape::Volume() const
{
  switch (this->kind)
  {
    case Kind::kBox:
      return this->size.X() * this->size.Y() * this->size.Z();
    case Kind::kCylinder:
      return GZ_PI * this->radius * this->radius * this->length;
    case Kind::kSphere:
      return 4.0 / 3.0 * GZ_PI * std::pow(this->radius, 3);
  }
  return 0.0;
}

//////////////////////////////////////////////////
std::optional<BuoyantShape> ShapeFromGeometry(const sdf::Geometry &_geometry,
    const math::Pose3d &_pose)
{
  BuoyantShape s;
  s.pose = _pose;
  switch (_geometry.Type())
  {
    case sdf::GeometryType::BOX:
      s.kind = BuoyantShape::Kind::kBox;
      s.size = _geometry.BoxShape()->Size();
      return s;
    case sdf::GeometryType::CYLINDER:
      s.kind = BuoyantShape::Kind::kCylinder;
      s.radius = _geometry.CylinderShape()->Radius();
      s.length = _geometry.CylinderShape()->Length();
      return s;
    case sdf::GeometryType::SPHERE:
      s.kind = BuoyantShape::Kind::kSphere;
      s.radius = _geometry.SphereShape()->Radius();
      return s;
    default:
      return std::nullopt;
  }
}

//////////////////////////////////////////////////
Displacement Below(const BuoyantShape &_shape,
    const math::Pose3d &_worldPose,
    const math::Vector3d &_surface,
    const math::Vector3d &_normal)
{
  Displacement out;
  out.centre = _worldPose.Pos();
  const double whole = _shape.Volume();
  if (!(whole > 0.0))
    return out;

  // The surface plane in the shape's frame: a shape point x is on it when
  // n . (p + R x - s) = 0, i.e. (R^T n) . x = n . (s - p).
  math::Vector3d normal =
      _worldPose.Rot().RotateVectorReverse(_normal.Normalized());

  // gz-math's Box::CenterOfVolumeBelow returns a centre far outside the box
  // once the plane is within about 1e-9 rad of parallel to a face, and a
  // floating box on calm water settles into exactly that. Snap components
  // too small to matter, as gz_buoyancy does. Fixed upstream in gz-math#847.
  static constexpr double kParallelTolerance{1e-6};
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(normal[i]) < kParallelTolerance)
      normal[i] = 0.0;
  }
  normal.Normalize();
  const double offset = normal.Dot(
      _worldPose.Rot().RotateVectorReverse(_surface - _worldPose.Pos()));
  const math::Planed plane(normal, offset);

  double volume{0.0};
  std::optional<math::Vector3d> centre;
  switch (_shape.kind)
  {
    case BuoyantShape::Kind::kBox:
      centre = Under(math::Boxd(_shape.size), plane, volume);
      break;
    case BuoyantShape::Kind::kCylinder:
      centre = Under(math::Cylinderd(_shape.length, _shape.radius), plane,
                     volume);
      break;
    case BuoyantShape::Kind::kSphere:
      centre = Under(math::Sphered(_shape.radius), plane, volume);
      break;
  }
  if (!centre || volume <= 0.0)
    return out;
  out.volume = std::min(volume, whole);
  out.centre = _worldPose.Pos() + _worldPose.Rot().RotateVector(*centre);
  return out;
}
}  // namespace gz::sim::waves_buoyancy
