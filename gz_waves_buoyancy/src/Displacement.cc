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
#include "Displacement.hh"

#include <algorithm>
#include <cmath>
#include <optional>

#include <gz/math/Box.hh>
#include <gz/math/Capsule.hh>
#include <gz/math/Cone.hh>
#include <gz/math/Cylinder.hh>
#include <gz/math/Ellipsoid.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Sphere.hh>

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

  /// \brief Call a function with the gz-math shape a marked shape is.
  /// \tparam F A callable taking any gz-math shape.
  /// \param[in] _shape The marked shape.
  /// \param[in] _f The function.
  /// \param[in] _otherwise What to return for a shape that does not float.
  /// \return What the function returns.
  template <typename F, typename R>
  R WithShape(const marked_shapes::Shape &_shape, F &&_f, R _otherwise)
  {
    const math::Vector3d &h = _shape.half;
    switch (_shape.type)
    {
      case sdf::GeometryType::BOX:
        return _f(math::Boxd(h * 2.0));
      case sdf::GeometryType::CYLINDER:
        return _f(math::Cylinderd(2.0 * h.Z(), h.X()));
      case sdf::GeometryType::SPHERE:
        return _f(math::Sphered(h.X()));
      case sdf::GeometryType::CAPSULE:
        // Its half height is half the length plus a radius.
        return _f(math::Capsuled(2.0 * (h.Z() - h.X()), h.X()));
      case sdf::GeometryType::ELLIPSOID:
        return _f(math::Ellipsoidd(h));
      case sdf::GeometryType::CONE:
        return _f(math::Coned(2.0 * h.Z(), h.X()));
      default:
        return _otherwise;
    }
  }

  /// \brief How far a shape reaches along a horizontal direction from its
  /// centre: its bounding box's half extents projected on it.
  /// \param[in] _half Half extents, shape frame.
  /// \param[in] _rot The shape's world orientation.
  /// \param[in] _dir A world direction.
  /// \return m.
  double Reach(const math::Vector3d &_half, const math::Quaterniond &_rot,
               const math::Vector3d &_dir)
  {
    return _half.X() * std::abs(_rot.RotateVector(math::Vector3d::UnitX)
                                    .Dot(_dir)) +
           _half.Y() * std::abs(_rot.RotateVector(math::Vector3d::UnitY)
                                    .Dot(_dir)) +
           _half.Z() * std::abs(_rot.RotateVector(math::Vector3d::UnitZ)
                                    .Dot(_dir));
  }
}

//////////////////////////////////////////////////
bool Floats(sdf::GeometryType _type)
{
  switch (_type)
  {
    case sdf::GeometryType::BOX:
    case sdf::GeometryType::CYLINDER:
    case sdf::GeometryType::SPHERE:
    case sdf::GeometryType::CAPSULE:
    case sdf::GeometryType::ELLIPSOID:
    case sdf::GeometryType::CONE:
      return true;
    default:
      return false;
  }
}

//////////////////////////////////////////////////
double Volume(const marked_shapes::Shape &_shape)
{
  return WithShape(_shape, [](const auto &_s) { return _s.Volume(); }, 0.0);
}

//////////////////////////////////////////////////
Surface FitSurface(const marked_shapes::Shape &_shape,
    const math::Pose3d &_worldPose, const HeightAt &_height)
{
  const math::Vector3d c = _worldPose.Pos();
  const math::Quaterniond &rot = _worldPose.Rot();

  // The shape's heading on the water: its x axis, or its y axis when x
  // stands upright, flattened.
  math::Vector3d along = rot.RotateVector(math::Vector3d::UnitX);
  along.Z(0.0);
  if (along.Length() < 1e-6)
  {
    along = rot.RotateVector(math::Vector3d::UnitY);
    along.Z(0.0);
  }
  along.Normalize();
  const math::Vector3d across = math::Vector3d::UnitZ.Cross(along);

  // Least squares through the centre and the four ends, symmetric about the
  // centre: the mean height, and the slope along each axis from its ends.
  const double h0 = _height(c.X(), c.Y());
  double sum = h0;
  int n = 1;
  math::Vector3d gradient;
  for (const math::Vector3d &dir : {along, across})
  {
    const double reach = Reach(_shape.half, rot, dir);
    if (reach < 1e-6)
      continue;
    const math::Vector3d a = c + dir * reach;
    const math::Vector3d b = c - dir * reach;
    const double ha = _height(a.X(), a.Y());
    const double hb = _height(b.X(), b.Y());
    sum += ha + hb;
    n += 2;
    gradient += dir * ((ha - hb) / (2.0 * reach));
  }

  Surface s;
  s.point = math::Vector3d(c.X(), c.Y(), sum / n);
  s.normal = math::Vector3d(-gradient.X(), -gradient.Y(), 1.0).Normalized();
  return s;
}

//////////////////////////////////////////////////
Displacement Below(const marked_shapes::Shape &_shape,
    const math::Pose3d &_worldPose, const Surface &_surface)
{
  Displacement out;
  out.centre = _worldPose.Pos();
  out.airCentre = _worldPose.Pos();
  const double whole = Volume(_shape);
  if (!(whole > 0.0))
    return out;

  // The whole shape's centre, in its frame: not its origin for a cone.
  const double above = 2.0 * _shape.half.Length() + 1.0;
  const math::Vector3d middle = WithShape(_shape,
      [&](const auto &_s)
      {
        return _s.CenterOfVolumeBelow(
            math::Planed(math::Vector3d::UnitZ, above))
            .value_or(math::Vector3d::Zero);
      }, math::Vector3d::Zero);
  out.airVolume = whole;
  out.airCentre = _worldPose.Pos() + _worldPose.Rot().RotateVector(middle);

  // The surface plane in the shape's frame: a shape point x is on it when
  // n . (p + R x - s) = 0, i.e. (R^T n) . x = n . (s - p).
  math::Vector3d normal =
      _worldPose.Rot().RotateVectorReverse(_surface.normal.Normalized());

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
  const double offset = normal.Dot(_worldPose.Rot().RotateVectorReverse(
      _surface.point - _worldPose.Pos()));
  const math::Planed plane(normal, offset);

  double volume{0.0};
  const auto centre = WithShape(_shape,
      [&](const auto &_s) { return Under(_s, plane, volume); },
      std::optional<math::Vector3d>());
  if (!centre || volume <= 0.0)
    return out;
  out.volume = std::min(volume, whole);
  out.centre = _worldPose.Pos() + _worldPose.Rot().RotateVector(*centre);
  out.airVolume = whole - out.volume;
  out.airCentre = out.airVolume > 1e-12 ?
      (out.airCentre * whole - out.centre * out.volume) / out.airVolume :
      out.centre;
  return out;
}
}  // namespace gz::sim::waves_buoyancy
