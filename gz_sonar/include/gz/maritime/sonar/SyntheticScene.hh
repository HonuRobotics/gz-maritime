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
#ifndef GZ_MARITIME_SONAR_SYNTHETICSCENE_HH_
#define GZ_MARITIME_SONAR_SYNTHETICSCENE_HH_

#include <cstdint>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Rays.hh"

namespace gz::maritime::sonar
{
/// \brief A scene of analytic shapes with an exact ray caster: the stand-in
/// for gz-rendering when the engine runs on its own, in the tests and in the
/// contributor walkthrough. World frame, z up.
class SyntheticScene
{
  /// \brief A sphere.
  /// \param[in] _center The center.
  /// \param[in] _radius The radius, m.
  /// \param[in] _properties Its acoustic properties.
  /// \param[in] _label Its label.
  public: void AddSphere(const math::Vector3d &_center, double _radius,
              const AcousticProperties &_properties, std::uint32_t _label);

  /// \brief An infinite plane through a point.
  /// \param[in] _point A point on the plane.
  /// \param[in] _normal The plane's normal.
  /// \param[in] _properties Its acoustic properties.
  /// \param[in] _label Its label.
  public: void AddPlane(const math::Vector3d &_point,
              const math::Vector3d &_normal,
              const AcousticProperties &_properties, std::uint32_t _label);

  /// \brief A box aligned with the world axes.
  /// \param[in] _min The corner with the smallest coordinates.
  /// \param[in] _max The corner with the largest coordinates.
  /// \param[in] _properties Its acoustic properties.
  /// \param[in] _label Its label.
  public: void AddBox(const math::Vector3d &_min, const math::Vector3d &_max,
              const AcousticProperties &_properties, std::uint32_t _label);

  /// \brief A flat rectangle: a center, its normal, one in-plane axis, and
  /// its half sizes along that axis and the other in-plane axis.
  /// \param[in] _center The center.
  /// \param[in] _normal The normal.
  /// \param[in] _axis The in-plane axis _halfLength is along.
  /// \param[in] _halfLength Half the size along _axis, m.
  /// \param[in] _halfWidth Half the size along the other in-plane axis, m.
  /// \param[in] _properties Its acoustic properties.
  /// \param[in] _label Its label.
  public: void AddRectangle(const math::Vector3d &_center,
              const math::Vector3d &_normal, const math::Vector3d &_axis,
              double _halfLength, double _halfWidth,
              const AcousticProperties &_properties, std::uint32_t _label);

  /// \brief Cast every ray from a sensor and return the first hit of each.
  /// \param[in] _sensorPose The sensor's pose in the world.
  /// \param[in] _rays The fan, in the sensor frame.
  /// \param[in] _maxRange Ignore hits farther than this, m.
  /// \return One hit per ray, in the order of _rays.
  public: std::vector<RayHit> Cast(const math::Pose3d &_sensorPose,
              const std::vector<Ray> &_rays, double _maxRange) const;

  private: enum class Kind { kSphere, kPlane, kBox, kRectangle };

  private: struct Shape
  {
    Kind kind;
    math::Vector3d a;
    math::Vector3d b;
    math::Vector3d c;
    double r{0.0};
    double s{0.0};
    AcousticProperties properties;
    std::uint32_t label{0};
  };

  /// \brief Distance along a ray to a shape, and the normal there.
  /// \param[in] _shape The shape.
  /// \param[in] _origin The ray's origin.
  /// \param[in] _dir The ray's unit direction.
  /// \param[out] _t The distance to the hit.
  /// \param[out] _normal The shape's normal at the hit.
  /// \return True if the ray hits the shape.
  private: static bool Intersect(const Shape &_shape,
              const math::Vector3d &_origin, const math::Vector3d &_dir,
              double &_t, math::Vector3d &_normal);

  private: std::vector<Shape> shapes;
};
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_SYNTHETICSCENE_HH_
