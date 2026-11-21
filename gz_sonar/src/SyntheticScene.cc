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
#include "gz/maritime/sonar/SyntheticScene.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace gz::maritime::sonar
{
namespace
{
constexpr double kEps = 1e-9;
}

void SyntheticScene::AddSphere(const math::Vector3d &_center, double _radius,
  const AcousticProperties &_properties, std::uint32_t _label)
{
  Shape s{Kind::kSphere, _center, {}, {}, _radius, 0.0, _properties, _label};
  this->shapes.push_back(s);
}

void SyntheticScene::AddPlane(const math::Vector3d &_point,
  const math::Vector3d &_normal, const AcousticProperties &_properties,
  std::uint32_t _label)
{
  Shape s{Kind::kPlane, _point, _normal.Normalized(), {}, 0.0, 0.0,
    _properties, _label};
  this->shapes.push_back(s);
}

void SyntheticScene::AddBox(const math::Vector3d &_min,
  const math::Vector3d &_max, const AcousticProperties &_properties,
  std::uint32_t _label)
{
  Shape s{Kind::kBox, _min, _max, {}, 0.0, 0.0, _properties, _label};
  this->shapes.push_back(s);
}

void SyntheticScene::AddRectangle(const math::Vector3d &_center,
  const math::Vector3d &_normal, const math::Vector3d &_axis,
  double _halfLength, double _halfWidth,
  const AcousticProperties &_properties, std::uint32_t _label)
{
  const math::Vector3d n = _normal.Normalized();
  // Keep the axis in the plane.
  const math::Vector3d u = (_axis - n * _axis.Dot(n)).Normalized();
  Shape s{Kind::kRectangle, _center, n, u, _halfLength, _halfWidth,
    _properties, _label};
  this->shapes.push_back(s);
}

bool SyntheticScene::Intersect(const Shape &_shape,
  const math::Vector3d &_origin, const math::Vector3d &_dir, double &_t,
  math::Vector3d &_normal)
{
  switch (_shape.kind)
  {
    case Kind::kSphere:
    {
      const math::Vector3d oc = _origin - _shape.a;
      const double b = oc.Dot(_dir);
      const double c = oc.Dot(oc) - _shape.r * _shape.r;
      const double disc = b * b - c;
      if (disc < 0.0)
        return false;
      const double sq = std::sqrt(disc);
      double t = -b - sq;
      if (t < kEps)
        t = -b + sq;
      if (t < kEps)
        return false;
      _t = t;
      _normal = (_origin + _dir * t - _shape.a).Normalized();
      return true;
    }
    case Kind::kPlane:
    case Kind::kRectangle:
    {
      const double denom = _dir.Dot(_shape.b);
      if (std::abs(denom) < kEps)
        return false;
      const double t = (_shape.a - _origin).Dot(_shape.b) / denom;
      if (t < kEps)
        return false;
      if (_shape.kind == Kind::kRectangle)
      {
        const math::Vector3d p = _origin + _dir * t - _shape.a;
        const math::Vector3d v = _shape.b.Cross(_shape.c);
        if (std::abs(p.Dot(_shape.c)) > _shape.r ||
            std::abs(p.Dot(v)) > _shape.s)
        {
          return false;
        }
      }
      _t = t;
      _normal = _shape.b;
      return true;
    }
    case Kind::kBox:
    {
      // Slabs.
      double tNear = -std::numeric_limits<double>::infinity();
      double tFar = std::numeric_limits<double>::infinity();
      int axisNear = 0;
      double signNear = 1.0;
      for (int i = 0; i < 3; ++i)
      {
        const double o = _origin[i];
        const double d = _dir[i];
        const double lo = _shape.a[i];
        const double hi = _shape.b[i];
        if (std::abs(d) < kEps)
        {
          if (o < lo || o > hi)
            return false;
          continue;
        }
        double t1 = (lo - o) / d;
        double t2 = (hi - o) / d;
        double sign = -1.0;
        if (t1 > t2)
        {
          std::swap(t1, t2);
          sign = 1.0;
        }
        if (t1 > tNear)
        {
          tNear = t1;
          axisNear = i;
          signNear = sign;
        }
        tFar = std::min(tFar, t2);
        if (tNear > tFar)
          return false;
      }
      if (tNear < kEps)
        return false;
      _t = tNear;
      _normal = math::Vector3d::Zero;
      _normal[axisNear] = signNear;
      return true;
    }
  }
  return false;
}

std::vector<RayHit> SyntheticScene::Cast(const math::Pose3d &_sensorPose,
  const std::vector<Ray> &_rays, double _maxRange) const
{
  std::vector<RayHit> hits;
  hits.reserve(_rays.size());
  const math::Vector3d origin = _sensorPose.Pos();
  for (const Ray &ray : _rays)
  {
    RayHit h;
    h.ray = ray;
    const math::Vector3d dir =
      _sensorPose.Rot().RotateVector(ray.Direction());
    double best = _maxRange;
    for (const Shape &s : this->shapes)
    {
      double t;
      math::Vector3d n;
      if (Intersect(s, origin, dir, t, n) && t < best)
      {
        best = t;
        h.hit = true;
        h.range = t;
        const double cosInc = std::min(1.0, std::abs(n.Dot(dir)));
        h.incidence = std::acos(cosInc);
        h.properties = s.properties;
        h.label = s.label;
      }
    }
    hits.push_back(h);
  }
  return hits;
}
}  // namespace gz::maritime::sonar
