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
#include "gz/sim/marked_shapes/MarkedShapes.hh"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Matrix3.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Sphere.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>

namespace gz::sim::marked_shapes
{
namespace
{
/// \brief Projected area of a box along each of its axes.
/// \param[in] _size Box size.
/// \return The area of the face normal to each axis.
math::Vector3d BoxAreas(const math::Vector3d &_size)
{
  return {_size.Y() * _size.Z(), _size.X() * _size.Z(),
          _size.X() * _size.Y()};
}
}  // namespace

//////////////////////////////////////////////////
Part Cut(const math::Pose3d &_worldPose, const math::Vector3d &_half,
         double _waterLevel, Side _side)
{
  const math::Matrix3d rot(_worldPose.Rot());

  // Vertical half extent of the shape's bounding box in the world.
  const double hz = std::abs(rot(2, 0)) * _half.X() +
                    std::abs(rot(2, 1)) * _half.Y() +
                    std::abs(rot(2, 2)) * _half.Z();
  const double top = _worldPose.Pos().Z() + hz;
  const double bottom = _worldPose.Pos().Z() - hz;

  Part part;
  double lo{bottom};
  double hi{top};
  if (Side::kAbove == _side)
  {
    if (top <= _waterLevel)
      return part;
    lo = std::max(bottom, _waterLevel);
  }
  else
  {
    if (bottom >= _waterLevel)
      return part;
    hi = std::min(top, _waterLevel);
  }

  // That part's share of the height, and its centre.
  part.any = true;
  part.fraction = hz > 0.0 ?
      std::clamp((hi - lo) / (2.0 * hz), 0.0, 1.0) : 1.0;
  part.centre = _worldPose.Pos();
  part.centre.Z((hi + lo) / 2.0);
  return part;
}

//////////////////////////////////////////////////
bool Resolve(const EntityComponentManager &_ecm, const Entity _collision,
             const std::string &_mark, const std::string &_cdMark,
             double _defaultCd, Shape &_shape)
{
  const auto *coll = _ecm.Component<components::CollisionElement>(_collision);
  if (nullptr == coll || nullptr == coll->Data().Element())
    return false;

  // Read as a bool, like the buoyancy mark, so "1" marks a shape too.
  const auto elem = coll->Data().Element();
  bool marked{false};
  if (!elem->HasAttribute(_mark) ||
      !elem->GetAttribute(_mark)->Get<bool>(marked) || !marked)
  {
    return false;
  }

  _shape.cd = _defaultCd;
  if (elem->HasAttribute(_cdMark))
  {
    double cd{0.0};
    if (elem->GetAttribute(_cdMark)->Get<double>(cd) && cd >= 0.0)
      _shape.cd = cd;
    else
      gzwarn << "Ignoring invalid " << _cdMark << " on a marked shape\n";
  }

  // The entity's pose is already resolved against any relative_to frame;
  // the raw pose is not.
  const auto *pose = _ecm.Component<components::Pose>(_collision);
  _shape.pose = nullptr != pose ? pose->Data() : coll->Data().RawPose();

  const sdf::Geometry *geom = coll->Data().Geom();
  if (nullptr == geom)
    return false;

  switch (geom->Type())
  {
    case sdf::GeometryType::BOX:
    {
      const math::Vector3d s = geom->BoxShape()->Size();
      _shape.half = s / 2.0;
      _shape.area = BoxAreas(s);
      break;
    }
    case sdf::GeometryType::CYLINDER:
    {
      const double r = geom->CylinderShape()->Radius();
      const double l = geom->CylinderShape()->Length();
      _shape.half.Set(r, r, l / 2.0);
      _shape.area.Set(2.0 * r * l, 2.0 * r * l, GZ_PI * r * r);
      break;
    }
    case sdf::GeometryType::SPHERE:
    {
      const double r = geom->SphereShape()->Radius();
      _shape.half.Set(r, r, r);
      _shape.area.Set(GZ_PI * r * r, GZ_PI * r * r, GZ_PI * r * r);
      break;
    }
    case sdf::GeometryType::CAPSULE:
    {
      const double r = geom->CapsuleShape()->Radius();
      const double l = geom->CapsuleShape()->Length();
      _shape.half.Set(r, r, l / 2.0 + r);
      const double side = 2.0 * r * l + GZ_PI * r * r;
      _shape.area.Set(side, side, GZ_PI * r * r);
      break;
    }
    case sdf::GeometryType::ELLIPSOID:
    {
      const math::Vector3d r = geom->EllipsoidShape()->Radii();
      _shape.half = r;
      _shape.area.Set(GZ_PI * r.Y() * r.Z(), GZ_PI * r.X() * r.Z(),
                      GZ_PI * r.X() * r.Y());
      break;
    }
    case sdf::GeometryType::MESH:
    {
      // A mesh is treated as its bounding box. loadMesh resolves model://
      // and Fuel URIs the way the rest of gz-sim does.
      const sdf::Mesh *meshSdf = geom->MeshShape();
      const common::Mesh *mesh = loadMesh(*meshSdf);
      if (nullptr == mesh)
      {
        gzwarn << "Cannot load marked shape mesh [" << meshSdf->Uri()
               << "]\n";
        return false;
      }
      math::Vector3d min;
      math::Vector3d max;
      math::Vector3d center;
      mesh->AABB(center, min, max);
      const math::Vector3d s = (max - min) * meshSdf->Scale();
      _shape.pose.Pos() += _shape.pose.Rot().RotateVector(
          center * meshSdf->Scale());
      _shape.half = s / 2.0;
      _shape.area = BoxAreas(s);
      break;
    }
    default:
      gzwarn << "Unsupported geometry on a marked shape, ignoring it\n";
      return false;
  }
  return true;
}

//////////////////////////////////////////////////
MarkedLinks::MarkedLinks(std::string _mark, std::string _cdMark)
  : mark(std::move(_mark)), cdMark(std::move(_cdMark))
{
}

//////////////////////////////////////////////////
void MarkedLinks::SetDefaultCd(double _cd)
{
  this->defaultCd = _cd;
}

//////////////////////////////////////////////////
void MarkedLinks::Find(EntityComponentManager &_ecm)
{
  std::vector<Entity> candidates;
  candidates.swap(this->newLinks);
  if (this->rescan)
  {
    // Collect first: creating components while iterating a view is unsafe.
    this->links.clear();
    candidates.clear();
    _ecm.Each<components::Link>(
        [&](const Entity &_link, const components::Link *) -> bool
        {
          candidates.push_back(_link);
          return true;
        });
    this->rescan = false;
  }

  for (const Entity link : candidates)
  {
    if (!_ecm.HasEntity(link))
      continue;
    std::vector<Shape> shapes;
    for (const Entity collision : _ecm.ChildrenByComponents(link,
        components::Collision()))
    {
      Shape shape;
      if (Resolve(_ecm, collision, this->mark, this->cdMark,
                  this->defaultCd, shape))
      {
        shapes.push_back(shape);
      }
    }
    if (shapes.empty())
    {
      this->links.erase(link);
      continue;
    }
    // The point velocities in ApplyDrag need the velocity components.
    Link(link).EnableVelocityChecks(_ecm);
    this->links[link] = std::move(shapes);
  }
}

//////////////////////////////////////////////////
void MarkedLinks::NoteNew(const EntityComponentManager &_ecm)
{
  _ecm.EachNew<components::Link>(
      [&](const Entity &_link, const components::Link *) -> bool
      {
        this->newLinks.push_back(_link);
        return true;
      });
}

//////////////////////////////////////////////////
void MarkedLinks::Reset()
{
  this->links.clear();
  this->newLinks.clear();
  this->rescan = true;
}

//////////////////////////////////////////////////
void MarkedLinks::ApplyDrag(const UpdateInfo &_info,
    EntityComponentManager &_ecm, Side _side, double _waterLevel,
    double _density, const FluidVelocity &_fluid)
{
  for (auto it = this->links.begin(); it != this->links.end();)
  {
    if (!_ecm.HasEntity(it->first))
    {
      it = this->links.erase(it);
      continue;
    }

    Link link(it->first);
    const auto linkPose = link.WorldPose(_ecm);
    const auto *inertial = _ecm.Component<components::Inertial>(it->first);
    if (!linkPose || nullptr == inertial)
    {
      ++it;
      continue;
    }
    // AddWorldForce takes the point relative to the centre of mass.
    const math::Vector3d com = inertial->Data().Pose().Pos();

    for (const Shape &shape : it->second)
    {
      const math::Pose3d worldPose = *linkPose * shape.pose;
      const Part part = Cut(worldPose, shape.half, _waterLevel, _side);
      if (!part.any)
        continue;

      // The fluid at the part's centre, relative to the shape, in the shape
      // frame.
      const math::Vector3d offset = linkPose->Rot().RotateVectorReverse(
          part.centre - linkPose->Pos());
      const auto pointVel = link.WorldLinearVelocity(_ecm, offset);
      const math::Vector3d fluid = _fluid(part.centre, _info.simTime);
      const math::Vector3d rel = worldPose.Rot().RotateVectorReverse(
          fluid - pointVel.value_or(math::Vector3d::Zero));

      // Quadratic drag per shape axis. The water level cuts the areas a
      // horizontal flow sees; the plan area is left whole.
      const math::Vector3d area(shape.area.X() * part.fraction,
                                shape.area.Y() * part.fraction,
                                shape.area.Z());
      const double q = 0.5 * _density * shape.cd;
      const math::Vector3d force(
          q * area.X() * std::abs(rel.X()) * rel.X(),
          q * area.Y() * std::abs(rel.Y()) * rel.Y(),
          q * area.Z() * std::abs(rel.Z()) * rel.Z());

      link.AddWorldForce(_ecm, worldPose.Rot().RotateVector(force),
                         offset - com);
    }
    ++it;
  }
}

//////////////////////////////////////////////////
const std::unordered_map<Entity, std::vector<Shape>> &MarkedLinks::Links()
    const
{
  return this->links;
}
}  // namespace gz::sim::marked_shapes
