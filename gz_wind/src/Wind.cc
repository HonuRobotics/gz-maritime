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
#include "Wind.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/wind.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
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
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/Conversions.hh>

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief The attribute a collision is marked with. Namespaced, so SDFormat
  /// keeps it without knowing it.
  const std::string kMark{"gz:wind"};

  /// \brief The optional per shape drag coefficient attribute.
  const std::string kCdMark{"gz:wind_cd"};

  /// \brief Read a numeric Any as a double.
  /// \param[in] _v The value.
  /// \param[out] _out The number, when there is one.
  /// \return True if the value is numeric.
  bool ReadDouble(const msgs::Any &_v, double &_out)
  {
    switch (_v.type())
    {
      case msgs::Any::DOUBLE: _out = _v.double_value(); return true;
      case msgs::Any::INT32: _out = _v.int_value(); return true;
      default: return false;
    }
  }

  /// \brief The wind velocity, in the world frame (ENU), of a wind that
  /// blows from a direction.
  /// \param[in] _speed Speed, m/s.
  /// \param[in] _from Direction the wind comes from, degrees clockwise from
  /// north, as in weather reports.
  /// \param[in] _vertical Vertical component, m/s.
  /// \return The velocity the air moves with.
  math::Vector3d Velocity(double _speed, double _from, double _vertical)
  {
    const double a = GZ_DTOR(_from);
    return {-_speed * std::sin(a), -_speed * std::cos(a), _vertical};
  }

  /// \brief Projected area of a box along each of its axes.
  /// \param[in] _size Box size.
  /// \return The area of the face normal to each axis.
  math::Vector3d BoxAreas(const math::Vector3d &_size)
  {
    return {_size.Y() * _size.Z(), _size.X() * _size.Z(),
            _size.X() * _size.Y()};
  }
}

/// \brief One marked shape of a link, resolved once when the link is found.
struct WindShape
{
  /// \brief Pose of the shape in the link frame.
  public: math::Pose3d pose;

  /// \brief Half extents of the shape's bounding box in the shape frame.
  public: math::Vector3d half;

  /// \brief Projected area of the whole shape along each shape axis, m^2.
  public: math::Vector3d area;

  /// \brief Drag coefficient.
  public: double cd{1.0};
};

class gz::sim::maritime::WindPrivate
{
  /// \brief Find the marked shapes of every link the system has not seen.
  /// \param[in] _ecm The entity component manager.
  public: void FindShapes(EntityComponentManager &_ecm);

  /// \brief Resolve one collision into a wind shape.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _collision The collision entity.
  /// \param[out] _shape The resolved shape.
  /// \return True if the collision is marked and its geometry is supported.
  public: bool Resolve(const EntityComponentManager &_ecm,
                       const Entity _collision, WindShape &_shape) const;

  /// \brief Marked shapes per link.
  public: std::unordered_map<Entity, std::vector<WindShape>> links;

  /// \brief The wind entity of the world.
  public: Entity windEntity{kNullEntity};

  /// \brief Air density, kg/m^3.
  public: double airDensity{1.225};

  /// \brief World z of the waterline.
  public: double waterLevel{0.0};

  /// \brief Drag coefficient for shapes that do not set their own.
  public: double defaultCd{1.0};

  /// \brief Scan every link instead of only the new ones on the next update.
  public: bool rescan{true};

  /// \brief set_parameters service handler, on a transport thread.
  /// \param[in] _req Keys `speed` and `direction`, either or both.
  /// \param[out] _rep True if a key was applied.
  /// \return True, the service always answers.
  public: bool OnSetParameters(const msgs::Param &_req, msgs::Boolean &_rep);

  /// \brief Write the wind into the wind entity.
  /// \param[in] _ecm The entity component manager.
  public: void WriteWind(EntityComponentManager &_ecm);

  /// \brief Wind speed, m/s.
  public: double speed{0.0};

  /// \brief Direction the wind comes from, degrees clockwise from north.
  public: double direction{0.0};

  /// \brief Vertical component, m/s, kept from the world's <wind>.
  public: double vertical{0.0};

  /// \brief Whether the wind changed and must be written.
  public: bool dirty{false};

  /// \brief A speed queued by the service.
  public: std::optional<double> pendingSpeed;

  /// \brief A direction queued by the service.
  public: std::optional<double> pendingDirection;

  /// \brief Guards the pending values across the transport and ECM threads.
  public: std::mutex mutex;

  /// \brief Transport node for the service and the ground truth.
  public: transport::Node node;

  /// \brief Ground truth publisher.
  public: transport::Node::Publisher windPub;

  /// \brief Ground truth publication period, simulation time.
  public: std::chrono::steady_clock::duration publishPeriod{
      std::chrono::milliseconds(100)};

  /// \brief Simulation time of the last publication.
  public: std::optional<std::chrono::steady_clock::duration> lastPublish;
};

//////////////////////////////////////////////////
bool WindPrivate::OnSetParameters(const msgs::Param &_req,
    msgs::Boolean &_rep)
{
  bool matched{false};
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    for (const auto &[key, value] : _req.params())
    {
      double d{0.0};
      if (!ReadDouble(value, d))
      {
        gzwarn << "Wind: set_parameters key '" << key
               << "' is not a number, ignored\n";
        continue;
      }
      if (key == "speed" && d >= 0.0)
      {
        this->pendingSpeed = d;
        matched = true;
      }
      else if (key == "direction")
      {
        this->pendingDirection = d;
        matched = true;
      }
      else
      {
        gzwarn << "Wind: set_parameters key '" << key
               << "' is unknown or out of range, ignored\n";
      }
    }
  }
  _rep.set_data(matched);
  return true;
}

//////////////////////////////////////////////////
void WindPrivate::WriteWind(EntityComponentManager &_ecm)
{
  if (kNullEntity == this->windEntity)
    return;
  const math::Vector3d vel = Velocity(this->speed, this->direction,
      this->vertical);
  auto *comp = _ecm.Component<components::WorldLinearVelocity>(
      this->windEntity);
  if (nullptr == comp)
  {
    _ecm.CreateComponent(this->windEntity,
        components::WorldLinearVelocity(vel));
  }
  else
  {
    comp->Data() = vel;
  }
}

//////////////////////////////////////////////////
bool WindPrivate::Resolve(const EntityComponentManager &_ecm,
    const Entity _collision, WindShape &_shape) const
{
  const auto *coll = _ecm.Component<components::CollisionElement>(_collision);
  if (nullptr == coll || nullptr == coll->Data().Element())
    return false;

  const auto elem = coll->Data().Element();
  if (!elem->HasAttribute(kMark) ||
      elem->GetAttribute(kMark)->GetAsString() != "true")
  {
    return false;
  }

  _shape.cd = this->defaultCd;
  if (elem->HasAttribute(kCdMark))
  {
    double cd{0.0};
    if (elem->GetAttribute(kCdMark)->Get<double>(cd) && cd >= 0.0)
      _shape.cd = cd;
    else
      gzwarn << "Ignoring invalid " << kCdMark << " on a wind shape\n";
  }

  _shape.pose = coll->Data().RawPose();

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
      // A mesh is treated as its bounding box.
      const sdf::Mesh *meshSdf = geom->MeshShape();
      const std::string file = asFullPath(meshSdf->Uri(),
          meshSdf->FilePath());
      const common::Mesh *mesh = common::MeshManager::Instance()->Load(file);
      if (nullptr == mesh)
      {
        gzwarn << "Cannot load wind shape mesh [" << file << "]\n";
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
      gzwarn << "Unsupported geometry on a wind shape, ignoring it\n";
      return false;
  }
  return true;
}

//////////////////////////////////////////////////
void WindPrivate::FindShapes(EntityComponentManager &_ecm)
{
  // Collect first: creating components while iterating a view is unsafe.
  std::vector<Entity> candidates;
  auto collect = [&](const Entity &_link, const components::Link *) -> bool
  {
    candidates.push_back(_link);
    return true;
  };

  if (this->rescan)
  {
    this->links.clear();
    _ecm.Each<components::Link>(collect);
    this->rescan = false;
  }
  else
  {
    _ecm.EachNew<components::Link>(collect);
  }

  for (const Entity link : candidates)
  {
    std::vector<WindShape> shapes;
    for (const Entity collision : _ecm.ChildrenByComponents(link,
        components::Collision()))
    {
      WindShape shape;
      if (this->Resolve(_ecm, collision, shape))
        shapes.push_back(shape);
    }
    if (shapes.empty())
    {
      this->links.erase(link);
      continue;
    }
    // The point velocities below need the velocity components.
    Link(link).EnableVelocityChecks(_ecm);
    this->links[link] = std::move(shapes);
  }
}

//////////////////////////////////////////////////
Wind::Wind()
  : dataPtr(std::make_unique<WindPrivate>())
{
}

//////////////////////////////////////////////////
Wind::~Wind() = default;

//////////////////////////////////////////////////
void Wind::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->airDensity = _sdf->Get<double>("air_density",
      this->dataPtr->airDensity).first;
  this->dataPtr->waterLevel = _sdf->Get<double>("water_level",
      this->dataPtr->waterLevel).first;
  this->dataPtr->defaultCd = _sdf->Get<double>("default_drag_coefficient",
      this->dataPtr->defaultCd).first;

  if (this->dataPtr->airDensity <= 0.0 || this->dataPtr->defaultCd < 0.0)
  {
    gzerr << "Wind: <air_density> must be positive and "
          << "<default_drag_coefficient> cannot be negative\n";
  }

  this->dataPtr->windEntity = _ecm.EntityByComponents(components::Wind());
  if (kNullEntity == this->dataPtr->windEntity)
  {
    gzwarn << "Wind: the world has no wind entity, nothing will be pushed\n";
    return;
  }

  // The world's <wind> is the starting point; <speed> and <direction>
  // override it. Without either, the wind is the world's until the service
  // changes it.
  math::Vector3d start = math::Vector3d::Zero;
  if (const auto *vel = _ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->windEntity))
  {
    start = vel->Data();
  }
  this->dataPtr->vertical = start.Z();
  this->dataPtr->speed = std::hypot(start.X(), start.Y());
  this->dataPtr->direction = this->dataPtr->speed > 0.0 ?
      GZ_RTOD(std::atan2(-start.X(), -start.Y())) : 0.0;
  if (_sdf->HasElement("speed"))
  {
    this->dataPtr->speed = std::max(0.0, _sdf->Get<double>("speed"));
    this->dataPtr->dirty = true;
  }
  if (_sdf->HasElement("direction"))
  {
    this->dataPtr->direction = _sdf->Get<double>("direction");
    this->dataPtr->dirty = true;
  }
  const double rate = _sdf->Get<double>("publish_rate", 10.0).first;
  if (rate > 0.0)
  {
    this->dataPtr->publishPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
  }

  // The world's name scopes the service and the ground truth, like the
  // waves' set_parameters.
  std::string worldName{"default"};
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  const std::string prefix = "/world/" + worldName + "/wind";
  if (!this->dataPtr->node.Advertise(prefix + "/set_parameters",
      &WindPrivate::OnSetParameters, this->dataPtr.get()))
  {
    gzerr << "Wind: cannot advertise " << prefix << "/set_parameters\n";
  }
  this->dataPtr->windPub =
      this->dataPtr->node.Advertise<msgs::Wind>(prefix + "_info");
}

//////////////////////////////////////////////////
void Wind::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  this->dataPtr->FindShapes(_ecm);

  // Apply what the service queued, then write the wind where Gazebo's rotor,
  // wing and air speed systems read it.
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (this->dataPtr->pendingSpeed)
    {
      this->dataPtr->speed = *this->dataPtr->pendingSpeed;
      this->dataPtr->dirty = true;
    }
    if (this->dataPtr->pendingDirection)
    {
      this->dataPtr->direction = *this->dataPtr->pendingDirection;
      this->dataPtr->dirty = true;
    }
    this->dataPtr->pendingSpeed.reset();
    this->dataPtr->pendingDirection.reset();
  }
  if (this->dataPtr->dirty)
  {
    this->dataPtr->WriteWind(_ecm);
    this->dataPtr->dirty = false;
  }

  if (_info.paused)
    return;

  math::Vector3d wind = math::Vector3d::Zero;
  if (const auto *vel = _ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->windEntity))
  {
    wind = vel->Data();
  }

  // Ground truth, at the publish rate in simulation time.
  if (!this->dataPtr->lastPublish ||
      _info.simTime - *this->dataPtr->lastPublish >=
      this->dataPtr->publishPeriod)
  {
    msgs::Wind msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));
    msgs::Set(msg.mutable_linear_velocity(), wind);
    msg.set_enable_wind(true);
    this->dataPtr->windPub.Publish(msg);
    this->dataPtr->lastPublish = _info.simTime;
  }

  for (auto it = this->dataPtr->links.begin();
       it != this->dataPtr->links.end();)
  {
    if (!_ecm.HasEntity(it->first))
    {
      it = this->dataPtr->links.erase(it);
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

    for (const WindShape &shape : it->second)
    {
      const math::Pose3d worldPose = *linkPose * shape.pose;
      const math::Matrix3d rot(worldPose.Rot());

      // Vertical half extent of the shape's bounding box in the world.
      const double hz = std::abs(rot(2, 0)) * shape.half.X() +
                        std::abs(rot(2, 1)) * shape.half.Y() +
                        std::abs(rot(2, 2)) * shape.half.Z();
      const double top = worldPose.Pos().Z() + hz;
      const double bottom = worldPose.Pos().Z() - hz;
      if (top <= this->dataPtr->waterLevel)
        continue;

      // The part above the waterline: its share of the height, and its centre.
      const double exposedBottom = std::max(bottom, this->dataPtr->waterLevel);
      const double fraction = hz > 0.0 ?
          std::clamp((top - exposedBottom) / (2.0 * hz), 0.0, 1.0) : 1.0;
      math::Vector3d centre = worldPose.Pos();
      centre.Z((top + exposedBottom) / 2.0);

      // Relative wind at the centre of the exposed part, in the shape frame.
      const math::Vector3d offset = linkPose->Rot().RotateVectorReverse(
          centre - linkPose->Pos());
      const auto pointVel = link.WorldLinearVelocity(_ecm, offset);
      const math::Vector3d rel = worldPose.Rot().RotateVectorReverse(
          wind - pointVel.value_or(math::Vector3d::Zero));

      // Quadratic drag per shape axis. The waterline cuts the areas the
      // horizontal wind sees; the plan area is left whole.
      const math::Vector3d area(shape.area.X() * fraction,
                                shape.area.Y() * fraction,
                                shape.area.Z());
      const double q = 0.5 * this->dataPtr->airDensity * shape.cd;
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
void Wind::Reset(const UpdateInfo &, EntityComponentManager &)
{
  this->dataPtr->links.clear();
  this->dataPtr->rescan = true;
}

GZ_ADD_PLUGIN(Wind,
              System,
              Wind::ISystemConfigure,
              Wind::ISystemPreUpdate,
              Wind::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(Wind, "gz::sim::maritime::Wind")
