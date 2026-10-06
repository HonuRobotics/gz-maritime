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
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/msgs/param.pb.h>
#include <gz/msgs/wind.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Angle.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
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

#include <gz/sim/Conversions.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/Wind.hh>

#include "gz/sim/components/Windfield.hh"
#include "gz/sim/wind/Windfield.hh"
#include "gz/sim/wind/WindSampler.hh"

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

  /// \brief Every parameter name the recipe takes.
  std::vector<std::string> ParameterNames()
  {
    std::vector<std::string> names{"seed"};
#define GZ_WIND_NAME(m, name) names.emplace_back(name);
    GZ_WIND_PARAM_TABLE(GZ_WIND_NAME)
#undef GZ_WIND_NAME
    return names;
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
  /// \brief Handler of the wind topic, on a transport thread.
  /// \param[in] _msg Parameter names and values.
  public: void OnSet(const msgs::Param &_msg);

  /// \brief Set one parameter of the recipe, drawing a seed for a 0 seed.
  /// \param[in] _name Parameter name.
  /// \param[in] _value Value.
  /// \return False if the name is unknown or the value out of range.
  public: bool Set(const std::string &_name, double _value);

  /// \brief Write the recipe into the world's Windfield component.
  /// \param[in] _ecm The entity component manager.
  public: void WriteRecipe(EntityComponentManager &_ecm);

  /// \brief Find the marked shapes of every link the system has not seen.
  /// \param[in] _ecm The entity component manager.
  public: void FindShapes(EntityComponentManager &_ecm);

  /// \brief Resolve one collision into a wind shape.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _collision The collision entity.
  /// \param[out] _shape The shape, when the collision is marked.
  /// \return True if the collision is a marked shape.
  public: bool Resolve(const EntityComponentManager &_ecm,
                       const Entity _collision, WindShape &_shape) const;

  /// \brief Push on the marked shapes of every link.
  /// \param[in] _info Update info.
  /// \param[in] _ecm The entity component manager.
  public: void ApplyWindage(const UpdateInfo &_info,
                            EntityComponentManager &_ecm);

  /// \brief Marked shapes per link.
  public: std::unordered_map<Entity, std::vector<WindShape>> links;

  /// \brief Scan every link instead of only the new ones on the next update.
  public: bool rescan{true};

  /// \brief Air density, kg/m^3.
  public: double airDensity{1.225};

  /// \brief Drag coefficient for shapes that do not set their own.
  public: double defaultCd{1.0};

  /// \brief The recipe this system owns.
  public: wind::WindfieldData recipe;

  /// \brief Samples the recipe, for the wind entity and the ground truth.
  public: wind::WindSampler sampler;

  /// \brief The world entity.
  public: Entity worldEntity{kNullEntity};

  /// \brief The world's wind entity.
  public: Entity windEntity{kNullEntity};

  /// \brief Whether the wind entity must be rewritten.
  public: bool dirty{true};

  /// \brief Parameters queued by the topic, applied at the next step.
  public: std::vector<std::pair<std::string, double>> pending;

  /// \brief Guards the queue across the transport and ECM threads.
  public: std::mutex mutex;

  /// \brief Transport node for the wind topic and the ground truth.
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
void WindPrivate::OnSet(const msgs::Param &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &[key, value] : _msg.params())
  {
    double d{0.0};
    if (!ReadDouble(value, d))
    {
      gzwarn << "Wind: key '" << key << "' is not a number, ignored\n";
      continue;
    }
    this->pending.emplace_back(key, d);
  }
}

//////////////////////////////////////////////////
bool WindPrivate::Set(const std::string &_name, double _value)
{
  // A requested 0 seed is resolved here, once, so every process that rebuilds
  // the model from the recipe gets the same wind.
  if (_name == "seed" && 0.0 == _value)
    _value = static_cast<double>(std::random_device{}() | 1u);
  if (!wind::SetParameter(this->recipe.params, _name, _value))
  {
    gzwarn << "Wind: key '" << _name << "' is unknown or out of range, "
           << "ignored\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
void WindPrivate::WriteRecipe(EntityComponentManager &_ecm)
{
  ++this->recipe.generation;
  // SetComponentData creates the component or updates it, but leaves the
  // change unmarked, and the mark is what replicates the recipe.
  if (_ecm.SetComponentData<components::Windfield>(this->worldEntity,
                                                   this->recipe))
  {
    _ecm.SetChanged(this->worldEntity, components::Windfield::typeId,
                    ComponentState::OneTimeChange);
  }
  this->dirty = true;
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
void WindPrivate::ApplyWindage(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  const double waterLevel = this->recipe.params.water_level;
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
      if (top <= waterLevel)
        continue;

      // The part above the water: its share of the height, and its centre.
      const double exposedBottom = std::max(bottom, waterLevel);
      const double fraction = hz > 0.0 ?
          std::clamp((top - exposedBottom) / (2.0 * hz), 0.0, 1.0) : 1.0;
      math::Vector3d centre = worldPose.Pos();
      centre.Z((top + exposedBottom) / 2.0);

      // The wind at that centre, relative to the shape, in the shape frame.
      const math::Vector3d offset = linkPose->Rot().RotateVectorReverse(
          centre - linkPose->Pos());
      const auto pointVel = link.WorldLinearVelocity(_ecm, offset);
      const math::Vector3d air = this->sampler.At(centre, _info.simTime);
      const math::Vector3d rel = worldPose.Rot().RotateVectorReverse(
          air - pointVel.value_or(math::Vector3d::Zero));

      // Quadratic drag per shape axis. The water cuts the areas the
      // horizontal wind sees; the plan area is left whole.
      const math::Vector3d area(shape.area.X() * fraction,
                                shape.area.Y() * fraction,
                                shape.area.Z());
      const double q = 0.5 * this->airDensity * shape.cd;
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
  auto &d = *this->dataPtr;
  d.worldEntity = _entity;
  d.windEntity = _ecm.EntityByComponents(components::Wind());
  d.airDensity = _sdf->Get<double>("air_density", d.airDensity).first;
  d.defaultCd = _sdf->Get<double>("default_drag_coefficient",
                                  d.defaultCd).first;
  if (d.airDensity <= 0.0 || d.defaultCd < 0.0)
  {
    gzerr << "Wind: <air_density> must be positive and "
          << "<default_drag_coefficient> cannot be negative\n";
  }

  // The world's <wind> is the starting point, in the world frame; the recipe
  // speaks of a speed and a direction from true north.
  math::Vector3d start = math::Vector3d::Zero;
  if (const auto *vel = _ecm.Component<components::WorldLinearVelocity>(
      d.windEntity))
  {
    start = vel->Data();
  }
  if (const auto *sc = _ecm.Component<components::SphericalCoordinates>(
      _entity))
  {
    const auto enu = sc->Data().VelocityTransform(
        math::CoordinateVector3::Metric(start),
        math::SphericalCoordinates::LOCAL, math::SphericalCoordinates::GLOBAL);
    if (enu && enu->IsMetric())
      start = enu->AsMetricVector().value_or(start);
  }
  d.recipe.params.vertical = start.Z();
  d.recipe.params.speed = std::hypot(start.X(), start.Y());
  const double from = GZ_RTOD(std::atan2(-start.X(), -start.Y()));
  wind::SetParameter(d.recipe.params, "direction",
                     d.recipe.params.speed > 0.0 ? from : 0.0);

  // The same names as the topic, so the world file and a message speak one
  // vocabulary.
  d.recipe.model = _sdf->Get<std::string>("model", "standard").first;
  d.Set("seed", _sdf->Get<double>("seed", 0.0).first);
  for (const auto &name : ParameterNames())
  {
    if ("seed" != name && _sdf->HasElement(name))
      d.Set(name, _sdf->Get<double>(name));
  }
  d.WriteRecipe(_ecm);

  const double rate = _sdf->Get<double>("publish_rate", 10.0).first;
  if (rate > 0.0)
  {
    d.publishPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
  }

  // The world's name scopes the wind topic and the ground truth.
  std::string worldName{"default"};
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  const std::string prefix = "/world/" + worldName + "/wind";
  if (!d.node.Subscribe(prefix + "/set", &WindPrivate::OnSet, &d))
    gzerr << "Wind: cannot subscribe to " << prefix << "/set\n";
  d.windPub = d.node.Advertise<msgs::Wind>(prefix + "_info");
}

//////////////////////////////////////////////////
void Wind::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.FindShapes(_ecm);

  // Apply what the topic queued; any change is a new recipe.
  bool changed{false};
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    for (const auto &[key, value] : d.pending)
      changed = d.Set(key, value) || changed;
    d.pending.clear();
  }
  if (changed)
    d.WriteRecipe(_ecm);

  // The wind above the world's origin, at the reference height, goes into
  // the wind entity, which Gazebo's rotor and wing systems read; they see
  // one wind for the whole world.
  if (!d.sampler.Sync(_ecm))
    return;
  const math::Vector3d reference(0.0, 0.0,
      d.recipe.params.water_level + d.recipe.params.reference_height);
  const math::Vector3d wind = d.sampler.At(reference, _info.simTime);
  if ((d.dirty || d.sampler.TimeVarying()) && kNullEntity != d.windEntity)
  {
    if (_ecm.SetComponentData<components::WorldLinearVelocity>(d.windEntity,
                                                               wind))
    {
      _ecm.SetChanged(d.windEntity, components::WorldLinearVelocity::typeId,
                      ComponentState::OneTimeChange);
    }
    d.dirty = false;
  }

  // Ground truth, at the publish rate in simulation time.
  if (!_info.paused && (!d.lastPublish ||
      _info.simTime - *d.lastPublish >= d.publishPeriod))
  {
    msgs::Wind msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));
    msgs::Set(msg.mutable_linear_velocity(), wind);
    msg.set_enable_wind(true);
    d.windPub.Publish(msg);
    d.lastPublish = _info.simTime;
  }

  if (!_info.paused)
    d.ApplyWindage(_info, _ecm);
}

//////////////////////////////////////////////////
void Wind::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;

  // A reset puts the world's recipe back as the world file set it. Start
  // again from that one, not from the changes made since, under a new
  // generation, so no consumer mistakes it for one it has already seen.
  if (const auto *comp = _ecm.Component<components::Windfield>(d.worldEntity))
  {
    d.recipe.model = comp->Data().model;
    d.recipe.params = comp->Data().params;
    d.WriteRecipe(_ecm);
  }
  d.links.clear();
  d.rescan = true;
  d.dirty = true;
  d.lastPublish.reset();
}

GZ_ADD_PLUGIN(Wind,
              System,
              Wind::ISystemConfigure,
              Wind::ISystemPreUpdate,
              Wind::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(Wind, "gz::sim::maritime::Wind")
