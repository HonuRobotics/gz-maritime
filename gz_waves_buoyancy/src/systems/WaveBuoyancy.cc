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
#include "WaveBuoyancy.hh"

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Collision.hh>

#include "gz/sim/components/WaveCoupling.hh"
#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves_buoyancy/Displacement.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief The attribute a collision is marked with, the flat buoyancy's.
  const std::string kMark{"gz:buoyancy"};
}

class gz::sim::maritime::WaveBuoyancyPrivate
{
  /// \brief Look at collisions not looked at yet, or at all of them.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _all Scan every collision.
  public: void Find(EntityComponentManager &_ecm, bool _all);

  /// \brief The marked shapes, by link.
  public: std::unordered_map<Entity, std::vector<
      waves_buoyancy::BuoyantShape>> links;

  /// \brief Collisions made since the last scan.
  public: std::vector<Entity> newCollisions;

  /// \brief Collisions looked at already: one found by a scan is reported
  /// new as well in the first steps, and must not float twice.
  public: std::unordered_set<Entity> known;

  /// \brief Scan every collision at the next update.
  public: bool rescan{true};

  /// \brief The world entity.
  public: Entity world{kNullEntity};

  /// \brief kg/m^3.
  public: double waterDensity{1025.0};
  public: double airDensity{1.0};

  /// \brief World z of the still water.
  public: double waterLevel{0.0};

  /// \brief Simulation time the wave field was advanced to.
  public: std::optional<double> advanced;
};

//////////////////////////////////////////////////
void WaveBuoyancyPrivate::Find(EntityComponentManager &_ecm, bool _all)
{
  std::vector<Entity> candidates;
  candidates.swap(this->newCollisions);
  if (_all)
  {
    this->links.clear();
    this->known.clear();
    candidates.clear();
    _ecm.Each<components::Collision>(
        [&](const Entity &_e, const components::Collision *) -> bool
        {
          candidates.push_back(_e);
          return true;
        });
  }

  for (const Entity collision : candidates)
  {
    if (!this->known.insert(collision).second)
      continue;
    const auto *element =
        _ecm.Component<components::CollisionElement>(collision);
    const auto *parent = _ecm.Component<components::ParentEntity>(collision);
    const auto *pose = _ecm.Component<components::Pose>(collision);
    if (nullptr == element || nullptr == parent || nullptr == pose)
      continue;
    const auto elem = element->Data().Element();
    bool marked{false};
    if (!elem || !elem->HasAttribute(kMark) ||
        !elem->GetAttribute(kMark)->Get<bool>(marked) || !marked)
    {
      continue;
    }
    const auto *geometry = element->Data().Geom();
    const auto shape = geometry ?
        waves_buoyancy::ShapeFromGeometry(*geometry, pose->Data()) :
        std::nullopt;
    if (!shape)
    {
      gzwarn << "WaveBuoyancy: collision [" << element->Data().Name()
             << "] is marked but is not a box, a cylinder or a sphere; "
             << "it does not float\n";
      continue;
    }
    this->links[parent->Data()].push_back(*shape);
  }
}

//////////////////////////////////////////////////
WaveBuoyancy::WaveBuoyancy()
  : dataPtr(std::make_unique<WaveBuoyancyPrivate>())
{
}

//////////////////////////////////////////////////
WaveBuoyancy::~WaveBuoyancy() = default;

//////////////////////////////////////////////////
void WaveBuoyancy::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.world = _entity;
  d.waterDensity = _sdf->Get<double>("water_density", d.waterDensity).first;
  d.airDensity = _sdf->Get<double>("air_density", d.airDensity).first;
  d.waterLevel = _sdf->Get<double>("water_level", d.waterLevel).first;

  // Tell the systems that damp against the water that it moves with the
  // waves here.
  _ecm.SetComponentData<components::WaveCoupling>(_entity, d.waterLevel);
}

//////////////////////////////////////////////////
void WaveBuoyancy::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.Find(_ecm, d.rescan);
  d.rescan = false;
  if (_info.paused || d.links.empty())
    return;

  const double t = std::chrono::duration<double>(_info.simTime).count();

  // The wave field at this step. The source system advances it a few times
  // a second; buoyancy needs it every step, or the forces jump.
  auto *field = _ecm.Component<components::Wavefield>(d.world);
  if (field && field->Data().simulation && (!d.advanced || *d.advanced != t))
  {
    waves::Advance(field->Data(), t);
    d.advanced = t;
  }

  math::Vector3d gravity(0, 0, -9.80665);
  if (const auto *g = _ecm.Component<components::Gravity>(d.world))
    gravity = g->Data();

  for (auto it = d.links.begin(); it != d.links.end();)
  {
    if (!_ecm.HasEntity(it->first))
    {
      it = d.links.erase(it);
      continue;
    }
    Link link(it->first);
    const math::Pose3d linkPose = worldPose(it->first, _ecm);
    const auto *inertial = _ecm.Component<components::Inertial>(it->first);
    // AddWorldForce takes the point relative to the centre of mass, in the
    // link frame.
    const math::Vector3d com = inertial ?
        inertial->Data().Pose().Pos() : math::Vector3d::Zero;

    for (const auto &shape : it->second)
    {
      const math::Pose3d pose = linkPose * shape.pose;
      const double x = pose.Pos().X();
      const double y = pose.Pos().Y();
      double eta{0.0};
      math::Vector3d normal = math::Vector3d::UnitZ;
      if (field)
      {
        eta = waves::SurfaceElevation(field->Data(), x, y, t);
        normal = waves::Normal(field->Data(), x, y, t);
      }
      const auto water = waves_buoyancy::Below(shape, pose,
          math::Vector3d(x, y, d.waterLevel + eta), normal);

      // The water under the surface, and the air above it, each pushes up
      // with the weight it displaces, at its own centre.
      const double whole = shape.Volume();
      const double air = whole - water.volume;
      auto push = [&](double _volume, double _density,
                      const math::Vector3d &_at)
      {
        if (_volume <= 0.0)
          return;
        const math::Vector3d force = -gravity * _density * _volume;
        const math::Vector3d offset =
            linkPose.Rot().RotateVectorReverse(_at - linkPose.Pos()) - com;
        link.AddWorldForce(_ecm, force, offset);
      };
      push(water.volume, d.waterDensity, water.centre);
      if (air > 1e-12)
      {
        push(air, d.airDensity,
             (pose.Pos() * whole - water.centre * water.volume) / air);
      }
    }
    ++it;
  }
}

//////////////////////////////////////////////////
void WaveBuoyancy::PostUpdate(const UpdateInfo &,
    const EntityComponentManager &_ecm)
{
  // EachNew only sees an entity in the step it was made in; noting it here,
  // after every PreUpdate, catches it whatever order the systems run in.
  _ecm.EachNew<components::Collision>(
      [&](const Entity &_e, const components::Collision *) -> bool
      {
        this->dataPtr->newCollisions.push_back(_e);
        return true;
      });
}

//////////////////////////////////////////////////
void WaveBuoyancy::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.rescan = true;
  d.newCollisions.clear();
  d.advanced.reset();
  _ecm.SetComponentData<components::WaveCoupling>(d.world, d.waterLevel);
}

GZ_ADD_PLUGIN(WaveBuoyancy,
              System,
              WaveBuoyancy::ISystemConfigure,
              WaveBuoyancy::ISystemPreUpdate,
              WaveBuoyancy::ISystemPostUpdate,
              WaveBuoyancy::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(WaveBuoyancy, "gz::sim::maritime::WaveBuoyancy")
