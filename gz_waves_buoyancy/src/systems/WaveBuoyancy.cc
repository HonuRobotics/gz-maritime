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
#include <unordered_set>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Volume.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/marked_shapes/MarkedShapes.hh"
#include "gz/sim/waves/Eval.hh"

#include "Displacement.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

class gz::sim::maritime::WaveBuoyancyPrivate
{
  /// \brief The links with collisions marked gz:buoyancy="true".
  public: marked_shapes::MarkedLinks marked{"gz:buoyancy", ""};

  /// \brief Links the flat buoyancy floats too, reported once each.
  public: std::unordered_set<Entity> doubled;

  /// \brief Links with a marked mesh, reported once each.
  public: std::unordered_set<Entity> meshes;

  /// \brief The world entity.
  public: Entity world{kNullEntity};

  /// \brief kg/m^3.
  public: double waterDensity{1025.0};

  /// \brief kg/m^3.
  public: double airDensity{1.0};

  /// \brief World z of the still water.
  public: double waterLevel{0.0};
};

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
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.world = _entity;
  const double water = _sdf->Get<double>("water_density", 1025.0).first;
  if (water > 0.0)
    d.waterDensity = water;
  else
    gzerr << "WaveBuoyancy: <water_density> must be positive, using 1025\n";
  const double air = _sdf->Get<double>("air_density", 1.0).first;
  if (air >= 0.0)
    d.airDensity = air;
  else
    gzerr << "WaveBuoyancy: <air_density> cannot be negative, using 1\n";
  d.waterLevel = _sdf->Get<double>("water_level", 0.0).first;
}

//////////////////////////////////////////////////
void WaveBuoyancy::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.marked.Find(_ecm);
  d.marked.Prune(_ecm);
  if (_info.paused || d.marked.Links().empty())
    return;

  const double t = std::chrono::duration<double>(_info.simTime).count();

  // The wave source system advances the field at its <update_rate>; this
  // reads it, and never advances it.
  const auto *field = _ecm.Component<components::Wavefield>(d.world);
  const waves_buoyancy::HeightAt height = [&](double _x, double _y)
  {
    return d.waterLevel +
           (field ? waves::SurfaceHeight(field->Data(), _x, _y, t) : 0.0);
  };

  math::Vector3d gravity(0, 0, -9.80665);
  if (const auto *g = _ecm.Component<components::Gravity>(d.world))
    gravity = g->Data();

  for (const auto &[entity, shapes] : d.marked.Links())
  {
    // A link the flat buoyancy floats too: it gives every link it floats
    // a Volume. Both would push it, so leave it to that one.
    if (_ecm.EntityHasComponentType(entity, components::Volume().TypeId()))
    {
      if (d.doubled.insert(entity).second)
      {
        gzerr << "WaveBuoyancy: link [" << entity << "] is floated by the "
              << "flat buoyancy as well; the wave buoyancy leaves it.\n";
      }
      continue;
    }
    Link link(entity);
    const auto linkPose = link.WorldPose(_ecm);
    if (!linkPose)
      continue;
    const auto *inertial = _ecm.Component<components::Inertial>(entity);
    // AddWorldForce takes the point relative to the centre of mass, in the
    // link frame.
    const math::Vector3d com = inertial ?
        inertial->Data().Pose().Pos() : math::Vector3d::Zero;
    auto push = [&](double _volume, double _density,
                    const math::Vector3d &_at)
    {
      if (_volume <= 1e-12 || _density <= 0.0)
        return;
      link.AddWorldForce(_ecm, -gravity * _density * _volume,
          linkPose->Rot().RotateVectorReverse(_at - linkPose->Pos()) - com);
    };

    for (const auto &shape : shapes)
    {
      if (!waves_buoyancy::Floats(shape.type))
      {
        if (d.meshes.insert(entity).second)
        {
          gzwarn << "WaveBuoyancy: link [" << entity << "] marks a mesh; "
                 << "only boxes, cylinders, spheres, capsules, ellipsoids "
                 << "and cones float\n";
        }
        continue;
      }
      const math::Pose3d pose = *linkPose * shape.pose;
      const auto surface = waves_buoyancy::FitSurface(shape, pose, height);

      // The water under the surface, and the air above it, each pushes up
      // with the weight it displaces, at its own centre.
      const auto cut = waves_buoyancy::Below(shape, pose, surface);
      push(cut.volume, d.waterDensity, cut.centre);
      push(cut.airVolume, d.airDensity, cut.airCentre);
    }
  }
}

//////////////////////////////////////////////////
void WaveBuoyancy::PostUpdate(const UpdateInfo &,
    const EntityComponentManager &_ecm)
{
  this->dataPtr->marked.NoteNew(_ecm);
}

//////////////////////////////////////////////////
void WaveBuoyancy::Reset(const UpdateInfo &, EntityComponentManager &)
{
  this->dataPtr->marked.Reset();
}

GZ_ADD_PLUGIN(WaveBuoyancy,
              System,
              WaveBuoyancy::ISystemConfigure,
              WaveBuoyancy::ISystemPreUpdate,
              WaveBuoyancy::ISystemPostUpdate,
              WaveBuoyancy::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(WaveBuoyancy, "gz::sim::maritime::WaveBuoyancy")
