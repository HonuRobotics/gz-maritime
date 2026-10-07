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
#include "OceanCurrent.hh"

#include <chrono>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/components/Name.hh>

#include "gz/sim/components/OceanCurrentfield.hh"
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"
#include "gz/sim/ocean_current/OceanCurrentfield.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief Every numeric parameter name the recipe takes.
  std::vector<std::string> ParameterNames()
  {
    std::vector<std::string> names{"seed"};
#define GZ_OCEAN_CURRENT_NAME(m, name) names.emplace_back(name);
    GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_NAME)
#undef GZ_OCEAN_CURRENT_NAME
    return names;
  }
}

class gz::sim::maritime::OceanCurrentPrivate
{
  /// \brief Set one numeric parameter of the recipe, drawing a seed for a 0
  /// seed.
  /// \param[in] _name Parameter name.
  /// \param[in] _value Value.
  public: void Set(const std::string &_name, double _value);

  /// \brief Write the recipe into the world's OceanCurrentfield component.
  /// \param[in] _ecm The entity component manager.
  public: void WriteRecipe(EntityComponentManager &_ecm);

  /// \brief The recipe this system owns.
  public: ocean_current::OceanCurrentfieldData recipe;

  /// \brief Samples the recipe, for the ground truth.
  public: ocean_current::OceanCurrentSampler sampler;

  /// \brief The world entity.
  public: Entity worldEntity{kNullEntity};

  /// \brief Transport node for the ground truth.
  public: transport::Node node;

  /// \brief Ground truth publisher.
  public: transport::Node::Publisher pub;

  /// \brief Ground truth publication period, simulation time.
  public: std::chrono::steady_clock::duration publishPeriod{
      std::chrono::milliseconds(100)};

  /// \brief Simulation time of the last publication.
  public: std::optional<std::chrono::steady_clock::duration> lastPublish;
};

//////////////////////////////////////////////////
void OceanCurrentPrivate::Set(const std::string &_name, double _value)
{
  // A requested 0 seed is resolved here, once, so every process that rebuilds
  // the model from the recipe gets the same current.
  if (_name == "seed" && 0.0 == _value)
    _value = static_cast<double>(std::random_device{}() | 1u);
  if (!ocean_current::SetParameter(this->recipe.params, _name, _value))
  {
    gzerr << "OceanCurrent: <" << _name << "> value " << _value
          << " is out of range, ignored\n";
  }
}

//////////////////////////////////////////////////
void OceanCurrentPrivate::WriteRecipe(EntityComponentManager &_ecm)
{
  ++this->recipe.generation;
  // SetComponentData creates the component or updates it, but leaves the
  // change unmarked, and the mark is what replicates the recipe.
  if (_ecm.SetComponentData<components::OceanCurrentfield>(
      this->worldEntity, this->recipe))
  {
    _ecm.SetChanged(this->worldEntity, components::OceanCurrentfield::typeId,
                    ComponentState::OneTimeChange);
  }
}

//////////////////////////////////////////////////
OceanCurrent::OceanCurrent()
  : dataPtr(std::make_unique<OceanCurrentPrivate>())
{
}

//////////////////////////////////////////////////
OceanCurrent::~OceanCurrent() = default;

//////////////////////////////////////////////////
void OceanCurrent::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.worldEntity = _entity;

  // The tags carry the parameter names, so the world file and the recipe
  // speak one vocabulary.
  d.recipe.model = _sdf->Get<std::string>("model", "standard").first;
  d.recipe.params.source = _sdf->Get<std::string>("source", "").first;
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

  // The world's name scopes the ground truth.
  std::string worldName{"default"};
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  d.pub = d.node.Advertise<msgs::Twist>(
      "/world/" + worldName + "/ocean_current_info");
}

//////////////////////////////////////////////////
void OceanCurrent::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (!d.sampler.Sync(_ecm))
    return;

  // Ground truth: the current at the world's origin, in the world frame, at
  // the publish rate in simulation time.
  if (!_info.paused && (!d.lastPublish ||
      _info.simTime - *d.lastPublish >= d.publishPeriod))
  {
    msgs::Twist twist;
    twist.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));
    auto *frame = twist.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("world");
    msgs::Set(twist.mutable_linear(),
              d.sampler.At(math::Vector3d::Zero, _info.simTime));
    d.pub.Publish(twist);
    d.lastPublish = _info.simTime;
  }
}

//////////////////////////////////////////////////
void OceanCurrent::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;

  // A reset puts the world's recipe back as the world file set it; write it
  // again under a new generation, so no consumer mistakes it for one it has
  // already seen.
  if (const auto *comp =
      _ecm.Component<components::OceanCurrentfield>(d.worldEntity))
  {
    d.recipe.model = comp->Data().model;
    d.recipe.params = comp->Data().params;
    d.WriteRecipe(_ecm);
  }
  d.lastPublish.reset();
}

GZ_ADD_PLUGIN(OceanCurrent,
              System,
              OceanCurrent::ISystemConfigure,
              OceanCurrent::ISystemPreUpdate,
              OceanCurrent::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(OceanCurrent, "gz::sim::maritime::OceanCurrent")
