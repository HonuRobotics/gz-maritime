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

#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/param.pb.h>
#include <gz/msgs/wind.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/components/LinearVelocity.hh>
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
}

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
  if (auto *comp = _ecm.Component<components::Windfield>(this->worldEntity))
  {
    comp->Data() = this->recipe;
    _ecm.SetChanged(this->worldEntity, components::Windfield::typeId,
                    ComponentState::OneTimeChange);
  }
  else
  {
    _ecm.CreateComponent(this->worldEntity,
                         components::Windfield(this->recipe));
  }
  this->dirty = true;
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
  d.recipe.params.direction = d.recipe.params.speed > 0.0 ?
      std::fmod(from + 360.0, 360.0) : 0.0;

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

  // The wind at the world's origin goes into the wind entity, which Gazebo's
  // rotor and wing systems read.
  if (!d.sampler.Sync(_ecm))
    return;
  const math::Vector3d wind = d.sampler.At(math::Vector3d::Zero,
                                           _info.simTime);
  if ((d.dirty || d.sampler.TimeVarying()) && kNullEntity != d.windEntity)
  {
    if (auto *comp = _ecm.Component<components::WorldLinearVelocity>(
        d.windEntity))
    {
      comp->Data() = wind;
    }
    else
    {
      _ecm.CreateComponent(d.windEntity,
                           components::WorldLinearVelocity(wind));
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
}

//////////////////////////////////////////////////
void Wind::Reset(const UpdateInfo &, EntityComponentManager &)
{
  this->dataPtr->dirty = true;
  this->dataPtr->lastPublish.reset();
}

GZ_ADD_PLUGIN(Wind,
              System,
              Wind::ISystemConfigure,
              Wind::ISystemPreUpdate,
              Wind::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(Wind, "gz::sim::maritime::Wind")
