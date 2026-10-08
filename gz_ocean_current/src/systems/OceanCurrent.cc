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
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/param.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/components/Name.hh>

#include "gz/sim/components/OceanCurrentfield.hh"
#include "gz/sim/marked_shapes/MarkedShapes.hh"
#include "gz/sim/ocean_current/OceanCurrentModel.hh"
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"
#include "gz/sim/ocean_current/OceanCurrentfield.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief The block of the parameters the model owns.
  const std::string kParameters{"parameters"};

  /// \brief The attribute a collision is marked with. Namespaced, so SDFormat
  /// keeps it without knowing it.
  const std::string kMark{"gz:ocean_current"};

  /// \brief The optional per shape drag coefficient attribute.
  const std::string kCdMark{"gz:ocean_current_cd"};

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

  /// \brief Every numeric parameter name the recipe takes.
  std::vector<std::string> ParameterNames()
  {
    std::vector<std::string> names{"seed"};
#define GZ_OCEAN_CURRENT_NAME(m, name) names.emplace_back(name);
    GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_NAME)
#undef GZ_OCEAN_CURRENT_NAME
    return names;
  }

  /// \brief Every element the plugin reads at its top level.
  std::set<std::string> KnownElements()
  {
    std::set<std::string> known{"model", "publish_rate", kParameters,
                                "water_density", "default_drag_coefficient"};
    for (const auto &name : ParameterNames())
      known.insert(name);
    return known;
  }

  /// \brief A number as text, at full precision, for a parameter the model
  /// owns.
  /// \param[in] _value The number.
  /// \return Its text.
  std::string ToText(double _value)
  {
    std::ostringstream os;
    os << std::setprecision(std::numeric_limits<double>::max_digits10)
       << _value;
    return os.str();
  }
}

class gz::sim::maritime::OceanCurrentPrivate
{
  /// \brief Handler of the ocean current topic, on a transport thread.
  /// \param[in] _msg Parameter names and values.
  public: void OnSet(const msgs::Param &_msg);

  /// \brief Set one numeric parameter, drawing a seed for a 0 seed.
  /// \param[in,out] _params Parameters.
  /// \param[in] _name Parameter name.
  /// \param[in] _value Value.
  /// \return False if the name is unknown or the value out of range.
  public: static bool Set(ocean_current::OceanCurrentParameters &_params,
                          const std::string &_name, double _value);

  /// \brief Apply one message from the topic, all of it or nothing: every
  /// key must be valid and the model must accept the result.
  /// \param[in] _msg The message.
  /// \return True if it changed the recipe.
  public: bool Apply(const msgs::Param &_msg);

  /// \brief Write the recipe into the world's OceanCurrentfield component.
  /// \param[in] _ecm The entity component manager.
  public: void WriteRecipe(EntityComponentManager &_ecm);

  /// \brief The recipe this system owns.
  public: ocean_current::OceanCurrentfieldData recipe;

  /// \brief Samples the recipe, for the ground truth.
  public: ocean_current::OceanCurrentSampler sampler;

  /// \brief The world entity.
  public: Entity worldEntity{kNullEntity};

  /// \brief The links carrying collisions marked gz:ocean_current, and the
  /// drag of the water on them.
  public: marked_shapes::MarkedLinks marked{kMark, kCdMark};

  /// \brief Water density, kg/m^3.
  public: double waterDensity{1025.0};

  /// \brief Messages queued by the topic, applied at the next step.
  public: std::vector<msgs::Param> pending;

  /// \brief Whether the model was checked against the registry, which
  /// happens once, at the first step, since a plugin may register it after
  /// this system is configured.
  public: bool modelChecked{false};

  /// \brief Guards the queue across the transport and ECM threads.
  public: std::mutex mutex;

  /// \brief Transport node for the ocean current topic and the ground
  /// truth.
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
void OceanCurrentPrivate::OnSet(const msgs::Param &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  this->pending.push_back(_msg);
}

//////////////////////////////////////////////////
bool OceanCurrentPrivate::Set(ocean_current::OceanCurrentParameters &_params,
    const std::string &_name, double _value)
{
  // A requested 0 seed is resolved here, once, so every process that rebuilds
  // the model from the recipe gets the same current.
  if (_name == "seed" && 0.0 == _value)
    _value = static_cast<double>(std::random_device{}() | 1u);
  return ocean_current::SetParameter(_params, _name, _value);
}

//////////////////////////////////////////////////
bool OceanCurrentPrivate::Apply(const msgs::Param &_msg)
{
  // Work on a copy, so a message with one bad key changes nothing.
  auto params = this->recipe.params;
  for (const auto &[key, value] : _msg.params())
  {
    double d{0.0};
    const bool numeric = ReadDouble(value, d);
    if (ocean_current::IsTypedParameter(key))
    {
      if (!numeric || !Set(params, key, d))
      {
        gzwarn << "OceanCurrent: key '" << key << "' needs a number in "
               << "range; message ignored\n";
        return false;
      }
    }
    else if (value.type() == msgs::Any::STRING)
    {
      params.extra[key] = value.string_value();
    }
    else if (numeric)
    {
      params.extra[key] = ToText(d);
    }
    else
    {
      gzwarn << "OceanCurrent: key '" << key << "' is neither a number nor "
             << "a string; message ignored\n";
      return false;
    }
  }

  const std::string why =
      ocean_current::ValidateOceanCurrentModel(this->recipe.model, params);
  if (!why.empty())
  {
    gzwarn << "OceanCurrent: " << why << "; message ignored\n";
    return false;
  }
  this->recipe.params = params;
  return true;
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
  // speak one vocabulary. A seed of 1 by default, so a run repeats; 0 asks
  // for a new one each run.
  d.recipe.model = _sdf->Get<std::string>("model", "standard").first;
  if (!OceanCurrentPrivate::Set(d.recipe.params, "seed",
                                _sdf->Get<double>("seed", 1.0).first))
  {
    gzwarn << "OceanCurrent: <seed> out of range, ignored\n";
  }
  for (const auto &name : ParameterNames())
  {
    if ("seed" != name && _sdf->HasElement(name) &&
        !OceanCurrentPrivate::Set(d.recipe.params, name,
                                  _sdf->Get<double>(name)))
    {
      gzwarn << "OceanCurrent: <" << name << "> out of range, ignored\n";
    }
  }

  // The parameters the model owns, as text, opaque to this system.
  if (_sdf->HasElement(kParameters))
  {
    auto block = _sdf->FindElement(kParameters);
    for (auto child = block->GetFirstElement(); child;
         child = child->GetNextElement())
    {
      d.recipe.params.extra[child->GetName()] = child->Get<std::string>();
    }
  }

  // A typo such as <speeed> would otherwise leave slack water in silence.
  const auto known = KnownElements();
  for (auto child = _sdf->GetFirstElement(); child;
       child = child->GetNextElement())
  {
    if (0u == known.count(child->GetName()))
    {
      gzwarn << "OceanCurrent: unknown element <" << child->GetName()
             << ">, ignored; a parameter of the model goes in <"
             << kParameters << ">\n";
    }
  }
  d.WriteRecipe(_ecm);

  // The load on marked shapes. An invalid value keeps the default: a
  // negative density or coefficient would make the water pull.
  const double density =
      _sdf->Get<double>("water_density", d.waterDensity).first;
  if (density > 0.0)
    d.waterDensity = density;
  else
    gzerr << "OceanCurrent: <water_density> must be positive, using "
          << d.waterDensity << "\n";
  const double cd = _sdf->Get<double>("default_drag_coefficient", 1.0).first;
  if (cd >= 0.0)
    d.marked.SetDefaultCd(cd);
  else
    gzerr << "OceanCurrent: <default_drag_coefficient> cannot be negative, "
          << "using 1\n";

  const double rate = _sdf->Get<double>("publish_rate", 10.0).first;
  if (rate > 0.0)
  {
    d.publishPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
  }

  // The world's name scopes the topic and the ground truth.
  std::string worldName{"default"};
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  const std::string prefix = "/world/" + worldName + "/ocean_current";
  if (!d.node.Subscribe(prefix + "/set", &OceanCurrentPrivate::OnSet, &d))
    gzerr << "OceanCurrent: cannot subscribe to " << prefix << "/set\n";
  d.pub = d.node.Advertise<msgs::Twist>(prefix + "_info");
}

//////////////////////////////////////////////////
void OceanCurrent::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;

  // Check the model once, here rather than in Configure, since a plugin
  // may register it after this system is configured. This is the one place
  // a bad model is reported; consumers only find no current.
  if (!d.modelChecked)
  {
    d.modelChecked = true;
    const std::string why = ocean_current::ValidateOceanCurrentModel(
        d.recipe.model, d.recipe.params);
    if (!why.empty())
      gzerr << "OceanCurrent: " << why << "; the world has no current\n";
  }

  // Apply what the topic queued; any change is a new recipe, which every
  // consumer sees on this step.
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    bool changed{false};
    for (const auto &msg : d.pending)
      changed = d.Apply(msg) || changed;
    d.pending.clear();
    if (changed)
      d.WriteRecipe(_ecm);
  }

  d.marked.Find(_ecm);
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

  // The water on the part of each marked shape below the water level,
  // relative to that part, at its centre. The water level is the recipe's,
  // so a change on the topic moves it too.
  if (!_info.paused)
  {
    d.marked.ApplyDrag(_info, _ecm, marked_shapes::Side::kBelow,
        d.recipe.params.water_level, d.waterDensity,
        [&d](const math::Vector3d &_point,
             const std::chrono::steady_clock::duration &_time)
        {
          return d.sampler.At(_point, _time);
        });
  }
}

//////////////////////////////////////////////////
void OceanCurrent::PostUpdate(const UpdateInfo &,
    const EntityComponentManager &_ecm)
{
  // A link is noted after every system's PreUpdate, whichever system made
  // it, and resolved at the next PreUpdate.
  this->dataPtr->marked.NoteNew(_ecm);
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
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    d.pending.clear();
  }
  d.marked.Reset();
  d.lastPublish.reset();
}

GZ_ADD_PLUGIN(OceanCurrent,
              System,
              OceanCurrent::ISystemConfigure,
              OceanCurrent::ISystemPreUpdate,
              OceanCurrent::ISystemPostUpdate,
              OceanCurrent::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(OceanCurrent, "gz::sim::maritime::OceanCurrent")
