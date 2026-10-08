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
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/wind.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocitySeed.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/Wind.hh>

#include "gz/sim/components/Windfield.hh"
#include "gz/sim/wind/Windfield.hh"
#include "gz/sim/wind/WindSampler.hh"
#include "gz/sim/marked_shapes/MarkedShapes.hh"

#include "Anemometer.hh"

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

}

/// \brief One anemometer and where it sits, resolved once when it is found.
struct AnemometerMount
{
  /// \brief The link it is on.
  public: Entity link{kNullEntity};

  /// \brief Its pose in the link frame.
  public: math::Pose3d pose;

  /// \brief The sensor, which publishes at its rate.
  public: std::unique_ptr<Anemometer> sensor;
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

  /// \brief Find the anemometers the system has not seen.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _all Scan every sensor, not only the new ones.
  public: void FindAnemometers(EntityComponentManager &_ecm, bool _all);

  /// \brief Publish what each anemometer reads, at its rate.
  /// \param[in] _info Update info.
  /// \param[in] _ecm The entity component manager.
  public: void ReadAnemometers(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm);

  /// \brief Write a velocity into a component of the wind entity.
  /// \tparam ComponentT The component.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _wind The velocity.
  public: template <typename ComponentT>
          void WriteEntity(EntityComponentManager &_ecm,
                           const math::Vector3d &_wind);

  /// \brief The links carrying collisions marked gz:wind, and the windage
  /// on them.
  public: marked_shapes::MarkedLinks marked{kMark, kCdMark};

  /// \brief Anemometers, by sensor entity.
  public: std::unordered_map<Entity, AnemometerMount> anemometers;

  /// \brief Scan every sensor instead of only the new ones on the next
  /// update.
  public: bool rescan{true};

  /// \brief Custom sensors created since the last scan, noted in
  /// PostUpdate, after every system's PreUpdate, so a sensor is seen
  /// whichever system made it.
  public: std::vector<Entity> newSensors;

  /// \brief Air density, kg/m^3.
  public: double airDensity{1.225};

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

  /// \brief The same ground truth as a twist, which ROS can bridge.
  public: transport::Node::Publisher twistPub;

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
template <typename ComponentT>
void WindPrivate::WriteEntity(EntityComponentManager &_ecm,
    const math::Vector3d &_wind)
{
  // SetComponentData creates or updates the component but leaves the change
  // unmarked.
  if (_ecm.SetComponentData<ComponentT>(this->windEntity, _wind))
  {
    _ecm.SetChanged(this->windEntity, ComponentT::typeId,
                    ComponentState::OneTimeChange);
  }
}

//////////////////////////////////////////////////
void WindPrivate::FindAnemometers(EntityComponentManager &_ecm, bool _all)
{
  std::vector<Entity> candidates;
  candidates.swap(this->newSensors);
  if (_all)
  {
    this->anemometers.clear();
    candidates.clear();
    _ecm.Each<components::CustomSensor>(
        [&](const Entity &_sensor, const components::CustomSensor *) -> bool
        {
          candidates.push_back(_sensor);
          return true;
        });
  }

  for (const Entity sensor : candidates)
  {
    const auto *custom = _ecm.Component<components::CustomSensor>(sensor);
    const auto *parent = _ecm.Component<components::ParentEntity>(sensor);
    const auto *pose = _ecm.Component<components::Pose>(sensor);
    if (nullptr == custom || nullptr == parent || nullptr == pose ||
        sensors::customType(custom->Data()) != "anemometer")
    {
      continue;
    }

    // The sensor base class reads the topic, frame id and update rate; the
    // name, which the frame id defaults to, and the topic default to the
    // sensor's scoped name.
    sdf::Sensor data = custom->Data();
    data.SetName(scopedName(sensor, _ecm, "::", false));
    if (data.Topic().empty())
      data.SetTopic("/" + scopedName(sensor, _ecm, "/", true) + "/anemometer");
    sensors::SensorFactory factory;
    auto created = factory.CreateSensor<Anemometer>(data);
    if (nullptr == created)
    {
      gzerr << "Wind: cannot create anemometer [" << data.Name() << "]\n";
      continue;
    }

    AnemometerMount mount;
    mount.link = parent->Data();
    mount.pose = pose->Data();
    mount.sensor = std::move(created);
    // The reading needs the velocity of the point it is taken at.
    Link(mount.link).EnableVelocityChecks(_ecm);
    this->anemometers[sensor] = std::move(mount);
  }
}

//////////////////////////////////////////////////
void WindPrivate::ReadAnemometers(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  for (auto it = this->anemometers.begin(); it != this->anemometers.end();)
  {
    if (!_ecm.HasEntity(it->first))
    {
      it = this->anemometers.erase(it);
      continue;
    }
    AnemometerMount &a = it->second;
    ++it;
    // Work the wind out only when the sensor is due to publish.
    if (_info.simTime < a.sensor->NextDataUpdateTime())
      continue;

    Link link(a.link);
    const auto linkPose = link.WorldPose(_ecm);
    if (!linkPose)
      continue;
    const math::Pose3d sensorPose = *linkPose * a.pose;
    const auto own = link.WorldLinearVelocity(_ecm, a.pose.Pos());

    // The apparent wind: the air's velocity relative to the sensor, in the
    // sensor frame, the way a vane and cups on a moving boat read it.
    a.sensor->SetApparentWind(sensorPose.Rot().RotateVectorReverse(
        this->sampler.At(sensorPose.Pos(), _info.simTime) -
        own.value_or(math::Vector3d::Zero)));
    a.sensor->Update(_info.simTime, false);
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
  // An invalid value keeps the default: a negative coefficient would make
  // the wind pull.
  const double density = _sdf->Get<double>("air_density",
                                           d.airDensity).first;
  if (density > 0.0)
    d.airDensity = density;
  else
    gzerr << "Wind: <air_density> must be positive, using "
          << d.airDensity << "\n";
  const double cd = _sdf->Get<double>("default_drag_coefficient", 1.0).first;
  if (cd >= 0.0)
    d.marked.SetDefaultCd(cd);
  else
    gzerr << "Wind: <default_drag_coefficient> cannot be negative, using 1\n";

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
  d.twistPub = d.node.Advertise<msgs::Twist>(prefix + "/velocity");
}

//////////////////////////////////////////////////
void Wind::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.marked.Find(_ecm);
  d.FindAnemometers(_ecm, d.rescan);
  d.rescan = false;

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
  // the wind entity, which Gazebo's rotor and wing systems read, and into
  // its seed, which Gazebo's air speed sensor reads; they see one wind for
  // the whole world.
  if (!d.sampler.Sync(_ecm))
    return;
  const math::Vector3d reference(0.0, 0.0,
      d.recipe.params.water_level + d.recipe.params.reference_height);
  const math::Vector3d wind = d.sampler.At(reference, _info.simTime);
  if ((d.dirty || d.sampler.TimeVarying()) && kNullEntity != d.windEntity)
  {
    d.WriteEntity<components::WorldLinearVelocity>(_ecm, wind);
    d.WriteEntity<components::WorldLinearVelocitySeed>(_ecm, wind);
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

    msgs::Twist twist;
    twist.mutable_header()->CopyFrom(msg.header());
    auto *frame = twist.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("world");
    msgs::Set(twist.mutable_linear(), wind);
    d.twistPub.Publish(twist);
    d.lastPublish = _info.simTime;
  }

  if (!_info.paused)
  {
    d.marked.ApplyDrag(_info, _ecm, marked_shapes::Side::kAbove,
        d.recipe.params.water_level, d.airDensity,
        [&d](const math::Vector3d &_point,
             const std::chrono::steady_clock::duration &_time)
        {
          return d.sampler.At(_point, _time);
        });
    d.ReadAnemometers(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void Wind::PostUpdate(const UpdateInfo &, const EntityComponentManager &_ecm)
{
  // EachNew only sees an entity in the step it was made in. Noting it here,
  // after every PreUpdate, catches it whatever order the systems run in;
  // it is resolved at the next PreUpdate.
  this->dataPtr->marked.NoteNew(_ecm);
  _ecm.EachNew<components::CustomSensor>(
      [&](const Entity &_sensor, const components::CustomSensor *) -> bool
      {
        this->dataPtr->newSensors.push_back(_sensor);
        return true;
      });
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
  d.marked.Reset();
  d.anemometers.clear();
  d.rescan = true;
  d.dirty = true;
  d.lastPublish.reset();
}

GZ_ADD_PLUGIN(Wind,
              System,
              Wind::ISystemConfigure,
              Wind::ISystemPreUpdate,
              Wind::ISystemPostUpdate,
              Wind::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(Wind, "gz::sim::maritime::Wind")
