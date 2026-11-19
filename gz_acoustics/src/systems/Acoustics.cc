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
#include "Acoustics.hh"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <list>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/msgs/param.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include "gz/sim/acoustics/Hearing.hh"
#include "gz/sim/acoustics/Pinger.hh"
#include "gz/sim/acoustics/Random.hh"
#include "gz/sim/components/AcousticPinger.hh"

#include "AcousticReceiver.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief The custom sensor type of a receiver.
  const std::string kReceiverType{"acoustic_receiver"};
}

/// \brief One receiver, where it sits and what it holds.
struct ReceiverMount
{
  /// \brief The link it is on.
  public: Entity link{kNullEntity};

  /// \brief Its pose in the link frame.
  public: math::Pose3d pose;

  /// \brief The sensor, which publishes.
  public: std::unique_ptr<AcousticReceiver> sensor;

  /// \brief Its scoped name, from the world.
  public: std::string name;

  /// \brief Draws its dropouts and outliers; its noise is gz-sensors'.
  public: acoustics::Generator rng;

  /// \brief Readings that arrived, in order of arrival, each with its
  /// pulse length: a reading waits that long, so a ping arriving on top of
  /// it can mark both invalid.
  public: std::deque<std::pair<ReceiverReading, acoustics::Duration>> held;
};

/// \brief One ping on its way.
struct Ping
{
  /// \brief Hz, as the pinger was set when it pinged.
  public: double frequency{0.0};

  /// \brief m, the farthest it is heard; 0 is no limit.
  public: double reach{0.0};

  /// \brief How long it lasts at a receiver.
  public: acoustics::Duration pulse{0};

  /// \brief World position it was sent from.
  public: math::Vector3d origin;

  /// \brief When it was sent.
  public: acoustics::Duration emitted{0};

  /// \brief Receivers it has not reached yet.
  public: std::unordered_set<Entity> waiting;
};

class gz::sim::maritime::AcousticsPrivate
{
  /// \brief Find the receivers the system has not seen.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _candidates Sensors to look at; with _all, ignored.
  /// \param[in] _all Scan every sensor, not only the candidates.
  public: void FindReceivers(const EntityComponentManager &_ecm,
                             std::vector<Entity> _candidates, bool _all);

  /// \brief Send the pings due in (after, now].
  /// \param[in] _after Exclusive start of the window.
  /// \param[in] _now Simulation time.
  /// \param[in] _ecm The entity component manager.
  public: void Emit(const acoustics::Duration &_after,
                    const acoustics::Duration &_now,
                    const EntityComponentManager &_ecm);

  /// \brief Move the pings on, and hand each to the receivers it reaches.
  /// \param[in] _now Simulation time.
  /// \param[in] _ecm The entity component manager.
  public: void Propagate(const acoustics::Duration &_now,
                         const EntityComponentManager &_ecm);

  /// \brief A receiver hears a ping, or does not.
  /// \param[in] _mount The receiver.
  /// \param[in] _pose Its world pose now.
  /// \param[in] _ping The ping.
  /// \param[in] _arrived When the sound gets there: a time between two
  /// steps, not the step that notices it.
  public: void Hear(ReceiverMount &_mount, const math::Pose3d &_pose,
                    const Ping &_ping, const acoustics::Duration &_arrived);

  /// \brief Publish the readings whose pulse has passed.
  /// \param[in] _now Simulation time.
  public: void Flush(const acoustics::Duration &_now);

  /// \brief Publish the pingers' ground truth and the list of receivers,
  /// at the publish rate: on simulation time while the world runs, on wall
  /// time while it is paused, so a GUI opened on a paused world gets both.
  /// \param[in] _now Simulation time.
  /// \param[in] _paused Whether the world is paused.
  /// \param[in] _ecm The entity component manager.
  public: void PublishPingers(const acoustics::Duration &_now, bool _paused,
                              const EntityComponentManager &_ecm);

  /// \brief World position of a pinger.
  /// \param[in] _link The link that carries it.
  /// \param[in] _pinger The pinger.
  /// \param[in] _ecm The entity component manager.
  /// \return Its position.
  public: math::Vector3d PingerPosition(const Entity &_link,
                                        const acoustics::PingerData &_pinger,
                                        const EntityComponentManager &_ecm)
                                        const;

  /// \brief Whether a point is under the water, or the rule is off.
  /// \param[in] _point World position.
  /// \return True if a ping can be sent or heard there.
  public: bool Wet(const math::Vector3d &_point) const;

  /// \brief Receivers, by sensor entity.
  public: std::unordered_map<Entity, ReceiverMount> receivers;

  /// \brief Pings on their way.
  public: std::list<Ping> pings;

  /// \brief Scan every sensor at the next update.
  public: bool rescan{true};

  /// \brief Simulation time of the last update; pings after it are due.
  public: std::optional<acoustics::Duration> last;

  /// \brief m/s.
  public: double speedOfSound{1500.0};

  /// \brief World z of the water.
  public: double waterLevel{0.0};

  /// \brief Whether both ends must be under the water.
  public: bool underwaterOnly{true};

  /// \brief The seed, resolved.
  public: std::uint32_t seed{1};

  /// \brief Transport node for the ground truth.
  public: transport::Node node;

  /// \brief Ground truth publisher.
  public: transport::Node::Publisher pingersPub;

  /// \brief The receivers' publisher.
  public: transport::Node::Publisher receiversPub;

  /// \brief Ground truth period, simulation time.
  public: acoustics::Duration publishPeriod{std::chrono::seconds(1)};

  /// \brief Simulation time of the last ground truth.
  public: std::optional<acoustics::Duration> lastPublish;

  /// \brief Wall time of the last ground truth sent while paused.
  public: std::optional<std::chrono::steady_clock::time_point>
      lastPausedPublish;
};

//////////////////////////////////////////////////
math::Vector3d AcousticsPrivate::PingerPosition(const Entity &_link,
    const acoustics::PingerData &_pinger,
    const EntityComponentManager &_ecm) const
{
  return (worldPose(_link, _ecm) * _pinger.pose).Pos();
}

//////////////////////////////////////////////////
bool AcousticsPrivate::Wet(const math::Vector3d &_point) const
{
  return !this->underwaterOnly || _point.Z() < this->waterLevel;
}

//////////////////////////////////////////////////
void AcousticsPrivate::FindReceivers(const EntityComponentManager &_ecm,
    std::vector<Entity> _candidates, bool _all)
{
  if (_all)
  {
    this->receivers.clear();
    _candidates.clear();
    _ecm.Each<components::CustomSensor>(
        [&](const Entity &_sensor, const components::CustomSensor *) -> bool
        {
          _candidates.push_back(_sensor);
          return true;
        });
  }

  for (const Entity sensor : _candidates)
  {
    const auto *custom = _ecm.Component<components::CustomSensor>(sensor);
    const auto *parent = _ecm.Component<components::ParentEntity>(sensor);
    const auto *pose = _ecm.Component<components::Pose>(sensor);
    if (nullptr == custom || nullptr == parent || nullptr == pose ||
        sensors::customType(custom->Data()) != kReceiverType ||
        this->receivers.count(sensor) > 0)
    {
      continue;
    }

    // The sensor base class reads the topic and the frame id; both default
    // to the sensor's scoped name.
    sdf::Sensor data = custom->Data();
    const std::string scoped = scopedName(sensor, _ecm, "::", false);
    data.SetName(scoped);
    if (data.Topic().empty())
    {
      data.SetTopic("/" + scopedName(sensor, _ecm, "/", true) + "/" +
                    kReceiverType);
    }
    sensors::SensorFactory factory;
    auto created = factory.CreateSensor<AcousticReceiver>(data);
    if (nullptr == created)
    {
      gzerr << "Acoustics: cannot create receiver [" << scoped << "]\n";
      continue;
    }

    ReceiverMount mount;
    mount.link = parent->Data();
    mount.pose = pose->Data();
    mount.sensor = std::move(created);
    mount.name = scoped;
    // Its own sequence for the dropouts and outliers, from the world's seed
    // and its name, so two receivers do not draw the same numbers; a hash
    // that is the same on every standard library, so a seeded run repeats
    // everywhere.
    mount.rng.seed(this->seed ^ acoustics::Hash(scoped));
    this->receivers[sensor] = std::move(mount);
  }
}

//////////////////////////////////////////////////
void AcousticsPrivate::Emit(const acoustics::Duration &_after,
    const acoustics::Duration &_now, const EntityComponentManager &_ecm)
{
  _ecm.Each<components::AcousticPinger>(
      [&](const Entity &_link, const components::AcousticPinger *_comp) -> bool
      {
        const auto &pinger = _comp->Data();
        const auto times = acoustics::EmissionTimes(pinger, _after, _now);
        if (times.empty())
          return true;
        // A pinger out of the water sends nothing.
        const math::Vector3d origin =
            this->PingerPosition(_link, pinger, _ecm);
        if (!this->Wet(origin))
          return true;
        for (const auto &t : times)
        {
          Ping ping;
          ping.frequency = pinger.frequency;
          ping.reach = pinger.reach;
          ping.pulse = acoustics::FromSeconds(pinger.pulse_length);
          ping.origin = origin;
          ping.emitted = t;
          for (const auto &[sensor, mount] : this->receivers)
            ping.waiting.insert(sensor);
          if (!ping.waiting.empty())
            this->pings.push_back(std::move(ping));
        }
        return true;
      });
}

//////////////////////////////////////////////////
void AcousticsPrivate::Propagate(const acoustics::Duration &_now,
    const EntityComponentManager &_ecm)
{
  // The world pose of each receiver, once per update.
  std::unordered_map<Entity, math::Pose3d> poses;
  for (auto it = this->receivers.begin(); it != this->receivers.end();)
  {
    if (!_ecm.HasEntity(it->first))
    {
      it = this->receivers.erase(it);
      continue;
    }
    poses[it->first] = worldPose(it->second.link, _ecm) * it->second.pose;
    ++it;
  }

  for (auto ping = this->pings.begin(); ping != this->pings.end();)
  {
    const double reach = ping->reach;
    const auto elapsed = std::chrono::duration<double>(
        _now - ping->emitted).count();
    // How far the sound has gone; with no speed, everywhere at once.
    const double radius = this->speedOfSound > 0.0 ?
        this->speedOfSound * elapsed : std::numeric_limits<double>::max();

    for (auto r = ping->waiting.begin(); r != ping->waiting.end();)
    {
      const auto pose = poses.find(*r);
      if (pose == poses.end())
      {
        r = ping->waiting.erase(r);
        continue;
      }
      const double distance = pose->second.Pos().Distance(ping->origin);
      if (reach > 0.0 && distance > reach)
      {
        // Beyond the reach it is never heard; forget it once the sound
        // has passed the reach.
        if (radius >= reach)
          r = ping->waiting.erase(r);
        else
          ++r;
        continue;
      }
      if (radius < distance)
      {
        ++r;
        continue;
      }
      // It arrived between the last step and this one: when, from the
      // distance, not from the step that notices it.
      this->Hear(this->receivers[*r], pose->second, *ping, ping->emitted +
          acoustics::TravelTime(distance, this->speedOfSound));
      r = ping->waiting.erase(r);
    }

    if (ping->waiting.empty())
      ping = this->pings.erase(ping);
    else
      ++ping;
  }
}

//////////////////////////////////////////////////
void AcousticsPrivate::Hear(ReceiverMount &_mount,
    const math::Pose3d &_pose, const Ping &_ping,
    const acoustics::Duration &_arrived)
{
  if (!this->Wet(_pose.Pos()))
    return;
  const ReceiverSettings settings = _mount.sensor->Settings();
  if (!acoustics::InBand(_ping.frequency, settings.frequency,
                         settings.bandwidth))
  {
    return;
  }

  const acoustics::Direction truth =
      acoustics::Measure(_pose, _ping.origin);
  if (!acoustics::InElevation(truth.elevation, settings.minElevation,
                              settings.maxElevation))
  {
    return;
  }
  const double lose = acoustics::DropoutProbability(truth.range,
      _ping.reach, settings.dropoutStart);
  if (lose > 0.0 && acoustics::Uniform(_mount.rng) < lose)
    return;

  ReceiverReading reading;
  using Channel = AcousticReceiver::Channel;
  auto &sensor = *_mount.sensor;
  const double dt = sensor.SinceLastReading(_arrived);
  reading.bearing = sensor.ApplyNoise(Channel::kBearing, truth.bearing, dt);
  if (settings.outlierProbability > 0.0 &&
      acoustics::Uniform(_mount.rng) < settings.outlierProbability)
  {
    reading.bearing = acoustics::WrapAngle(
        (2.0 * acoustics::Uniform(_mount.rng) - 1.0) * GZ_PI);
  }
  reading.elevation =
      sensor.ApplyNoise(Channel::kElevation, truth.elevation, dt);
  reading.range = sensor.ApplyNoise(Channel::kRange, truth.range, dt);
  reading.frequency = _ping.frequency;
  reading.amplitude = sensor.ApplyNoise(Channel::kAmplitude,
      acoustics::Amplitude(truth.range), dt);
  reading.emitted = _ping.emitted;
  reading.arrived = _arrived;

  // Two pings in band, the later arriving while the earlier still sounds,
  // are both spoilt.
  for (auto &[held, heldPulse] : _mount.held)
  {
    if (acoustics::Overlap(held.arrived, heldPulse, _arrived, _ping.pulse))
    {
      held.valid = false;
      reading.valid = false;
    }
  }
  // Two pings can be noticed in one step in either order: keep the
  // readings in order of arrival, which is the order they are published in.
  const auto at = std::upper_bound(_mount.held.begin(), _mount.held.end(),
      _arrived,
      [](const acoustics::Duration &_t, const auto &_held)
      {
        return _t < _held.first.arrived;
      });
  _mount.held.emplace(at, reading, _ping.pulse);
}

//////////////////////////////////////////////////
void AcousticsPrivate::Flush(const acoustics::Duration &_now)
{
  for (auto &[sensor, mount] : this->receivers)
  {
    // A reading leaves once its pulse has passed: any ping that could
    // spoil it has been noticed by then.
    while (!mount.held.empty() &&
           mount.held.front().first.arrived + mount.held.front().second <=
           _now)
    {
      mount.sensor->Publish(mount.held.front().first);
      mount.held.pop_front();
    }
  }
}

//////////////////////////////////////////////////
void AcousticsPrivate::PublishPingers(const acoustics::Duration &_now,
    bool _paused, const EntityComponentManager &_ecm)
{
  if (_paused)
  {
    // Simulation time stands still: keep the rate on wall time.
    const auto wall = std::chrono::steady_clock::now();
    if (this->lastPausedPublish &&
        wall - *this->lastPausedPublish < this->publishPeriod)
    {
      return;
    }
    this->lastPausedPublish = wall;
  }
  else
  {
    if (this->lastPublish && _now - *this->lastPublish < this->publishPeriod)
      return;
    this->lastPublish = _now;
  }

  msgs::Param msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  _ecm.Each<components::AcousticPinger>(
      [&](const Entity &_link, const components::AcousticPinger *_comp) -> bool
      {
        const auto &p = _comp->Data();
        const math::Vector3d pos = this->PingerPosition(_link, p, _ecm);
        auto *child = msg.add_children();
        auto set = [child](const std::string &_key, double _value)
        {
          auto &any = (*child->mutable_params())[_key];
          any.set_type(msgs::Any::DOUBLE);
          any.set_double_value(_value);
        };
        auto &name = (*child->mutable_params())["name"];
        name.set_type(msgs::Any::STRING);
        name.set_string_value(p.name);
        set("x", pos.X());
        set("y", pos.Y());
        set("z", pos.Z());
#define GZ_ACOUSTICS_PUB(m, key) set(key, p.m);
        GZ_ACOUSTICS_PINGER_TABLE(GZ_ACOUSTICS_PUB)
#undef GZ_ACOUSTICS_PUB
        auto &active = (*child->mutable_params())["active"];
        active.set_type(msgs::Any::BOOLEAN);
        active.set_bool_value(p.active);
        return true;
      });
  this->pingersPub.Publish(msg);

  // The receivers, which a GUI cannot tell from other custom sensors: the
  // GUI's copy of a sensor loses its gz:type.
  msgs::Param receiversMsg;
  *receiversMsg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  for (const auto &[entity, mount] : this->receivers)
  {
    const ReceiverSettings settings = mount.sensor->Settings();
    auto *child = receiversMsg.add_children();
    auto setString = [child](const std::string &_key, const std::string &_v)
    {
      auto &any = (*child->mutable_params())[_key];
      any.set_type(msgs::Any::STRING);
      any.set_string_value(_v);
    };
    auto setDouble = [child](const std::string &_key, double _v)
    {
      auto &any = (*child->mutable_params())[_key];
      any.set_type(msgs::Any::DOUBLE);
      any.set_double_value(_v);
    };
    setString("name", mount.name);
    setString("topic", mount.sensor->Topic());
    setString("frame_id", mount.sensor->FrameId());
    setDouble("frequency", settings.frequency);
    setDouble("bandwidth", settings.bandwidth);
    setDouble("dropout_start", settings.dropoutStart);
    setDouble("min_elevation", settings.minElevation);
    setDouble("max_elevation", settings.maxElevation);
  }
  this->receiversPub.Publish(receiversMsg);
}

//////////////////////////////////////////////////
Acoustics::Acoustics()
  : dataPtr(std::make_unique<AcousticsPrivate>())
{
}

//////////////////////////////////////////////////
Acoustics::~Acoustics() = default;

//////////////////////////////////////////////////
void Acoustics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.speedOfSound = std::max(
      _sdf->Get<double>("speed_of_sound", d.speedOfSound).first, 0.0);
  d.waterLevel = _sdf->Get<double>("water_level", d.waterLevel).first;
  d.underwaterOnly =
      _sdf->Get<bool>("underwater_only", d.underwaterOnly).first;

  const auto seed = _sdf->Get<std::uint32_t>("seed", 0u).first;
  // The seed of the receivers' own generators only: gz-math's global
  // generator, which every other sensor draws from, is left alone.
  d.seed = 0u != seed ? seed : std::random_device{}();

  const double rate = _sdf->Get<double>("publish_rate", 1.0).first;
  if (rate > 0.0)
  {
    d.publishPeriod = std::chrono::duration_cast<acoustics::Duration>(
        std::chrono::duration<double>(1.0 / rate));
  }

  std::string worldName{"default"};
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  d.pingersPub = d.node.Advertise<msgs::Param>(
      "/world/" + worldName + "/acoustics/pingers");
  d.receiversPub = d.node.Advertise<msgs::Param>(
      "/world/" + worldName + "/acoustics/receivers");
}

//////////////////////////////////////////////////
void Acoustics::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;

  // EachNew only sees an entity in the step it was made in; noting it in
  // PostUpdate, after every PreUpdate, catches it whatever order the
  // systems run in.
  std::vector<Entity> newSensors;
  _ecm.EachNew<components::CustomSensor>(
      [&](const Entity &_sensor, const components::CustomSensor *) -> bool
      {
        newSensors.push_back(_sensor);
        return true;
      });
  d.FindReceivers(_ecm, std::move(newSensors), d.rescan);
  d.rescan = false;

  // Paused, nothing pings, but the lists still go out.
  d.PublishPingers(_info.simTime, _info.paused, _ecm);
  if (_info.paused)
    return;

  // At the first update only the step before is due, not every ping since
  // time zero: the system may be added to a world that has run for long.
  const acoustics::Duration after = d.last.value_or(
      acoustics::FirstWindowStart(_info.simTime, _info.dt));
  d.Emit(after, _info.simTime, _ecm);
  d.Propagate(_info.simTime, _ecm);
  d.Flush(_info.simTime);
  d.last = _info.simTime;
}

//////////////////////////////////////////////////
void Acoustics::Reset(const UpdateInfo &, EntityComponentManager &)
{
  auto &d = *this->dataPtr;
  d.pings.clear();
  d.receivers.clear();
  d.rescan = true;
  d.last.reset();
  d.lastPublish.reset();
  d.lastPausedPublish.reset();
}

GZ_ADD_PLUGIN(Acoustics,
              System,
              Acoustics::ISystemConfigure,
              Acoustics::ISystemPostUpdate,
              Acoustics::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(Acoustics, "gz::sim::maritime::Acoustics")
