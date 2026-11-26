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
#include "WaterOptics.hh"

#include <chrono>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/param.pb.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Name.hh>

#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/water_optics/WaterOptics.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief Three numbers from text.
  std::optional<water_optics::Rgb> ParseRgb(const std::string &_text)
  {
    std::istringstream is(_text);
    double r{0}, g{0}, b{0};
    if (!(is >> r >> g >> b))
      return std::nullopt;
    std::string rest;
    if (is >> rest)
      return std::nullopt;
    return water_optics::Rgb(r, g, b);
  }

  /// \brief A number from a numeric or text Any.
  std::optional<double> ReadNumber(const msgs::Any &_v)
  {
    switch (_v.type())
    {
      case msgs::Any::DOUBLE: return _v.double_value();
      case msgs::Any::INT32: return _v.int_value();
      case msgs::Any::STRING:
      {
        std::istringstream is(_v.string_value());
        double d{0};
        std::string rest;
        if (is >> d && !(is >> rest))
          return d;
        return std::nullopt;
      }
      default: return std::nullopt;
    }
  }

  /// \brief Three numbers as text, at full precision.
  std::string ToText(const water_optics::Rgb &_v)
  {
    std::ostringstream os;
    os << std::setprecision(std::numeric_limits<double>::max_digits10)
       << _v.X() << ' ' << _v.Y() << ' ' << _v.Z();
    return os.str();
  }
}

class gz::sim::maritime::WaterOpticsPrivate
{
  /// \brief Handler of the set topic, on a transport thread.
  /// \param[in] _msg Keys and values.
  public: void OnSet(const msgs::Param &_msg);

  /// \brief Apply one message to the spec, all of it or nothing.
  /// \param[in] _msg The message.
  /// \return True if it changed the optics.
  public: bool Apply(const msgs::Param &_msg);

  /// \brief Write the optics into the world's component, marked changed.
  /// \param[in] _ecm The entity component manager.
  public: void Write(EntityComponentManager &_ecm);

  /// \brief What the world asks for.
  public: water_optics::WaterOpticsSpec spec;

  /// \brief The spec as the world file set it, for a reset.
  public: water_optics::WaterOpticsSpec initialSpec;

  /// \brief Whether the world file's spec was valid: if not, a reset goes
  /// back to clear water.
  public: bool initialValid{false};

  /// \brief The resolved optics.
  public: water_optics::WaterOpticsData optics;

  /// \brief Whether the world has water optics. Not until a valid spec
  /// came, from the world file or, after a bad one, the topic: until then
  /// the world has clear water.
  public: bool valid{false};

  /// \brief The world entity.
  public: Entity world{kNullEntity};

  /// \brief Messages from the topic, applied at the next step.
  public: std::vector<msgs::Param> pending;

  /// \brief Guards the queue across threads.
  public: std::mutex mutex;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Ground truth publisher.
  public: transport::Node::Publisher pub;

  /// \brief Ground truth period, wall time, so it keeps coming while the
  /// world is paused; none to publish only on a change.
  public: std::optional<std::chrono::steady_clock::duration> publishPeriod{
      std::chrono::seconds(1)};

  /// \brief Wall time of the last publication; none to publish at the
  /// next step, e.g. after a change.
  public: std::optional<std::chrono::steady_clock::time_point> lastPublish;
};

//////////////////////////////////////////////////
void WaterOpticsPrivate::OnSet(const msgs::Param &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  this->pending.push_back(_msg);
}

//////////////////////////////////////////////////
bool WaterOpticsPrivate::Apply(const msgs::Param &_msg)
{
  auto spec = this->spec;
  for (const auto &[key, value] : _msg.params())
  {
    const bool text = value.type() == msgs::Any::STRING;
    const bool empty = text && value.string_value().empty();
    if ("jerlov" == key)
    {
      if (!text)
      {
        gzwarn << "WaterOptics: 'jerlov' needs a string; message ignored\n";
        return false;
      }
      spec.jerlov = value.string_value();
      continue;
    }
    bool known = false;
    for (const auto &param : water_optics::NumberParams())
    {
      if (key != param.name)
        continue;
      known = true;
      if (empty && param.removable)
      {
        param.set(spec, std::nullopt);
        break;
      }
      const auto number = ReadNumber(value);
      if (!number)
      {
        gzwarn << "WaterOptics: '" << key << "' needs a number; message "
               << "ignored\n";
        return false;
      }
      param.set(spec, *number);
      break;
    }
    for (const auto &param : water_optics::ColourParams())
    {
      if (known || key != param.name)
        continue;
      known = true;
      if (empty)
      {
        (spec.*param.spec).reset();
        break;
      }
      const auto rgb = text ? ParseRgb(value.string_value()) : std::nullopt;
      if (!rgb)
      {
        gzwarn << "WaterOptics: '" << key << "' needs three numbers as a "
               << "string, \"r g b\"; message ignored\n";
        return false;
      }
      spec.*param.spec = *rgb;
      break;
    }
    if (!known)
    {
      gzwarn << "WaterOptics: unknown key '" << key << "'; message "
             << "ignored\n";
      return false;
    }
  }

  water_optics::WaterOpticsData optics = this->optics;
  std::string error, warning;
  if (!water_optics::Resolve(spec, optics, error, warning))
  {
    gzwarn << "WaterOptics: " << error << "; message ignored\n";
    return false;
  }
  if (!warning.empty())
    gzwarn << "WaterOptics: " << warning << "\n";
  this->spec = spec;
  // The same water, e.g. an empty message or the values it has: nothing
  // to replicate or publish.
  if (this->valid && water_optics::SameWater(optics, this->optics))
    return false;
  this->optics = optics;
  this->valid = true;
  return true;
}

//////////////////////////////////////////////////
void WaterOpticsPrivate::Write(EntityComponentManager &_ecm)
{
  ++this->optics.generation;
  if (!_ecm.Component<components::WaterOptics>(this->world))
  {
    // The first good water, after a world file with a bad one
    _ecm.CreateComponent(this->world, components::WaterOptics(this->optics));
    return;
  }
  // SetComponentData leaves the change unmarked, and the mark is what
  // replicates the optics to the GUI and sensors.
  if (_ecm.SetComponentData<components::WaterOptics>(this->world,
                                                      this->optics))
  {
    _ecm.SetChanged(this->world, components::WaterOptics::typeId,
                    ComponentState::OneTimeChange);
  }
}

//////////////////////////////////////////////////
WaterOptics::WaterOptics()
  : dataPtr(std::make_unique<WaterOpticsPrivate>())
{
}

//////////////////////////////////////////////////
WaterOptics::~WaterOptics() = default;

//////////////////////////////////////////////////
void WaterOptics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.world = _entity;

  // Every element the plugin reads; any other is warned about, so a typo
  // such as <visibilty> does not leave the water unchanged in silence.
  std::set<std::string> known{"jerlov", "publish_rate"};
  for (const auto &param : water_optics::NumberParams())
    known.insert(param.name);
  for (const auto &param : water_optics::ColourParams())
    known.insert(param.name);
  for (auto e = _sdf->GetFirstElement(); e; e = e->GetNextElement())
  {
    if (!known.count(e->GetName()))
    {
      gzwarn << "WaterOptics: unknown element <" << e->GetName()
             << ">, ignored\n";
    }
  }

  const double rate = _sdf->Get<double>("publish_rate", 1.0).first;
  if (rate > 0.0)
  {
    d.publishPeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
  }
  else if (rate == 0.0)
  {
    d.publishPeriod.reset();
  }
  else
  {
    gzwarn << "WaterOptics: <publish_rate> must not be negative; 1 Hz\n";
  }

  // The topics come first: after a bad world file, a good message on the
  // set topic still brings the water.
  std::string worldName = "default";
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  const std::string base = "/world/" + worldName;
  d.node.Subscribe(base + "/water_optics/set", &WaterOpticsPrivate::OnSet,
                   &d);
  d.pub = d.node.Advertise<msgs::Param>(base + "/water_optics_info");

  auto &spec = d.spec;
  spec.jerlov = _sdf->Get<std::string>("jerlov", "").first;
  for (const auto &param : water_optics::NumberParams())
  {
    if (_sdf->HasElement(param.name))
      param.set(spec, _sdf->Get<double>(param.name));
  }
  for (const auto &param : water_optics::ColourParams())
  {
    if (!_sdf->HasElement(param.name))
      continue;
    const auto rgb = ParseRgb(_sdf->Get<std::string>(param.name));
    if (!rgb)
    {
      gzerr << "WaterOptics: <" << param.name << "> needs three numbers, "
            << "r g b; the world has clear water until a good one comes on "
            << base << "/water_optics/set\n";
      d.spec = water_optics::WaterOpticsSpec{};
      return;
    }
    spec.*param.spec = *rgb;
  }

  std::string error, warning;
  if (!water_optics::Resolve(spec, d.optics, error, warning))
  {
    gzerr << "WaterOptics: " << error << "; the world has clear water until "
          << "a good one comes on " << base << "/water_optics/set\n";
    d.spec = water_optics::WaterOpticsSpec{};
    return;
  }
  if (!warning.empty())
    gzwarn << "WaterOptics: " << warning << "\n";
  d.initialSpec = spec;
  d.initialValid = true;
  d.valid = true;
  d.Write(_ecm);

  gzmsg << "WaterOptics: Jerlov " << d.optics.jerlov << ", visibility "
        << d.optics.Visibility() << " m, attenuation "
        << ToText(d.optics.attenuation) << " 1/m\n";
}

//////////////////////////////////////////////////
void WaterOptics::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  std::vector<msgs::Param> pending;
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    pending.swap(d.pending);
  }
  bool changed = false;
  for (const auto &msg : pending)
    changed = d.Apply(msg) || changed;
  if (changed)
  {
    d.Write(_ecm);
    d.lastPublish.reset();
  }
}

//////////////////////////////////////////////////
void WaterOptics::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &)
{
  auto &d = *this->dataPtr;
  if (!d.valid)
    return;
  // On a change, and every period of wall time, paused or not: the optics
  // do not depend on time, and a paused world still has its water.
  const auto now = std::chrono::steady_clock::now();
  if (d.lastPublish &&
      (!d.publishPeriod || now - *d.lastPublish < *d.publishPeriod))
  {
    return;
  }

  msgs::Param msg;
  auto &p = *msg.mutable_params();
  auto setText = [&p](const std::string &_k, const std::string &_v)
  {
    p[_k].set_type(msgs::Any::STRING);
    p[_k].set_string_value(_v);
  };
  auto setNumber = [&p](const std::string &_k, double _v)
  {
    p[_k].set_type(msgs::Any::DOUBLE);
    p[_k].set_double_value(_v);
  };
  setText("jerlov", d.optics.jerlov);
  for (const auto &param : water_optics::NumberParams())
    setNumber(param.name, param.get(d.optics));
  for (const auto &param : water_optics::ColourParams())
    setText(param.name, ToText(d.optics.*param.data));
  setNumber("generation", static_cast<double>(d.optics.generation));
  d.pub.Publish(msg);
  d.lastPublish = now;
}

//////////////////////////////////////////////////
void WaterOptics::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    d.pending.clear();
  }
  if (!d.initialValid)
  {
    // Back to the clear water of a bad world file
    d.spec = water_optics::WaterOpticsSpec{};
    d.valid = false;
    if (_ecm.Component<components::WaterOptics>(d.world))
      _ecm.RemoveComponent<components::WaterOptics>(d.world);
    return;
  }
  d.spec = d.initialSpec;
  std::string error, warning;
  water_optics::Resolve(d.spec, d.optics, error, warning);
  d.valid = true;
  d.Write(_ecm);
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    d.pending.clear();
  }
  d.lastPublish.reset();
}

GZ_ADD_PLUGIN(WaterOptics,
              System,
              WaterOptics::ISystemConfigure,
              WaterOptics::ISystemPreUpdate,
              WaterOptics::ISystemPostUpdate,
              WaterOptics::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(WaterOptics, "gz::sim::maritime::WaterOptics")
