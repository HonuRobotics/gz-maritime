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
  /// \brief The per-channel tags, and the spec member each sets.
  struct VectorTag
  {
    const char *name;
    std::optional<water_optics::Rgb> water_optics::WaterOpticsSpec::*member;
  };
  const VectorTag kVectorTags[] = {
      {"attenuation", &water_optics::WaterOpticsSpec::attenuation},
      {"backscatter", &water_optics::WaterOpticsSpec::backscatter},
      {"veil", &water_optics::WaterOpticsSpec::veil},
      {"diffuse_attenuation",
       &water_optics::WaterOpticsSpec::diffuseAttenuation}};

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

  /// \brief The resolved optics.
  public: water_optics::WaterOpticsData optics;

  /// \brief Whether the world file's spec was valid. If not, the world has
  /// no water optics (clear water) and the topic is not offered.
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

  /// \brief Ground truth period, simulation time.
  public: std::chrono::steady_clock::duration publishPeriod{
      std::chrono::seconds(1)};

  /// \brief Simulation time of the last publication.
  public: std::optional<std::chrono::steady_clock::duration> lastPublish;
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
    if ("visibility" == key || "water_level" == key)
    {
      if ("visibility" == key && empty)
      {
        spec.visibility.reset();
        continue;
      }
      const auto number = ReadNumber(value);
      if (!number)
      {
        gzwarn << "WaterOptics: '" << key << "' needs a number; message "
               << "ignored\n";
        return false;
      }
      if ("visibility" == key)
        spec.visibility = *number;
      else
        spec.waterLevel = *number;
      continue;
    }
    bool known = false;
    for (const auto &tag : kVectorTags)
    {
      if (key != tag.name)
        continue;
      known = true;
      if (empty)
      {
        (spec.*tag.member).reset();
        break;
      }
      const auto rgb = text ? ParseRgb(value.string_value()) : std::nullopt;
      if (!rgb)
      {
        gzwarn << "WaterOptics: '" << key << "' needs three numbers as a "
               << "string, \"r g b\"; message ignored\n";
        return false;
      }
      spec.*tag.member = *rgb;
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
  this->optics = optics;
  return true;
}

//////////////////////////////////////////////////
void WaterOpticsPrivate::Write(EntityComponentManager &_ecm)
{
  ++this->optics.generation;
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
  const std::set<std::string> known{"jerlov", "visibility", "water_level",
      "publish_rate", "attenuation", "backscatter", "veil",
      "diffuse_attenuation"};
  for (auto e = _sdf->GetFirstElement(); e; e = e->GetNextElement())
  {
    if (!known.count(e->GetName()))
    {
      gzwarn << "WaterOptics: unknown element <" << e->GetName()
             << ">, ignored\n";
    }
  }

  auto &spec = d.spec;
  spec.jerlov = _sdf->Get<std::string>("jerlov", "").first;
  if (_sdf->HasElement("visibility"))
    spec.visibility = _sdf->Get<double>("visibility");
  spec.waterLevel = _sdf->Get<double>("water_level", 0.0).first;
  for (const auto &tag : kVectorTags)
  {
    if (!_sdf->HasElement(tag.name))
      continue;
    const auto rgb = ParseRgb(_sdf->Get<std::string>(tag.name));
    if (!rgb)
    {
      gzerr << "WaterOptics: <" << tag.name << "> needs three numbers, "
            << "r g b; the world has clear water\n";
      return;
    }
    spec.*tag.member = *rgb;
  }
  const double rate = _sdf->Get<double>("publish_rate", 1.0).first;
  if (rate > 0.0)
  {
    d.publishPeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
  }

  std::string error, warning;
  if (!water_optics::Resolve(spec, d.optics, error, warning))
  {
    gzerr << "WaterOptics: " << error << "; the world has clear water\n";
    return;
  }
  if (!warning.empty())
    gzwarn << "WaterOptics: " << warning << "\n";
  d.initialSpec = spec;
  d.valid = true;
  d.Write(_ecm);

  std::string worldName = "default";
  if (const auto *name = _ecm.Component<components::Name>(_entity))
    worldName = name->Data();
  const std::string base = "/world/" + worldName;
  d.node.Subscribe(base + "/water_optics/set", &WaterOpticsPrivate::OnSet,
                   &d);
  d.pub = d.node.Advertise<msgs::Param>(base + "/water_optics_info");

  gzmsg << "WaterOptics: Jerlov " << d.optics.jerlov << ", visibility "
        << d.optics.Visibility() << " m, attenuation "
        << ToText(d.optics.attenuation) << " 1/m\n";
}

//////////////////////////////////////////////////
void WaterOptics::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (!d.valid)
    return;
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
void WaterOptics::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &)
{
  auto &d = *this->dataPtr;
  if (!d.valid || _info.paused)
    return;
  if (d.lastPublish && _info.simTime - *d.lastPublish < d.publishPeriod)
    return;

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
  setNumber("visibility", d.optics.Visibility());
  setText("attenuation", ToText(d.optics.attenuation));
  setText("backscatter", ToText(d.optics.backscatter));
  setText("veil", ToText(d.optics.veil));
  setText("diffuse_attenuation", ToText(d.optics.diffuseAttenuation));
  setNumber("water_level", d.optics.waterLevel);
  setNumber("generation", static_cast<double>(d.optics.generation));
  d.pub.Publish(msg);
  d.lastPublish = _info.simTime;
}

//////////////////////////////////////////////////
void WaterOptics::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (!d.valid)
    return;
  d.spec = d.initialSpec;
  std::string error, warning;
  water_optics::Resolve(d.spec, d.optics, error, warning);
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
