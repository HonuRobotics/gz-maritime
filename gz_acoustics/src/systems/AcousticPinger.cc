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
#include "AcousticPinger.hh"

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/param.pb.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>

#include "gz/sim/acoustics/Pinger.hh"
#include "gz/sim/components/AcousticPinger.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

class gz::sim::maritime::AcousticPingerPrivate
{
  /// \brief Handler of the set topic, on a transport thread.
  /// \param[in] _msg Parameter names and values.
  public: void OnSet(const msgs::Param &_msg);

  /// \brief Write the pinger into its link's component.
  /// \param[in] _ecm The entity component manager.
  public: void Write(EntityComponentManager &_ecm);

  /// \brief The link that carries the pinger.
  public: Entity link{kNullEntity};

  /// \brief The pinger as it is now.
  public: acoustics::PingerData pinger;

  /// \brief The pinger as the SDF set it, for a reset.
  public: acoustics::PingerData initial;

  /// \brief Changes queued by the topic, applied at the next step.
  public: std::vector<std::pair<std::string, double>> pending;

  /// \brief Guards the queue across the transport and ECM threads.
  public: std::mutex mutex;

  /// \brief Transport node for the set topic.
  public: transport::Node node;
};

//////////////////////////////////////////////////
void AcousticPingerPrivate::OnSet(const msgs::Param &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &[key, value] : _msg.params())
  {
    double d{0.0};
    switch (value.type())
    {
      case msgs::Any::DOUBLE: d = value.double_value(); break;
      case msgs::Any::INT32: d = value.int_value(); break;
      case msgs::Any::BOOLEAN: d = value.bool_value() ? 1.0 : 0.0; break;
      default:
        gzwarn << "AcousticPinger [" << this->pinger.name << "]: key '" << key
               << "' is not a number or a boolean, ignored\n";
        continue;
    }
    this->pending.emplace_back(key, d);
  }
}

//////////////////////////////////////////////////
void AcousticPingerPrivate::Write(EntityComponentManager &_ecm)
{
  ++this->pinger.generation;
  // SetComponentData creates the component or updates it, but leaves the
  // change unmarked, and the mark is what replicates it.
  if (_ecm.SetComponentData<components::AcousticPinger>(this->link,
                                                         this->pinger))
  {
    _ecm.SetChanged(this->link, components::AcousticPinger::typeId,
                    ComponentState::OneTimeChange);
  }
}

//////////////////////////////////////////////////
AcousticPinger::AcousticPinger()
  : dataPtr(std::make_unique<AcousticPingerPrivate>())
{
}

//////////////////////////////////////////////////
AcousticPinger::~AcousticPinger() = default;

//////////////////////////////////////////////////
void AcousticPinger::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "AcousticPinger: must be attached to a model\n";
    return;
  }

  // The <name> element's own value: Get("name") on the plugin element
  // would answer with the plugin's name attribute.
  d.pinger.name = _sdf->HasElement("name") ?
      _sdf->FindElement("name")->Get<std::string>() : model.Name(_ecm);
  const std::string linkName = _sdf->Get<std::string>("link_name", "").first;
  d.link = linkName.empty() ? model.CanonicalLink(_ecm)
                            : model.LinkByName(_ecm, linkName);
  if (kNullEntity == d.link)
  {
    gzerr << "AcousticPinger [" << d.pinger.name << "]: model ["
          << model.Name(_ecm) << "] has no link [" << linkName << "]\n";
    return;
  }
  if (nullptr != _ecm.Component<components::AcousticPinger>(d.link))
  {
    gzerr << "AcousticPinger [" << d.pinger.name << "]: link already carries "
          << "a pinger; one pinger per link\n";
    d.link = kNullEntity;
    return;
  }

  d.pinger.pose = _sdf->Get<math::Pose3d>("pose", math::Pose3d::Zero).first;
  d.pinger.active = _sdf->Get<bool>("active", true).first;
#define GZ_ACOUSTICS_LOAD(m, key)                                       \
  if (_sdf->HasElement(key) &&                                          \
      !acoustics::SetParameter(d.pinger, key, _sdf->Get<double>(key))) \
  {                                                                      \
    gzwarn << "AcousticPinger [" << d.pinger.name << "]: <" << key       \
           << "> out of range, default kept\n";                          \
  }
  GZ_ACOUSTICS_PINGER_TABLE(GZ_ACOUSTICS_LOAD)
#undef GZ_ACOUSTICS_LOAD

  d.initial = d.pinger;
  d.Write(_ecm);

  // The world's name scopes the topic, like the wind's.
  std::string worldName{"default"};
  if (const auto *name =
      _ecm.Component<components::Name>(worldEntity(_ecm)))
  {
    worldName = name->Data();
  }
  const std::string topic =
      "/world/" + worldName + "/acoustics/" + d.pinger.name + "/set";
  if (!d.node.Subscribe(topic, &AcousticPingerPrivate::OnSet, &d))
    gzerr << "AcousticPinger: cannot subscribe to " << topic << "\n";
}

//////////////////////////////////////////////////
void AcousticPinger::PreUpdate(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (kNullEntity == d.link)
    return;

  bool changed{false};
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    for (const auto &[key, value] : d.pending)
    {
      if (acoustics::SetParameter(d.pinger, key, value))
      {
        changed = true;
      }
      else
      {
        gzwarn << "AcousticPinger [" << d.pinger.name << "]: key '" << key
               << "' is unknown or out of range, ignored\n";
      }
    }
    d.pending.clear();
  }
  if (changed)
    d.Write(_ecm);
}

//////////////////////////////////////////////////
void AcousticPinger::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (kNullEntity == d.link)
    return;
  // Back to what the SDF set, under a new generation.
  const auto generation = d.pinger.generation;
  d.pinger = d.initial;
  d.pinger.generation = generation;
  d.Write(_ecm);
}

GZ_ADD_PLUGIN(AcousticPinger,
              System,
              AcousticPinger::ISystemConfigure,
              AcousticPinger::ISystemPreUpdate,
              AcousticPinger::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(AcousticPinger, "gz::sim::maritime::AcousticPinger")
