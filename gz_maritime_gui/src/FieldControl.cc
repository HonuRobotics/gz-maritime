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
#include "gz/sim/maritime_gui/FieldControl.hh"

#include <algorithm>
#include <cmath>
#include <functional>
#include <locale>
#include <mutex>
#include <optional>
#include <sstream>

#include <gz/common/Console.hh>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief Updates without a recipe before the control asks the world's
  /// state service, about a second at the GUI's update rate.
  constexpr int kFetchAfter{30};
}

class FieldControlPrivate
{
  /// \brief Constructor.
  /// \param[in] _spec The field.
  public: explicit FieldControlPrivate(const FieldSpec &_spec)
    : steering(_spec.MakeSteering())
  {
  }

  /// \brief The field's name on its topics.
  public: std::string field;

  /// \brief The name the plugin logs under.
  public: std::string log;

  /// \brief Steering by hand.
  public: Steering steering;

  /// \brief Whether steering stopped short, until the next change.
  public: bool yielded{false};

  /// \brief The `set` topic's publisher, advertised as soon as the world's
  /// name is known: a message published right after advertising is lost,
  /// before the subscriber is discovered.
  public: std::optional<transport::Node::Publisher> publisher;

  /// \brief The world's name, once the ECM has it.
  public: std::string worldName;

  /// \brief What became of the last change sent.
  public: ChangeTracker tracker;

  /// \brief Updates without the recipe in the GUI's ECM.
  public: int missing{0};

  /// \brief Whether the state service was asked.
  public: bool fetchAsked{false};

  /// \brief The state service's answer, until an update applies it.
  public: std::optional<msgs::SerializedStepMap> fetched;

  /// \brief A copy of the world from the state service.
  public: std::unique_ptr<EntityComponentManager> snapshot;

  /// \brief An empty ECM, for a source before any copy came.
  public: EntityComponentManager empty;

  /// \brief Guards `fetched` across the transport and Qt threads.
  public: std::mutex mutex;

  /// \brief Transport node. Last, so it is destroyed first: a state service
  /// reply arriving while the panel closes must not find `fetched` and
  /// `mutex` gone.
  public: transport::Node node;
};

//////////////////////////////////////////////////
FieldControl::FieldControl(const FieldSpec &_spec, const std::string &_log)
  : dataPtr(std::make_unique<FieldControlPrivate>(_spec))
{
  this->dataPtr->field = _spec.name;
  this->dataPtr->log = _log;
}

//////////////////////////////////////////////////
FieldControl::~FieldControl() = default;

//////////////////////////////////////////////////
void FieldControl::Update(const EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (d.worldName.empty())
  {
    const Entity world = _ecm.EntityByComponents(components::World());
    if (const auto *name = _ecm.Component<components::Name>(world))
    {
      d.worldName = name->Data();
      d.publisher = d.node.Advertise<msgs::Param>(
          "/world/" + d.worldName + "/" + d.field + "/set");
    }
  }

  const std::lock_guard<std::mutex> lock(d.mutex);
  if (d.fetched)
  {
    d.snapshot = std::make_unique<EntityComponentManager>();
    d.snapshot->SetState(d.fetched->state());
    d.fetched.reset();
  }
}

//////////////////////////////////////////////////
const EntityComponentManager &FieldControl::Source(
    const EntityComponentManager &_ecm, bool _inGui)
{
  auto &d = *this->dataPtr;
  if (_inGui)
    return _ecm;

  // A plugin loaded after the GUI received the world's first state never
  // got the recipe; ask once for the whole state.
  if (!d.fetchAsked && ++d.missing > kFetchAfter && !d.worldName.empty())
  {
    d.fetchAsked = true;
    gzdbg << d.log << ": no recipe in the GUI's state yet; asking the "
          << "world's state service\n";
    std::function<void(const msgs::SerializedStepMap &, const bool)> cb =
        [&d](const msgs::SerializedStepMap &_rep, const bool _result)
        {
          if (!_result)
            return;
          const std::lock_guard<std::mutex> lock(d.mutex);
          d.fetched = _rep;
        };
    d.node.Request("/world/" + d.worldName + "/state", cb);
  }
  return d.snapshot ? *d.snapshot : d.empty;
}

//////////////////////////////////////////////////
void FieldControl::LogSource(const std::string &_what, bool _fromGui,
    std::uint64_t _generation) const
{
  gzdbg << this->dataPtr->log << ": " << _what << " from the "
        << (_fromGui ? "GUI's state" : "state service") << ", generation "
        << _generation << "\n";
}

//////////////////////////////////////////////////
void FieldControl::Send(const msgs::Param &_msg, std::uint64_t _generation,
    ChangeTracker::Clock::time_point _now)
{
  auto &d = *this->dataPtr;
  if (!d.publisher || !d.publisher->HasConnections() ||
      !d.publisher->Publish(_msg))
  {
    d.tracker.NotSent();
    return;
  }
  d.tracker.Sent(_generation, _now);
  d.yielded = false;
}

//////////////////////////////////////////////////
ChangeTracker::State FieldControl::Track(std::uint64_t _generation,
    ChangeTracker::Clock::time_point _now)
{
  return this->dataPtr->tracker.Update(_generation, _now);
}

//////////////////////////////////////////////////
void FieldControl::Steer(double _speed, double _direction)
{
  this->dataPtr->steering.SetTarget(_speed, _direction);
  this->dataPtr->yielded = false;
}

//////////////////////////////////////////////////
void FieldControl::StepSteering(double _speed, double _direction,
    std::uint64_t _generation, Steering::Clock::time_point _now)
{
  auto &d = *this->dataPtr;
  const auto msg = d.steering.Step(_speed, _direction, _now);
  if (d.steering.Yielded())
    d.yielded = true;
  if (!msg)
    return;
  if (d.steering.Arrived())
  {
    // The last step is tracked like a change from Apply.
    this->Send(*msg, _generation, _now);
  }
  else if (d.publisher && d.publisher->HasConnections())
  {
    d.publisher->Publish(*msg);
  }
}

//////////////////////////////////////////////////
bool FieldControl::IsSteering() const
{
  return this->dataPtr->steering.Active();
}

//////////////////////////////////////////////////
bool FieldControl::Yielded() const
{
  return this->dataPtr->yielded;
}

//////////////////////////////////////////////////
std::string StatusText(ChangeTracker::State _state)
{
  switch (_state)
  {
    case ChangeTracker::State::kPending:
      return "Sending...";
    case ChangeTracker::State::kApplied:
      return "Applied.";
    case ChangeTracker::State::kRefused:
      return "Not applied: the server kept its values; its log says why.";
    case ChangeTracker::State::kUnheard:
      return "Not sent: nothing is listening on the topic yet.";
    default:
      return "";
  }
}

//////////////////////////////////////////////////
std::string YieldedText()
{
  return "Stopped steering: the field was changed elsewhere.";
}

//////////////////////////////////////////////////
bool ParseNumber(const std::string &_text, double &_value)
{
  std::string text = _text;
  std::replace(text.begin(), text.end(), ',', '.');
  // The classic locale, so the process's (which Qt sets from the
  // environment) cannot make a decimal point unreadable.
  std::istringstream is(text);
  is.imbue(std::locale::classic());
  double v{0.0};
  if (!(is >> v))
    return false;
  is >> std::ws;
  if (!is.eof() || !std::isfinite(v))
    return false;
  _value = v;
  return true;
}
}  // namespace gz::sim::maritime_gui
