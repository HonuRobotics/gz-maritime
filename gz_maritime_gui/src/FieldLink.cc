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
#include "gz/sim/maritime_gui/FieldLink.hh"

#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
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
  /// \brief The id of a field's marker. Not 0: MarkerManager gives a marker
  /// with id 0 a new random id, so every redraw would add one.
  constexpr std::uint64_t kMarkerId{1};

  /// \brief Updates without a recipe before the link asks the world's state
  /// service, about a second at the GUI's update rate.
  constexpr int kFetchAfter{30};

  /// \brief Period of simulation time between two redraws of a field that
  /// changes on its own.
  constexpr std::chrono::milliseconds kRedrawPeriod{200};

  /// \brief A child element's number, if it has one.
  /// \param[in] _elem The plugin element.
  /// \param[in] _name The child's name.
  /// \param[in,out] _value Set when the child is a number.
  void ReadDouble(const tinyxml2::XMLElement *_elem, const char *_name,
                  double &_value)
  {
    if (const auto *child = _elem->FirstChildElement(_name))
      child->QueryDoubleText(&_value);
  }
}

class FieldLinkPrivate
{
  /// \brief Send a marker to MarkerManager. Its service is one way: it
  /// takes a marker and answers nothing.
  /// \param[in] _marker The marker.
  public: void SendMarker(const msgs::Marker &_marker)
  {
    this->node.Request(this->markerService, _marker);
  }

  /// \brief The field's name on its topics.
  public: std::string field;

  /// \brief The name the plugin logs under.
  public: std::string log;

  /// \brief Marker namespace.
  public: std::string ns;

  /// \brief Arrows' colour.
  public: math::Color color;

  /// \brief m of arrow per m/s.
  public: double scale{1.0};

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief The `set` topic's publisher, advertised as soon as the world's
  /// name is known: a message published right after advertising is lost,
  /// before the subscriber is discovered.
  public: std::optional<transport::Node::Publisher> publisher;

  /// \brief The world's name, once the ECM has it.
  public: std::string worldName;

  /// \brief MarkerManager's service.
  public: std::string markerService{"/marker"};

  /// \brief What became of the last change sent.
  public: ChangeTracker tracker;

  /// \brief The grid.
  public: Grid grid;

  /// \brief Whether the user wants the arrows.
  public: bool show{false};

  /// \brief Whether a marker is on the scene.
  public: bool drawn{false};

  /// \brief A setting changed since the last drawing.
  public: bool dirty{true};

  /// \brief The recipe generation last drawn.
  public: std::uint64_t drawnGeneration{0};

  /// \brief Simulation time of the last drawing.
  public: std::chrono::steady_clock::duration lastDraw{0};

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
};

//////////////////////////////////////////////////
FieldLink::FieldLink(const std::string &_field, const std::string &_log,
    const math::Color &_color, double _scale)
  : dataPtr(std::make_unique<FieldLinkPrivate>())
{
  auto &d = *this->dataPtr;
  d.field = _field;
  d.log = _log;
  d.ns = "gz_maritime_gui/" + _field;
  d.color = _color;
  d.scale = _scale;
}

//////////////////////////////////////////////////
FieldLink::~FieldLink()
{
  // Closing the panel takes its arrows off the scene.
  if (this->dataPtr->drawn)
    this->dataPtr->SendMarker(DeleteMarker(this->dataPtr->ns, kMarkerId));
}

//////////////////////////////////////////////////
void FieldLink::LoadConfig(const tinyxml2::XMLElement *_elem)
{
  if (!_elem)
    return;
  auto &d = *this->dataPtr;
  if (const auto *child = _elem->FirstChildElement("show_arrows"))
    child->QueryBoolText(&d.show);
  ReadDouble(_elem, "extent", d.grid.extent);
  ReadDouble(_elem, "spacing", d.grid.spacing);
  double x{d.grid.center.X()};
  double y{d.grid.center.Y()};
  ReadDouble(_elem, "center_x", x);
  ReadDouble(_elem, "center_y", y);
  d.grid.center.Set(x, y);
  ReadDouble(_elem, "scale", d.scale);
  if (const auto *child = _elem->FirstChildElement("marker_service"))
  {
    if (child->GetText())
      d.markerService = child->GetText();
  }

  if (!(d.grid.extent >= 0.0))
  {
    gzwarn << d.log << ": <extent> cannot be negative, using 40\n";
    d.grid.extent = 40.0;
  }
  if (!(d.grid.spacing > 0.0))
  {
    gzwarn << d.log << ": <spacing> must be positive, using 5\n";
    d.grid.spacing = 5.0;
  }
}

//////////////////////////////////////////////////
void FieldLink::Update(const EntityComponentManager &_ecm)
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
const EntityComponentManager &FieldLink::Source(
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
void FieldLink::LogSource(const std::string &_what, bool _fromGui,
    std::uint64_t _generation) const
{
  gzdbg << this->dataPtr->log << ": " << _what << " from the "
        << (_fromGui ? "GUI's state" : "state service") << ", generation "
        << _generation << "\n";
}

//////////////////////////////////////////////////
void FieldLink::Send(const msgs::Param &_msg, std::uint64_t _generation)
{
  auto &d = *this->dataPtr;
  if (!d.publisher || !d.publisher->HasConnections() ||
      !d.publisher->Publish(_msg))
  {
    d.tracker.NotSent();
    return;
  }
  d.tracker.Sent(_generation, ChangeTracker::Clock::now());
}

//////////////////////////////////////////////////
ChangeTracker::State FieldLink::Track(std::uint64_t _generation)
{
  return this->dataPtr->tracker.Update(_generation,
                                       ChangeTracker::Clock::now());
}

//////////////////////////////////////////////////
void FieldLink::Draw(bool _available, std::uint64_t _generation,
    bool _timeVarying, double _z,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    const std::chrono::steady_clock::duration &_simTime)
{
  auto &d = *this->dataPtr;
  if (!d.show || !_available)
  {
    if (d.drawn)
      d.SendMarker(DeleteMarker(d.ns, kMarkerId));
    d.drawn = false;
    return;
  }

  const bool due = !d.drawn || d.dirty || _generation != d.drawnGeneration ||
      (_timeVarying && _simTime - d.lastDraw >= kRedrawPeriod);
  if (!due)
    return;

  const auto segments =
      ArrowSegments(GridPoints(d.grid, _z), _velocity, d.scale);
  // A field with no velocity anywhere has nothing to draw.
  if (segments.empty())
    d.SendMarker(DeleteMarker(d.ns, kMarkerId));
  else
    d.SendMarker(ArrowMarker(d.ns, kMarkerId, d.color, segments));
  d.drawn = true;
  d.dirty = false;
  d.drawnGeneration = _generation;
  d.lastDraw = _simTime;
}

//////////////////////////////////////////////////
bool FieldLink::ShowArrows() const
{
  return this->dataPtr->show;
}

//////////////////////////////////////////////////
void FieldLink::SetShowArrows(bool _show)
{
  this->dataPtr->show = _show;
  this->dataPtr->dirty = true;
}

//////////////////////////////////////////////////
const Grid &FieldLink::ArrowGrid() const
{
  return this->dataPtr->grid;
}

//////////////////////////////////////////////////
bool FieldLink::SetExtent(double _extent)
{
  if (!(_extent >= 0.0) || !std::isfinite(_extent))
    return false;
  this->dataPtr->grid.extent = _extent;
  this->dataPtr->dirty = true;
  return true;
}

//////////////////////////////////////////////////
bool FieldLink::SetSpacing(double _spacing)
{
  if (!(_spacing > 0.0) || !std::isfinite(_spacing))
    return false;
  this->dataPtr->grid.spacing = _spacing;
  this->dataPtr->dirty = true;
  return true;
}

//////////////////////////////////////////////////
double FieldLink::Scale() const
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
void FieldLink::Redraw()
{
  this->dataPtr->dirty = true;
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
