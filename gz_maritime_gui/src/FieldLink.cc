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
#include <map>
#include <optional>
#include <set>
#include <sstream>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief The id of a drawing's first colour bin. Not 0: MarkerManager
  /// gives a marker with id 0 a new random id, so every redraw would add
  /// one.
  constexpr std::uint64_t kFirstId{1};

  /// \brief Period of simulation time between two frames of the trails.
  constexpr std::chrono::milliseconds kTrailPeriod{40};

  /// \brief Longest step a trail takes, s: after a pause or a stall the
  /// particles do not jump.
  constexpr double kMaxTrailStep{0.2};

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

  /// \brief Send a drawing's colour bins in one request to MarkerManager's
  /// array service: the bins with points added or replaced, and the bins
  /// drawn before and empty now deleted, since a marker sent with no points
  /// keeps its old ones.
  /// \param[in] _bins The bins.
  /// \param[in] _ns The drawing's namespace.
  /// \param[in,out] _drawn The ids on the scene.
  public: void SendBins(const ColorBins &_bins, const std::string &_ns,
                        std::set<std::uint64_t> &_drawn)
  {
    msgs::Marker_V markers;
    for (auto &m : _bins.Markers(_ns, kFirstId))
    {
      if (m.point_size() > 0)
      {
        _drawn.insert(m.id());
        *markers.add_marker() = std::move(m);
      }
      else if (_drawn.erase(m.id()) > 0u)
      {
        *markers.add_marker() = DeleteMarker(_ns, m.id());
      }
    }
    if (markers.marker_size() > 0)
      this->SendArray(markers);
  }

  /// \brief Take a drawing off the scene.
  /// \param[in] _ns The drawing's namespace.
  /// \param[in,out] _drawn The ids on the scene, emptied.
  public: void ClearBins(const std::string &_ns,
                         std::set<std::uint64_t> &_drawn)
  {
    msgs::Marker_V markers;
    for (const auto id : _drawn)
      *markers.add_marker() = DeleteMarker(_ns, id);
    _drawn.clear();
    if (markers.marker_size() > 0)
      this->SendArray(markers);
  }

  /// \brief Send markers to the array service, without waiting.
  /// \param[in] _markers The markers.
  public: void SendArray(const msgs::Marker_V &_markers)
  {
    std::function<void(const msgs::Boolean &, const bool)> cb =
        [](const msgs::Boolean &, const bool) {};
    this->node.Request(this->markerService + "_array", _markers, cb);
  }

  /// \brief The namespaces of the arrows and of the trails.
  public: std::string ArrowsNs() const { return this->ns + "/arrows"; }
  public: std::string TrailsNs() const { return this->ns + "/trails"; }

  /// \brief The field's name on its topics.
  public: std::string field;

  /// \brief The name the plugin logs under.
  public: std::string log;

  /// \brief Marker namespace.
  public: std::string ns;

  /// \brief Colour by speed.
  public: Colormap colormap{Colormap::Water(1.0)};

  /// \brief The trails, and their settings but the region and the plane.
  public: ParticleField particles;
  public: ParticleField::Settings trails;

  /// \brief Whether the trails need configuring again (region or plane).
  public: bool trailsDirty{true};

  /// \brief The plane the trails were configured on.
  public: double trailsZ{0.0};

  /// \brief Simulation time the trails last moved to.
  public: std::optional<std::chrono::steady_clock::duration> trailTime;

  /// \brief Steering by hand.
  public: std::optional<maritime_gui::Steering> steering;

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

  /// \brief How the field is drawn.
  public: Display display{Display::kOff};

  /// \brief The colour bins of each drawing on the scene.
  public: std::set<std::uint64_t> arrowsDrawn;
  public: std::set<std::uint64_t> trailsDrawn;

  /// \brief The markers on the scene of each extra drawing, by its name.
  public: std::map<std::string, std::set<std::uint64_t>> extraDrawn;

  /// \brief Whether the arrows were drawn since they were last cleared,
  /// even if the field was still and no bin had points.
  public: bool arrowsShown{false};

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
    const Colormap &_colormap, double _scale,
    const ParticleField::Settings &_trails,
    const maritime_gui::Steering &_steering)
  : dataPtr(std::make_unique<FieldLinkPrivate>())
{
  auto &d = *this->dataPtr;
  d.field = _field;
  d.log = _log;
  d.ns = "gz_maritime_gui/" + _field;
  d.colormap = _colormap;
  d.scale = _scale;
  d.trails = _trails;
  d.steering = _steering;
}

//////////////////////////////////////////////////
FieldLink::~FieldLink()
{
  // Closing the panel takes its drawing off the scene.
  auto &d = *this->dataPtr;
  d.ClearBins(d.ArrowsNs(), d.arrowsDrawn);
  d.ClearBins(d.TrailsNs(), d.trailsDrawn);
  for (auto &[kind, drawn] : d.extraDrawn)
    d.ClearBins(d.ns + "/" + kind, drawn);
}

//////////////////////////////////////////////////
void FieldLink::LoadConfig(const tinyxml2::XMLElement *_elem)
{
  if (!_elem)
    return;
  auto &d = *this->dataPtr;
  if (const auto *child = _elem->FirstChildElement("display"))
  {
    if (!child->GetText() || !ParseDisplay(child->GetText(), d.display))
    {
      gzwarn << d.log << ": <display> is off, arrows or trails; using "
             << DisplayName(d.display) << "\n";
    }
  }
  double maxSpeed{d.colormap.MaxSpeed()};
  ReadDouble(_elem, "max_speed", maxSpeed);
  if (maxSpeed > 0.0)
    d.colormap.SetMaxSpeed(maxSpeed);
  else
    gzwarn << d.log << ": <max_speed> must be positive, ignored\n";
  if (const auto *child = _elem->FirstChildElement("particles"))
    child->QueryIntText(&d.trails.count);
  if (const auto *child = _elem->FirstChildElement("trail_length"))
    child->QueryIntText(&d.trails.trailLength);
  ReadDouble(_elem, "lifetime", d.trails.lifetime);
  ReadDouble(_elem, "speedup", d.trails.speedup);
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
void FieldLink::Steer(double _speed, double _direction)
{
  this->dataPtr->steering->SetTarget(_speed, _direction);
}

//////////////////////////////////////////////////
void FieldLink::StepSteering(double _speed, double _direction,
    std::uint64_t _generation)
{
  auto &d = *this->dataPtr;
  const auto msg = d.steering->Step(_speed, _direction,
                                    maritime_gui::Steering::Clock::now());
  if (!msg)
    return;
  if (d.steering->Arrived())
  {
    // The last step is tracked like a change from Apply.
    this->Send(*msg, _generation);
  }
  else if (d.publisher && d.publisher->HasConnections())
  {
    d.publisher->Publish(*msg);
  }
}

//////////////////////////////////////////////////
bool FieldLink::IsSteering() const
{
  return this->dataPtr->steering->Active();
}

//////////////////////////////////////////////////
void FieldLink::Draw(bool _available, std::uint64_t _generation,
    bool _timeVarying, double _z,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    const std::chrono::steady_clock::duration &_simTime)
{
  auto &d = *this->dataPtr;
  const Display display = _available ? d.display : Display::kOff;

  // Take off what this display does not draw.
  if (display != Display::kArrows)
  {
    d.ClearBins(d.ArrowsNs(), d.arrowsDrawn);
    d.arrowsShown = false;
  }
  if (display != Display::kTrails)
  {
    d.ClearBins(d.TrailsNs(), d.trailsDrawn);
    d.trailTime.reset();
  }

  if (display == Display::kArrows)
  {
    const bool due = d.dirty || !d.arrowsShown ||
        _generation != d.drawnGeneration ||
        (_timeVarying && _simTime - d.lastDraw >= kRedrawPeriod);
    if (!due)
      return;
    d.SendBins(ColoredArrows(GridPoints(d.grid, _z), _velocity, d.scale,
                             d.colormap), d.ArrowsNs(), d.arrowsDrawn);
    d.arrowsShown = true;
    d.dirty = false;
    d.drawnGeneration = _generation;
    d.lastDraw = _simTime;
  }
  else if (display == Display::kTrails)
  {
    if (d.trailsDirty || d.dirty || _z != d.trailsZ)
    {
      auto settings = d.trails;
      settings.region = d.grid;
      settings.z = _z;
      d.particles.Configure(settings);
      d.trailsZ = _z;
      d.trailsDirty = false;
      d.dirty = false;
    }
    // The particles move with simulation time, so a paused world freezes
    // them; a frame every few tens of milliseconds of it.
    if (d.trailTime && _simTime - *d.trailTime < kTrailPeriod)
      return;
    const double dt = d.trailTime ? std::min(kMaxTrailStep,
        std::chrono::duration<double>(_simTime - *d.trailTime).count()) :
        0.0;
    d.trailTime = _simTime;
    d.particles.Step(dt, _velocity);
    d.SendBins(d.particles.Trails(d.colormap), d.TrailsNs(), d.trailsDrawn);
  }
}

//////////////////////////////////////////////////
void FieldLink::DrawExtra(const std::string &_kind,
    std::vector<msgs::Marker> _markers)
{
  auto &d = *this->dataPtr;
  const std::string ns = d.ns + "/" + _kind;
  auto &drawn = d.extraDrawn[_kind];
  std::set<std::uint64_t> now;
  msgs::Marker_V out;
  for (auto &m : _markers)
  {
    m.set_ns(ns);
    if (m.point_size() > 0)
    {
      now.insert(m.id());
      *out.add_marker() = std::move(m);
    }
  }
  for (const auto id : drawn)
  {
    if (now.count(id) == 0u)
      *out.add_marker() = DeleteMarker(ns, id);
  }
  drawn = now;
  if (out.marker_size() > 0)
    d.SendArray(out);
}

//////////////////////////////////////////////////
void FieldLink::ClearExtra(const std::string &_kind)
{
  auto &d = *this->dataPtr;
  auto it = d.extraDrawn.find(_kind);
  if (it != d.extraDrawn.end())
    d.ClearBins(d.ns + "/" + _kind, it->second);
}

//////////////////////////////////////////////////
Display FieldLink::CurrentDisplay() const
{
  return this->dataPtr->display;
}

//////////////////////////////////////////////////
void FieldLink::SetDisplay(Display _display)
{
  this->dataPtr->display = _display;
  this->dataPtr->dirty = true;
}

//////////////////////////////////////////////////
const Colormap &FieldLink::Colors() const
{
  return this->dataPtr->colormap;
}

//////////////////////////////////////////////////
bool FieldLink::SetMaxSpeed(double _maxSpeed)
{
  if (!(_maxSpeed > 0.0) || !std::isfinite(_maxSpeed))
    return false;
  this->dataPtr->colormap.SetMaxSpeed(_maxSpeed);
  this->dataPtr->dirty = true;
  return true;
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
bool ParseDisplay(const std::string &_name, Display &_display)
{
  if (_name == "off")
    _display = Display::kOff;
  else if (_name == "arrows")
    _display = Display::kArrows;
  else if (_name == "trails")
    _display = Display::kTrails;
  else
    return false;
  return true;
}

//////////////////////////////////////////////////
std::string DisplayName(Display _display)
{
  switch (_display)
  {
    case Display::kArrows: return "arrows";
    case Display::kTrails: return "trails";
    default: return "off";
  }
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
