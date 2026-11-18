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
#include "gz/sim/maritime_gui/FieldDrawing.hh"

#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>

#include "gz/sim/maritime_gui/MarkerLayer.hh"

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief The id of a drawing's first colour bin. Not 0: MarkerManager
  /// gives a marker with id 0 a new random id, so every redraw would add
  /// one.
  constexpr std::uint64_t kFirstId{1};

  /// \brief The names of the two drawings of the field itself.
  const char kArrowsKind[] = "arrows";
  const char kTrailsKind[] = "trails";

  /// \brief Period of simulation time between two frames of the trails.
  constexpr std::chrono::milliseconds kTrailPeriod{40};

  /// \brief Longest step a trail takes, s: after a pause or a stall the
  /// particles do not jump.
  constexpr double kMaxTrailStep{0.2};

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

class FieldDrawingPrivate
{
  /// \brief A drawing of this panel on the scene, made at its first use,
  /// so after the configuration named the marker service.
  /// \param[in] _kind Its name: the namespace's last part.
  /// \return The layer.
  public: MarkerLayer &Layer(const std::string &_kind)
  {
    auto it = this->layers.find(_kind);
    if (it == this->layers.end())
    {
      it = this->layers.emplace(_kind, std::make_unique<MarkerLayer>(
          this->ns + "/" + _kind, this->markerService)).first;
    }
    return *it->second;
  }

  /// \brief Take a drawing off the scene, if it was ever made.
  /// \param[in] _kind Its name.
  public: void ClearLayer(const std::string &_kind)
  {
    const auto it = this->layers.find(_kind);
    if (it != this->layers.end() && it->second->Drawn())
      it->second->Clear();
  }

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

  /// \brief m of arrow per m/s.
  public: double scale{1.0};

  /// \brief MarkerManager's service.
  public: std::string markerService{kMarkerService};

  /// \brief The grid.
  public: Grid grid;

  /// \brief How the field is drawn.
  public: Display display{Display::kOff};

  /// \brief The drawings on the scene, by name: the arrows, the trails and
  /// each extra one. Each takes what it drew off the scene when it goes.
  public: std::map<std::string, std::unique_ptr<MarkerLayer>> layers;

  /// \brief Whether the arrows were drawn since they were last cleared,
  /// even if the field was still and no bin had points.
  public: bool arrowsShown{false};

  /// \brief A setting changed since the last drawing.
  public: bool dirty{true};

  /// \brief Only the colours changed: the arrows are redrawn, the trails
  /// take the new colours at their next frame without starting again.
  public: bool recolor{false};

  /// \brief The recipe generation last drawn.
  public: std::uint64_t drawnGeneration{0};

  /// \brief Simulation time of the last drawing.
  public: std::chrono::steady_clock::duration lastDraw{0};

};

//////////////////////////////////////////////////
FieldDrawing::FieldDrawing(const FieldSpec &_spec, const std::string &_log)
  : dataPtr(std::make_unique<FieldDrawingPrivate>())
{
  auto &d = *this->dataPtr;
  d.log = _log;
  d.ns = "gz_maritime_gui/" + _spec.name;
  d.colormap = _spec.colormap;
  d.scale = _spec.scale;
  d.trails = _spec.trails;
}

//////////////////////////////////////////////////
// Closing the panel takes its drawings off the scene: each layer does.
FieldDrawing::~FieldDrawing() = default;

//////////////////////////////////////////////////
void FieldDrawing::LoadConfig(const tinyxml2::XMLElement *_elem)
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
void FieldDrawing::Draw(bool _available, std::uint64_t _generation,
    bool _timeVarying, double _z,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    const std::chrono::steady_clock::duration &_simTime)
{
  auto &d = *this->dataPtr;
  const Display display = _available ? d.display : Display::kOff;

  // Simulation time going back is a world reset: the timers would wait
  // for it to catch up, so they start again from the new time.
  if (_simTime < d.lastDraw || (d.trailTime && _simTime < *d.trailTime))
  {
    d.lastDraw = _simTime;
    d.trailTime.reset();
    d.arrowsShown = false;
  }

  // Take off what this display does not draw.
  if (display != Display::kArrows)
  {
    d.ClearLayer(kArrowsKind);
    d.arrowsShown = false;
  }
  if (display != Display::kTrails)
  {
    d.ClearLayer(kTrailsKind);
    d.trailTime.reset();
  }

  if (display == Display::kArrows)
  {
    const bool due = d.dirty || d.recolor || !d.arrowsShown ||
        _generation != d.drawnGeneration ||
        (_timeVarying && _simTime - d.lastDraw >= kRedrawPeriod);
    if (!due)
      return;
    auto &layer = d.Layer(kArrowsKind);
    layer.Draw(ColoredArrows(GridPoints(d.grid, _z), _velocity, d.scale,
                             d.colormap).Markers(layer.Ns(), kFirstId));
    d.arrowsShown = true;
    d.dirty = false;
    d.recolor = false;
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
    auto &layer = d.Layer(kTrailsKind);
    layer.Draw(d.particles.Trails(d.colormap).Markers(layer.Ns(), kFirstId));
    d.recolor = false;
  }
}

//////////////////////////////////////////////////
void FieldDrawing::DrawExtra(const std::string &_kind,
    std::vector<msgs::Marker> _markers)
{
  this->dataPtr->Layer(_kind).Draw(_markers);
}

//////////////////////////////////////////////////
void FieldDrawing::ClearExtra(const std::string &_kind)
{
  this->dataPtr->ClearLayer(_kind);
}

//////////////////////////////////////////////////
Display FieldDrawing::CurrentDisplay() const
{
  return this->dataPtr->display;
}

//////////////////////////////////////////////////
void FieldDrawing::SetDisplay(Display _display)
{
  this->dataPtr->display = _display;
  this->dataPtr->dirty = true;
}

//////////////////////////////////////////////////
const Colormap &FieldDrawing::Colors() const
{
  return this->dataPtr->colormap;
}

//////////////////////////////////////////////////
bool FieldDrawing::SetMaxSpeed(double _maxSpeed)
{
  if (!(_maxSpeed > 0.0) || !std::isfinite(_maxSpeed))
    return false;
  this->dataPtr->colormap.SetMaxSpeed(_maxSpeed);
  this->dataPtr->recolor = true;
  return true;
}

//////////////////////////////////////////////////
const Grid &FieldDrawing::ArrowGrid() const
{
  return this->dataPtr->grid;
}

//////////////////////////////////////////////////
bool FieldDrawing::SetExtent(double _extent)
{
  if (!(_extent >= 0.0) || !std::isfinite(_extent))
    return false;
  this->dataPtr->grid.extent = _extent;
  this->dataPtr->dirty = true;
  return true;
}

//////////////////////////////////////////////////
bool FieldDrawing::SetSpacing(double _spacing)
{
  if (!(_spacing > 0.0) || !std::isfinite(_spacing))
    return false;
  this->dataPtr->grid.spacing = _spacing;
  this->dataPtr->dirty = true;
  return true;
}

//////////////////////////////////////////////////
double FieldDrawing::Scale() const
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
void FieldDrawing::Redraw()
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

}  // namespace gz::sim::maritime_gui
