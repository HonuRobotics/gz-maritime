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
#include "OceanCurrentPanel.hh"

#include <tinyxml2.h>

#include <chrono>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include <gz/plugin/Register.hh>

#include <cmath>

#include <QColor>

#include "gz/sim/maritime_gui/FieldLink.hh"
#include "gz/sim/maritime_gui/Flow.hh"
#include "gz/sim/maritime_gui/Vessels.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

namespace
{
  /// \brief The arrows sit this far above the water, which hides anything
  /// below its surface.
  constexpr double kAboveWater{0.1};
}

class gz::sim::maritime_gui::OceanCurrentPanelPrivate
{
  /// \brief The link to the world: topic, state service, drawing,
  /// steering. Water colours up to 1 m/s; arrows of 4 m per m/s, so a
  /// 0.5 m/s current is 2 m; trails sped up eight times, so a slow current
  /// still visibly flows; steering at 0.5 m/s and 90 degrees a second.
  public: FieldLink link{"ocean_current", "OceanCurrentPanel",
                         Colormap::Water(1.0), 4.0,
                         TrailDefaults(), Steering(0.5, 90.0)};

  /// \brief The trails' defaults for the current.
  /// \return The settings.
  public: static ParticleField::Settings TrailDefaults()
  {
    ParticleField::Settings s;
    s.count = 400;
    s.trailLength = 8;
    s.lifetime = 6.0;
    s.speedup = 8.0;
    return s;
  }

  /// \brief The current as last read, if the world has one.
  public: std::optional<CurrentValues> current;

  /// \brief What became of the last change.
  public: ChangeTracker::State state{ChangeTracker::State::kIdle};

  /// \brief Whether steering was under way at the last update.
  public: bool steering{false};

  /// \brief A field that is not a number, reported until the next Apply.
  public: std::string invalid;

  /// \brief Samples the current for the arrows.
  public: ocean_current::OceanCurrentSampler sampler;

  /// \brief Whether the vehicles' drawings are on.
  public: bool onVehicles{false};

  /// \brief The vehicles.
  public: VesselTracker vessels;

  /// \brief Simulation time of the vehicles' last drawing.
  public: std::optional<std::chrono::steady_clock::duration> vesselsDrawn;
};

//////////////////////////////////////////////////
OceanCurrentPanel::OceanCurrentPanel()
  : dataPtr(std::make_unique<OceanCurrentPanelPrivate>())
{
}

//////////////////////////////////////////////////
OceanCurrentPanel::~OceanCurrentPanel() = default;

//////////////////////////////////////////////////
void OceanCurrentPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Ocean current";
  this->dataPtr->link.LoadConfig(_pluginElem);
  if (_pluginElem)
  {
    if (const auto *elem = _pluginElem->FirstChildElement("on_vehicles"))
      elem->QueryBoolText(&this->dataPtr->onVehicles);
  }
  // The panel was built before the configuration was read.
  emit this->ArrowsChanged();
}

//////////////////////////////////////////////////
void OceanCurrentPanel::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.link.Update(_ecm);

  const bool inGui = ReadCurrent(_ecm).has_value();
  const auto &source = d.link.Source(_ecm, inGui);
  const auto current = ReadCurrent(source);

  if (current)
  {
    if (!d.current || d.current->generation != current->generation)
    {
      if (!d.current)
        d.link.LogSource("ocean current", inGui, current->generation);
      d.current = current;
      emit this->CurrentChanged();
    }
    const auto state = d.link.Track(current->generation);
    if (state != d.state)
    {
      d.state = state;
      emit this->StatusChanged();
    }
    d.sampler.Sync(source);
    d.link.StepSteering(d.current->speed, d.current->direction,
                        d.current->generation);
    if (d.link.IsSteering() != d.steering)
    {
      d.steering = d.link.IsSteering();
      emit this->StatusChanged();
    }
  }
  else if (d.current)
  {
    d.current.reset();
    emit this->CurrentChanged();
  }

  // Sampled at the water's surface, drawn just above it.
  const double level = d.current ? d.current->waterLevel : 0.0;
  d.link.Draw(d.current.has_value() && d.sampler.Valid(),
      d.current ? d.current->generation : 0u, d.sampler.TimeVarying(),
      level + kAboveWater,
      [&](const math::Vector3d &_p)
      {
        return d.sampler.At({_p.X(), _p.Y(), level}, _info.simTime);
      },
      _info.simTime);

  // On each vehicle: the water past its hull, its track and its heading,
  // ten times a second of simulation time.
  if (!d.onVehicles || !d.current || !d.sampler.Valid())
  {
    d.link.ClearExtra("vessels");
    d.vesselsDrawn.reset();
    return;
  }
  d.vessels.Update(_ecm, _info.simTime);
  if (d.vesselsDrawn &&
      _info.simTime - *d.vesselsDrawn < std::chrono::milliseconds(100) &&
      _info.simTime >= *d.vesselsDrawn)
  {
    return;
  }
  d.vesselsDrawn = _info.simTime;
  const auto vessels = d.vessels.Vessels();
  auto markers = VesselArrows(vessels, [&](const Vessel &_v)
      {
        math::Vector3d rel = d.sampler.At(
            {_v.pose.Pos().X(), _v.pose.Pos().Y(), level}, _info.simTime) -
            _v.velocity;
        rel.Z(0.0);
        return rel;
      }, 1.0, d.link.Scale(), 0.25, d.link.Colors()).Markers("", 1);
  for (auto &m : TracksAndHeadings(vessels, level + 0.12, 3.0).Markers("", 101))
    markers.push_back(std::move(m));
  d.link.DrawExtra("vessels", std::move(markers));
}

//////////////////////////////////////////////////
void OceanCurrentPanel::Apply(const QVariantMap &_edits)
{
  auto &d = *this->dataPtr;
  if (!d.current)
    return;
  std::map<std::string, double> edits;
  d.invalid.clear();
  for (auto it = _edits.begin(); it != _edits.end(); ++it)
  {
    double value{0.0};
    if (!ParseNumber(it.value().toString().toStdString(), value))
    {
      d.invalid = it.key().toStdString();
      emit this->StatusChanged();
      return;
    }
    edits[it.key().toStdString()] = value;
  }
  const auto msg = ChangedKeys(edits, Editable(*d.current));
  if (msg.params().empty())
  {
    emit this->StatusChanged();
    return;
  }
  d.link.Send(msg, d.current->generation);
  d.state = d.link.Track(d.current->generation);
  emit this->StatusChanged();
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SetExtent(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->link.SetExtent(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SetSpacing(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->link.SetSpacing(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
QVariantMap OceanCurrentPanel::Current() const
{
  const auto &d = *this->dataPtr;
  QVariantMap m;
  m["available"] = d.current.has_value();
  if (!d.current)
    return m;
  m["model"] = QString::fromStdString(d.current->model);
  m["speed"] = d.current->speed;
  m["direction"] = d.current->direction;
  m["waterLevel"] = d.current->waterLevel;
  m["flowHeading"] = FlowHeading(d.current->direction, false);
  std::ostringstream os;
  for (const auto &[key, value] : d.current->extra)
    os << key << " = " << value << "\n";
  m["parameters"] = QString::fromStdString(os.str()).trimmed();
  return m;
}

//////////////////////////////////////////////////
QString OceanCurrentPanel::Status() const
{
  const auto &d = *this->dataPtr;
  if (d.steering)
    return "Steering...";
  if (!d.invalid.empty())
  {
    return QString::fromStdString("Not sent: " + d.invalid +
                                  " is not a number.");
  }
  return QString::fromStdString(StatusText(d.state));
}

//////////////////////////////////////////////////
QString OceanCurrentPanel::CurrentDisplay() const
{
  return QString::fromStdString(
      DisplayName(this->dataPtr->link.CurrentDisplay()));
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SetDisplay(const QString &_display)
{
  Display display{Display::kOff};
  if (ParseDisplay(_display.toStdString(), display))
  {
    this->dataPtr->link.SetDisplay(display);
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
double OceanCurrentPanel::MaxSpeed() const
{
  return this->dataPtr->link.Colors().MaxSpeed();
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SetMaxSpeed(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->link.SetMaxSpeed(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
QStringList OceanCurrentPanel::Legend() const
{
  QStringList out;
  for (const auto &c : this->dataPtr->link.Colors().Stops())
    out << QColor::fromRgbF(c.R(), c.G(), c.B()).name();
  return out;
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SteerFlow(double _speed, double _flowHeading)
{
  if (!std::isfinite(_speed) || !std::isfinite(_flowHeading))
    return;
  this->dataPtr->invalid.clear();
  this->dataPtr->link.Steer(_speed, _flowHeading);
  this->dataPtr->steering = true;
  emit this->StatusChanged();
}

//////////////////////////////////////////////////
double OceanCurrentPanel::Extent() const
{
  return this->dataPtr->link.ArrowGrid().extent;
}

//////////////////////////////////////////////////
double OceanCurrentPanel::Spacing() const
{
  return this->dataPtr->link.ArrowGrid().spacing;
}

//////////////////////////////////////////////////
double OceanCurrentPanel::Scale() const
{
  return this->dataPtr->link.Scale();
}

//////////////////////////////////////////////////
bool OceanCurrentPanel::OnVehicles() const
{
  return this->dataPtr->onVehicles;
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SetOnVehicles(bool _on)
{
  this->dataPtr->onVehicles = _on;
  emit this->ArrowsChanged();
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::maritime_gui::OceanCurrentPanel, gz::gui::Plugin)
