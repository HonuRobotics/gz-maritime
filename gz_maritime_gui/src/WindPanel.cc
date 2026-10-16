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
#include "WindPanel.hh"

#include <chrono>
#include <locale>
#include <map>
#include <sstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <tinyxml2.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <cmath>

#include <QColor>

#include "gz/sim/maritime_gui/FieldControl.hh"
#include "gz/sim/maritime_gui/FieldDrawing.hh"
#include "gz/sim/maritime_gui/FieldSpec.hh"
#include "gz/sim/maritime_gui/Flow.hh"
#include "gz/sim/maritime_gui/Vessels.hh"
#include "gz/sim/maritime_gui/WindSock.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/wind/WindSampler.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

namespace
{
  /// \brief m of a vehicle's apparent wind arrow per m/s: longer than the
  /// field's arrows, since one arrow on a boat has room to be read.
  constexpr double kVesselWindScale{0.5};
}

class gz::sim::maritime_gui::WindPanelPrivate
{
  /// \brief The control of the field: topic, state service, steering.
  public: FieldControl control{WindSpec(), "WindPanel"};

  /// \brief The drawing of the field.
  public: FieldDrawing drawing{WindSpec(), "WindPanel"};

  /// \brief The wind as last read, if the world has one.
  public: std::optional<WindValues> wind;

  /// \brief Height above the water the arrows are drawn at, m.
  public: double height{3.0};

  /// \brief What became of the last change.
  public: ChangeTracker::State state{ChangeTracker::State::kIdle};

  /// \brief Whether steering was under way at the last update.
  public: bool steering{false};

  /// \brief Whether steering stopped short at the last update.
  public: bool yielded{false};

  /// \brief A field that is not a number, reported until the next Apply.
  public: std::string invalid;

  /// \brief Samples the wind for the arrows.
  public: wind::WindSampler sampler;

  /// \brief Whether the vehicles' apparent wind is drawn.
  public: bool onVehicles{false};

  /// \brief The vehicles.
  public: VesselTracker vessels;

  /// \brief Where the wind socks' poles stand, and the way each pointed
  /// last, for still air.
  public: std::vector<math::Vector3d> socks;
  public: std::vector<math::Vector3d> sockAxes;

  /// \brief The poles' height, m.
  public: double sockHeight{3.0};

  /// \brief Whether the wind socks are drawn.
  public: bool showSocks{true};

  /// \brief Simulation time of the last drawing of the vehicles and socks.
  public: std::optional<std::chrono::steady_clock::duration> extrasDrawn;
};

//////////////////////////////////////////////////
WindPanel::WindPanel()
  : dataPtr(std::make_unique<WindPanelPrivate>())
{
}

//////////////////////////////////////////////////
WindPanel::~WindPanel() = default;

//////////////////////////////////////////////////
void WindPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Wind";
  this->dataPtr->drawing.LoadConfig(_pluginElem);
  auto &d = *this->dataPtr;
  if (_pluginElem)
  {
    if (const auto *elem = _pluginElem->FirstChildElement("height"))
      elem->QueryDoubleText(&d.height);
    if (const auto *elem = _pluginElem->FirstChildElement("on_vehicles"))
      elem->QueryBoolText(&d.onVehicles);
    if (const auto *elem = _pluginElem->FirstChildElement("show_socks"))
      elem->QueryBoolText(&d.showSocks);
    if (const auto *elem = _pluginElem->FirstChildElement("sock_height"))
      elem->QueryDoubleText(&d.sockHeight);
    for (auto *elem = _pluginElem->FirstChildElement("wind_sock"); elem;
         elem = elem->NextSiblingElement("wind_sock"))
    {
      std::istringstream is(elem->GetText() ? elem->GetText() : "");
      is.imbue(std::locale::classic());
      double x{0.0}, y{0.0}, z{0.0};
      if (is >> x >> y >> z)
      {
        d.socks.emplace_back(x, y, z);
        d.sockAxes.push_back(math::Vector3d::UnitX);
      }
      else
      {
        gzwarn << "WindPanel: <wind_sock> takes x y z; ignored\n";
      }
    }
  }
  // The panel was built before the configuration was read.
  emit this->ArrowsChanged();
}

//////////////////////////////////////////////////
void WindPanel::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.control.Update(_ecm);

  const bool inGui = ReadWind(_ecm).has_value();
  const auto &source = d.control.Source(_ecm, inGui);
  const auto wind = ReadWind(source);

  if (wind)
  {
    if (!d.wind || d.wind->generation != wind->generation)
    {
      if (!d.wind)
        d.control.LogSource("wind", inGui, wind->generation);
      d.wind = wind;
      emit this->WindChanged();
    }
    const auto state = d.control.Track(wind->generation);
    if (state != d.state)
    {
      d.state = state;
      emit this->StatusChanged();
    }
    d.sampler.Sync(source);
    d.control.StepSteering(d.wind->speed, d.wind->direction,
                           d.wind->generation);
    if (d.control.IsSteering() != d.steering ||
        d.control.Yielded() != d.yielded)
    {
      d.steering = d.control.IsSteering();
      d.yielded = d.control.Yielded();
      emit this->StatusChanged();
    }
  }
  else if (d.wind)
  {
    d.wind.reset();
    emit this->WindChanged();
  }

  // Sampled where it is drawn, so the profile with height shows.
  const double level = d.wind ? d.wind->waterLevel : 0.0;
  d.drawing.Draw(d.wind.has_value() && d.sampler.Valid(),
      d.wind ? d.wind->generation : 0u, d.sampler.TimeVarying(),
      level + d.height,
      [&](const math::Vector3d &_p)
      {
        return d.sampler.At(_p, _info.simTime);
      },
      _info.simTime);

  // The vehicles' apparent wind and the wind socks, ten times a second of
  // simulation time; they move with the gusts.
  const bool windy = d.wind.has_value() && d.sampler.Valid();
  if (!windy || !d.onVehicles)
    d.drawing.ClearExtra("vessels");
  if (!windy || !d.showSocks)
    d.drawing.ClearExtra("socks");
  if (!windy || (!d.onVehicles && (!d.showSocks || d.socks.empty())))
  {
    d.extrasDrawn.reset();
    return;
  }
  if (d.onVehicles)
    d.vessels.Update(_ecm, _info.simTime);
  if (d.extrasDrawn &&
      _info.simTime - *d.extrasDrawn < std::chrono::milliseconds(100) &&
      _info.simTime >= *d.extrasDrawn)
  {
    return;
  }
  d.extrasDrawn = _info.simTime;

  if (d.onVehicles)
  {
    // The apparent wind, 2 m above the vehicle, where a sailor feels it.
    d.drawing.DrawExtra("vessels", VesselArrows(d.vessels.Vessels(),
        [&](const Vessel &_v)
        {
          math::Vector3d rel = d.sampler.At(
              _v.pose.Pos() + math::Vector3d(0, 0, 2.0), _info.simTime) -
              _v.velocity;
          rel.Z(0.0);
          return rel;
        }, 2.0, kVesselWindScale, 0.35, d.drawing.Colors()).Markers("", 1));
  }
  if (d.showSocks && !d.socks.empty())
  {
    std::vector<msgs::Marker> markers;
    for (std::size_t i = 0; i < d.socks.size(); ++i)
    {
      const math::Vector3d top =
          d.socks[i] + math::Vector3d(0, 0, d.sockHeight);
      d.sockAxes[i] = SockAxis(d.sampler.At(top, _info.simTime),
                               d.sockAxes[i]);
      for (auto &m : WindSockMarkers("", 1 + 3 * i, d.socks[i],
                                     d.sockHeight, d.sockAxes[i], 2.0))
      {
        markers.push_back(std::move(m));
      }
    }
    d.drawing.DrawExtra("socks", std::move(markers));
  }
}

//////////////////////////////////////////////////
void WindPanel::Apply(const QVariantMap &_edits)
{
  auto &d = *this->dataPtr;
  if (!d.wind)
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
  const auto msg = ChangedKeys(edits, Editable(*d.wind));
  if (msg.params().empty())
  {
    emit this->StatusChanged();
    return;
  }
  d.control.Send(msg, d.wind->generation);
  d.state = d.control.Track(d.wind->generation);
  emit this->StatusChanged();
}

//////////////////////////////////////////////////
void WindPanel::SetExtent(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->drawing.SetExtent(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
void WindPanel::SetSpacing(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->drawing.SetSpacing(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
void WindPanel::SetHeight(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value))
  {
    this->dataPtr->height = value;
    this->dataPtr->drawing.Redraw();
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
QVariantMap WindPanel::Wind() const
{
  const auto &d = *this->dataPtr;
  QVariantMap m;
  m["available"] = d.wind.has_value();
  if (!d.wind)
    return m;
  m["model"] = QString::fromStdString(d.wind->model);
  m["speed"] = d.wind->speed;
  m["direction"] = d.wind->direction;
  m["speedGust"] = d.wind->speedGust;
  m["speedGustTime"] = d.wind->speedGustTime;
  m["directionGust"] = d.wind->directionGust;
  m["directionGustTime"] = d.wind->directionGustTime;
  m["referenceHeight"] = d.wind->referenceHeight;
  m["flowHeading"] = FlowHeading(d.wind->direction, true);
  return m;
}

//////////////////////////////////////////////////
QString WindPanel::Status() const
{
  const auto &d = *this->dataPtr;
  if (d.steering)
    return "Steering...";
  if (d.yielded)
    return QString::fromStdString(YieldedText());
  if (!d.invalid.empty())
  {
    return QString::fromStdString("Not sent: " + d.invalid +
                                  " is not a number.");
  }
  return QString::fromStdString(StatusText(d.state));
}

//////////////////////////////////////////////////
QString WindPanel::CurrentDisplay() const
{
  return QString::fromStdString(
      DisplayName(this->dataPtr->drawing.CurrentDisplay()));
}

//////////////////////////////////////////////////
void WindPanel::SetDisplay(const QString &_display)
{
  Display display{Display::kOff};
  if (ParseDisplay(_display.toStdString(), display))
  {
    this->dataPtr->drawing.SetDisplay(display);
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
double WindPanel::MaxSpeed() const
{
  return this->dataPtr->drawing.Colors().MaxSpeed();
}

//////////////////////////////////////////////////
void WindPanel::SetMaxSpeed(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->drawing.SetMaxSpeed(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
QStringList WindPanel::Legend() const
{
  QStringList out;
  for (const auto &c : this->dataPtr->drawing.Colors().Stops())
    out << QColor::fromRgbF(c.R(), c.G(), c.B()).name();
  return out;
}

//////////////////////////////////////////////////
void WindPanel::SteerFlow(double _speed, double _flowHeading)
{
  if (!std::isfinite(_speed) || !std::isfinite(_flowHeading))
    return;
  this->dataPtr->invalid.clear();
  this->dataPtr->control.Steer(_speed, _flowHeading + 180.0);
  this->dataPtr->steering = true;
  emit this->StatusChanged();
}

//////////////////////////////////////////////////
double WindPanel::Extent() const
{
  return this->dataPtr->drawing.ArrowGrid().extent;
}

//////////////////////////////////////////////////
double WindPanel::Spacing() const
{
  return this->dataPtr->drawing.ArrowGrid().spacing;
}

//////////////////////////////////////////////////
double WindPanel::Height() const
{
  return this->dataPtr->height;
}

//////////////////////////////////////////////////
double WindPanel::Scale() const
{
  return this->dataPtr->drawing.Scale();
}

//////////////////////////////////////////////////
bool WindPanel::OnVehicles() const
{
  return this->dataPtr->onVehicles;
}

//////////////////////////////////////////////////
void WindPanel::SetOnVehicles(bool _on)
{
  this->dataPtr->onVehicles = _on;
  emit this->ArrowsChanged();
}

//////////////////////////////////////////////////
bool WindPanel::ShowSocks() const
{
  return this->dataPtr->showSocks;
}

//////////////////////////////////////////////////
void WindPanel::SetShowSocks(bool _show)
{
  this->dataPtr->showSocks = _show;
  emit this->ArrowsChanged();
}

//////////////////////////////////////////////////
bool WindPanel::HasSocks() const
{
  return !this->dataPtr->socks.empty();
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::maritime_gui::WindPanel, gz::gui::Plugin)
