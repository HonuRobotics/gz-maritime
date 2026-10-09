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

#include <map>
#include <optional>
#include <string>

#include <tinyxml2.h>

#include <gz/plugin/Register.hh>

#include "gz/sim/maritime_gui/FieldLink.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/wind/WindSampler.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

class gz::sim::maritime_gui::WindPanelPrivate
{
  /// \brief The link to the world: topic, state service, arrows. Yellow,
  /// and 0.3 m per m/s, so a 10 m/s wind is 3 m, within the default 5 m
  /// spacing.
  public: FieldLink link{"wind", "WindPanel",
                         math::Color(1.0f, 0.85f, 0.2f, 1.0f), 0.3};

  /// \brief The wind as last read, if the world has one.
  public: std::optional<WindValues> wind;

  /// \brief Height above the water the arrows are drawn at, m.
  public: double height{3.0};

  /// \brief What became of the last change.
  public: ChangeTracker::State state{ChangeTracker::State::kIdle};

  /// \brief A field that is not a number, reported until the next Apply.
  public: std::string invalid;

  /// \brief Samples the wind for the arrows.
  public: wind::WindSampler sampler;
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
  this->dataPtr->link.LoadConfig(_pluginElem);
  if (_pluginElem)
  {
    if (const auto *elem = _pluginElem->FirstChildElement("height"))
      elem->QueryDoubleText(&this->dataPtr->height);
  }
  // The panel was built before the configuration was read.
  emit this->ArrowsChanged();
}

//////////////////////////////////////////////////
void WindPanel::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.link.Update(_ecm);

  const bool inGui = ReadWind(_ecm).has_value();
  const auto &source = d.link.Source(_ecm, inGui);
  const auto wind = ReadWind(source);

  if (wind)
  {
    if (!d.wind || d.wind->generation != wind->generation)
    {
      if (!d.wind)
        d.link.LogSource("wind", inGui, wind->generation);
      d.wind = wind;
      emit this->WindChanged();
    }
    const auto state = d.link.Track(wind->generation);
    if (state != d.state)
    {
      d.state = state;
      emit this->StatusChanged();
    }
    d.sampler.Sync(source);
  }
  else if (d.wind)
  {
    d.wind.reset();
    emit this->WindChanged();
  }

  // Sampled where it is drawn, so the profile with height shows.
  const double level = d.wind ? d.wind->waterLevel : 0.0;
  d.link.Draw(d.wind.has_value() && d.sampler.Valid(),
      d.wind ? d.wind->generation : 0u, d.sampler.TimeVarying(),
      level + d.height,
      [&](const math::Vector3d &_p)
      {
        return d.sampler.At(_p, _info.simTime);
      },
      _info.simTime);
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
  d.link.Send(msg, d.wind->generation);
  d.state = d.link.Track(d.wind->generation);
  emit this->StatusChanged();
}

//////////////////////////////////////////////////
void WindPanel::SetExtent(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->link.SetExtent(value))
  {
    emit this->ArrowsChanged();
  }
}

//////////////////////////////////////////////////
void WindPanel::SetSpacing(const QString &_text)
{
  double value{0.0};
  if (ParseNumber(_text.toStdString(), value) &&
      this->dataPtr->link.SetSpacing(value))
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
    this->dataPtr->link.Redraw();
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
  if (!d.invalid.empty())
  {
    return QString::fromStdString("Not sent: " + d.invalid +
                                  " is not a number.");
  }
  return QString::fromStdString(StatusText(d.state));
}

//////////////////////////////////////////////////
bool WindPanel::ShowArrows() const
{
  return this->dataPtr->link.ShowArrows();
}

//////////////////////////////////////////////////
void WindPanel::SetShowArrows(bool _show)
{
  this->dataPtr->link.SetShowArrows(_show);
  emit this->ArrowsChanged();
}

//////////////////////////////////////////////////
double WindPanel::Extent() const
{
  return this->dataPtr->link.ArrowGrid().extent;
}

//////////////////////////////////////////////////
double WindPanel::Spacing() const
{
  return this->dataPtr->link.ArrowGrid().spacing;
}

//////////////////////////////////////////////////
double WindPanel::Height() const
{
  return this->dataPtr->height;
}

//////////////////////////////////////////////////
double WindPanel::Scale() const
{
  return this->dataPtr->link.Scale();
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::maritime_gui::WindPanel, gz::gui::Plugin)
