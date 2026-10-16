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
#include "ScenarioPanel.hh"

#include <tinyxml2.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/plugin/Register.hh>

#include "gz/sim/maritime_gui/FieldControl.hh"
#include "gz/sim/maritime_gui/FieldSpec.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/maritime_gui/Flow.hh"
#include "gz/sim/maritime_gui/Presets.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

class gz::sim::maritime_gui::ScenarioPanelPrivate
{
  /// \brief The control of the two fields: topics, state service,
  /// steering. The field panels draw them.
  public: FieldControl current{OceanCurrentSpec(), "ScenarioPanel"};
  public: FieldControl wind{WindSpec(), "ScenarioPanel"};

  /// \brief The presets.
  public: std::vector<Preset> presets{DefaultPresets()};

  /// \brief The demo.
  public: Demo demo;

  /// \brief The preset applied last, if any.
  public: std::optional<std::size_t> active;

  /// \brief The fields as last read.
  public: std::optional<CurrentValues> currentValues;
  public: std::optional<WindValues> windValues;

  /// \brief Whether a ramp was under way at the last update.
  public: bool steering{false};

  /// \brief Whether a ramp stopped short at the last update.
  public: bool yielded{false};

  /// \brief Whether the world had a field at the last update.
  public: bool available{false};
};

//////////////////////////////////////////////////
ScenarioPanel::ScenarioPanel()
  : dataPtr(std::make_unique<ScenarioPanelPrivate>())
{
}

//////////////////////////////////////////////////
ScenarioPanel::~ScenarioPanel() = default;

//////////////////////////////////////////////////
void ScenarioPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Scenarios";
  auto &d = *this->dataPtr;
  auto parsed = ParsePresets(_pluginElem);
  if (!parsed.empty())
    d.presets = std::move(parsed);
  bool demo{false};
  if (_pluginElem)
  {
    if (const auto *elem = _pluginElem->FirstChildElement("demo"))
      elem->QueryBoolText(&demo);
  }
  if (demo)
    d.demo.Start();
  emit this->StateChanged();
}

//////////////////////////////////////////////////
void ScenarioPanel::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  d.current.Update(_ecm);
  d.wind.Update(_ecm);

  d.currentValues = ReadCurrent(d.current.Source(
      _ecm, ReadCurrent(_ecm).has_value()));
  d.windValues = ReadWind(d.wind.Source(_ecm, ReadWind(_ecm).has_value()));

  // The demo moves on in simulation time.
  if (const auto next = d.demo.Update(d.presets, _info.simTime))
    this->ApplyPreset(static_cast<int>(*next));

  if (d.currentValues)
  {
    d.current.StepSteering(d.currentValues->speed,
        d.currentValues->direction, d.currentValues->generation);
  }
  if (d.windValues)
  {
    d.wind.StepSteering(d.windValues->speed, d.windValues->direction,
                        d.windValues->generation);
  }
  const bool steering = d.current.IsSteering() || d.wind.IsSteering();
  const bool yielded = d.current.Yielded() || d.wind.Yielded();
  const bool available = d.currentValues.has_value() ||
      d.windValues.has_value();
  if (steering != d.steering || yielded != d.yielded ||
      available != d.available)
  {
    d.steering = steering;
    d.yielded = yielded;
    d.available = available;
    emit this->StateChanged();
  }
}

//////////////////////////////////////////////////
void ScenarioPanel::ApplyPreset(int _index)
{
  auto &d = *this->dataPtr;
  if (_index < 0 || static_cast<std::size_t>(_index) >= d.presets.size())
    return;
  const Preset &p = d.presets[static_cast<std::size_t>(_index)];
  if (p.current && d.currentValues)
  {
    d.current.Steer(p.current->speed, p.current->direction);
  }
  if (p.wind && d.windValues)
  {
    d.wind.Steer(p.wind->speed, p.wind->direction);
  }
  if (p.gusts && d.windValues)
  {
    // The gusts' statistics change at once; the wind itself is steered.
    msgs::Param msg;
    auto &s = (*msg.mutable_params())["speed_gust"];
    s.set_type(msgs::Any::DOUBLE);
    s.set_double_value(p.gusts->speed);
    auto &g = (*msg.mutable_params())["direction_gust"];
    g.set_type(msgs::Any::DOUBLE);
    g.set_double_value(p.gusts->direction);
    d.wind.Send(msg, d.windValues->generation);
  }
  d.active = static_cast<std::size_t>(_index);
  d.steering = d.current.IsSteering() || d.wind.IsSteering();
  emit this->StateChanged();
}

//////////////////////////////////////////////////
void ScenarioPanel::SetDemo(bool _run)
{
  if (_run)
    this->dataPtr->demo.Start();
  else
    this->dataPtr->demo.Stop();
  emit this->StateChanged();
}

//////////////////////////////////////////////////
QStringList ScenarioPanel::Presets() const
{
  QStringList out;
  for (const auto &p : this->dataPtr->presets)
    out << QString::fromStdString(p.name);
  return out;
}

//////////////////////////////////////////////////
bool ScenarioPanel::DemoRunning() const
{
  return this->dataPtr->demo.Running();
}

//////////////////////////////////////////////////
int ScenarioPanel::Active() const
{
  const auto &d = *this->dataPtr;
  return d.active ? static_cast<int>(*d.active) : -1;
}

//////////////////////////////////////////////////
QString ScenarioPanel::Status() const
{
  const auto &d = *this->dataPtr;
  if (!d.currentValues && !d.windValues)
    return "This world has no ocean current or wind system.";
  std::string s;
  if (d.active)
  {
    const auto &name = d.presets[*d.active].name;
    if (d.steering)
      s = "Steering to " + name + "...";
    else if (d.yielded)
      s = name + " stopped: a field was changed elsewhere.";
    else
      s = name + ".";
  }
  if (d.demo.Running())
    s += s.empty() ? "Demo running." : " Demo running.";
  return QString::fromStdString(s);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::maritime_gui::ScenarioPanel, gz::gui::Plugin)
