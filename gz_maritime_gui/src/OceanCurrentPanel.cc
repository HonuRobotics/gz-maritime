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

#include <map>
#include <optional>
#include <sstream>
#include <string>

#include <gz/plugin/Register.hh>

#include "gz/sim/maritime_gui/FieldLink.hh"
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
  /// \brief The link to the world: topic, state service, arrows. Blue, and
  /// 4 m per m/s, so a 0.5 m/s current is 2 m.
  public: FieldLink link{"ocean_current", "OceanCurrentPanel",
                         math::Color(0.1f, 0.6f, 1.0f, 1.0f), 4.0};

  /// \brief The current as last read, if the world has one.
  public: std::optional<CurrentValues> current;

  /// \brief What became of the last change.
  public: ChangeTracker::State state{ChangeTracker::State::kIdle};

  /// \brief A field that is not a number, reported until the next Apply.
  public: std::string invalid;

  /// \brief Samples the current for the arrows.
  public: ocean_current::OceanCurrentSampler sampler;
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
  if (!d.invalid.empty())
  {
    return QString::fromStdString("Not sent: " + d.invalid +
                                  " is not a number.");
  }
  return QString::fromStdString(StatusText(d.state));
}

//////////////////////////////////////////////////
bool OceanCurrentPanel::ShowArrows() const
{
  return this->dataPtr->link.ShowArrows();
}

//////////////////////////////////////////////////
void OceanCurrentPanel::SetShowArrows(bool _show)
{
  this->dataPtr->link.SetShowArrows(_show);
  emit this->ArrowsChanged();
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

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::maritime_gui::OceanCurrentPanel, gz::gui::Plugin)
