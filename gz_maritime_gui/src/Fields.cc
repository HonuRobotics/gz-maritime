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
#include "gz/sim/maritime_gui/Fields.hh"

#include <algorithm>
#include <cmath>

#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/Utility.hh>

#include <gz/sim/components/OceanCurrentfield.hh>
#include <gz/sim/components/Windfield.hh>
#include <gz/sim/components/World.hh>

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief Two values a panel shows as the same.
  constexpr double kSame{1e-9};

  /// \brief The head's strokes, as a fraction of the shaft.
  constexpr double kHead{0.25};

  /// \brief The head's half angle, radians.
  constexpr double kHeadAngle{GZ_DTOR(25.0)};

  /// \brief The world entity's component, if any.
  /// \param[in] _ecm The entity component manager.
  /// \return The component, or null.
  template <typename C>
  const C *WorldComponent(const EntityComponentManager &_ecm)
  {
    const Entity world = _ecm.EntityByComponents(components::World());
    if (kNullEntity == world)
      return nullptr;
    return _ecm.Component<C>(world);
  }
}

//////////////////////////////////////////////////
std::optional<CurrentValues> ReadCurrent(const EntityComponentManager &_ecm)
{
  const auto *comp = WorldComponent<components::OceanCurrentfield>(_ecm);
  if (!comp)
    return std::nullopt;
  const auto &d = comp->Data();
  CurrentValues v;
  v.generation = d.generation;
  v.model = d.model;
  v.speed = d.params.speed;
  v.direction = d.params.direction;
  v.waterLevel = d.params.water_level;
  v.extra = d.params.extra;
  return v;
}

//////////////////////////////////////////////////
std::optional<WindValues> ReadWind(const EntityComponentManager &_ecm)
{
  const auto *comp = WorldComponent<components::Windfield>(_ecm);
  if (!comp)
    return std::nullopt;
  const auto &d = comp->Data();
  WindValues v;
  v.generation = d.generation;
  v.model = d.model;
  v.speed = d.params.speed;
  v.direction = d.params.direction;
  v.speedGust = d.params.speed_gust;
  v.speedGustTime = d.params.speed_gust_time;
  v.directionGust = d.params.direction_gust;
  v.directionGustTime = d.params.direction_gust_time;
  v.referenceHeight = d.params.reference_height;
  v.waterLevel = d.params.water_level;
  return v;
}

//////////////////////////////////////////////////
std::map<std::string, double> Editable(const CurrentValues &_current)
{
  return {{"speed", _current.speed},
          {"direction", _current.direction},
          {"water_level", _current.waterLevel}};
}

//////////////////////////////////////////////////
std::map<std::string, double> Editable(const WindValues &_wind)
{
  return {{"speed", _wind.speed},
          {"direction", _wind.direction},
          {"speed_gust", _wind.speedGust},
          {"speed_gust_time", _wind.speedGustTime},
          {"direction_gust", _wind.directionGust},
          {"direction_gust_time", _wind.directionGustTime}};
}

//////////////////////////////////////////////////
msgs::Param ChangedKeys(const std::map<std::string, double> &_edits,
                        const std::map<std::string, double> &_recipe)
{
  msgs::Param msg;
  for (const auto &[key, value] : _edits)
  {
    const auto it = _recipe.find(key);
    if (it != _recipe.end() && std::abs(it->second - value) <= kSame)
      continue;
    auto &any = (*msg.mutable_params())[key];
    any.set_type(msgs::Any::DOUBLE);
    any.set_double_value(value);
  }
  return msg;
}

//////////////////////////////////////////////////
double FlowHeading(double _direction, bool _comesFrom)
{
  double heading = std::fmod(_direction + (_comesFrom ? 180.0 : 0.0), 360.0);
  if (heading < 0.0)
    heading += 360.0;
  return heading;
}

//////////////////////////////////////////////////
std::vector<math::Vector3d> GridPoints(const Grid &_grid, double _z)
{
  std::vector<math::Vector3d> points;
  if (!(_grid.extent >= 0.0) || !(_grid.spacing > 0.0))
    return points;
  const double spacing = std::max(_grid.spacing,
      _grid.extent / static_cast<double>(kMaxArrowsPerSide - 1));
  const int n = static_cast<int>(std::floor(_grid.extent / spacing + 1e-9)) + 1;
  const double start = -0.5 * spacing * (n - 1);
  points.reserve(static_cast<std::size_t>(n) * n);
  for (int j = 0; j < n; ++j)
  {
    for (int i = 0; i < n; ++i)
    {
      points.emplace_back(_grid.center.X() + start + i * spacing,
                          _grid.center.Y() + start + j * spacing, _z);
    }
  }
  return points;
}

//////////////////////////////////////////////////
std::vector<math::Vector3d> ArrowSegments(
    const std::vector<math::Vector3d> &_points,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    double _scale)
{
  std::vector<math::Vector3d> segments;
  segments.reserve(_points.size() * 6);
  for (const auto &p : _points)
  {
    const math::Vector3d v = _velocity(p) * _scale;
    const double length = v.Length();
    if (length < 1e-6)
      continue;
    const math::Vector3d tip = p + v;
    const math::Vector3d back = -v / length * (kHead * length);

    // The head's strokes turn the shaft back about the vertical, so the head
    // of a horizontal arrow lies flat on the water; an arrow pointing up or
    // down turns about x instead.
    const math::Vector3d axis = std::abs(v.Z()) > 0.99 * length ?
        math::Vector3d::UnitX : math::Vector3d::UnitZ;
    const math::Quaterniond left(axis, kHeadAngle);
    const math::Quaterniond right(axis, -kHeadAngle);

    segments.push_back(p);
    segments.push_back(tip);
    segments.push_back(tip);
    segments.push_back(tip + left.RotateVector(back));
    segments.push_back(tip);
    segments.push_back(tip + right.RotateVector(back));
  }
  return segments;
}

//////////////////////////////////////////////////
msgs::Marker ArrowMarker(const std::string &_ns, std::uint64_t _id,
                         const math::Color &_color,
                         const std::vector<math::Vector3d> &_segments)
{
  msgs::Marker marker;
  marker.set_ns(_ns);
  marker.set_id(_id);
  marker.set_action(msgs::Marker::ADD_MODIFY);
  marker.set_type(msgs::Marker::LINE_LIST);
  marker.set_visibility(msgs::Marker::GUI);
  msgs::Set(marker.mutable_pose(), math::Pose3d::Zero);
  auto *material = marker.mutable_material();
  msgs::Set(material->mutable_ambient(), _color);
  msgs::Set(material->mutable_diffuse(), _color);
  msgs::Set(material->mutable_emissive(), _color);
  for (const auto &point : _segments)
    msgs::Set(marker.add_point(), point);
  return marker;
}

//////////////////////////////////////////////////
msgs::Marker DeleteMarker(const std::string &_ns, std::uint64_t _id)
{
  msgs::Marker marker;
  marker.set_ns(_ns);
  marker.set_id(_id);
  marker.set_action(msgs::Marker::DELETE_MARKER);
  return marker;
}

//////////////////////////////////////////////////
ChangeTracker::ChangeTracker(Clock::duration _timeout)
  : timeout(_timeout)
{
}

//////////////////////////////////////////////////
void ChangeTracker::Sent(std::uint64_t _generation, Clock::time_point _now)
{
  this->state = State::kPending;
  this->sentGeneration = _generation;
  this->sentTime = _now;
}

//////////////////////////////////////////////////
void ChangeTracker::NotSent()
{
  this->state = State::kUnheard;
}

//////////////////////////////////////////////////
ChangeTracker::State ChangeTracker::Update(std::uint64_t _generation,
    Clock::time_point _now)
{
  if (State::kPending != this->state)
    return this->state;
  if (_generation != this->sentGeneration)
    this->state = State::kApplied;
  else if (_now - this->sentTime >= this->timeout)
    this->state = State::kRefused;
  return this->state;
}
}  // namespace gz::sim::maritime_gui
