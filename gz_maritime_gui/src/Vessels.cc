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
#include "gz/sim/maritime_gui/Vessels.hh"

#include <algorithm>
#include <cmath>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/World.hh>

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief Shortest interval of simulation time a velocity is estimated
  /// over, s: shorter ones are mostly noise.
  constexpr double kMinVelocityDt{0.1};

  /// \brief Weight of a new velocity estimate against the smoothed one.
  constexpr double kVelocitySmoothing{0.5};

  /// \brief A seconds count.
  /// \param[in] _d A duration.
  /// \return Seconds.
  double Seconds(const std::chrono::steady_clock::duration &_d)
  {
    return std::chrono::duration<double>(_d).count();
  }
}

//////////////////////////////////////////////////
VesselTracker::VesselTracker(double _trackSeconds, double _trackStep)
  : trackSeconds(std::max(0.0, _trackSeconds)),
    trackStep(_trackStep > 0.0 ? _trackStep : 0.5)
{
}

//////////////////////////////////////////////////
void VesselTracker::Update(const EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_simTime)
{
  for (auto &[entity, state] : this->states)
    state.seen = false;

  const Entity world = _ecm.EntityByComponents(components::World());
  _ecm.Each<components::Model, components::Name, components::ParentEntity>(
      [&](const Entity &_model, const components::Model *,
          const components::Name *_name,
          const components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() != world)
          return true;
        const auto *isStatic = _ecm.Component<components::Static>(_model);
        if (isStatic && isStatic->Data())
          return true;
        if (kNullEntity == _ecm.EntityByComponents(components::Link(),
                components::ParentEntity(_model)))
        {
          return true;
        }

        const math::Pose3d pose = worldPose(_model, _ecm);
        auto [it, fresh] = this->states.try_emplace(_model);
        State &s = it->second;
        s.seen = true;
        s.vessel.name = _name->Data();
        s.vessel.pose = pose;
        if (fresh)
        {
          s.lastPosition = pose.Pos();
          s.lastTime = _simTime;
        }

        // Velocity from the poses, over at least kMinVelocityDt, smoothed.
        const double dt = Seconds(_simTime - s.lastTime);
        if (dt >= kMinVelocityDt)
        {
          const math::Vector3d v = (pose.Pos() - s.lastPosition) / dt;
          s.vessel.velocity = s.hasVelocity ?
              s.vessel.velocity * (1.0 - kVelocitySmoothing) +
                  v * kVelocitySmoothing : v;
          s.hasVelocity = true;
          s.lastPosition = pose.Pos();
          s.lastTime = _simTime;
        }
        else if (dt < 0.0)
        {
          // Time went back: a reset.
          s.lastPosition = pose.Pos();
          s.lastTime = _simTime;
          s.vessel.velocity = math::Vector3d::Zero;
          s.vessel.track.clear();
          s.lastTrack.reset();
        }

        // The track, a point every trackStep, trackSeconds of it.
        if (!s.lastTrack || Seconds(_simTime - *s.lastTrack) >= this->trackStep)
        {
          s.vessel.track.push_back(pose.Pos());
          s.lastTrack = _simTime;
          const auto keep = static_cast<std::size_t>(
              std::ceil(this->trackSeconds / this->trackStep)) + 1u;
          while (s.vessel.track.size() > keep)
            s.vessel.track.pop_front();
        }
        return true;
      });

  // A vehicle removed is forgotten.
  for (auto it = this->states.begin(); it != this->states.end();)
    it = it->second.seen ? std::next(it) : this->states.erase(it);
}

//////////////////////////////////////////////////
std::vector<Vessel> VesselTracker::Vessels() const
{
  std::vector<Vessel> out;
  out.reserve(this->states.size());
  for (const auto &[entity, state] : this->states)
    out.push_back(state.vessel);
  std::sort(out.begin(), out.end(), [](const Vessel &_a, const Vessel &_b)
      { return _a.name < _b.name; });
  return out;
}

//////////////////////////////////////////////////
void VesselTracker::Clear()
{
  this->states.clear();
}

//////////////////////////////////////////////////
void FlatArrow(ColorBins &_bins, const math::Vector3d &_start,
    const math::Vector3d &_vector, double _width, double _speed)
{
  const math::Vector3d along(_vector.X(), _vector.Y(), 0.0);
  const double length = along.Length();
  if (length < 1e-6)
    return;
  const math::Vector3d u = along / length;
  // The head: a third of the arrow, at most three widths long.
  const double head = std::min(length / 3.0, 3.0 * _width);
  const double w = 0.5 * _width;
  const math::Vector3d neck = _start + u * (length - head);
  const math::Vector3d tip = _start + u * length;

  // Two copies crossed along the arrow, one flat and one upright, so it
  // reads from above and from a camera low over the water alike.
  for (const math::Vector3d &side :
       {math::Vector3d(-u.Y(), u.X(), 0.0), math::Vector3d::UnitZ})
  {
    // The shaft: two triangles.
    _bins.AddTriangle(_start + side * w, _start - side * w, neck - side * w,
                      _speed);
    _bins.AddTriangle(_start + side * w, neck - side * w, neck + side * w,
                      _speed);
    // The head.
    _bins.AddTriangle(neck + side * (2.0 * w), neck - side * (2.0 * w), tip,
                      _speed);
  }
}

//////////////////////////////////////////////////
ColorBins VesselArrows(const std::vector<Vessel> &_vessels,
    const std::function<math::Vector3d(const Vessel &)> &_relative,
    double _height, double _scale, double _width,
    const Colormap &_colormap)
{
  ColorBins bins(_colormap, 8, 1, msgs::Marker::TRIANGLE_LIST);
  for (const auto &v : _vessels)
  {
    const math::Vector3d rel = _relative(v);
    const math::Vector3d start = v.pose.Pos() + math::Vector3d(0, 0, _height);
    FlatArrow(bins, start, rel * _scale, _width, rel.Length());
  }
  return bins;
}

//////////////////////////////////////////////////
ColorBins TracksAndHeadings(const std::vector<Vessel> &_vessels, double _z,
    double _headingLength)
{
  // White to dark grey; "speed" here is only the bin, the full colour at 1.
  ColorBins bins(Colormap({math::Color::White}, 1.0), 1, 4);
  for (const auto &v : _vessels)
  {
    const std::size_t n = v.track.size();
    for (std::size_t i = 1; i < n; ++i)
    {
      math::Vector3d a = v.track[i - 1];
      math::Vector3d b = v.track[i];
      a.Z(_z);
      b.Z(_z);
      bins.Add(a, b, 1.0, static_cast<double>(i) / static_cast<double>(n));
    }
    if (n > 0u)
    {
      // Join the track's newest point to the vehicle.
      math::Vector3d a = v.track.back();
      math::Vector3d b = v.pose.Pos();
      a.Z(_z);
      b.Z(_z);
      bins.Add(a, b, 1.0, 1.0);
    }
    if (_headingLength > 0.0)
    {
      math::Vector3d x = v.pose.Rot().RotateVector(math::Vector3d::UnitX);
      x.Z(0.0);
      if (x.Length() > 1e-6)
      {
        math::Vector3d a = v.pose.Pos();
        a.Z(_z);
        bins.Add(a, a + x.Normalized() * _headingLength, 1.0, 1.0);
      }
    }
  }
  return bins;
}
}  // namespace gz::sim::maritime_gui
