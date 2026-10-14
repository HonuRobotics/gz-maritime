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
#ifndef GZ_SIM_MARITIME_GUI_VESSELS_HH_
#define GZ_SIM_MARITIME_GUI_VESSELS_HH_

#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "gz/sim/maritime_gui/Flow.hh"

/// \brief The water and the wind each vehicle meets, without Qt: the
/// vehicles found in the GUI's ECM (every non-static model with a link,
/// buoys included), the velocity of each model's origin over the ground
/// from its poses, its track, and the drawings of the water and the wind
/// relative to it at its origin. A picture, not the forces: those act at
/// the centre of mass or on each marked shape.
namespace gz::sim::maritime_gui
{
/// \brief One vehicle as the GUI sees it.
struct Vessel
{
  /// \brief Its model's name.
  std::string name;

  /// \brief Its model's world pose.
  math::Pose3d pose;

  /// \brief Its velocity over the ground, world frame, estimated from its
  /// poses.
  math::Vector3d velocity;

  /// \brief Where it has been, oldest first.
  std::deque<math::Vector3d> track;
};

/// \brief Finds the vehicles in the GUI's ECM, every model that is not
/// static, sits on the world and has a link, spawned later included, and
/// follows them: velocity from their poses over simulation time, smoothed,
/// and a track of where they have been. The GUI's ECM carries poses, not
/// velocities, so the velocity is estimated.
class VesselTracker
{
  /// \brief Constructor.
  /// \param[in] _trackSeconds Seconds of track kept.
  /// \param[in] _trackStep Seconds between two points of the track.
  public: explicit VesselTracker(double _trackSeconds = 30.0,
                                 double _trackStep = 0.5);

  /// \brief Look at the ECM again.
  /// \param[in] _ecm The GUI's ECM.
  /// \param[in] _simTime Simulation time.
  public: void Update(const EntityComponentManager &_ecm,
                      const std::chrono::steady_clock::duration &_simTime);

  /// \brief The vehicles, by name.
  /// \return The vehicles.
  public: std::vector<Vessel> Vessels() const;

  /// \brief Forget every vehicle, after a reset.
  public: void Clear();

  /// \brief What it keeps of one vehicle.
  private: struct State
  {
    /// \brief The vehicle.
    Vessel vessel;

    /// \brief The pose and time the velocity was last estimated from.
    math::Vector3d lastPosition;
    std::chrono::steady_clock::duration lastTime{0};

    /// \brief Simulation time of the track's last point.
    std::optional<std::chrono::steady_clock::duration> lastTrack;

    /// \brief Whether the velocity has one estimate yet.
    bool hasVelocity{false};

    /// \brief Seen in the last update.
    bool seen{false};
  };

  /// \brief Seconds of track kept.
  private: double trackSeconds;

  /// \brief Seconds between two points of the track.
  private: double trackStep;

  /// \brief The vehicles, by entity.
  private: std::map<Entity, State> states;
};

/// \brief A solid arrow: a shaft and a head as triangles, twice, crossed
/// along the arrow, one copy flat in the horizontal plane through its start
/// and one upright, so it reads from above and from the side alike.
/// \param[in,out] _bins Where the triangles go, coloured by `_speed`.
/// \param[in] _start Where it starts.
/// \param[in] _vector Its direction and length; only x and y are used.
/// \param[in] _width Its shaft's width, m; the head is twice as wide.
/// \param[in] _speed m/s, for its colour.
void FlatArrow(ColorBins &_bins, const math::Vector3d &_start,
               const math::Vector3d &_vector, double _width, double _speed);

/// \brief One flat arrow per vehicle, for a velocity relative to it: the
/// water past the hull, or the apparent wind.
/// \param[in] _vessels The vehicles.
/// \param[in] _relative The velocity to draw at a vehicle.
/// \param[in] _height m above the vehicle's origin the arrow is drawn at.
/// \param[in] _scale m of arrow per m/s.
/// \param[in] _width The shaft's width, m.
/// \param[in] _colormap Colour by speed.
/// \return The arrows in colour bins of triangles.
ColorBins VesselArrows(const std::vector<Vessel> &_vessels,
    const std::function<math::Vector3d(const Vessel &)> &_relative,
    double _height, double _scale, double _width,
    const Colormap &_colormap);

/// \brief Each vehicle's track over the ground, fading towards its oldest
/// end, and a line from its origin along its heading (its x axis): where it
/// goes against where it points.
/// \param[in] _vessels The vehicles.
/// \param[in] _z World z of the drawing.
/// \param[in] _headingLength m of the heading line.
/// \return The track in colour bins of lines, white to dark.
ColorBins TracksAndHeadings(const std::vector<Vessel> &_vessels, double _z,
                            double _headingLength);
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_VESSELS_HH_
