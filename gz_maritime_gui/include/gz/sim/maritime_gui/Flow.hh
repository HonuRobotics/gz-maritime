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
#ifndef GZ_SIM_MARITIME_GUI_FLOW_HH_
#define GZ_SIM_MARITIME_GUI_FLOW_HH_

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/param.pb.h>

#include "gz/sim/maritime_gui/Fields.hh"

/// \brief The livelier half of the maritime GUI, without Qt: colour by
/// speed, flow trails that drift with a field, and steering a field by hand
/// with a smooth ramp.
namespace gz::sim::maritime_gui
{
/// \brief A colour map from speed to colour: stops evenly spaced from 0 to
/// a maximum speed, linearly blended, the last one beyond the maximum.
class Colormap
{
  /// \brief Constructor.
  /// \param[in] _stops The colours, slowest first; at least one.
  /// \param[in] _maxSpeed m/s the last stop stands for.
  public: Colormap(std::vector<math::Color> _stops, double _maxSpeed);

  /// \brief The colour for a speed.
  /// \param[in] _speed m/s.
  /// \return The colour.
  public: math::Color At(double _speed) const;

  /// \brief m/s the last stop stands for.
  /// \return The maximum speed.
  public: double MaxSpeed() const;

  /// \brief Set the maximum speed.
  /// \param[in] _maxSpeed m/s, positive; anything else is ignored.
  public: void SetMaxSpeed(double _maxSpeed);

  /// \brief The stops, for a legend.
  /// \return The colours, slowest first.
  public: const std::vector<math::Color> &Stops() const;

  /// \brief The water's map: deep blue, through cyan, to white.
  /// \param[in] _maxSpeed m/s.
  /// \return The map.
  public: static Colormap Water(double _maxSpeed);

  /// \brief The air's map: indigo, through magenta and orange, to yellow.
  /// \param[in] _maxSpeed m/s.
  /// \return The map.
  public: static Colormap Air(double _maxSpeed);

  /// \brief The stops.
  private: std::vector<math::Color> stops;

  /// \brief m/s the last stop stands for.
  private: double maxSpeed;
};

/// \brief Line segments sorted into colour bins, one LINE_LIST marker per
/// bin. Gazebo's renderer draws a line list in its material's colour and
/// ignores colours given point by point, so colour by speed and a trail that
/// fades take one marker per colour: the speed in `speedBins` steps from 0
/// to the colour map's maximum, times the brightness in `fadeBins` steps.
class ColorBins
{
  /// \brief Constructor.
  /// \param[in] _colormap Colour by speed.
  /// \param[in] _speedBins Steps of speed, at least 1.
  /// \param[in] _fadeBins Steps of brightness, at least 1.
  /// \param[in] _type LINE_LIST (Add) or TRIANGLE_LIST (AddTriangle).
  public: ColorBins(const Colormap &_colormap, int _speedBins = 8,
                    int _fadeBins = 3,
                    msgs::Marker::Type _type = msgs::Marker::LINE_LIST);

  /// \brief Add a segment.
  /// \param[in] _a One end.
  /// \param[in] _b The other.
  /// \param[in] _speed m/s, for its colour.
  /// \param[in] _brightness 0 (darkest bin) to 1 (full colour).
  public: void Add(const math::Vector3d &_a, const math::Vector3d &_b,
                   double _speed, double _brightness = 1.0);

  /// \brief Add a triangle, seen from both sides.
  /// \param[in] _a A corner.
  /// \param[in] _b Another.
  /// \param[in] _c The third.
  /// \param[in] _speed m/s, for its colour.
  /// \param[in] _brightness 0 (darkest bin) to 1 (full colour).
  public: void AddTriangle(const math::Vector3d &_a,
                           const math::Vector3d &_b,
                           const math::Vector3d &_c, double _speed,
                           double _brightness = 1.0);

  /// \brief How many bins, so ids `_firstId` to `_firstId + Count() - 1`.
  /// \return The number of bins.
  public: int Count() const;

  /// \brief One marker per bin, empty bins included (with no points), each
  /// an unlit LINE_LIST or TRIANGLE_LIST in its bin's colour.
  /// \param[in] _ns Marker namespace.
  /// \param[in] _firstId Id of the first bin, not 0.
  /// \return The markers, bin by bin.
  public: std::vector<msgs::Marker> Markers(const std::string &_ns,
                                            std::uint64_t _firstId) const;

  /// \brief A bin's colour.
  /// \param[in] _bin The bin.
  /// \return Its colour.
  public: math::Color BinColor(int _bin) const;

  /// \brief Colour by speed.
  private: Colormap colormap;

  /// \brief Steps of speed and of brightness.
  private: int speedBins;
  private: int fadeBins;

  /// \brief The bin a speed and a brightness fall in.
  /// \param[in] _speed m/s.
  /// \param[in] _brightness 0 to 1.
  /// \return The bin.
  private: std::size_t Bin(double _speed, double _brightness) const;

  /// \brief The markers' type.
  private: msgs::Marker::Type type;

  /// \brief Each bin's points, two per segment or three per triangle.
  private: std::vector<std::vector<math::Vector3d>> points;
};

/// \brief A marker of one colour: the type's points, unlit.
/// \param[in] _ns Namespace.
/// \param[in] _id Id, not 0.
/// \param[in] _type LINE_LIST, TRIANGLE_LIST, ...
/// \param[in] _color Its colour.
/// \param[in] _points The points.
/// \return The marker.
msgs::Marker SolidMarker(const std::string &_ns, std::uint64_t _id,
                         msgs::Marker::Type _type, const math::Color &_color,
                         const std::vector<math::Vector3d> &_points);

/// \brief Arrows coloured by speed.
/// \param[in] _points Where the arrows start.
/// \param[in] _velocity The field at a point.
/// \param[in] _scale m of arrow per m/s.
/// \param[in] _colormap Colour by speed.
/// \return The arrows' segments in colour bins.
ColorBins ColoredArrows(const std::vector<math::Vector3d> &_points,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    double _scale, const Colormap &_colormap);

/// \brief Flow trails: particles that drift with a field over a square
/// region on a horizontal plane, drawn as a soft disc that fades out towards
/// the region's edge, each leaving a short trail that fades
/// behind it, and starting again somewhere else when it grows old or leaves
/// the region. They move at the field's velocity times a speed-up, so a
/// slow current still visibly flows.
class ParticleField
{
  /// \brief Settings.
  public: struct Settings
  {
    /// \brief The region, and its plane's z.
    Grid region;

    /// \brief World z of the plane.
    double z{0.0};

    /// \brief How many particles.
    int count{400};

    /// \brief Points kept in a trail, the particle's own included.
    int trailLength{8};

    /// \brief Seconds a particle lives, at most; each draws its own
    /// lifetime between half of this and this, so they do not all restart
    /// at once.
    double lifetime{6.0};

    /// \brief The field's velocity is multiplied by this to move them.
    double speedup{1.0};

    /// \brief Seconds between two points of a trail.
    double trailStep{0.1};
  };

  /// \brief Constructor.
  /// \param[in] _seed Seed of the particles' positions and lifetimes.
  public: explicit ParticleField(std::uint32_t _seed = 1u);

  /// \brief Change the settings; the particles start again.
  /// \param[in] _settings The settings.
  public: void Configure(const Settings &_settings);

  /// \brief The settings.
  /// \return The settings.
  public: const Settings &Config() const;

  /// \brief Move every particle.
  /// \param[in] _dt Seconds of simulation time since the last step;
  /// negative or zero moves nothing.
  /// \param[in] _velocity The field at a point.
  public: void Step(double _dt,
      const std::function<math::Vector3d(const math::Vector3d &)> &_velocity);

  /// \brief The trails, each segment coloured by the speed where it is and
  /// faded towards the trail's tail and as the particle is born and dies.
  /// \param[in] _colormap Colour by speed.
  /// \return The trails' segments in colour bins.
  public: ColorBins Trails(const Colormap &_colormap) const;

  /// \brief The particles' current positions, for tests.
  /// \return One position per particle.
  public: std::vector<math::Vector3d> Positions() const;

  /// \brief One particle.
  private: struct Particle
  {
    /// \brief Its trail, newest last; its position is the last point.
    std::vector<math::Vector3d> trail;

    /// \brief The field's speed at each point of the trail.
    std::vector<double> speed;

    /// \brief Seconds lived.
    double age{0.0};

    /// \brief Seconds it lives.
    double lifetime{1.0};

    /// \brief Seconds since the trail's last point.
    double sinceTrail{0.0};
  };

  /// \brief Start a particle again somewhere in the region.
  /// \param[in,out] _p The particle.
  /// \param[in] _randomAge Start it at a random age, so a fresh field does
  /// not restart all at once.
  private: void Respawn(Particle &_p, bool _randomAge);

  /// \brief How bright a trail is at a point: full in the middle of the
  /// region, fading to nothing at half its side from the centre, so the
  /// trails fill a soft disc.
  /// \param[in] _p The point.
  /// \return 0 to 1.
  private: double EdgeFade(const math::Vector3d &_p) const;

  /// \brief Whether a point is inside the region.
  /// \param[in] _p The point.
  /// \return True if it is.
  private: bool Inside(const math::Vector3d &_p) const;

  /// \brief The settings.
  private: Settings settings;

  /// \brief The particles.
  private: std::vector<Particle> particles;

  /// \brief Random numbers for positions and lifetimes.
  private: std::mt19937 rng;
};

/// \brief Steers a field towards a target set by hand, in small steps: the
/// speed and the direction move towards the target no faster than their
/// rates, one `set` message per period, so the field ramps instead of
/// jumping (a jump is a force spike for a vehicle with added mass). The
/// direction takes the shorter way round.
class Steering
{
  /// \brief The clock the steps run on.
  public: using Clock = std::chrono::steady_clock;

  /// \brief Constructor.
  /// \param[in] _speedRate Fastest change of speed, m/s per second.
  /// \param[in] _turnRate Fastest change of direction, degrees per second.
  /// \param[in] _period Time between two steps.
  public: Steering(double _speedRate, double _turnRate,
      Clock::duration _period = std::chrono::milliseconds(100));

  /// \brief Steer towards a target. The first target after the field was
  /// idle starts from the recipe's values, read at the next Step.
  /// \param[in] _speed m/s, not negative.
  /// \param[in] _direction Degrees, in the field's own convention.
  public: void SetTarget(double _speed, double _direction);

  /// \brief Whether it is still on its way to the target.
  /// \return True if a step is to come.
  public: bool Active() const;

  /// \brief The next step, when one is due.
  /// \param[in] _speed The recipe's speed now, m/s.
  /// \param[in] _direction The recipe's direction now, degrees.
  /// \param[in] _now The time now.
  /// \return A `set` message with speed and direction, or nothing.
  public: std::optional<msgs::Param> Step(double _speed, double _direction,
                                          Clock::time_point _now);

  /// \brief Whether the last step returned reached the target.
  /// \return True if it did.
  public: bool Arrived() const;

  /// \brief Fastest change of speed, m/s per second.
  private: double speedRate;

  /// \brief Fastest change of direction, degrees per second.
  private: double turnRate;

  /// \brief Time between two steps.
  private: Clock::duration period;

  /// \brief The target, once set.
  private: std::optional<std::pair<double, double>> target;

  /// \brief What the last step commanded, once steering started.
  private: std::optional<std::pair<double, double>> commanded;

  /// \brief When the last step was sent.
  private: std::optional<Clock::time_point> lastStep;

  /// \brief Whether the last step reached the target.
  private: bool arrived{false};
};
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_FLOW_HH_
