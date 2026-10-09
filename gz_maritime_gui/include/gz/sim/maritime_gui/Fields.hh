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
#ifndef GZ_SIM_MARITIME_GUI_FIELDS_HH_
#define GZ_SIM_MARITIME_GUI_FIELDS_HH_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/sim/EntityComponentManager.hh>

/// \brief What the maritime GUI does without Qt: read the wind and the
/// ocean current from their recipes, build the messages that change them,
/// sample them on a grid of arrows, and tell a refused change from an
/// applied one. The GUI plugin is a thin layer over it.
namespace gz::sim::maritime_gui
{
/// \brief The ocean current as its recipe gives it.
struct CurrentValues
{
  /// \brief Generation of the recipe; a new one on every change.
  std::uint64_t generation{0};

  /// \brief The registered model.
  std::string model;

  /// \brief m/s.
  double speed{0.0};

  /// \brief Degrees clockwise from north the current sets towards.
  double direction{0.0};

  /// \brief World z of the water's surface.
  double waterLevel{0.0};

  /// \brief The parameters the model owns, as text.
  std::map<std::string, std::string> extra;
};

/// \brief The wind as its recipe gives it.
struct WindValues
{
  /// \brief Generation of the recipe; a new one on every change.
  std::uint64_t generation{0};

  /// \brief The registered model.
  std::string model;

  /// \brief m/s at the reference height.
  double speed{0.0};

  /// \brief Degrees clockwise from north the wind comes from.
  double direction{0.0};

  /// \brief Standard deviation of the speed gusts, m/s.
  double speedGust{0.0};

  /// \brief Correlation time of the speed gusts, s.
  double speedGustTime{0.0};

  /// \brief Standard deviation of the direction gusts, degrees.
  double directionGust{0.0};

  /// \brief Correlation time of the direction gusts, s.
  double directionGustTime{0.0};

  /// \brief m above the water the speed is given at.
  double referenceHeight{0.0};

  /// \brief World z of the water.
  double waterLevel{0.0};
};

/// \brief The ocean current recipe on the world, if there is one.
/// \param[in] _ecm The entity component manager.
/// \return The current, or nothing when the world has no ocean current.
std::optional<CurrentValues> ReadCurrent(const EntityComponentManager &_ecm);

/// \brief The wind recipe on the world, if there is one.
/// \param[in] _ecm The entity component manager.
/// \return The wind, or nothing when the world has no wind system.
std::optional<WindValues> ReadWind(const EntityComponentManager &_ecm);

/// \brief The keys a panel edits, by their names on the `set` topic.
/// \param[in] _current The current.
/// \return speed, direction and water_level.
std::map<std::string, double> Editable(const CurrentValues &_current);

/// \brief The keys a panel edits, by their names on the `set` topic.
/// \param[in] _wind The wind.
/// \return speed, direction and the four gust keys.
std::map<std::string, double> Editable(const WindValues &_wind);

/// \brief A `set` message with the edited keys whose value differs from the
/// recipe's, so a message carries only what the user changed.
/// \param[in] _edits The panel's values, by key.
/// \param[in] _recipe The recipe's values, by key.
/// \return The message; empty when nothing changed.
msgs::Param ChangedKeys(const std::map<std::string, double> &_edits,
                        const std::map<std::string, double> &_recipe);

/// \brief The heading the flow goes towards, degrees clockwise from north
/// in [0, 360): a current's own direction, a wind's direction plus 180.
/// \param[in] _direction The field's direction in its own convention.
/// \param[in] _comesFrom True for a direction given by where the flow comes
/// from (the wind), false for where it goes (the current).
/// \return The flow's heading.
double FlowHeading(double _direction, bool _comesFrom);

/// \brief A square grid of arrows in the horizontal plane.
struct Grid
{
  /// \brief Its centre, world x and y.
  math::Vector2d center{0.0, 0.0};

  /// \brief Length of its side, m.
  double extent{40.0};

  /// \brief Distance between two arrows, m.
  double spacing{5.0};
};

/// \brief Most arrows on one side of a grid, so a small spacing over a wide
/// extent cannot ask for millions of samples.
constexpr int kMaxArrowsPerSide{51};

/// \brief The points of a grid at a height.
/// \param[in] _grid The grid. A spacing that would put more than
/// kMaxArrowsPerSide arrows on a side is widened to fit.
/// \param[in] _z World z.
/// \return The points, row by row.
std::vector<math::Vector3d> GridPoints(const Grid &_grid, double _z);

/// \brief Arrows as line segments: a shaft from each point along the
/// velocity there, `_scale` metres per m/s, and two strokes of a head. A
/// point with no velocity draws nothing.
/// \param[in] _points Where the arrows start.
/// \param[in] _velocity The field's velocity at a point, world frame.
/// \param[in] _scale Metres of arrow per m/s.
/// \return Pairs of points, one pair per segment.
std::vector<math::Vector3d> ArrowSegments(
    const std::vector<math::Vector3d> &_points,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    double _scale);

/// \brief A LINE_LIST marker drawing segments, added or replaced.
/// \param[in] _ns Marker namespace.
/// \param[in] _id Marker id, not 0: MarkerManager gives a marker with id 0
/// a new random id, so it would add a marker instead of replacing one.
/// \param[in] _color Its colour.
/// \param[in] _segments Pairs of points.
/// \return The marker.
msgs::Marker ArrowMarker(const std::string &_ns, std::uint64_t _id,
                         const math::Color &_color,
                         const std::vector<math::Vector3d> &_segments);

/// \brief A marker deleting what ArrowMarker drew.
/// \param[in] _ns Marker namespace.
/// \param[in] _id Marker id.
/// \return The marker.
msgs::Marker DeleteMarker(const std::string &_ns, std::uint64_t _id);

/// \brief Tells an applied change from a refused one. The server applies a
/// `set` message whole or refuses it, logging why, and says nothing to the
/// sender; a change it applies is a new recipe generation, so a sent
/// message with no new generation within a timeout was refused.
class ChangeTracker
{
  /// \brief What became of the last message sent.
  public: enum class State
  {
    /// \brief Nothing sent yet.
    kIdle,

    /// \brief Sent, no answer yet.
    kPending,

    /// \brief A new generation came.
    kApplied,

    /// \brief No new generation within the timeout.
    kRefused,

    /// \brief Not sent: nothing subscribed to the topic.
    kUnheard
  };

  /// \brief The clock the timeout runs on.
  public: using Clock = std::chrono::steady_clock;

  /// \brief Constructor.
  /// \param[in] _timeout How long a change may take to show. The default
  /// leaves room for a GUI that receives the world's state slowly, such as
  /// one rendering in software.
  public: explicit ChangeTracker(
      Clock::duration _timeout = std::chrono::seconds(5));

  /// \brief A message was sent.
  /// \param[in] _generation The recipe's generation when it was sent.
  /// \param[in] _now The time it was sent.
  public: void Sent(std::uint64_t _generation, Clock::time_point _now);

  /// \brief A message could not be sent: nothing subscribed to the topic.
  public: void NotSent();

  /// \brief Look at the recipe again.
  /// \param[in] _generation The recipe's generation now.
  /// \param[in] _now The time now.
  /// \return The state of the last message sent.
  public: State Update(std::uint64_t _generation, Clock::time_point _now);

  /// \brief How long a change may take to show.
  private: Clock::duration timeout;

  /// \brief The state of the last message sent.
  private: State state{State::kIdle};

  /// \brief The generation when it was sent.
  private: std::uint64_t sentGeneration{0};

  /// \brief When it was sent.
  private: Clock::time_point sentTime;
};
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_FIELDS_HH_
