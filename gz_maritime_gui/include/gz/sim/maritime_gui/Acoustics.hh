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
#ifndef GZ_SIM_MARITIME_GUI_ACOUSTICS_HH_
#define GZ_SIM_MARITIME_GUI_ACOUSTICS_HH_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/sim/EntityComponentManager.hh>

#include "gz/sim/acoustics/Pinger.hh"

/// \brief The acoustics as the GUI shows them, without Qt: the pingers
/// read from the GUI's ECM and from the acoustics system's ground truth,
/// the receivers and their readings heard on the acoustics system's topics,
/// and the drawings of each (the reach of a pinger on the water, its pings
/// spreading, and the bearing each receiver reports), as MarkerManager
/// markers.
namespace gz::sim::maritime_gui
{
/// \brief One pinger as the GUI sees it.
struct PingerView
{
  /// \brief How it is set.
  public: acoustics::PingerData data;

  /// \brief Its world position.
  public: math::Vector3d position;

  /// \brief Simulation time its schedule last changed, or it went under
  /// the water: no ping before it is drawn, since none was sent as drawn.
  /// Unset for a pinger not seen to change.
  public: std::optional<std::chrono::steady_clock::duration> since;
};

/// \brief Every pinger in an ECM, by name.
/// \param[in] _ecm The GUI's ECM.
/// \return The pingers, sorted by name.
std::vector<PingerView> ReadPingers(const EntityComponentManager &_ecm);

/// \brief The acoustics system's ground truth message as a list.
/// \param[in] _msg The message on `/world/<world>/acoustics/pingers`.
/// \return The pingers, sorted by name.
std::vector<PingerView> ParsePingers(const msgs::Param &_msg);

/// \brief The pingers a panel shows: those the acoustics system lists, each
/// replaced by the GUI's own state of it when the GUI has one, which is
/// newer. A panel loaded with the GUI has every pinger in its state; one
/// loaded later has only those that changed since, and the list fills in
/// the rest.
/// \param[in] _listed The pingers of the ground truth topic.
/// \param[in] _state The pingers of the GUI's ECM.
/// \return The pingers, sorted by name.
std::vector<PingerView> MergePingers(std::vector<PingerView> _listed,
                                     const std::vector<PingerView> &_state);

/// \brief Follows the pingers from one update to the next, to know since
/// when each pings as it does now: the rings are drawn from the schedule,
/// and a pinger switched on a moment ago has sent no ping before that.
class PingerChanges
{
  /// \brief Set each pinger's `since`: the time its schedule (active,
  /// period, offset) changed or it went under the water, unset while it is
  /// as it was first seen.
  /// \param[in,out] _pingers The pingers.
  /// \param[in] _now Simulation time; going back, a reset, forgets all.
  /// \param[in] _waterLevel World z of the water.
  public: void Track(std::vector<PingerView> &_pingers,
                     const std::chrono::steady_clock::duration &_now,
                     double _waterLevel);

  /// \brief What a pinger was last seen as.
  private: struct Seen
  {
    /// \brief Its schedule and whether it was under the water, as text.
    public: std::string schedule;

    /// \brief When that changed, if it was seen to.
    public: std::optional<std::chrono::steady_clock::duration> since;
  };

  /// \brief The pingers by name.
  private: std::map<std::string, Seen> seen;

  /// \brief Simulation time of the last call.
  private: std::chrono::steady_clock::duration last{0};
};

/// \brief One receiver, as the acoustics system lists it.
struct ReceiverInfo
{
  /// \brief Its scoped name, from the world.
  public: std::string name;

  /// \brief The topic it publishes its readings on.
  public: std::string topic;

  /// \brief The frame its readings are in.
  public: std::string frameId;

  /// \brief Hz, its band's centre; 0 hears every frequency.
  public: double frequency{0.0};

  /// \brief Hz, its band's full width.
  public: double bandwidth{0.0};
};

/// \brief One reading, as a receiver published it.
struct HeardPing
{
  /// \brief rad, in the sensor frame.
  public: double bearing{0.0};

  /// \brief rad, positive up.
  public: double elevation{0.0};

  /// \brief Hz.
  public: double frequency{0.0};

  /// \brief dB relative to 1 m.
  public: double amplitude{0.0};

  /// \brief False when it overlapped another.
  public: bool valid{true};

  /// \brief Simulation time it arrived.
  public: std::chrono::steady_clock::duration stamp{0};
};

/// \brief A colour per frequency, the same on the panel and on the scene:
/// the RoboNation band, 25 to 40 kHz, spread over the hues from orange to
/// violet; below and above it, the ends.
/// \param[in] _hz Hz.
/// \return The colour.
math::Color FrequencyColor(double _hz);

/// \brief The radius of the circle where a sphere around a point cuts a
/// horizontal plane.
/// \param[in] _radius The sphere's radius, m.
/// \param[in] _depth How far the plane is from the sphere's centre, m.
/// \return The circle's radius; 0 when the sphere does not reach the plane.
double CutRadius(double _radius, double _depth);

/// \brief A horizontal circle as LINE_LIST segments.
/// \param[in] _centre Its centre.
/// \param[in] _radius m.
/// \param[in] _dashed Every other segment only.
/// \param[in] _segments Segments around, at least 8.
/// \return Pairs of points.
std::vector<math::Vector3d> CircleSegments(const math::Vector3d &_centre,
                                           double _radius, bool _dashed,
                                           int _segments = 96);

/// \brief The pings of a pinger whose sound is still spreading at a time:
/// those sent in (now - window, now].
/// \param[in] _pinger The pinger.
/// \param[in] _now Simulation time.
/// \param[in] _window How long a ping is drawn.
/// \return When they were sent, oldest first.
std::vector<std::chrono::steady_clock::duration> RecentPings(
    const acoustics::PingerData &_pinger,
    const std::chrono::steady_clock::duration &_now,
    const std::chrono::steady_clock::duration &_window);

/// \brief The reach of each pinger on a horizontal plane: the circle where
/// its reach sphere cuts the plane, solid in its frequency's colour when it
/// pings, grey and dashed when it is off. A pinger with no reach limit, or
/// whose reach does not get to the plane, draws none.
/// \param[in] _pingers The pingers.
/// \param[in] _planeZ World z of the plane: the water, or a depth under it.
/// \param[in] _ns Marker namespace.
/// \return One LINE_LIST marker per pinger whose reach meets the plane, ids
/// from 1.
std::vector<msgs::Marker> ReachMarkers(const std::vector<PingerView> &_pingers,
                                       double _planeZ,
                                       const std::string &_ns);

/// \brief The most rings drawn for one pinger. Slowed down a lot, more
/// pings than this are on their way at once: every second, third... ping is
/// then drawn, always the same ones, so the rings still go out to the reach
/// and what is sent stays bounded.
constexpr int kMaxRings{6};

/// \brief The pings spreading from each active pinger under the water,
/// where their sound meets a horizontal plane: a ring per ping, out to the
/// pinger's reach, fading as it grows. Only the pings the pinger sent: none
/// from a pinger above the water, none from before its `since`.
/// \param[in] _pingers The pingers.
/// \param[in] _planeZ World z of the plane: the water, or a depth under it.
/// \param[in] _waterLevel World z of the water: a pinger at or above it
/// sends nothing.
/// \param[in] _now Simulation time.
/// \param[in] _speed m/s the rings spread at: the speed of sound, or less
/// so the eye can follow them.
/// \param[in] _maxRadius m, the farthest a ring is drawn for a pinger with
/// no reach limit; one with a reach is drawn out to its reach.
/// \param[in] _ns Marker namespace.
/// \return LINE_LIST markers, a few per pinger, one per shade.
std::vector<msgs::Marker> PingMarkers(const std::vector<PingerView> &_pingers,
    double _planeZ, double _waterLevel,
    const std::chrono::steady_clock::duration &_now,
    double _speed, double _maxRadius, const std::string &_ns);

/// \brief A bearing a receiver reported, to draw.
struct BearingRay
{
  /// \brief The receiver's world pose when the reading came: the bearing
  /// is in the sensor frame as it was then, so a ray drawn from a later
  /// pose would turn with the vehicle and point away from the pinger.
  public: math::Pose3d pose;

  /// \brief The reading.
  public: HeardPing ping;
};

/// \brief Keep, for each receiver, its last reading with the pose the
/// receiver had when the reading came: a new reading takes the pose given
/// now, one already kept keeps its own.
/// \param[in,out] _kept The rays, by the receiver's topic.
/// \param[in] _last The last reading of each receiver, by topic.
/// \param[in] _poses The receivers' world poses now, by topic; a new
/// reading of a receiver without one waits for it.
void KeepRays(std::map<std::string, BearingRay> &_kept,
              const std::map<std::string, HeardPing> &_last,
              const std::map<std::string, math::Pose3d> &_poses);

/// \brief The rays still to draw: those whose reading arrived within a
/// lifetime of now.
/// \param[in] _kept The rays, by the receiver's topic.
/// \param[in] _now Simulation time.
/// \param[in] _lifetime How long a ray stays.
/// \return The rays.
std::vector<BearingRay> LiveRays(
    const std::map<std::string, BearingRay> &_kept,
    const std::chrono::steady_clock::duration &_now,
    const std::chrono::steady_clock::duration &_lifetime);

/// \brief A ray from each receiver along the bearing and elevation of its
/// last reading, in its frequency's colour, red when the reading was
/// invalid.
/// \param[in] _rays The rays.
/// \param[in] _length m.
/// \param[in] _ns Marker namespace.
/// \return LINE_LIST markers, one per colour, ids from 1.
std::vector<msgs::Marker> RayMarkers(const std::vector<BearingRay> &_rays,
                                     double _length, const std::string &_ns);

/// \brief Listens to the acoustics system: the pingers and the receivers
/// it lists, and the readings each receiver publishes. Thread safe.
class AcousticListener
{
  /// \brief Constructor.
  public: AcousticListener();

  /// \brief Destructor.
  public: ~AcousticListener();

  /// \brief Listen to a world's acoustics, once its name is known; a call
  /// with the same world again does nothing.
  /// \param[in] _world The world's name.
  public: void Listen(const std::string &_world);

  /// \brief The receivers, as last listed.
  /// \return They, by name.
  public: std::vector<ReceiverInfo> Receivers() const;

  /// \brief The pingers, as the ground truth last listed them.
  /// \return They, by name.
  public: std::vector<PingerView> Pingers() const;

  /// \brief The last reading of each receiver that has published one.
  /// \return Readings by topic.
  public: std::map<std::string, HeardPing> Last() const;

  /// \brief Whether the acoustics system has listed its receivers.
  /// \return True once it has.
  public: bool Heard() const;

  /// \brief The private data.
  private: struct Impl;

  /// \brief Private data pointer.
  private: std::unique_ptr<Impl> impl;
};

/// \brief A reading's message as a HeardPing.
/// \param[in] _msg The message.
/// \return The reading, if it has a bearing.
std::optional<HeardPing> ParseReading(const msgs::Param &_msg);

/// \brief The acoustics system's receivers message as a list.
/// \param[in] _msg The message.
/// \return The receivers, by name.
std::vector<ReceiverInfo> ParseReceivers(const msgs::Param &_msg);
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_ACOUSTICS_HH_
