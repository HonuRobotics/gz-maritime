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
/// read from the GUI's ECM, the receivers and their readings heard on the
/// acoustics system's topics, and the drawings of each (the reach of a
/// pinger on the water, its pings spreading, and the bearing each receiver
/// reports), as MarkerManager markers.
namespace gz::sim::maritime_gui
{
/// \brief One pinger as the GUI sees it.
struct PingerView
{
  /// \brief How it is set.
  public: acoustics::PingerData data;

  /// \brief Its world position.
  public: math::Vector3d position;
};

/// \brief Every pinger in an ECM, by name.
/// \param[in] _ecm The GUI's ECM, or a copy of the world's state.
/// \return The pingers, sorted by name.
std::vector<PingerView> ReadPingers(const EntityComponentManager &_ecm);

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

  /// \brief Fraction of a pinger's reach where its pings start to drop; 1
  /// is no dropout.
  public: double dropoutStart{1.0};
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

/// \brief Which horizontal plane the reach and the pings are drawn on.
enum class Plane
{
  /// \brief The water's surface: the view from above, and where a boat's
  /// hydrophone listens, give or take its draft.
  kWater,

  /// \brief A depth under the water, set by hand.
  kDepth,

  /// \brief The depth of a receiver, following it: the slice a vehicle
  /// under the water is listening in.
  kReceiver,

  /// \brief Not a plane: the reach and the pings as wireframe spheres, cut
  /// at the water, where sound stops.
  k3D
};

/// \brief A plane's name in a configuration: water, depth, receiver or 3d.
/// \param[in] _name The name.
/// \param[out] _plane The plane, when the name is one.
/// \return True if the name is a plane.
bool ParsePlane(const std::string &_name, Plane &_plane);

/// \brief A plane's name.
/// \param[in] _plane The plane.
/// \return water, depth, receiver or 3d.
std::string PlaneName(Plane _plane);

/// \brief The world z to draw at.
/// \param[in] _plane Which plane.
/// \param[in] _waterLevel World z of the water.
/// \param[in] _depth m under the water, for kDepth; not negative.
/// \param[in] _receiverZ The receiver's world z, for kReceiver, when known.
/// \return The z. On the water, a little above it, so the drawn sea does
/// not hide the drawing; a receiver not known yet falls back to the water.
double PlaneZ(Plane _plane, double _waterLevel, double _depth,
              const std::optional<double> &_receiverZ);

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

/// \brief The pings spreading from each active pinger, where their sound
/// meets a horizontal plane: a ring per ping, out to the pinger's reach or
/// to `_maxRadius`, fading as it grows.
/// \param[in] _pingers The pingers.
/// \param[in] _planeZ World z of the plane: the water, or a depth under it.
/// \param[in] _now Simulation time.
/// \param[in] _speed m/s the rings spread at: the speed of sound, or less
/// so the eye can follow them.
/// \param[in] _maxRadius m, the farthest a ring is drawn.
/// \param[in] _ns Marker namespace.
/// \return LINE_LIST markers, a few per pinger, one per shade.
std::vector<msgs::Marker> PingMarkers(const std::vector<PingerView> &_pingers,
    double _planeZ, const std::chrono::steady_clock::duration &_now,
    double _speed, double _maxRadius, const std::string &_ns);

/// \brief A wireframe sphere as LINE_LIST segments, cut at a height: its
/// meridians and its parallels, and the circle where it meets the cut.
/// \param[in] _centre Its centre.
/// \param[in] _radius m.
/// \param[in] _ceiling World z above which nothing is drawn.
/// \param[in] _meridians Half great circles through the poles, at least 2.
/// \param[in] _parallels Circles of latitude between the poles, at least 1.
/// \param[in] _segments Segments around a circle, at least 8.
/// \return Pairs of points, none above the ceiling.
std::vector<math::Vector3d> SphereSegments(const math::Vector3d &_centre,
    double _radius, double _ceiling, int _meridians, int _parallels,
    int _segments = 48);

/// \brief The reach of each pinger as a wireframe sphere, cut at the water,
/// solid in its frequency's colour when it pings, grey when it is off. A
/// pinger with no reach limit draws none.
/// \param[in] _pingers The pingers.
/// \param[in] _waterLevel World z of the water.
/// \param[in] _ns Marker namespace.
/// \return One LINE_LIST marker per pinger with a reach, ids from 1.
std::vector<msgs::Marker> ReachSpheres(const std::vector<PingerView> &_pingers,
                                       double _waterLevel,
                                       const std::string &_ns);

/// \brief The pings spreading from each active pinger as wireframe
/// spheres, sparser than the reach, cut at the water, out to the reach or
/// `_maxRadius`, fading as they grow.
/// \param[in] _pingers The pingers.
/// \param[in] _waterLevel World z of the water.
/// \param[in] _now Simulation time.
/// \param[in] _speed m/s the spheres grow at.
/// \param[in] _maxRadius m, the largest drawn.
/// \param[in] _ns Marker namespace.
/// \return LINE_LIST markers, a few per pinger, one per shade.
std::vector<msgs::Marker> PingSpheres(const std::vector<PingerView> &_pingers,
    double _waterLevel, const std::chrono::steady_clock::duration &_now,
    double _speed, double _maxRadius, const std::string &_ns);

/// \brief Where pings start to drop around each pinger with a reach: a
/// dashed ring, lighter than the reach, for each dropout start the
/// receivers use, where the sphere of that radius cuts a horizontal plane.
/// \param[in] _pingers The pingers.
/// \param[in] _starts Fractions of the reach; those not in (0, 1) and
/// repeats are skipped.
/// \param[in] _planeZ World z of the plane.
/// \param[in] _ns Marker namespace.
/// \param[in] _firstId The first marker's id, after the reach's.
/// \return One LINE_LIST marker per pinger and start that meets the plane.
std::vector<msgs::Marker> DropoutMarkers(
    const std::vector<PingerView> &_pingers, std::vector<double> _starts,
    double _planeZ, const std::string &_ns, std::uint64_t _firstId = 1001);

/// \brief A bearing a receiver reported, to draw.
struct BearingRay
{
  /// \brief The receiver's world pose when it was drawn.
  public: math::Pose3d pose;

  /// \brief The reading.
  public: HeardPing ping;

  /// \brief How far through its life the ray is, 0 new to 1 gone: it
  /// fades towards white.
  public: double fade{0.0};
};

/// \brief A ray from each receiver along the bearing and elevation of its
/// last reading, in its frequency's colour, red when the reading was
/// invalid, fading in three steps as it ages.
/// \param[in] _rays The rays.
/// \param[in] _length m.
/// \param[in] _ns Marker namespace.
/// \return LINE_LIST markers, one per colour, ids from 1.
std::vector<msgs::Marker> RayMarkers(const std::vector<BearingRay> &_rays,
                                     double _length, const std::string &_ns);

/// \brief A drawing on the scene, through MarkerManager's array service:
/// the markers drawn last are replaced, and those no longer drawn deleted.
class MarkerLayer
{
  /// \brief Constructor.
  /// \param[in] _ns The drawing's namespace.
  /// \param[in] _service MarkerManager's service.
  public: MarkerLayer(const std::string &_ns, const std::string &_service);

  /// \brief Destructor: takes the drawing off the scene.
  public: ~MarkerLayer();

  /// \brief Draw: the markers with points added or replaced, those drawn
  /// before and empty or missing now deleted.
  /// \param[in] _markers The markers, in this layer's namespace.
  public: void Draw(const std::vector<msgs::Marker> &_markers);

  /// \brief Take the drawing off the scene.
  public: void Clear();

  /// \brief Whether anything is drawn.
  /// \return True if it is.
  public: bool Drawn() const;

  /// \brief The namespace.
  /// \return It.
  public: const std::string &Ns() const;

  /// \brief The private data.
  private: struct Impl;

  /// \brief Private data pointer.
  private: std::unique_ptr<Impl> impl;
};

/// \brief Listens to the acoustics system: the receivers it lists, and the
/// readings each publishes. Thread safe.
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
