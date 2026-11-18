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
// The acoustics panel's logic, without a GUI or a scene: the pingers read
// from an ECM and from the acoustics system's list, the drawings as markers,
// the messages parsed, a listener hearing a fake acoustics system, and a
// marker layer sending to a fake MarkerManager in this process.

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/param.pb.h>

#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/Helpers.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AcousticPinger.hh"
#include "gz/sim/maritime_gui/Acoustics.hh"
#include "gz/sim/maritime_gui/MarkerLayer.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;
using namespace std::chrono_literals;

namespace
{
/// \brief A pinger at a place.
/// \param[in] _name Name.
/// \param[in] _pos World position.
/// \param[in] _hz Frequency.
/// \param[in] _reach m.
/// \param[in] _active On.
/// \return The view.
PingerView Pinger(const std::string &_name, const math::Vector3d &_pos,
                  double _hz, double _reach, bool _active)
{
  PingerView p;
  p.data.name = _name;
  p.data.frequency = _hz;
  p.data.reach = _reach;
  p.data.active = _active;
  p.position = _pos;
  return p;
}

/// \brief The distance of every point of a marker from a centre, in xy.
/// \param[in] _m The marker.
/// \param[in] _c The centre.
/// \return The distances.
std::vector<double> Radii(const msgs::Marker &_m, const math::Vector3d &_c)
{
  std::vector<double> out;
  for (const auto &p : _m.point())
    out.push_back(std::hypot(p.x() - _c.X(), p.y() - _c.Y()));
  return out;
}

/// \brief The radii of the rings in a set of markers, in cm.
/// \param[in] _markers The markers.
/// \param[in] _c The rings' centre.
/// \return The distinct radii.
std::set<long> RingRadii(const std::vector<msgs::Marker> &_markers,
                         const math::Vector3d &_c)
{
  std::set<long> out;
  for (const auto &m : _markers)
  {
    for (const auto r : Radii(m, _c))
      out.insert(std::lround(r * 100));
  }
  return out;
}

/// \brief Stands for MarkerManager: keeps what comes on its array service.
class FakeMarkerManager
{
  /// \brief Constructor.
  /// \param[in] _service The marker service.
  public: explicit FakeMarkerManager(const std::string &_service)
  {
    this->node.Advertise(_service + "_array", &FakeMarkerManager::OnArray,
                         this);
  }

  /// \brief A request came.
  /// \param[in] _req The markers.
  /// \param[out] _rep Taken.
  /// \return True.
  public: bool OnArray(const msgs::Marker_V &_req, msgs::Boolean &_rep)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->received.push_back(_req);
    _rep.set_data(true);
    return true;
  }

  /// \brief The markers so far, and forget them.
  /// \return The markers.
  public: std::vector<msgs::Marker> Take()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    std::vector<msgs::Marker> out;
    for (const auto &r : this->received)
      out.insert(out.end(), r.marker().begin(), r.marker().end());
    this->received.clear();
    return out;
  }

  /// \brief The requests.
  private: std::vector<msgs::Marker_V> received;

  /// \brief Guards them.
  private: std::mutex mutex;

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};
}

//////////////////////////////////////////////////
/// The pingers in an ECM, by name, where their link and offset put them.
TEST(AcousticsGui, ReadPingers)
{
  EntityComponentManager ecm;
  auto addLink = [&ecm](const std::string &_name, const math::Pose3d &_pose,
                        const std::string &_pinger, double _z)
  {
    const Entity link = ecm.CreateEntity();
    ecm.CreateComponent(link, components::Link());
    ecm.CreateComponent(link, components::Name(_name));
    ecm.CreateComponent(link, components::Pose(_pose));
    acoustics::PingerData d;
    d.name = _pinger;
    d.pose = math::Pose3d(0, 0, _z, 0, 0, 0);
    ecm.CreateComponent(link, components::AcousticPinger(d));
  };
  addLink("b", math::Pose3d(40, -15, 0, 0, 0, 0), "pinger_red", -1.5);
  addLink("a", math::Pose3d(40, 15, 0, 0, 0, 0), "pinger_green", -1.5);

  const auto pingers = ReadPingers(ecm);
  ASSERT_EQ(2u, pingers.size());
  EXPECT_EQ("pinger_green", pingers[0].data.name);
  EXPECT_EQ(math::Vector3d(40, 15, -1.5), pingers[0].position);
  EXPECT_EQ("pinger_red", pingers[1].data.name);
}

//////////////////////////////////////////////////
/// A colour per frequency, from orange at 25 kHz to violet at 40, the ends
/// beyond the band.
TEST(AcousticsGui, FrequencyColor)
{
  EXPECT_EQ(FrequencyColor(20000), FrequencyColor(25000));
  EXPECT_EQ(FrequencyColor(45000), FrequencyColor(40000));
  EXPECT_NE(FrequencyColor(30000), FrequencyColor(37000));
  // Orange: red full, some green, no blue.
  const auto low = FrequencyColor(25000);
  EXPECT_FLOAT_EQ(1.0f, low.R());
  EXPECT_FLOAT_EQ(0.0f, low.B());
}

//////////////////////////////////////////////////
/// The circle where a sphere meets a plane.
TEST(AcousticsGui, CutRadius)
{
  EXPECT_DOUBLE_EQ(4.0, CutRadius(5.0, 3.0));
  EXPECT_DOUBLE_EQ(5.0, CutRadius(5.0, 0.0));
  EXPECT_DOUBLE_EQ(0.0, CutRadius(1.0, 1.5));
}

//////////////////////////////////////////////////
/// A circle as segments, every point on it; dashed is half of them.
TEST(AcousticsGui, CircleSegments)
{
  const math::Vector3d c(1, 2, 0.1);
  const auto solid = CircleSegments(c, 10.0, false, 32);
  const auto dashed = CircleSegments(c, 10.0, true, 32);
  EXPECT_EQ(64u, solid.size());
  EXPECT_EQ(32u, dashed.size());
  for (const auto &p : solid)
  {
    EXPECT_NEAR(10.0, std::hypot(p.X() - c.X(), p.Y() - c.Y()), 1e-9);
    EXPECT_DOUBLE_EQ(c.Z(), p.Z());
  }
  EXPECT_TRUE(CircleSegments(c, 0.0, false).empty());
}

//////////////////////////////////////////////////
/// The pings still spreading: those of the last window.
TEST(AcousticsGui, RecentPings)
{
  acoustics::PingerData p;
  p.period = 1.0;
  p.offset = 0.5;
  const auto t = RecentPings(p, 3700ms, 2s);
  ASSERT_EQ(2u, t.size());
  EXPECT_EQ(std::chrono::steady_clock::duration(2500ms), t[0]);
  EXPECT_EQ(std::chrono::steady_clock::duration(3500ms), t[1]);
}

//////////////////////////////////////////////////
/// The reach on the water: the cut of the reach sphere, solid when it
/// pings, dashed and grey when off; none without a limit.
TEST(AcousticsGui, ReachMarkers)
{
  const math::Vector3d at(10, 0, -3);
  const auto markers = ReachMarkers(
      {Pinger("on", at, 30000, 5, true), Pinger("off", at, 30000, 5, false),
       Pinger("unlimited", at, 30000, 0, true)}, 0.0, "ns");
  ASSERT_EQ(2u, markers.size());
  EXPECT_EQ(1u, markers[0].id());
  EXPECT_EQ(2u, markers[1].id());
  for (const auto r : Radii(markers[0], at))
    EXPECT_NEAR(4.0, r, 1e-9);
  EXPECT_EQ(2 * markers[1].point_size(), markers[0].point_size());
  EXPECT_EQ(msgs::Marker::LINE_LIST, markers[0].type());
  EXPECT_EQ(msgs::Marker::GUI, markers[0].visibility());
  EXPECT_NE(markers[0].material().diffuse().r(),
            markers[1].material().diffuse().r());
}

//////////////////////////////////////////////////
/// The planes: just above the water, a depth under it, or a receiver's
/// depth, the water until the receiver is known.
TEST(AcousticsGui, Planes)
{
  Plane plane{Plane::kWater};
  EXPECT_TRUE(ParsePlane("receiver", plane));
  EXPECT_EQ(Plane::kReceiver, plane);
  EXPECT_TRUE(ParsePlane("3d", plane));
  EXPECT_EQ(Plane::k3D, plane);
  EXPECT_EQ("3d", PlaneName(Plane::k3D));
  EXPECT_FALSE(ParsePlane("sky", plane));
  EXPECT_EQ("depth", PlaneName(Plane::kDepth));

  EXPECT_NEAR(0.08, PlaneZ(Plane::kWater, 0.0, 5.0, -3.0), 1e-12);
  EXPECT_DOUBLE_EQ(-5.0, PlaneZ(Plane::kDepth, 0.0, 5.0, std::nullopt));
  EXPECT_DOUBLE_EQ(0.0, PlaneZ(Plane::kDepth, 0.0, -2.0, std::nullopt));
  EXPECT_DOUBLE_EQ(-3.0, PlaneZ(Plane::kReceiver, 0.0, 5.0, -3.0));
  EXPECT_NEAR(0.08, PlaneZ(Plane::kReceiver, 0.0, 5.0, std::nullopt), 1e-12);
}

//////////////////////////////////////////////////
/// The reach on a plane at the pinger's depth is the reach itself; on the
/// water it is less, and on a plane the reach does not get to, nothing.
TEST(AcousticsGui, ReachOnAPlaneUnderTheWater)
{
  const math::Vector3d at(0, 0, -10);
  const std::vector<PingerView> pingers{Pinger("p", at, 30000, 20, true)};

  const auto atDepth = ReachMarkers(pingers, -10.0, "ns");
  ASSERT_EQ(1u, atDepth.size());
  for (const auto r : Radii(atDepth[0], at))
    EXPECT_NEAR(20.0, r, 1e-9);
  EXPECT_DOUBLE_EQ(-10.0, atDepth[0].point(0).z());

  const auto onWater = ReachMarkers(pingers, 0.0, "ns");
  ASSERT_EQ(1u, onWater.size());
  for (const auto r : Radii(onWater[0], at))
    EXPECT_NEAR(std::sqrt(300.0), r, 1e-9);

  EXPECT_TRUE(ReachMarkers(pingers, -35.0, "ns").empty());
}

//////////////////////////////////////////////////
/// A wireframe sphere: every point on it, nothing above the cut, and the
/// cut itself closed by a ring.
TEST(AcousticsGui, SphereSegments)
{
  const math::Vector3d c(5, -3, -10);
  const auto whole = SphereSegments(c, 4.0, 0.0, 6, 3, 32);
  ASSERT_FALSE(whole.empty());
  EXPECT_EQ(0u, whole.size() % 2);
  for (const auto &p : whole)
    EXPECT_NEAR(4.0, p.Distance(c), 1e-9);

  // Cut 6 m up a 10 m sphere: the top is gone, the cut ring is 8 m across.
  const auto cut = SphereSegments(c, 10.0, -4.0, 6, 3, 32);
  int onCut{0};
  for (const auto &p : cut)
  {
    EXPECT_LE(p.Z(), -4.0 + 1e-9);
    if (std::abs(p.Z() + 4.0) < 1e-9)
    {
      ++onCut;
      EXPECT_NEAR(8.0, std::hypot(p.X() - c.X(), p.Y() - c.Y()), 1e-9);
    }
    else
    {
      EXPECT_NEAR(10.0, p.Distance(c), 1e-9);
    }
  }
  EXPECT_GE(onCut, 64);

  // Wholly above the cut: nothing.
  EXPECT_TRUE(SphereSegments(c, 4.0, -20.0, 6, 3).empty());
}

//////////////////////////////////////////////////
/// The reach and the pings as spheres, cut at the water: grey when off,
/// one shell per ping still spreading.
TEST(AcousticsGui, Spheres)
{
  const math::Vector3d at(0, 0, -3);
  const auto reach = ReachSpheres(
      {Pinger("on", at, 30000, 20, true), Pinger("off", at, 30000, 20, false),
       Pinger("unlimited", at, 30000, 0, true)}, 0.0, "ns");
  ASSERT_EQ(2u, reach.size());
  for (const auto &m : reach)
  {
    for (const auto &p : m.point())
      EXPECT_LE(p.z(), 1e-9);
  }
  EXPECT_NE(reach[0].material().diffuse().r(),
            reach[1].material().diffuse().r());

  // At 10 m/s, at 2.5 s: shells of 15 and 5 m; the 25 m one is past the
  // reach.
  const auto pings = PingSpheres({Pinger("on", at, 30000, 20, true)}, 0.0,
                                 2500ms, 10.0, 200.0, "ns");
  std::set<long> radii;
  for (const auto &m : pings)
  {
    for (const auto &p : m.point())
    {
      if (p.z() < -1e-6)
        radii.insert(std::lround(math::Vector3d(p.x(), p.y(), p.z())
                                 .Distance(at) * 100));
    }
  }
  EXPECT_EQ((std::set<long>{500, 1500}), radii);
}

//////////////////////////////////////////////////
/// A ring per ping where its sound meets the water, out to the reach; an
/// inactive pinger draws none.
TEST(AcousticsGui, PingMarkers)
{
  const math::Vector3d at(0, 0, -3);
  // Rings at 10 m/s from pings every second: at 2.5 s, the pings of 0, 1
  // and 2 s have spheres of 25, 15 and 5 m; the reach of 20 m drops the
  // first, and the water at 3 m cuts the others to 14.70 and 4 m.
  const auto markers = PingMarkers(
      {Pinger("on", at, 30000, 20, true), Pinger("off", at, 30000, 20, false)},
      0.0, 0.0, 2500ms, 10.0, 200.0, "ns");
  ASSERT_EQ(3u, markers.size());
  EXPECT_EQ((std::set<long>{400, 1470}), RingRadii(markers, at));
}

//////////////////////////////////////////////////
/// The rings go out to the reach, whatever the cap: the cap is for a pinger
/// with no reach, whose rings would never end.
TEST(AcousticsGui, RingsReachTheReachTheCapIsForNone)
{
  const math::Vector3d at(0, 0, -3);
  // At 30 m/s, a ping every 2 s: after 9.9 s the pings of 0, 2, 4, 6 and
  // 8 s have spheres of 297, 237, 177, 117 and 57 m.
  const auto reach = PingMarkers({Pinger("reach", at, 30000, 300, true)},
      0.0, 0.0, 9900ms, 30.0, 200.0, "ns");
  const auto far = RingRadii(reach, at);
  ASSERT_EQ(5u, far.size());
  EXPECT_NEAR(std::sqrt(297.0 * 297.0 - 9.0), *far.rbegin() / 100.0, 0.01);

  // With no reach the cap of 200 m ends them.
  const auto capped = PingMarkers({Pinger("none", at, 30000, 0, true)},
      0.0, 0.0, 9900ms, 30.0, 200.0, "ns");
  const auto near = RingRadii(capped, at);
  ASSERT_EQ(3u, near.size());
  EXPECT_LT(*near.rbegin(), 200 * 100);
}

//////////////////////////////////////////////////
/// Only pings that were sent ring: none from a pinger above the water, and
/// none from before a pinger was switched on.
TEST(AcousticsGui, RingsOnlyOfPingsSent)
{
  // The same pinger a metre above the water.
  const math::Vector3d dry(0, 0, 1);
  for (const auto &m : PingMarkers({Pinger("dry", dry, 30000, 20, true)},
                                   0.0, 0.0, 2500ms, 10.0, 200.0, "ns"))
  {
    EXPECT_EQ(0, m.point_size());
  }

  // Off at 1 s, on again at 2.2 s: at 2.5 s the schedule alone would draw
  // the pings of 1 and 2 s, neither of which was sent.
  const math::Vector3d at(0, 0, -3);
  PingerChanges changes;
  std::vector<PingerView> pingers{Pinger("p", at, 30000, 20, true)};
  changes.Track(pingers, 500ms, 0.0);
  EXPECT_FALSE(pingers[0].since.has_value()) << "as first seen";
  EXPECT_EQ((std::set<long>{400}),
            RingRadii(PingMarkers(pingers, 0.0, 0.0, 500ms, 10.0, 200.0,
                                  "ns"), at));
  pingers[0].data.active = false;
  changes.Track(pingers, 1000ms, 0.0);
  pingers[0].data.active = true;
  changes.Track(pingers, 2200ms, 0.0);
  ASSERT_TRUE(pingers[0].since.has_value());
  EXPECT_EQ(std::chrono::steady_clock::duration(2200ms), *pingers[0].since);
  changes.Track(pingers, 2500ms, 0.0);
  EXPECT_EQ(std::chrono::steady_clock::duration(2200ms), *pingers[0].since)
      << "unchanged since";
  EXPECT_TRUE(RingRadii(PingMarkers(pingers, 0.0, 0.0, 2500ms, 10.0, 200.0,
                                    "ns"), at).empty());
  // The ping of 3 s was sent.
  changes.Track(pingers, 3500ms, 0.0);
  EXPECT_EQ((std::set<long>{400}),
            RingRadii(PingMarkers(pingers, 0.0, 0.0, 3500ms, 10.0, 200.0,
                                  "ns"), at));

  // Lifted out and put back: it starts again from when it went under.
  pingers[0].position.Z(1.0);
  changes.Track(pingers, 4000ms, 0.0);
  pingers[0].position.Z(-3.0);
  changes.Track(pingers, 4200ms, 0.0);
  EXPECT_EQ(std::chrono::steady_clock::duration(4200ms), *pingers[0].since);

  // A reset, simulation time going back, forgets it.
  changes.Track(pingers, 100ms, 0.0);
  EXPECT_FALSE(pingers[0].since.has_value());
}

//////////////////////////////////////////////////
/// Slowed down a lot, many pings are on their way at once: no more rings
/// than the limit are drawn, the same pings from one frame to the next,
/// and still out towards the reach.
TEST(AcousticsGui, RingsAreBounded)
{
  const math::Vector3d at(0, 0, -1);
  // At 1 m/s a 300 m reach holds 300 pings a second apart.
  const std::vector<PingerView> pingers{Pinger("p", at, 30000, 300, true)};
  const auto a = RingRadii(PingMarkers(pingers, 0.0, 0.0, 1000s, 1.0, 200.0,
                                       "ns"), at);
  EXPECT_LE(a.size(), static_cast<std::size_t>(kMaxRings));
  EXPECT_GE(a.size(), static_cast<std::size_t>(kMaxRings - 1));
  EXPECT_GT(*a.rbegin(), 200 * 100) << "out past the cap, towards the reach";

  // Half a second on, the same pings, each half a metre further out.
  const auto b = RingRadii(PingMarkers(pingers, 0.0, 0.0, 1000s + 500ms, 1.0,
                                       200.0, "ns"), at);
  ASSERT_EQ(a.size(), b.size());
  auto ia = a.begin();
  for (auto ib = b.begin(); ib != b.end(); ++ia, ++ib)
    EXPECT_NEAR(50.0, static_cast<double>(*ib - *ia), 2.0);
}

//////////////////////////////////////////////////
/// A reading's ray keeps the pose the receiver had when the reading came:
/// the boat turning afterwards does not turn the ray off the pinger.
TEST(AcousticsGui, ARayKeepsThePoseItWasHeardAt)
{
  std::map<std::string, BearingRay> kept;
  HeardPing heard;
  heard.bearing = 0.3;
  heard.stamp = 1s;
  const math::Pose3d then(5, 0, -0.3, 0, 0, 0);
  KeepRays(kept, {{"/a", heard}}, {{"/a", then}});
  ASSERT_EQ(1u, kept.size());
  EXPECT_EQ(then, kept.at("/a").pose);

  // A second later the boat has yawed 0.4 rad and moved on; same reading.
  const math::Pose3d later(6, 1, -0.3, 0, 0, 0.4);
  KeepRays(kept, {{"/a", heard}}, {{"/a", later}});
  EXPECT_EQ(then, kept.at("/a").pose);
  const auto markers =
      RayMarkers(LiveRays(kept, 2s, 1500ms), 10.0, "ns");
  ASSERT_EQ(1u, markers.size());
  EXPECT_NEAR(5 + 10 * std::cos(0.3), markers[0].point(1).x(), 1e-9);
  EXPECT_NEAR(10 * std::sin(0.3), markers[0].point(1).y(), 1e-9);

  // The next reading takes the pose of its own moment.
  heard.stamp = 2s;
  KeepRays(kept, {{"/a", heard}}, {{"/a", later}});
  EXPECT_EQ(later, kept.at("/a").pose);

  // A ray lives for its lifetime, and a receiver that is gone has none.
  EXPECT_EQ(1u, LiveRays(kept, 3s, 1500ms).size());
  EXPECT_TRUE(LiveRays(kept, 4s, 1500ms).empty());
  EXPECT_TRUE(LiveRays(kept, 1s, 1500ms).empty()) << "before it came";
  KeepRays(kept, {}, {});
  EXPECT_TRUE(kept.empty());

  // A reading whose receiver has no pose yet waits for one.
  KeepRays(kept, {{"/b", heard}}, {});
  EXPECT_TRUE(kept.empty());
}

//////////////////////////////////////////////////
/// The pingers a panel shows: the acoustics system's list, each as the
/// GUI's state has it when it does. A pinger that changed arrives alone in
/// the state of a panel loaded late; the others stay.
TEST(AcousticsGui, PingersFromTheListAndTheState)
{
  msgs::Param list;
  auto add = [&list](const std::string &_name, double _x, bool _active)
  {
    auto *c = list.add_children();
    auto &n = (*c->mutable_params())["name"];
    n.set_type(msgs::Any::STRING);
    n.set_string_value(_name);
    for (const auto &[key, value] : std::map<std::string, double>{
        {"x", _x}, {"y", 2.0}, {"z", -1.5}, {"frequency", 37000.0},
        {"period", 2.0}, {"offset", 0.5}, {"reach", 300.0},
        {"pulse_length", 0.004}})
    {
      auto &a = (*c->mutable_params())[key];
      a.set_type(msgs::Any::DOUBLE);
      a.set_double_value(value);
    }
    auto &a = (*c->mutable_params())["active"];
    a.set_type(msgs::Any::BOOLEAN);
    a.set_bool_value(_active);
  };
  add("red", 40.0, false);
  add("green", 10.0, true);

  const auto listed = ParsePingers(list);
  ASSERT_EQ(2u, listed.size());
  EXPECT_EQ("green", listed[0].data.name);
  EXPECT_EQ(math::Vector3d(10, 2, -1.5), listed[0].position);
  EXPECT_DOUBLE_EQ(37000.0, listed[0].data.frequency);
  EXPECT_DOUBLE_EQ(2.0, listed[0].data.period);
  EXPECT_DOUBLE_EQ(0.5, listed[0].data.offset);
  EXPECT_DOUBLE_EQ(300.0, listed[0].data.reach);
  EXPECT_TRUE(listed[0].data.active);
  EXPECT_FALSE(listed[1].data.active);
  EXPECT_TRUE(ParsePingers(msgs::Param()).empty());

  // Nothing in the state: the list.
  EXPECT_EQ(2u, MergePingers(listed, {}).size());
  // Red alone in the state, switched on: both stay, red as the state has it.
  auto red = Pinger("red", math::Vector3d(41, 2, -1.5), 37000, 300, true);
  const auto merged = MergePingers(listed, {red});
  ASSERT_EQ(2u, merged.size());
  EXPECT_EQ("green", merged[0].data.name);
  EXPECT_TRUE(merged[1].data.active);
  EXPECT_DOUBLE_EQ(41.0, merged[1].position.X());
  // Before the list comes, the state alone.
  EXPECT_EQ(1u, MergePingers({}, {red}).size());
}

//////////////////////////////////////////////////
/// The listener hears the pingers the acoustics system lists.
TEST(AcousticsGui, ListenerHearsThePingers)
{
  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(
      "/world/listened/acoustics/pingers");
  AcousticListener listener;
  listener.Listen("listened");
  EXPECT_TRUE(listener.Pingers().empty());

  msgs::Param list;
  auto &n = (*list.add_children()->mutable_params())["name"];
  n.set_type(msgs::Any::STRING);
  n.set_string_value("only");
  for (int i = 0; i < 100 && listener.Pingers().empty(); ++i)
  {
    pub.Publish(list);
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_EQ(1u, listener.Pingers().size());
  EXPECT_EQ("only", listener.Pingers()[0].data.name);
}

//////////////////////////////////////////////////
/// Where pings start to drop: a dashed ring at that fraction of the reach,
/// one per start the receivers use, none for no dropout or a repeat.
TEST(AcousticsGui, DropoutMarkers)
{
  const math::Vector3d at(0, 0, -3);
  const auto markers = DropoutMarkers(
      {Pinger("on", at, 30000, 10, true), Pinger("unlimited", at, 30000, 0,
                                                 true)},
      {0.5, 1.0, 0.8, 0.5, 0.0}, 0.0, "ns");
  ASSERT_EQ(2u, markers.size());
  EXPECT_EQ(1001u, markers[0].id());
  std::set<long> radii;
  for (const auto &m : markers)
  {
    for (const auto r : Radii(m, at))
      radii.insert(std::lround(r * 100));
  }
  // 5 m and 8 m spheres around a pinger 3 m down meet the water at 4 m and
  // about 7.42 m.
  EXPECT_EQ((std::set<long>{400, 742}), radii);
}

//////////////////////////////////////////////////
/// A ray fades in three steps as it ages.
TEST(AcousticsGui, RaysFade)
{
  BearingRay fresh;
  fresh.ping.frequency = 30000;
  BearingRay old = fresh;
  old.fade = 0.9;
  const auto markers = RayMarkers({fresh, old}, 10.0, "ns");
  ASSERT_EQ(2u, markers.size());
  // 30 kHz is green; fading towards white raises its red.
  EXPECT_LT(markers[0].material().diffuse().r(),
            markers[1].material().diffuse().r());
}

//////////////////////////////////////////////////
/// A ray along the reading's bearing and elevation from the receiver's
/// pose, red when the reading was invalid.
TEST(AcousticsGui, RayMarkers)
{
  BearingRay ray;
  ray.pose = math::Pose3d(1, 2, -0.3, 0, 0, GZ_PI / 2);
  ray.ping.bearing = GZ_PI / 2;
  ray.ping.elevation = 0.0;
  ray.ping.frequency = 30000;
  BearingRay bad = ray;
  bad.ping.valid = false;

  const auto markers = RayMarkers({ray, bad}, 10.0, "ns");
  ASSERT_EQ(2u, markers.size());
  // Turned to +y, a bearing to port points to -x.
  const auto &p = markers[0].point(1);
  EXPECT_NEAR(1 - 10, p.x(), 1e-9);
  EXPECT_NEAR(2, p.y(), 1e-9);
  bool red{false};
  for (const auto &m : markers)
  {
    red = red || (m.material().diffuse().r() > 0.85 &&
                  m.material().diffuse().g() < 0.2);
  }
  EXPECT_TRUE(red);
}

//////////////////////////////////////////////////
/// A reading and the receivers' list, as the acoustics system sends them.
TEST(AcousticsGui, ParseMessages)
{
  msgs::Param reading;
  auto set = [&reading](const std::string &_k, double _v)
  {
    auto &a = (*reading.mutable_params())[_k];
    a.set_type(msgs::Any::DOUBLE);
    a.set_double_value(_v);
  };
  set("bearing", 0.5);
  set("elevation", -0.1);
  set("frequency", 30000);
  set("amplitude", -32);
  auto &valid = (*reading.mutable_params())["valid"];
  valid.set_type(msgs::Any::BOOLEAN);
  valid.set_bool_value(false);
  reading.mutable_header()->mutable_stamp()->set_sec(4);
  const auto h = ParseReading(reading);
  ASSERT_TRUE(h.has_value());
  EXPECT_DOUBLE_EQ(0.5, h->bearing);
  EXPECT_FALSE(h->valid);
  EXPECT_EQ(std::chrono::steady_clock::duration(4s), h->stamp);
  EXPECT_FALSE(ParseReading(msgs::Param()).has_value());

  msgs::Param list;
  for (const auto &[name, topic] : {std::pair{"w::b::l::s", "/b"},
                                    std::pair{"w::a::l::s", "/a"}})
  {
    auto *c = list.add_children();
    auto &n = (*c->mutable_params())["name"];
    n.set_type(msgs::Any::STRING);
    n.set_string_value(name);
    auto &t = (*c->mutable_params())["topic"];
    t.set_type(msgs::Any::STRING);
    t.set_string_value(topic);
    auto &f = (*c->mutable_params())["frequency"];
    f.set_type(msgs::Any::DOUBLE);
    f.set_double_value(30000);
  }
  const auto receivers = ParseReceivers(list);
  ASSERT_EQ(2u, receivers.size());
  EXPECT_EQ("/a", receivers[0].topic);
  EXPECT_DOUBLE_EQ(30000, receivers[1].frequency);
}

//////////////////////////////////////////////////
/// A layer replaces what it drew, deletes what it no longer draws, never
/// sends an empty marker, and takes everything off when it goes.
TEST(AcousticsGui, MarkerLayerAddsAndDeletes)
{
  FakeMarkerManager manager("/acoustics_test_marker");
  auto ring = [](std::uint64_t _id, bool _empty)
  {
    msgs::Marker m;
    m.set_id(_id);
    m.set_action(msgs::Marker::ADD_MODIFY);
    if (!_empty)
      msgs::Set(m.add_point(), math::Vector3d::Zero);
    return m;
  };
  auto ids = [](const std::vector<msgs::Marker> &_ms,
                msgs::Marker::Action _action)
  {
    std::set<std::uint64_t> out;
    for (const auto &m : _ms)
    {
      if (m.action() == _action)
        out.insert(m.id());
    }
    return out;
  };

  {
    MarkerLayer layer("acoustics/test", "/acoustics_test_marker");
    layer.Draw({ring(1, false), ring(2, false)});
    auto sent = manager.Take();
    EXPECT_EQ((std::set<std::uint64_t>{1, 2}),
              ids(sent, msgs::Marker::ADD_MODIFY));
    for (const auto &m : sent)
      EXPECT_EQ("acoustics/test", m.ns());

    layer.Draw({ring(1, false), ring(2, true)});
    sent = manager.Take();
    EXPECT_EQ((std::set<std::uint64_t>{1}),
              ids(sent, msgs::Marker::ADD_MODIFY));
    EXPECT_EQ((std::set<std::uint64_t>{2}),
              ids(sent, msgs::Marker::DELETE_MARKER));
    EXPECT_TRUE(layer.Drawn());

    // Asked for again unchanged, nothing is sent; changed, it is; and a
    // plain Draw sends whatever is there.
    EXPECT_FALSE(layer.DrawIfChanged({ring(1, false), ring(2, true)}));
    std::this_thread::sleep_for(100ms);
    EXPECT_TRUE(manager.Take().empty());
    EXPECT_TRUE(layer.DrawIfChanged({ring(1, false), ring(3, false)}));
    sent = manager.Take();
    EXPECT_EQ((std::set<std::uint64_t>{1, 3}),
              ids(sent, msgs::Marker::ADD_MODIFY));
    layer.Draw({ring(1, false), ring(3, false)});
    EXPECT_EQ(2u, manager.Take().size());
    layer.Draw({ring(1, false)});
    manager.Take();
  }
  // Gone: what it drew is deleted.
  const auto last = manager.Take();
  EXPECT_EQ((std::set<std::uint64_t>{1}),
            ids(last, msgs::Marker::DELETE_MARKER));
}
