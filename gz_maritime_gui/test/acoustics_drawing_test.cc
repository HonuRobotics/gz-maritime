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
// from an ECM, the drawings as markers, the messages parsed, and a marker
// layer sending to a fake MarkerManager in this process.

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/param.pb.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <set>
#include <string>
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
      0.0, 2500ms, 10.0, 200.0, "ns");
  ASSERT_EQ(3u, markers.size());
  std::set<long> radii;
  for (const auto &m : markers)
  {
    for (const auto r : Radii(m, at))
      radii.insert(std::lround(r * 100));
  }
  EXPECT_EQ((std::set<long>{400, 1470}), radii);
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
  }
  // Gone: what it drew is deleted.
  const auto last = manager.Take();
  EXPECT_EQ((std::set<std::uint64_t>{1}),
            ids(last, msgs::Marker::DELETE_MARKER));
}
