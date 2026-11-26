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
// The world system and the sensor in a running server: scene B (a box 5 m
// ahead of an imaging sonar on a flat seabed, the sonar's own hull in front
// of it) rendered headless and stepped from here. One server serves every
// test, in the order written, because each one leaves the world as the next
// expects it and a rendering server is slow to start. It needs a GPU or
// llvmpipe; where rendering starts but no ping comes within 30 s, every
// test skips. Where ogre2 itself fails to start, gz-rendering can crash the
// process before then; that is outside what a test here can catch.

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "gz/maritime/sonar/sonar_image.pb.h"
#include "gz/maritime/sonar/sonar_labels.pb.h"

using namespace gz;
using namespace std::chrono_literals;

namespace
{
const std::string kWorld = "/world/scene_b";
const std::string kTopic = "rov/sonar_forward/image";
const std::string kSpawnTopic = "rov2/sonar_forward/image";
constexpr double kRate = 25.0;

/// \brief One ping as it arrived: the image and, matched by ping number,
/// its labels.
struct Ping
{
  std::uint64_t ping{0};
  std::int64_t stamp{0};
  gz::maritime::sonar::SonarImage image;
};

/// \brief The images and labels a sonar publishes, from a subscription
/// made before the server starts.
class Pings
{
  public: explicit Pings(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &Pings::OnImage, this);
    this->node.Subscribe(_topic + "/labels", &Pings::OnLabels, this);
  }

  /// \brief Forget what came so far.
  public: void Clear()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->images.clear();
    this->labels.clear();
  }

  /// \brief What came since the last Clear, in arrival order.
  public: std::vector<Ping> Images()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->images;
  }

  /// \brief The labels of a ping, if they came.
  public: bool LabelsOf(std::uint64_t _ping,
    gz::maritime::sonar::SonarLabels &_out)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    const auto it = this->labels.find(_ping);
    if (it == this->labels.end())
      return false;
    _out = it->second;
    return true;
  }

  private: void OnImage(const gz::maritime::sonar::SonarImage &_msg)
  {
    Ping p;
    p.ping = _msg.ping();
    p.stamp = static_cast<std::int64_t>(_msg.header().stamp().sec()) *
      1000000000 + _msg.header().stamp().nsec();
    p.image = _msg;
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->images.push_back(std::move(p));
  }

  private: void OnLabels(const gz::maritime::sonar::SonarLabels &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->labels[_msg.ping()] = _msg;
  }

  private: std::mutex mutex;
  private: std::vector<Ping> images;
  private: std::map<std::uint64_t, gz::maritime::sonar::SonarLabels> labels;

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief When ping _k is due on a _rate grid, in nanoseconds; the system's
/// own rule (DueTime in Sonar.cc), restated so the test checks it.
std::int64_t Due(std::uint64_t _k, double _rate)
{
  return std::llround(static_cast<double>(_k) * 1e9 / _rate);
}

/// \brief Ping numbers that rise by one each time, on the _rate grid.
/// \return An empty string, or what is wrong with the first bad ping.
std::string OnGrid(const std::vector<Ping> &_pings, double _rate)
{
  for (std::size_t i = 0; i < _pings.size(); ++i)
  {
    const auto &p = _pings[i];
    if (p.stamp != Due(p.ping, _rate))
    {
      return "ping " + std::to_string(p.ping) + " stamped " +
        std::to_string(p.stamp) + " ns, due " +
        std::to_string(Due(p.ping, _rate)) + " ns";
    }
    if (i > 0 && p.ping != _pings[i - 1].ping + 1)
    {
      return "ping " + std::to_string(p.ping) + " after " +
        std::to_string(_pings[i - 1].ping);
    }
  }
  return "";
}

/// \brief The sonar's model in SDF, as scene B has it, under another name.
std::string Rov(const std::string &_name)
{
  return
    "<sdf version='1.12' xmlns:gz='http://gazebosim.org/schema'>"
    "<model name='" + _name + "'><static>true</static><pose>0 0 2 0 0 0</pose>"
    "<link name='base_link'>"
    "<visual name='hull'><pose>0.3 0 0 0 0 0</pose>"
    "<geometry><box><size>0.05 0.6 0.6</size></box></geometry></visual>"
    "<sensor name='sonar_forward' type='custom' gz:type='imaging_sonar'>"
    "<pose>0 0 0 0 0.2618 0</pose><update_rate>25</update_rate>"
    "<topic>" + _name + "/sonar_forward/image</topic>"
    "<gz:sonar><range><min>0.2</min><max>10</max></range>"
    "<samples>512</samples><labels>true</labels></gz:sonar>"
    "</sensor></link></model></sdf>";
}

/// \brief Whether a label's scoped name, world prefix and all, is the
/// top-level model _model's.
bool Names(const std::string &_scoped, const std::string &_model)
{
  const std::string suffix = "::" + _model;
  return _scoped == _model || (_scoped.size() > suffix.size() &&
    _scoped.compare(_scoped.size() - suffix.size(), suffix.size(), suffix) == 0);
}

/// \brief The path a loaded shared library came from, or empty.
std::string Loaded(const std::string &_name)
{
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line))
  {
    const auto at = line.find('/');
    if (at != std::string::npos && line.find(_name) != std::string::npos)
      return line.substr(at);
  }
  return "";
}
}  // namespace

class SonarSystem : public ::testing::Test
{
  protected: static void SetUpTestSuite()
  {
    sim::ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "scene_b.sdf"));
    config.SetHeadlessRendering(true);
    config.SetSeed(7);
    pings = std::make_unique<Pings>(kTopic);
    spawned = std::make_unique<Pings>(kSpawnTopic);
    server = std::make_unique<sim::Server>(config);

    // Rendering starts on its own thread; step until the first ping, or
    // give up: this machine cannot render.
    const auto end = std::chrono::steady_clock::now() + 30s;
    while (std::chrono::steady_clock::now() < end)
    {
      server->Run(true, 100, false);
      if (!pings->Images().empty())
      {
        rendering = true;
        break;
      }
    }
  }

  protected: static void TearDownTestSuite()
  {
    server.reset();
    spawned.reset();
    pings.reset();
  }

  protected: void SetUp() override
  {
    if (!rendering)
      GTEST_SKIP() << "no ping: this machine cannot render";
  }

  /// \brief Step the server _n times (ms of sim time), then let the
  /// worker's queue drain.
  protected: static void Step(std::uint64_t _n)
  {
    ASSERT_TRUE(server->Run(true, _n, false));
    std::this_thread::sleep_for(300ms);
  }

  /// \brief Step _settle ms, forget what came, step _n ms more; what came
  /// in those.
  protected: static std::vector<Ping> Window(Pings &_p, std::uint64_t _n,
    std::uint64_t _settle = 100)
  {
    Step(_settle);
    _p.Clear();
    Step(_n);
    return _p.Images();
  }

  protected: template <typename Req>
  static bool Call(const std::string &_service, const Req &_req)
  {
    msgs::Boolean rep;
    bool result{false};
    return node->Request(_service, _req, 3000, rep, result) && result &&
      rep.data();
  }

  protected: static std::unique_ptr<sim::Server> server;
  protected: static std::unique_ptr<Pings> pings;
  protected: static std::unique_ptr<Pings> spawned;
  protected: static std::unique_ptr<transport::Node> node;
  protected: static bool rendering;
};

std::unique_ptr<sim::Server> SonarSystem::server;
std::unique_ptr<Pings> SonarSystem::pings;
std::unique_ptr<Pings> SonarSystem::spawned;
std::unique_ptr<transport::Node> SonarSystem::node =
  std::make_unique<transport::Node>();
bool SonarSystem::rendering = false;

//////////////////////////////////////////////////
// The system under test is this build's, in its build tree or where it
// installs, not another installed copy that the environment's plugin and
// library paths might reach first.
TEST_F(SonarSystem, LoadsTheBuiltSystem)
{
  auto ours = [](const std::string &_path) {
    return _path.rfind(BUILD_DIR, 0) == 0 || _path.rfind(INSTALL_DIR, 0) == 0;
  };
  const auto system = Loaded("libgz-maritime-sonar-system.so");
  const auto library = Loaded("libgz_sonar.so");
  EXPECT_TRUE(ours(system)) << system;
  EXPECT_TRUE(ours(library)) << library;
}

//////////////////////////////////////////////////
// Every ping on the 25 Hz grid, numbered without a repeat or a gap: the
// grid is counted in whole nanoseconds (DueTime), so 0.04 s, which a double
// cannot hold, never makes a ping due twice or skips one.
TEST_F(SonarSystem, PingsOnTheRateGrid)
{
  const auto got = Window(*pings, 2000);
  ASSERT_GE(got.size(), 49u);
  EXPECT_LE(got.size(), 51u);
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// The box: its label in the samples and beams where it stands, 4.5 to
// 5.5 m ahead and 0.5 m to either side, and an echo above the seabed's,
// since it is marked 12 dB brighter. The hull in front of the sonar, on
// the sonar's own model, is hidden from it: no sample carries its label,
// and nothing comes back from within a meter, where it would.
TEST_F(SonarSystem, SeesTheBoxNotItsHull)
{
  const auto got = Window(*pings, 400);
  ASSERT_FALSE(got.empty());
  const auto &ping = got.back();
  gz::maritime::sonar::SonarLabels labels;
  ASSERT_TRUE(pings->LabelsOf(ping.ping, labels));
  const auto &img = ping.image;
  ASSERT_EQ(gz::maritime::sonar::SonarImage::UINT8, img.data_type());
  ASSERT_EQ(img.beam_count(), labels.beam_count());
  ASSERT_EQ(img.sample_count(), labels.sample_count());
  const std::size_t beams = img.beam_count();
  const std::size_t samples = img.sample_count();
  ASSERT_EQ(beams * samples, img.data().size());
  ASSERT_EQ(beams * samples, labels.labels().size());

  // The label table is the world's, so the hull has a label too; it must
  // be in no sample.
  int box = -1;
  int seabed = -1;
  int hull = -1;
  for (int i = 0; i < labels.names_size(); ++i)
  {
    if (Names(labels.names(i), "rov"))
      hull = i;
    if (Names(labels.names(i), "box"))
      box = i;
    if (Names(labels.names(i), "seabed"))
      seabed = i;
  }
  ASSERT_GT(box, 0);
  ASSERT_GT(seabed, 0);

  const double maxAz = std::atan(0.5 / 4.5) + 2.0 * img.beam_width_horizontal();
  std::size_t boxCells = 0;
  std::size_t hullCells = 0;
  std::size_t stray = 0;
  std::size_t near = 0;
  double boxSum = 0;
  double seabedSum = 0;
  std::size_t seabedCells = 0;
  for (std::size_t b = 0; b < beams; ++b)
  {
    for (std::size_t s = 0; s < samples; ++s)
    {
      const auto i = b * samples + s;
      const int l = static_cast<unsigned char>(labels.labels()[i]);
      const double v = static_cast<unsigned char>(img.data()[i]);
      const double r = img.ranges(static_cast<int>(s));
      const double az = img.azimuths(static_cast<int>(b));
      if (l != 0 && r < 1.0)
        ++near;
      if (l == hull)
        ++hullCells;
      if (l == box)
      {
        ++boxCells;
        boxSum += v;
        // Slant ranges from the sonar, 2 m up, to the box's near bottom
        // edge and far top edge are 4.9 and 5.7 m; azimuths within
        // atan(0.5 / 4.5), widened by two beam widths, over which the beam
        // pattern spreads a hit at the box's edge.
        if (r < 4.5 || r > 6.0 || std::abs(az) > maxAz)
          ++stray;
      }
      else if (l == seabed)
      {
        ++seabedCells;
        seabedSum += v;
      }
    }
  }
  EXPECT_EQ(0u, hullCells);
  EXPECT_GE(boxCells, 20u);
  EXPECT_EQ(0u, stray) << "of " << boxCells << " box samples";
  EXPECT_EQ(0u, near) << "samples within 1 m carry a label";
  ASSERT_GT(seabedCells, 0u);
  EXPECT_GT(boxSum / boxCells, seabedSum / seabedCells);
}

//////////////////////////////////////////////////
// Off through <topic>/enable, no pings; on again, they resume on the grid,
// where they would have been had it never been off.
TEST_F(SonarSystem, EnableStopsAndResumes)
{
  msgs::Boolean off;
  off.set_data(false);
  ASSERT_TRUE(node->Request(kTopic + "/enable", off));
  EXPECT_TRUE(Window(*pings, 1000).empty());

  msgs::Boolean on;
  on.set_data(true);
  ASSERT_TRUE(node->Request(kTopic + "/enable", on));
  const auto got = Window(*pings, 1000);
  EXPECT_GE(got.size(), 24u);
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// <topic>/set_rate (gz-sensors' oneway service) moves the sonar to another
// grid in sim time; set back, it returns to the first.
TEST_F(SonarSystem, SetRateChangesTheRate)
{
  msgs::Double rate;
  rate.set_data(5.0);
  ASSERT_TRUE(node->Request(kTopic + "/set_rate", rate));
  auto got = Window(*pings, 2000);
  EXPECT_GE(got.size(), 9u);
  EXPECT_LE(got.size(), 11u);
  EXPECT_EQ("", OnGrid(got, 5.0));

  rate.set_data(kRate);
  ASSERT_TRUE(node->Request(kTopic + "/set_rate", rate));
  got = Window(*pings, 1000);
  EXPECT_GE(got.size(), 24u);
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// A reset of the whole world: the server goes on, sim time goes back to
// zero, and the pings start over on the grid from there, as at the start.
TEST_F(SonarSystem, ResetStartsOverOnTheGrid)
{
  Step(500);
  const auto before = pings->Images();
  ASSERT_FALSE(before.empty());
  const auto last = before.back().ping;

  msgs::WorldControl reset;
  reset.mutable_reset()->set_all(true);
  ASSERT_TRUE(Call(kWorld + "/control", reset));
  pings->Clear();
  Step(1000);
  const auto got = pings->Images();
  ASSERT_GE(got.size(), 20u);
  EXPECT_LT(got.front().ping, last);
  EXPECT_LT(got.front().stamp, Due(5, kRate));
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// The sonar's model removed: the server keeps stepping and the sonar stops.
// Removing a vehicle once crashed the server, on cameras destroyed in the
// middle of a frame. Then a sonar spawned after the last one is gone pings,
// which one once never did, and it too sees the box and not its own hull.
//
// This runs last: a reset now would bring the removed model back without
// its visual upstream (gz-sim does not recreate visuals of entities a reset
// restores), so the sonar on it would have nothing to render from. That is
// a gz-sim limitation, not checked here.
TEST_F(SonarSystem, RemoveThenSpawn)
{
  msgs::Entity rov;
  rov.set_name("rov");
  rov.set_type(msgs::Entity::MODEL);
  ASSERT_TRUE(Call(kWorld + "/remove", rov));
  const auto iterations = server->IterationCount();
  EXPECT_TRUE(Window(*pings, 1000, 200).empty());
  ASSERT_TRUE(iterations.has_value());
  EXPECT_GE(*server->IterationCount(), *iterations + 1200);

  msgs::EntityFactory factory;
  factory.set_sdf(Rov("rov2"));
  ASSERT_TRUE(Call(kWorld + "/create", factory));
  // Its cameras take a few frames to build.
  const auto got = Window(*spawned, 1000, 500);
  ASSERT_GE(got.size(), 20u);
  EXPECT_EQ("", OnGrid(got, kRate));

  gz::maritime::sonar::SonarLabels labels;
  ASSERT_TRUE(spawned->LabelsOf(got.back().ping, labels));
  int box = -1;
  int hull = -1;
  for (int i = 0; i < labels.names_size(); ++i)
  {
    if (Names(labels.names(i), "rov2"))
      hull = i;
    if (Names(labels.names(i), "box"))
      box = i;
  }
  ASSERT_GT(box, 0);
  std::size_t boxCells = 0;
  std::size_t hullCells = 0;
  for (const char c : labels.labels())
  {
    const int l = static_cast<unsigned char>(c);
    boxCells += l == box;
    hullCells += l == hull;
  }
  EXPECT_GT(boxCells, 0u);
  EXPECT_EQ(0u, hullCells);
}
