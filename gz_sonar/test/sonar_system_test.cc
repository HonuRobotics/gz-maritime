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
// llvmpipe. Every test skips only where the render engine cannot load; where
// it loads and no ping comes, every test fails. Where ogre2 itself fails to
// start, gz-rendering can crash the process before then; that is outside
// what a test here can catch.
//
// Nothing here waits a fixed time for the sonar's worker: a wait is for the
// ping stamped at a known sim time, or for a topic to appear or go, each
// with a generous limit that only a failure reaches. The test runs in a
// gz-transport partition of its own, made up per run.

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <spdlog/sinks/base_sink.h>

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

/// \brief The longest any wait may take. Only a failure reaches it, so it
/// is set for a slow build on a busy machine, not for this one.
constexpr auto kLimit = 120s;

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

  /// \brief Forget what came so far and the latest stamp too: after a world
  /// reset, when sim time starts over.
  public: void Restart()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->images.clear();
    this->labels.clear();
    this->latest = -1;
  }

  /// \brief Wait for a ping stamped _stamp or later, with its labels. A
  /// sonar's pings arrive in order, so every earlier one has come by then.
  /// \param[in] _stamp The stamp, ns of sim time.
  /// \return False if none came within kLimit.
  public: bool WaitFor(std::int64_t _stamp)
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    return this->cv.wait_for(lock, kLimit, [&]() {
      return this->latest >= _stamp && this->latestLabels >= this->latest;
    });
  }

  /// \brief The stamp of the latest ping, or -1.
  /// \return The stamp, ns of sim time.
  public: std::int64_t Latest()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->latest;
  }

  /// \brief What came since the last Clear, in arrival order.
  /// \return The pings.
  public: std::vector<Ping> Images()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->images;
  }

  /// \brief The labels of a ping, if they came.
  /// \param[in] _ping The ping number.
  /// \param[out] _out The labels.
  /// \return True if they came.
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
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      this->latest = p.stamp;
      this->images.push_back(std::move(p));
    }
    this->cv.notify_all();
  }

  private: void OnLabels(const gz::maritime::sonar::SonarLabels &_msg)
  {
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      this->latestLabels =
        static_cast<std::int64_t>(_msg.header().stamp().sec()) * 1000000000 +
        _msg.header().stamp().nsec();
      this->labels[_msg.ping()] = _msg;
    }
    this->cv.notify_all();
  }

  private: std::mutex mutex;
  private: std::condition_variable cv;
  private: std::int64_t latest{-1};
  private: std::int64_t latestLabels{-1};
  private: std::vector<Ping> images;
  private: std::map<std::uint64_t, gz::maritime::sonar::SonarLabels> labels;

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

//////////////////////////////////////////////////
/// \brief When ping _k is due on a _rate grid, in nanoseconds; the system's
/// own rule (DueTime in Sonar.cc), restated so the test checks it.
/// \param[in] _k The ping number.
/// \param[in] _rate Pings per second.
/// \return The due time, ns.
std::int64_t Due(std::uint64_t _k, double _rate)
{
  return std::llround(static_cast<double>(_k) * 1e9 / _rate);
}

//////////////////////////////////////////////////
/// \brief The last due time on a _rate grid at or before _t.
/// \param[in] _t The sim time, ns.
/// \param[in] _rate Pings per second.
/// \return The due time, ns.
std::int64_t LastDue(std::int64_t _t, double _rate)
{
  auto k = static_cast<std::uint64_t>(
    std::max(0.0, std::floor(static_cast<double>(_t) * 1e-9 * _rate)));
  while (Due(k + 1, _rate) <= _t)
    ++k;
  while (k > 0 && Due(k, _rate) > _t)
    --k;
  return Due(k, _rate);
}

/// \brief What the server logs, kept for the tests to read: some of what
/// the system does (a warning, a camera reused, a sonar left unwatched)
/// shows nowhere else from outside.
class Log : public spdlog::sinks::base_sink<std::mutex>
{
  /// \brief How many messages so far contain every one of _parts.
  /// \param[in] _parts The texts to look for.
  /// \return The count.
  public: std::size_t Count(const std::vector<std::string> &_parts)
  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    std::size_t n = 0;
    for (const auto &line : this->lines)
    {
      bool all = true;
      for (const auto &part : _parts)
        all = all && line.find(part) != std::string::npos;
      n += all;
    }
    return n;
  }

  // Documentation inherited
  protected: void sink_it_(const spdlog::details::log_msg &_msg) override
  {
    this->lines.emplace_back(_msg.payload.data(), _msg.payload.size());
  }

  // Documentation inherited
  protected: void flush_() override {}

  private: std::vector<std::string> lines;
};

//////////////////////////////////////////////////
/// \brief Threads in this process.
/// \return The count.
std::size_t Threads()
{
  std::size_t n = 0;
  for (common::DirIter it("/proc/self/task"); it != common::DirIter(); ++it)
    ++n;
  return n;
}

//////////////////////////////////////////////////
/// \brief Once per process, before the first node or server: a partition
/// of this run's own, and the log sink.
/// \return The sink.
std::shared_ptr<Log> Prepare()
{
  static std::shared_ptr<Log> log;
  if (log)
    return log;
  // A partition no other run has: two workspaces testing at once, or two
  // runs in one, must not read each other's pings.
  const std::string partition = "sonar_system_test_" +
    std::to_string(getpid()) + "_" + std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
  setenv("GZ_PARTITION", partition.c_str(), 1);

  // Debug messages too, into a sink the tests read.
  common::Console::SetVerbosity(4);
  log = std::make_shared<Log>();
  common::Console::Root().RawLogger().sinks().push_back(log);
  return log;
}

/// \brief What the system says, at debug level, when a sonar starts to
/// ping, stops for want of a subscriber, or is turned off.
const char *const kPinging = "]: pinging";
const char *const kUnwatched = "]: no subscriber, not pinging";
const char *const kOff = "]: off, not pinging";

/// \brief The text of the warning about a world without the Sensors system.
const char *const kNoSensorsWarning = "no frame rendered";

/// \brief Sim time as the server's last step saw it, read from inside the
/// server so the test never has to keep its own count across a reset.
class Clock : public sim::System, public sim::ISystemPostUpdate
{
  // Documentation inherited
  public: void PostUpdate(const sim::UpdateInfo &_info,
    const sim::EntityComponentManager &) override
  {
    this->now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count();
  }

  public: std::atomic<std::int64_t> now{0};
};

//////////////////////////////////////////////////
/// \brief Ping numbers that rise by one each time, on the _rate grid.
/// \return An empty string, or what is wrong with the first bad ping.
/// \param[in] _pings The pings, in arrival order.
/// \param[in] _rate Pings per second.
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

//////////////////////////////////////////////////
/// \brief The sonar's model in SDF, as scene B has it, under another name.
/// \param[in] _name The model's name.
/// \param[in] _sonar What to put in its <gz:sonar> in place of scene B's
/// range of 0.2 to 10 m.
/// \param[in] _pose Where, in place of scene B's 2 m above the origin.
/// \return The model's SDF.
std::string Rov(const std::string &_name, const std::string &_sonar =
  "<range><min>0.2</min><max>10</max></range>",
  const std::string &_pose = "0 0 2 0 0 0")
{
  return
    "<sdf version='1.12' xmlns:gz='http://gazebosim.org/schema'>"
    "<model name='" + _name + "'><static>true</static><pose>" + _pose +
    "</pose>"
    "<link name='base_link'>"
    "<visual name='hull'><pose>0.3 0 0 0 0 0</pose>"
    "<geometry><box><size>0.05 0.6 0.6</size></box></geometry></visual>"
    "<sensor name='sonar_forward' type='custom' gz:type='imaging_sonar'>"
    "<pose>0 0 0 0 0.2618 0</pose><update_rate>25</update_rate>"
    "<topic>" + _name + "/sonar_forward/image</topic>"
    "<gz:sonar>" + _sonar +
    "<samples>512</samples><labels>true</labels></gz:sonar>"
    "</sensor></link></model></sdf>";
}

//////////////////////////////////////////////////
/// \brief Whether a label's scoped name, world prefix and all, is the
/// top-level model _model's.
/// \param[in] _scoped The label's scoped name.
/// \param[in] _model The model's name.
/// \return True if the label names the model.
bool Names(const std::string &_scoped, const std::string &_model)
{
  const std::string suffix = "::" + _model;
  return _scoped == _model || (_scoped.size() > suffix.size() &&
    _scoped.compare(_scoped.size() - suffix.size(), suffix.size(), suffix) == 0);
}

//////////////////////////////////////////////////
/// \brief The path a loaded shared library came from, or empty.
/// \param[in] _name Part of the library's file name.
/// \return The path.
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
    log = Prepare();

    sim::ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "scene_b.sdf"));
    config.SetHeadlessRendering(true);
    config.SetSeed(7);
    node = std::make_unique<transport::Node>();
    pings = std::make_unique<Pings>(kTopic);
    spawned = std::make_unique<Pings>(kSpawnTopic);
    server = std::make_unique<sim::Server>(config);
    clock = std::make_shared<Clock>();
    server->AddSystem(clock);

    // Rendering starts on its own thread; step until the first ping.
    const auto end = std::chrono::steady_clock::now() + kLimit;
    while (std::chrono::steady_clock::now() < end)
    {
      server->Run(true, 100, false);
      if (pings->Latest() >= 0)
      {
        state = State::kPings;
        return;
      }
    }

    // No ping. That excuses the tests only on a machine that cannot render
    // at all, so find out: if the server loaded no engine, try here. An
    // engine that loads, in the server or here, makes the missing ping the
    // system's fault (it never asked for a frame, say), and every test
    // fails.
    bool renders = !rendering::loadedEngines().empty();
    if (!renders)
    {
      server.reset();
      try
      {
        renders = rendering::engine("ogre2",
          std::map<std::string, std::string>{{"headless", "1"}}) != nullptr;
      }
      catch (...)
      {
        renders = false;
      }
    }
    state = renders ? State::kNoPings : State::kNoEngine;
  }

  protected: static void TearDownTestSuite()
  {
    server.reset();
    clock.reset();
    spawned.reset();
    pings.reset();
    node.reset();
  }

  // Documentation inherited
  protected: void SetUp() override
  {
    if (state == State::kNoEngine)
      GTEST_SKIP() << "the render engine does not load on this machine";
    ASSERT_EQ(State::kPings, state)
      << "the render engine loads, but no ping came in "
      << kLimit.count() << " s";
  }

  /// \brief Step the server _n times (ms of sim time).
  /// \param[in] _n The number of steps.
  protected: static void Step(std::uint64_t _n)
  {
    ASSERT_TRUE(server->Run(true, _n, false));
  }

  /// \brief Sim time now.
  /// \return The time, ns.
  protected: static std::int64_t Now()
  {
    return clock->now;
  }

  /// \brief Wait until a pinging sonar's worker has caught up with the
  /// server: until the ping stamped at the last due time has arrived.
  /// \param[in,out] _p The subscriber.
  /// \param[in] _rate The sonar's rate.
  protected: static void Settle(Pings &_p, double _rate = kRate)
  {
    const auto due = LastDue(Now(), _rate);
    ASSERT_TRUE(_p.WaitFor(due))
      << "no ping stamped " << due << " ns; the latest is " << _p.Latest();
  }

  /// \brief Step _settle ms and catch up, forget what came, step _n ms more
  /// and catch up; what came in those: every ping stamped after the first
  /// catch-up, to the last due time.
  /// \param[in,out] _p The subscriber.
  /// \param[in] _n Steps to collect over.
  /// \param[in] _settle Steps to settle first.
  /// \param[in] _rate The sonar's rate.
  /// \return The pings.
  protected: static std::vector<Ping> Window(Pings &_p, std::uint64_t _n,
    std::uint64_t _settle = 100, double _rate = kRate)
  {
    Step(_settle);
    Settle(_p, _rate);
    _p.Clear();
    Step(_n);
    Settle(_p, _rate);
    return _p.Images();
  }

  /// \brief Whether anything advertises a topic.
  /// \param[in] _topic The topic, without the leading slash.
  /// \return True if it has a publisher.
  protected: static bool Advertised(const std::string &_topic)
  {
    std::vector<transport::MessagePublisher> publishers;
    std::vector<transport::MessagePublisher> subscribers;
    node->TopicInfo("/" + _topic, publishers, subscribers);
    return !publishers.empty();
  }

  /// \brief Step, _chunk ms at a time, until _done or the limit.
  /// \param[in] _done The condition.
  /// \param[in] _chunk Steps between checks.
  /// \return What _done returned last.
  protected: template <typename F>
  static bool StepUntil(F _done, std::uint64_t _chunk = 20)
  {
    const auto end = std::chrono::steady_clock::now() + kLimit;
    while (!_done() && std::chrono::steady_clock::now() < end)
    {
      if (!server->Run(true, _chunk, false))
        break;
      // Let the rendering thread and the workers have the processor.
      std::this_thread::yield();
    }
    return _done();
  }

  protected: template <typename Req>
  static bool Call(const std::string &_service, const Req &_req)
  {
    msgs::Boolean rep;
    bool result{false};
    return node->Request(_service, _req, 3000, rep, result) && result &&
      rep.data();
  }

  /// \brief Cameras in the scene: two for each camera pair the system has
  /// made, since it destroys none while the scene runs.
  /// \return The count.
  protected: static std::size_t Cameras()
  {
    const auto scene = rendering::sceneFromFirstRenderEngine();
    return scene ? scene->SensorCount() : 0;
  }

  /// \brief Spawn a vehicle and wait for its sonar's first ping.
  /// \param[in] _name The model's name.
  /// \param[in] _sonar Its <gz:sonar>; see Rov.
  /// \param[in] _pose Its pose; see Rov.
  /// \return The labels of that ping, empty if something failed.
  protected: static std::string SpawnAndPing(const std::string &_name,
    const std::string &_sonar, const std::string &_pose = "0 0 2 0 0 0")
  {
    Pings sub(_name + "/sonar_forward/image");
    msgs::EntityFactory factory;
    factory.set_sdf(Rov(_name, _sonar, _pose));
    if (!Call(kWorld + "/create", factory) ||
        !StepUntil([&]() { return sub.Latest() >= 0; }) ||
        !sub.WaitFor(sub.Latest()))
    {
      return "";
    }
    const auto got = sub.Images();
    gz::maritime::sonar::SonarLabels labels;
    if (got.empty() || !sub.LabelsOf(got.back().ping, labels))
      return "";
    return labels.labels();
  }

  /// \brief Remove a vehicle and wait until its sonar is gone, so its
  /// cameras are parked.
  /// \param[in] _name The model's name.
  /// \return False if something failed.
  protected: static bool Remove(const std::string &_name)
  {
    msgs::Entity model;
    model.set_name(_name);
    model.set_type(msgs::Entity::MODEL);
    return Call(kWorld + "/remove", model) && StepUntil([&]() {
      return !Advertised(_name + "/sonar_forward/image");
    });
  }

  /// \brief SpawnAndPing, then Remove.
  /// \param[in] _name The model's name.
  /// \param[in] _sonar Its <gz:sonar>; see Rov.
  /// \return The ping's labels, empty if something failed.
  protected: static std::string SpawnPingRemove(const std::string &_name,
    const std::string &_sonar)
  {
    const auto labels = SpawnAndPing(_name, _sonar);
    return Remove(_name) ? labels : "";
  }

  /// \brief Whether the first ping came, and if not, why.
  protected: enum class State {kPings, kNoEngine, kNoPings};

  protected: static std::unique_ptr<sim::Server> server;
  protected: static std::shared_ptr<Clock> clock;
  protected: static std::unique_ptr<Pings> pings;
  protected: static std::unique_ptr<Pings> spawned;
  protected: static std::unique_ptr<transport::Node> node;
  protected: static std::shared_ptr<Log> log;
  protected: static State state;
};

std::unique_ptr<sim::Server> SonarSystem::server;
std::shared_ptr<Clock> SonarSystem::clock;
std::unique_ptr<Pings> SonarSystem::pings;
std::unique_ptr<Pings> SonarSystem::spawned;
std::unique_ptr<transport::Node> SonarSystem::node;
std::shared_ptr<Log> SonarSystem::log;
SonarSystem::State SonarSystem::state = SonarSystem::State::kNoPings;

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
  // Exactly the 50 due in 2 s, since the window is cut at pings, not at
  // whatever had arrived when a sleep ran out.
  const auto got = Window(*pings, 2000);
  EXPECT_EQ(50u, got.size());
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
//
// The service is one way, and gz-transport runs a one-way request to a
// responder in the same process before Request returns, so the sonar is off
// (or at its new rate, below) from the next frame. The server has caught up
// when the request is made, so no earlier ping is still on its way; and a
// ping made while off would arrive before the first one after, so none is
// missed by looking too soon.
TEST_F(SonarSystem, EnableStopsAndResumes)
{
  Step(100);
  Settle(*pings);
  pings->Clear();
  const auto off = Now();
  msgs::Boolean request;
  request.set_data(false);
  ASSERT_TRUE(node->Request(kTopic + "/enable", request));
  Step(1000);
  // Turned on at a time that is not due: at a due time, the last frame of
  // the off period can still be on the rendering thread when the request
  // lands, and the first ping is then stamped exactly `on`.
  if (Now() == LastDue(Now(), kRate))
    Step(1);
  const auto on = Now();
  request.set_data(true);
  ASSERT_TRUE(node->Request(kTopic + "/enable", request));
  Step(1000);
  Settle(*pings);

  const auto got = pings->Images();
  ASSERT_EQ(25u, got.size());
  EXPECT_GT(got.front().stamp, on) << "a ping while off, " << off << " to "
                                   << on << " ns";
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// <topic>/set_rate (gz-sensors' oneway service) moves the sonar to another
// grid in sim time; set back, it returns to the first.
TEST_F(SonarSystem, SetRateChangesTheRate)
{
  Step(100);
  Settle(*pings);
  msgs::Double rate;
  rate.set_data(5.0);
  ASSERT_TRUE(node->Request(kTopic + "/set_rate", rate));
  auto got = Window(*pings, 2000, 100, 5.0);
  EXPECT_EQ(10u, got.size());
  EXPECT_EQ("", OnGrid(got, 5.0));

  rate.set_data(kRate);
  ASSERT_TRUE(node->Request(kTopic + "/set_rate", rate));
  got = Window(*pings, 1000);
  EXPECT_EQ(25u, got.size());
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// A rate too small to schedule is held at the floor, 0.001 Hz: the server
// keeps stepping, where 1e-11 Hz once overflowed the due times and hung it;
// and the sonar comes back on the grid when a rate is set again.
TEST_F(SonarSystem, ATinyRateDoesNotHangTheServer)
{
  Step(100);
  Settle(*pings);
  msgs::Double rate;
  for (const double tiny : {1e-11, 1e-300, 0.0005})
  {
    rate.set_data(tiny);
    ASSERT_TRUE(node->Request(kTopic + "/set_rate", rate));
    const auto before = Now();
    Step(200);
    EXPECT_EQ(before + 200'000'000, Now()) << "at " << tiny << " Hz";
  }
  rate.set_data(kRate);
  ASSERT_TRUE(node->Request(kTopic + "/set_rate", rate));
  const auto got = Window(*pings, 1000, 100);
  EXPECT_EQ(25u, got.size());
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// A reset of the whole world: the server goes on, sim time goes back to
// zero, and the pings start over on the grid from there, as at the start.
TEST_F(SonarSystem, ResetStartsOverOnTheGrid)
{
  Step(500);
  Settle(*pings);
  const auto before = pings->Images();
  ASSERT_FALSE(before.empty());
  const auto last = before.back().ping;

  // Nothing from before the reset is still on its way, so what comes now
  // is from after it.
  msgs::WorldControl reset;
  reset.mutable_reset()->set_all(true);
  ASSERT_TRUE(Call(kWorld + "/control", reset));
  pings->Restart();
  Step(1000);
  Settle(*pings);
  const auto got = pings->Images();
  ASSERT_GE(got.size(), 20u);
  EXPECT_LT(got.front().ping, last);
  EXPECT_LT(got.front().stamp, Due(5, kRate));
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// A sonar pings only while something subscribes to its image or its labels.
// What it does with nobody listening shows only in what it logs at each
// change, so that is what is read here; that it keeps to the rate grid
// through the gap shows in the pings after it.
TEST_F(SonarSystem, PingsOnlyWhileSubscribed)
{
  const std::string name = "rov::base_link::sonar_forward";
  Step(100);
  Settle(*pings);
  // Counted from here: the tests before turned it off and on.
  const auto quietBefore = log->Count({name, kUnwatched});
  const auto pingingBefore = log->Count({name, kPinging});
  auto quiet = [&]() { return log->Count({name, kUnwatched}) - quietBefore; };
  auto pinging = [&]() {
    return log->Count({name, kPinging}) - pingingBefore;
  };

  // The tests after this one read `pings`: put a subscriber back however
  // this test ends, so one failure here does not take the rest down.
  struct Restore
  {
    ~Restore()
    {
      if (!pings)
        pings = std::make_unique<Pings>(kTopic);
    }
  } restore;

  // The only subscriber goes.
  pings.reset();
  ASSERT_TRUE(StepUntil([&]() { return quiet() == 1; }));
  Step(1000);
  EXPECT_EQ(0u, pinging());

  // One to the labels alone is a subscriber too.
  {
    transport::Node labels;
    std::function<void(const gz::maritime::sonar::SonarLabels &)> ignore =
      [](const gz::maritime::sonar::SonarLabels &) {};
    ASSERT_TRUE(labels.Subscribe(kTopic + "/labels", ignore));
    ASSERT_TRUE(StepUntil([&]() { return pinging() == 1; }));
    Step(200);
    EXPECT_EQ(1u, quiet());
  }
  ASSERT_TRUE(StepUntil([&]() { return quiet() == 2; }));

  // A subscriber again: pings again, numbered as if they had never stopped.
  // Waited for in the log, not by a first ping: the last ping made for the
  // labels' subscriber may still be on its way and arrive here.
  pings = std::make_unique<Pings>(kTopic);
  ASSERT_TRUE(StepUntil([&]() { return pinging() == 2; }));
  const auto got = Window(*pings, 1000);
  EXPECT_EQ(25u, got.size());
  EXPECT_EQ("", OnGrid(got, kRate));
  EXPECT_EQ(2u, quiet());
}

//////////////////////////////////////////////////
// <always_ping> in <gz:sonar>: the sonar pings with no subscriber. Its
// enable service still turns
// it off. A value that is neither true nor false refuses the sonar.
TEST_F(SonarSystem, AlwaysPingNeedsNoSubscriber)
{
  const std::string name = "always::base_link::sonar_forward";
  const std::string topic = "always/sonar_forward/image";
  msgs::EntityFactory factory;
  factory.set_sdf(Rov("always", "<range><min>0.2</min><max>10</max></range>"
    "<always_ping>true</always_ping>"));
  ASSERT_TRUE(Call(kWorld + "/create", factory));
  ASSERT_TRUE(StepUntil([&]() {
    return log->Count({name, kPinging}) == 1;
  }));
  Step(1000);
  EXPECT_EQ(0u, log->Count({name, kUnwatched}));
  EXPECT_EQ(0u, log->Count({"unknown element <always_ping>"}));

  // A subscriber that comes later finds it on the grid, like any other.
  {
    Pings late(topic);
    ASSERT_TRUE(StepUntil([&]() { return late.Latest() >= 0; }));
    const auto got = Window(late, 400);
    EXPECT_EQ(10u, got.size());
    EXPECT_EQ("", OnGrid(got, kRate));
  }
  Step(400);
  EXPECT_EQ(0u, log->Count({name, kUnwatched}));

  msgs::Boolean request;
  request.set_data(false);
  ASSERT_TRUE(node->Request(topic + "/enable", request));
  ASSERT_TRUE(StepUntil([&]() {
    return log->Count({name, kOff}) == 1;
  }));

  msgs::Entity model;
  model.set_name("always");
  model.set_type(msgs::Entity::MODEL);
  ASSERT_TRUE(Call(kWorld + "/remove", model));
  EXPECT_TRUE(StepUntil([&]() { return !Advertised(topic); }));

  // false is the default said aloud; anything else is refused.
  factory.set_sdf(Rov("never", "<range><min>0.2</min><max>10</max></range>"
    "<always_ping>false</always_ping>"));
  ASSERT_TRUE(Call(kWorld + "/create", factory));
  ASSERT_TRUE(StepUntil([&]() {
    return log->Count({"never::base_link::sonar_forward", kUnwatched}) == 1;
  }));
  model.set_name("never");
  ASSERT_TRUE(Call(kWorld + "/remove", model));
  EXPECT_TRUE(StepUntil([&]() {
    return !Advertised("never/sonar_forward/image");
  }));

  factory.set_sdf(Rov("maybe", "<range><min>0.2</min><max>10</max></range>"
    "<always_ping>maybe</always_ping>"));
  ASSERT_TRUE(Call(kWorld + "/create", factory));
  ASSERT_TRUE(StepUntil([&]() {
    return log->Count({"<always_ping> must be true or false"}) == 1;
  }));
  EXPECT_FALSE(Advertised("maybe/sonar_forward/image"));
  model.set_name("maybe");
  ASSERT_TRUE(Call(kWorld + "/remove", model));
  Step(100);
  Settle(*pings);
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
  Step(100);
  Settle(*pings);
  ASSERT_TRUE(Advertised(kTopic));
  ASSERT_TRUE(Call(kWorld + "/remove", rov));
  const auto iterations = server->IterationCount();
  // The sonar goes with its topic; once that is gone its worker has been
  // joined, so nothing more can come from it.
  EXPECT_TRUE(StepUntil([]() { return !Advertised(kTopic); }));
  pings->Clear();
  // More than the 5 s after which a world with a sonar and no frame is
  // warned about its missing Sensors system.
  Step(5500);
  EXPECT_TRUE(pings->Images().empty());
  ASSERT_TRUE(iterations.has_value());
  EXPECT_GE(*server->IterationCount(), *iterations + 5500);

  msgs::EntityFactory factory;
  factory.set_sdf(Rov("rov2"));
  ASSERT_TRUE(Call(kWorld + "/create", factory));
  // Its cameras take a few frames to build.
  ASSERT_TRUE(StepUntil([]() { return spawned->Latest() >= 0; }));
  const auto got = Window(*spawned, 1000);
  EXPECT_EQ(25u, got.size());
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

  // The Sensors system is here and rendering. The warning once counted its
  // 5 s from the first sonar the world ever had, so a later one, spawned
  // after the render events had been left, was warned about at once.
  EXPECT_EQ(0u, log->Count({kNoSensorsWarning}));
}

//////////////////////////////////////////////////
// A removed sonar's cameras cannot be destroyed while the scene runs, so
// they are kept for the next sonar. They are matched on image size and far
// class (the far plane, a power of two meters), which a camera keeps for
// life, and take the new sonar's field of view and far clip at each use.
// So vehicles that come and go with ranges a little apart share cameras,
// where once each took a new set and the GPU memory with it.
TEST_F(SonarSystem, ParkedCamerasServeOtherRanges)
{
  Step(100);
  Settle(*spawned);
  const auto reusedBefore = log->Count({"camera pair(s)", ", 2 reused"});
  auto reused = [&]() {
    return log->Count({"camera pair(s)", ", 2 reused"}) - reusedBefore;
  };
  auto range = [](double _max) {
    return "<range><min>0.2</min><max>" + std::to_string(_max) +
      "</max></range>";
  };

  // rov2 has the two pairs the world started with, and the vehicle of an
  // earlier test left two more idle. Twenty vehicles in turn, their ranges
  // 2 cm apart, make do with those: far clips of 13.5 to 14.07 m, all in
  // the 16 m class.
  const auto cameras = Cameras();
  for (int i = 0; i < 20; ++i)
  {
    ASSERT_FALSE(SpawnPingRemove("near" + std::to_string(i),
      range(9.0 + 0.02 * i)).empty()) << i;
  }
  EXPECT_EQ(cameras, Cameras());
  EXPECT_EQ(20u, reused());

  // The ends of the class, 8 m and 16 m of far clip, still share them...
  ASSERT_FALSE(SpawnPingRemove("low", range(5.34)).empty());
  ASSERT_FALSE(SpawnPingRemove("high", range(10.66)).empty());
  EXPECT_EQ(cameras, Cameras());
  // ...and the next class takes a set of its own, once.
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_FALSE(SpawnPingRemove("far" + std::to_string(i),
      range(10.7 + 5.0 * i)).empty()) << i;
  }
  EXPECT_EQ(cameras + 4, Cameras());
  EXPECT_EQ(24u, reused());
}

//////////////////////////////////////////////////
// A reused camera gives what a new one would. Two sonars with images of
// one size and fields of view well apart share cameras: scene B's, two
// cameras 63.4 degrees wide, and a narrower fan whose one camera has the
// same 604 by 675 pixels at 44.2 degrees. In a far class of their own, so
// that every camera is new: the first holds its cameras (from behind the
// other, out of its view) while the second gets new ones and its labels,
// which are free of noise, are taken. Both removed, the first before the
// second, the second spawned again gets a camera the first had, and must
// see what it saw on its own. Then the other way around in another class.
TEST_F(SonarSystem, AReusedCameraSeesAsANewOne)
{
  const std::string narrow = "<horizontal_fov>0.661</horizontal_fov>"
    "<vertical_fov>0.261</vertical_fov><beams>123</beams>";
  const auto sizesBefore = log->Count({"camera pair(s) of 604x675"});
  for (const double max : {40.0, 80.0})
  {
    const std::string range = "<range><min>0.2</min><max>" +
      std::to_string(max) + "</max></range>";
    const std::string holder = max < 50.0 ? range + narrow : range;
    const std::string sonar = max < 50.0 ? range : range + narrow;
    const std::string tag = std::to_string(static_cast<int>(max));
    const auto reusedBefore = log->Count({"c" + tag, " 0 reused"});

    ASSERT_FALSE(SpawnAndPing("a" + tag, holder, "-20 0 2 0 0 0").empty());
    const auto fresh = SpawnAndPing("b" + tag, sonar);
    ASSERT_FALSE(fresh.empty());
    const auto made = Cameras();
    ASSERT_TRUE(Remove("a" + tag));
    ASSERT_TRUE(Remove("b" + tag));
    const auto again = SpawnPingRemove("c" + tag, sonar);
    EXPECT_EQ(made, Cameras());
    EXPECT_EQ(reusedBefore, log->Count({"c" + tag, " 0 reused"}));
    EXPECT_TRUE(fresh == again) << "labels differ on a reused camera";
  }
  // All six had images of the one size, or the above showed nothing.
  EXPECT_EQ(sizesBefore + 6, log->Count({"camera pair(s) of 604x675"}));
}

//////////////////////////////////////////////////
// What bounds the pool: the most sonars of one size and far class there
// have been at once. 33 small sonars together make 33 pairs; removed, the
// pairs are idle, which the system warns about once, past 32, since their
// memory stays taken; 33 more then make none.
TEST_F(SonarSystem, ThePoolGrowsToTheMostAtOnce)
{
  const std::string small = "<range><min>0.2</min><max>10</max></range>"
    "<horizontal_fov>0.5</horizontal_fov><beams>16</beams>"
    "<rays_per_beam>2</rays_per_beam><elevation_rays>8</elevation_rays>";
  const int count = 33;
  const char *const kIdle = "camera pairs of removed sonars are idle";
  const auto warned = log->Count({kIdle});
  std::size_t cameras = Cameras();
  for (const std::string round : {"first", "second"})
  {
    const auto built = log->Count({round, "camera pair(s)"});
    for (int i = 0; i < count; ++i)
    {
      msgs::EntityFactory factory;
      factory.set_sdf(Rov(round + std::to_string(i), small));
      ASSERT_TRUE(Call(kWorld + "/create", factory));
    }
    ASSERT_TRUE(StepUntil([&]() {
      return log->Count({round, "camera pair(s)"}) == built + count;
    }));
    if (round == "first")
    {
      EXPECT_EQ(cameras + 2 * count, Cameras());
      cameras = Cameras();
      EXPECT_EQ(warned, log->Count({kIdle}));
    }
    else
    {
      EXPECT_EQ(cameras, Cameras());
    }
    for (int i = 0; i < count; ++i)
    {
      msgs::Entity model;
      model.set_name(round + std::to_string(i));
      model.set_type(msgs::Entity::MODEL);
      ASSERT_TRUE(Call(kWorld + "/remove", model));
    }
    ASSERT_TRUE(StepUntil([&]() {
      return !Advertised(round + std::to_string(count - 1) +
        "/sonar_forward/image") && !Advertised(round +
        "0/sonar_forward/image");
    }));
    EXPECT_EQ(warned + 1, log->Count({kIdle}));
  }
  Step(100);
  Settle(*spawned);
}

//////////////////////////////////////////////////
// Vehicles spawned and removed a step or a few apart, before the scene has
// their link visuals, so before their sonars' cameras are built: each sonar
// goes all the same, with its topics and its threads. A removal once
// dropped only the cameras, and a sonar still waiting for them stayed for
// good.
TEST_F(SonarSystem, QuickSpawnAndRemoveLeavesNothing)
{
  Step(100);
  Settle(*spawned);
  const auto threads = Threads();

  const int count = 30;
  auto topic = [](int _i) {
    return "quick" + std::to_string(_i) + "/sonar_forward/image";
  };
  for (int i = 0; i < count; ++i)
  {
    const std::string name = "quick" + std::to_string(i);
    msgs::EntityFactory factory;
    factory.set_sdf(Rov(name));
    ASSERT_TRUE(Call(kWorld + "/create", factory));
    // One to three steps before the removal and after it, in every pairing.
    Step(1 + i % 3);
    msgs::Entity model;
    model.set_name(name);
    model.set_type(msgs::Entity::MODEL);
    ASSERT_TRUE(Call(kWorld + "/remove", model));
    Step(1 + (i / 3) % 3);
  }

  auto left = [&]() {
    int n = 0;
    for (int i = 0; i < count; ++i)
      n += Advertised(topic(i));
    return n;
  };
  EXPECT_TRUE(StepUntil([&]() { return left() == 0 && Threads() <= threads; }))
    << left() << " sonar topics still advertised, " << Threads()
    << " threads against " << threads << " before";

  // The one that was there throughout still pings.
  const auto got = Window(*spawned, 400);
  EXPECT_EQ(10u, got.size());
  EXPECT_EQ("", OnGrid(got, kRate));
}

//////////////////////////////////////////////////
// A reset brings the removed rov back, and gz-sim gives its link no
// rendering visual again (see RemoveThenSpawn), so its sonar waits for its
// cameras for as long as it exists. Removed again, it goes like any other;
// and with every sonar gone and none waiting, the system leaves the render
// events, which it once could not while a request waited.
//
// This runs last: it leaves the world without a sonar.
TEST_F(SonarSystem, ARestoredSonarCanBeRemoved)
{
  Step(100);
  Settle(*spawned);
  ASSERT_FALSE(Advertised(kTopic));
  msgs::WorldControl reset;
  reset.mutable_reset()->set_all(true);
  ASSERT_TRUE(Call(kWorld + "/control", reset));
  ASSERT_TRUE(StepUntil([]() { return Advertised(kTopic); }));
  Step(200);

  const auto left = log->Count({"left the render events"});
  for (const std::string name : {"rov", "rov2"})
  {
    const std::string topic = name + "/sonar_forward/image";
    if (!Advertised(topic))
      continue;
    msgs::Entity model;
    model.set_name(name);
    model.set_type(msgs::Entity::MODEL);
    ASSERT_TRUE(Call(kWorld + "/remove", model));
    EXPECT_TRUE(StepUntil([&]() { return !Advertised(topic); })) << topic;
  }
  EXPECT_TRUE(StepUntil([&]() {
    return log->Count({"left the render events"}) > left;
  }));
  EXPECT_EQ(0u, log->Count({kNoSensorsWarning}));
}

//////////////////////////////////////////////////
// Scene B without the Sensors system: no frame is ever rendered, and the
// system says so, once, 5 s of sim time after the sonar appeared; not
// before, and not again for a later sonar or after a reset. A server of its
// own, after the other has gone; it renders nothing, so it runs anywhere.
TEST(SonarSystemAlone, WarnsOnceWithoutTheSensorsSystem)
{
  const auto log = Prepare();
  std::ifstream file(common::joinPaths(TEST_WORLD_DIR, "scene_b.sdf"));
  std::string world((std::istreambuf_iterator<char>(file)),
    std::istreambuf_iterator<char>());
  const auto from = world.find("<plugin filename=\"gz-sim-sensors-system\"");
  ASSERT_NE(std::string::npos, from);
  const std::string close = "</plugin>";
  const auto to = world.find(close, from);
  ASSERT_NE(std::string::npos, to);
  world.erase(from, to + close.size() - from);

  sim::ServerConfig config;
  config.SetSdfString(world);
  sim::Server server(config);
  transport::Node node;
  auto call = [&](const std::string &_service, const auto &_req) {
    msgs::Boolean rep;
    bool result{false};
    return node.Request(kWorld + _service, _req, 3000, rep, result) &&
      result && rep.data();
  };
  const auto before = log->Count({kNoSensorsWarning});
  auto warnings = [&]() { return log->Count({kNoSensorsWarning}) - before; };

  ASSERT_TRUE(server.Run(true, 4900, false));
  EXPECT_EQ(0u, warnings());
  ASSERT_TRUE(server.Run(true, 200, false));
  EXPECT_EQ(1u, warnings());

  // The sonar removed and, 6 s later, another spawned, then 6 s more.
  msgs::Entity rov;
  rov.set_name("rov");
  rov.set_type(msgs::Entity::MODEL);
  ASSERT_TRUE(call("/remove", rov));
  ASSERT_TRUE(server.Run(true, 6000, false));
  msgs::EntityFactory factory;
  factory.set_sdf(Rov("rov2"));
  ASSERT_TRUE(call("/create", factory));
  ASSERT_TRUE(server.Run(true, 6000, false));
  // And a reset, which takes sim time back, then 6 s more.
  msgs::WorldControl reset;
  reset.mutable_reset()->set_all(true);
  ASSERT_TRUE(call("/control", reset));
  ASSERT_TRUE(server.Run(true, 6000, false));
  EXPECT_EQ(1u, warnings());
}
