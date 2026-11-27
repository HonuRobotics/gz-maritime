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
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/water_optics/Jerlov.hh"
#include "gz/sim/water_optics/WaterOptics.hh"
#include "gz/sim/water_optics/WaterOpticsSampler.hh"

using namespace gz;
using namespace sim;
using water_optics::Rgb;

namespace
{
/// \brief A transport partition of this process's own, set before any
/// node is made, so copies of the test run at once do not hear each other.
const bool kOwnPartition = []
{
  const std::string partition =
      "gz_water_optics_test_" + std::to_string(getpid());
  return setenv("GZ_PARTITION", partition.c_str(), 1) == 0;
}();

std::string World(const std::string &_name)
{
  return common::joinPaths(TEST_WORLD_DIR, _name);
}

/// \brief A world, run step by step, with the optics read after each step.
class Sim
{
  public: explicit Sim(const std::string &_world)
    : fixture(World(_world))
  {
    this->fixture.OnPostUpdate(
        [this](const UpdateInfo &_info, const EntityComponentManager &_ecm)
        {
          const std::lock_guard<std::mutex> lock(this->mutex);
          this->time = _info.simTime;
          this->sampler.Sync(_ecm);
          this->valid = this->sampler.Valid();
          this->optics = this->sampler.Data();
          const auto *f = _ecm.Component<components::Wavefield>(
              worldEntity(_ecm));
          this->field.reset();
          if (f && f->Data().simulation)
            this->field = f->Data();
          if (this->probe)
            this->probe(_info, this->sampler);
        }).Finalize();
  }

  public: void Steps(int _n) { this->fixture.Server()->Run(true, _n, false); }

  public: TestFixture fixture;
  public: water_optics::WaterOpticsSampler sampler;
  public: std::mutex mutex;
  public: bool valid{false};
  public: water_optics::WaterOpticsData optics;
  public: std::chrono::steady_clock::duration time{0};
  public: std::optional<waves::WavefieldData> field;
  public: std::function<void(const UpdateInfo &,
                             const water_optics::WaterOpticsSampler &)> probe;
};

msgs::Param Set(const std::string &_key, const std::string &_value)
{
  msgs::Param msg;
  auto &v = (*msg.mutable_params())[_key];
  v.set_type(msgs::Any::STRING);
  v.set_string_value(_value);
  return msg;
}

msgs::Param SetNumber(const std::string &_key, double _value)
{
  msgs::Param msg;
  auto &v = (*msg.mutable_params())[_key];
  v.set_type(msgs::Any::DOUBLE);
  v.set_double_value(_value);
  return msg;
}

msgs::Param SetInt(const std::string &_key, int _value)
{
  msgs::Param msg;
  auto &v = (*msg.mutable_params())[_key];
  v.set_type(msgs::Any::INT32);
  v.set_int_value(_value);
  return msg;
}

/// \brief Publish until the subscriber is there, then let it arrive.
void Send(transport::Node &_node, const std::string &_topic,
          const msgs::Param &_msg)
{
  auto pub = _node.Advertise<msgs::Param>(_topic);
  for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(pub.HasConnections()) << "nobody listens on " << _topic;
  pub.Publish(_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/// \brief Step until the optics' generation passes one.
/// \return False if it did not within a few seconds.
bool StepPast(Sim &_run, std::uint64_t _generation)
{
  for (int i = 0; i < 200; ++i)
  {
    _run.Steps(1);
    {
      const std::lock_guard<std::mutex> lock(_run.mutex);
      if (_run.valid && _run.optics.generation > _generation)
        return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

/// \brief Send a message and then a marker, a new water level, on the same
/// topic, and step until the marker is applied: the message was handled
/// before it. Messages from one publisher arrive in order.
/// \return The generation before the marker, as the message left it.
std::uint64_t SendThenMark(Sim &_run, transport::Node &_node,
                           const std::string &_topic, const msgs::Param &_msg)
{
  auto pub = _node.Advertise<msgs::Param>(_topic);
  for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(pub.HasConnections()) << "nobody listens on " << _topic;
  std::uint64_t before{0};
  double level{0.0};
  {
    const std::lock_guard<std::mutex> lock(_run.mutex);
    before = _run.optics.generation;
    level = _run.optics.waterLevel;
  }
  // The message alone first, applied in a step of its own, so a change it
  // makes and the marker's are two writes
  pub.Publish(_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  _run.Steps(2);
  pub.Publish(SetNumber("water_level", level + 0.25));
  for (int i = 0; i < 200; ++i)
  {
    _run.Steps(1);
    {
      const std::lock_guard<std::mutex> lock(_run.mutex);
      if (_run.optics.waterLevel == level + 0.25)
        return _run.optics.generation - 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ADD_FAILURE() << "the marker never came";
  return before;
}

/// \brief Reset the world, stepping until the condition holds.
/// \return Whether the reset was accepted.
bool ResetUntil(Sim &_run, transport::Node &_node, const std::string &_world,
                const std::function<bool()> &_done)
{
  msgs::WorldControl control;
  control.mutable_reset()->set_all(true);
  msgs::Boolean reply;
  bool result{false};
  std::thread request([&]
  {
    _node.Request("/world/" + _world + "/control", control, 5000, reply,
                  result);
  });
  for (int i = 0; i < 100; ++i)
  {
    _run.Steps(1);
    {
      const std::lock_guard<std::mutex> lock(_run.mutex);
      if (_done())
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  request.join();
  return result;
}

/// \brief The ground truth messages a world publishes, as they come.
struct Truth
{
  explicit Truth(transport::Node &_node, const std::string &_topic)
  {
    std::function<void(const msgs::Param &)> cb =
        [this](const msgs::Param &_msg)
        {
          const std::lock_guard<std::mutex> lock(this->mutex);
          this->messages.push_back(_msg);
        };
    _node.Subscribe(_topic, cb);
  }

  std::size_t Count()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->messages.size();
  }

  std::mutex mutex;
  std::vector<msgs::Param> messages;
};
}  // namespace

/////////////////////////////////////////////////
// The world file's water is the recipe every system reads.
TEST(WaterOpticsSystem, WritesTheRecipe)
{
  Sim run("optics.sdf");
  run.Steps(2);
  ASSERT_TRUE(run.valid);
  water_optics::WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.waterLevel = 0.5;
  water_optics::WaterOpticsData expected;
  std::string error, warning;
  ASSERT_TRUE(water_optics::Resolve(spec, expected, error, warning));
  EXPECT_EQ(run.optics.jerlov, "3C");
  EXPECT_EQ(run.optics.attenuation, expected.attenuation);
  EXPECT_EQ(run.optics.veil, expected.veil);
  EXPECT_DOUBLE_EQ(run.optics.waterLevel, 0.5);
  EXPECT_GE(run.optics.generation, 1u);
}

/////////////////////////////////////////////////
// The set topic changes the water while the world runs, the ground truth
// follows, and a bad message changes nothing.
TEST(WaterOpticsSystem, SetTopicAndGroundTruth)
{
  Sim run("optics.sdf");
  run.Steps(2);
  ASSERT_TRUE(run.valid);
  const auto before = run.optics;

  transport::Node node;
  std::mutex mutex;
  std::optional<msgs::Param> info;
  std::function<void(const msgs::Param &)> onInfo =
      [&](const msgs::Param &_msg)
      {
        const std::lock_guard<std::mutex> lock(mutex);
        info = _msg;
      };
  ASSERT_TRUE(node.Subscribe("/world/optics/water_optics_info", onInfo));

  // Visibility, as a number.
  Send(node, "/world/optics/water_optics/set", SetNumber("visibility", 5.0));
  run.Steps(2);
  EXPECT_NEAR(run.optics.Visibility(), 5.0, 1e-9);
  EXPECT_EQ(run.optics.jerlov, "3C");
  EXPECT_GT(run.optics.generation, before.generation);
  // Clearer than 3C's own 3.9 m: closer to pure water, which scatters less
  EXPECT_LT(run.optics.veil.Y(), before.veil.Y());

  // The ground truth reports it.
  for (int i = 0; i < 50; ++i)
  {
    run.Steps(10);
    const std::lock_guard<std::mutex> lock(mutex);
    if (info && std::abs(info->params().at("visibility").double_value() -
                         5.0) < 1e-6)
    {
      break;
    }
  }
  {
    const std::lock_guard<std::mutex> lock(mutex);
    ASSERT_TRUE(info);
    EXPECT_EQ(info->params().at("jerlov").string_value(), "3C");
    EXPECT_NEAR(info->params().at("visibility").double_value(), 5.0, 1e-6);
    EXPECT_NEAR(info->params().at("water_level").double_value(), 0.5, 1e-12);
  }

  // A type and an explicit coefficient, as strings; then the coefficient
  // removed again with an empty string.
  msgs::Param both = Set("jerlov", "9C");
  (*both.mutable_params())["veil"].set_type(msgs::Any::STRING);
  (*both.mutable_params())["veil"].set_string_value("0.1 0.2 0.3");
  Send(node, "/world/optics/water_optics/set", both);
  run.Steps(2);
  EXPECT_EQ(run.optics.jerlov, "9C");
  EXPECT_EQ(run.optics.veil, Rgb(0.1, 0.2, 0.3));
  EXPECT_NEAR(run.optics.Visibility(), 5.0, 1e-9) << "the visibility stays";

  Send(node, "/world/optics/water_optics/set", Set("veil", ""));
  run.Steps(2);
  EXPECT_NE(run.optics.veil, Rgb(0.1, 0.2, 0.3));

  // The visibility removed: 9C's own
  Send(node, "/world/optics/water_optics/set", Set("visibility", ""));
  run.Steps(2);
  EXPECT_NEAR(run.optics.Visibility(),
              water_optics::kVisibilityConstant /
                  (water_optics::Jerlov("9C")->absorption.Y() +
                   water_optics::Jerlov("9C")->scattering.Y()), 1e-9);

  // Bad messages, each whole: nothing changes. A marker sent after each
  // shows it was handled.
  const std::string topic = "/world/optics/water_optics/set";
  msgs::Param mixed = SetNumber("visibility", 2.0);
  (*mixed.mutable_params())["jerlov"].set_type(msgs::Any::STRING);
  (*mixed.mutable_params())["jerlov"].set_string_value("XX");
  for (const auto &bad : {Set("jerlov", "4C"), SetNumber("visibility", -2),
                          Set("attenuation", "1 2"), Set("colour", "red"),
                          mixed})
  {
    const auto good = run.optics;
    EXPECT_EQ(SendThenMark(run, node, topic, bad), good.generation);
    EXPECT_EQ(run.optics.attenuation, good.attenuation);
  }
}

/////////////////////////////////////////////////
// Numbers come as doubles, integers or strings; a message that leaves the
// water as it is changes nothing, so nothing replicates.
TEST(WaterOpticsSystem, NumbersAndMessagesThatChangeNothing)
{
  Sim run("optics.sdf");
  run.Steps(2);
  ASSERT_TRUE(run.valid);
  transport::Node node;
  const std::string topic = "/world/optics/water_optics/set";

  Send(node, topic, SetInt("water_level", 2));
  ASSERT_TRUE(StepPast(run, run.optics.generation));
  EXPECT_DOUBLE_EQ(run.optics.waterLevel, 2.0);
  Send(node, topic, Set("visibility", "3"));
  ASSERT_TRUE(StepPast(run, run.optics.generation));
  EXPECT_NEAR(run.optics.Visibility(), 3.0, 1e-9);

  // The same values again, and an empty message: no new generation
  const auto generation = run.optics.generation;
  EXPECT_EQ(SendThenMark(run, node, topic, Set("visibility", "3")),
            generation);
  const auto afterMarker = run.optics.generation;
  EXPECT_EQ(SendThenMark(run, node, topic, msgs::Param()), afterMarker);
}

/////////////////////////////////////////////////
// Caustics and marine snow out of 0 to 1, from the world file and from the
// topic: clamped, the rest of the water kept, not thrown away.
TEST(WaterOpticsSystem, StrengthsOutOfRangeAreClamped)
{
  Sim run("optics_strengths.sdf");
  run.Steps(2);
  ASSERT_TRUE(run.valid) << "the whole water was thrown away";
  EXPECT_EQ(run.optics.jerlov, "3C");
  EXPECT_DOUBLE_EQ(run.optics.caustics, 1.0);
  EXPECT_DOUBLE_EQ(run.optics.marineSnow, 0.0);

  transport::Node node;
  const std::string topic = "/world/optics/water_optics/set";
  Send(node, topic, SetNumber("marine_snow", 3.5));
  ASSERT_TRUE(StepPast(run, run.optics.generation));
  EXPECT_DOUBLE_EQ(run.optics.marineSnow, 1.0);
  Send(node, topic, SetNumber("caustics", -0.5));
  ASSERT_TRUE(StepPast(run, run.optics.generation));
  EXPECT_DOUBLE_EQ(run.optics.caustics, 0.0);
  EXPECT_EQ(run.optics.jerlov, "3C");
}

/////////////////////////////////////////////////
// A reset brings back the world file's water.
TEST(WaterOpticsSystem, ResetBringsBackTheWorldFile)
{
  Sim run("optics.sdf");
  run.Steps(2);
  ASSERT_TRUE(run.valid);
  const auto initial = run.optics;
  transport::Node node;
  Send(node, "/world/optics/water_optics/set", Set("jerlov", "9C"));
  ASSERT_TRUE(StepPast(run, initial.generation));
  ASSERT_EQ(run.optics.jerlov, "9C");

  EXPECT_TRUE(ResetUntil(run, node, "optics",
                         [&] { return run.optics.jerlov == "3C"; }));
  EXPECT_EQ(run.optics.jerlov, "3C");
  EXPECT_EQ(run.optics.attenuation, initial.attenuation);
  EXPECT_DOUBLE_EQ(run.optics.waterLevel, 0.5);
}

/////////////////////////////////////////////////
// The ground truth keeps coming while the world is paused.
TEST(WaterOpticsSystem, GroundTruthWhilePaused)
{
  TestFixture fixture(World("optics.sdf"));
  fixture.Finalize();
  transport::Node node;
  Truth truth(node, "/world/optics/water_optics_info");
  ASSERT_TRUE(fixture.Server()->Run(false, 0, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  // 10 Hz of wall time: over a dozen in 1.5 s, paused
  EXPECT_GT(truth.Count(), 5u);
}

/////////////////////////////////////////////////
// With a publish rate of 0, the ground truth comes on a change only. The
// world's unknown element is warned about and does not stop the water.
TEST(WaterOpticsSystem, PublishRateZeroPublishesOnAChange)
{
  Sim run("optics_rate0.sdf");
  transport::Node node;
  Truth truth(node, "/world/optics_rate0/water_optics_info");
  run.Steps(2);
  ASSERT_TRUE(run.valid);
  EXPECT_EQ(run.optics.jerlov, "3C");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  run.Steps(50);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto atStart = truth.Count();
  EXPECT_LE(atStart, 1u);
  // Two and a half seconds: the default 1 Hz would publish twice
  for (int i = 0; i < 25; ++i)
  {
    run.Steps(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(truth.Count(), atStart) << "published without a change";

  Send(node, "/world/optics_rate0/water_optics/set", Set("jerlov", "7C"));
  ASSERT_TRUE(StepPast(run, run.optics.generation));
  run.Steps(2);
  for (int i = 0; i < 50 && truth.Count() == atStart; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(truth.Count(), atStart + 1);
}

/////////////////////////////////////////////////
TEST(WaterOpticsSystem, ClearWithoutTheSystem)
{
  Sim run("optics_none.sdf");
  run.Steps(2);
  EXPECT_FALSE(run.valid);
  EXPECT_EQ(run.sampler.Data().attenuation, Rgb::Zero);
  EXPECT_EQ(run.sampler.Veil(5.0), Rgb::Zero);
}

/////////////////////////////////////////////////
// A bad world file gives clear water, and the set topic is still there:
// a good message brings the water. A reset goes back to clear water.
TEST(WaterOpticsSystem, BadWorldFileGivesClearWaterUntilAGoodMessage)
{
  for (const std::string world : {"optics_bad", "optics_badvec"})
  {
    SCOPED_TRACE(world);
    Sim run(world + ".sdf");
    run.Steps(2);
    EXPECT_FALSE(run.valid);
    transport::Node node;
    Send(node, "/world/" + world + "/water_optics/set", Set("jerlov", "5C"));
    ASSERT_TRUE(StepPast(run, 0));
    EXPECT_EQ(run.optics.jerlov, "5C");

    // A reset: the world file's clear water again
    EXPECT_TRUE(ResetUntil(run, node, world, [&] { return !run.valid; }));
    EXPECT_FALSE(run.valid);
    EXPECT_EQ(run.sampler.Data().attenuation, Rgb::Zero);

    // The same water as before the reset brings it back
    Send(node, "/world/" + world + "/water_optics/set", Set("jerlov", "5C"));
    EXPECT_TRUE(StepPast(run, 0));
    EXPECT_EQ(run.optics.jerlov, "5C");
  }
}

/////////////////////////////////////////////////
// Explicit colours in the world file, over a type, and a visibility
// ignored when every coefficient is given.
TEST(WaterOpticsSystem, ExplicitColoursFromTheWorldFile)
{
  Sim run("optics_explicit.sdf");
  run.Steps(2);
  ASSERT_TRUE(run.valid);
  EXPECT_EQ(run.optics.jerlov, "");
  EXPECT_EQ(run.optics.attenuation, Rgb(1.0, 0.5, 0.25));
  EXPECT_EQ(run.optics.backscatter, Rgb(0.9, 0.4, 0.2));
  EXPECT_EQ(run.optics.veil, Rgb(0.1, 0.2, 0.3));
  EXPECT_EQ(run.optics.diffuseAttenuation, Rgb(0.2, 0.1, 0.05));

  Sim veil("optics_veil.sdf");
  veil.Steps(2);
  ASSERT_TRUE(veil.valid);
  EXPECT_EQ(veil.optics.jerlov, "3C");
  EXPECT_EQ(veil.optics.veil, Rgb(0.05, 0.3, 0.2));
}

/////////////////////////////////////////////////
// Under a wave, depth is measured below the local surface: at five points
// along the wave, at one instant, it is the wave field's elevation minus z.
TEST(WaterOpticsSystem, DepthBelowTheWaves)
{
  Sim run("optics_waves.sdf");
  std::atomic<int> checked{0};
  double maxElevation{0.0};
  run.probe = [&](const UpdateInfo &_info,
                  const water_optics::WaterOpticsSampler &_sampler)
  {
    if (!run.field || _info.simTime < std::chrono::seconds(2))
      return;
    const double t = std::chrono::duration<double>(_info.simTime).count();
    for (const double x : {0.0, 5.0, 10.0, 15.0, 20.0})
    {
      const double eta = waves::SurfaceElevation(*run.field, x, 0.0, t);
      maxElevation = std::max(maxElevation, std::abs(eta));
      EXPECT_NEAR(_sampler.Depth({x, 0.0, -2.0}, _info.simTime), eta + 2.0,
                  1e-3) << "x " << x;
      // Asked from another thread, as a renderer: an engine of its own,
      // the same wave
      double other{0.0};
      std::thread([&] { other = _sampler.Depth({x, 0.0, -2.0},
                                               _info.simTime); }).join();
      EXPECT_NEAR(other, eta + 2.0, 1e-3) << "x " << x << ", other thread";
    }
    ++checked;
  };
  run.Steps(300);
  EXPECT_GT(checked, 50);
  EXPECT_GT(maxElevation, 0.1) << "the wave never rose: the test saw flat "
                                  "water";
  EXPECT_NEAR(run.optics.Visibility(), 8.0, 1e-9);
}
