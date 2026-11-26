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

#include <gz/msgs/param.pb.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/water_optics/WaterOptics.hh"
#include "gz/sim/water_optics/WaterOpticsSampler.hh"

using namespace gz;
using namespace sim;
using water_optics::Rgb;

namespace
{
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

  // Bad messages, each whole: nothing changes.
  const auto good = run.optics;
  for (const auto &bad : {Set("jerlov", "4C"), SetNumber("visibility", -2),
                          Set("attenuation", "1 2"), Set("colour", "red")})
  {
    Send(node, "/world/optics/water_optics/set", bad);
    run.Steps(2);
    EXPECT_EQ(run.optics.generation, good.generation);
    EXPECT_EQ(run.optics.attenuation, good.attenuation);
  }

  // A message with one good key and one bad: nothing changes either.
  msgs::Param mixed = SetNumber("visibility", 2.0);
  (*mixed.mutable_params())["jerlov"].set_type(msgs::Any::STRING);
  (*mixed.mutable_params())["jerlov"].set_string_value("XX");
  Send(node, "/world/optics/water_optics/set", mixed);
  run.Steps(2);
  EXPECT_EQ(run.optics.generation, good.generation);
}

/////////////////////////////////////////////////
TEST(WaterOpticsSystem, ClearWithoutTheSystem)
{
  Sim run("optics_none.sdf");
  run.Steps(2);
  EXPECT_FALSE(run.valid);
  EXPECT_EQ(run.sampler.Transmittance(50.0), Rgb::One);
  EXPECT_EQ(run.sampler.Veil(5.0), Rgb::Zero);
}

/////////////////////////////////////////////////
TEST(WaterOpticsSystem, BadWorldFileGivesClearWater)
{
  Sim run("optics_bad.sdf");
  run.Steps(2);
  EXPECT_FALSE(run.valid);
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
    }
    ++checked;
  };
  run.Steps(300);
  EXPECT_GT(checked, 50);
  EXPECT_GT(maxElevation, 0.1) << "the wave never rose: the test saw flat "
                                  "water";
  EXPECT_NEAR(run.optics.Visibility(), 8.0, 1e-9);
}
