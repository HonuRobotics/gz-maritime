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
// The water moves with the waves in a world whose physics follows them
// (one that runs gz_waves_buoyancy): a damped body just under the surface is
// carried back and forth by the waves' orbital velocity, and one deep down
// hardly at all. In a world whose waves are drawn only, nothing changes.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>

#include <gz/common/Filesystem.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief How far a model swung along x after the wave ramped in, m.
struct Swing
{
  double min{1e9};
  double max{-1e9};
  double Span() const { return this->max - this->min; }
};

/// \brief Run a world and measure each sphere's swing along the wave.
/// \param[in] _file World file name.
/// \return Swing by model name.
std::map<std::string, Swing> Swings(const std::string &_file)
{
  std::map<std::string, Swing> out;
  TestFixture fixture(common::joinPaths(TEST_WORLD_DIR, _file));
  fixture.OnPostUpdate(
      [&out](const UpdateInfo &_info, const EntityComponentManager &_ecm)
      {
        if (std::chrono::duration<double>(_info.simTime).count() < 8.0)
          return;
        for (const std::string name : {"shallow", "deep"})
        {
          const Entity model = _ecm.EntityByComponents(
              components::Model(), components::Name(name));
          if (kNullEntity == model)
            continue;
          const double x = worldPose(model, _ecm).Pos().X();
          out[name].min = std::min(out[name].min, x);
          out[name].max = std::max(out[name].max, x);
        }
      });
  fixture.Finalize();
  // 16 s at 2 ms: two periods after the ramp, then two more measured.
  fixture.Server()->Run(true, 8000, false);
  return out;
}
}

//////////////////////////////////////////////////
TEST(HydrodynamicsWaves, CarriedNearTheSurfaceHardlyDeepDown)
{
  const auto swing = Swings("waves_coupled.sdf");
  ASSERT_EQ(2u, swing.size());
  const double shallow = swing.at("shallow").Span();
  const double deep = swing.at("deep").Span();
  RecordProperty("shallow_swing_m", std::to_string(shallow));
  RecordProperty("deep_swing_m", std::to_string(deep));

  // A 0.3 m wave of 4 s moves the water 0.3 m each way at the surface; its
  // orbit fades as e^(-k d), k = 0.25 / m: to 0.86 at 0.6 m, 0.28 at 5 m.
  EXPECT_GT(shallow, 0.25);
  EXPECT_LT(deep, 0.5 * shallow);
}

//////////////////////////////////////////////////
TEST(HydrodynamicsWaves, DrawnWavesMoveNothing)
{
  const auto swing = Swings("waves_drawn.sdf");
  ASSERT_EQ(2u, swing.size());
  EXPECT_LT(swing.at("shallow").Span(), 0.01);
  EXPECT_LT(swing.at("deep").Span(), 0.01);
}
