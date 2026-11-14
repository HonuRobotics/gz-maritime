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

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/WaveCoupling.hh"
#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/Eval.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief The step of the test worlds, s.
constexpr double kStep{0.002};

/// \brief Where the box's centre rests on still water: half its 0.4 m
/// height above a draft of 200 / 1025 m.
const double kRest{0.2 - 200.0 / 1025.0};

/// \brief One sample of a run.
struct Sample
{
  /// \brief Simulation time, s.
  double t{0.0};

  /// \brief The box's centre and the water surface above it, world z.
  double box{0.0};
  double eta{0.0};

  /// \brief The sphere's centre, world z.
  double sphere{0.0};
};

/// \brief A world under test, sampled every step.
class WaveWorld
{
  /// \brief Load a world.
  /// \param[in] _file World file name.
  public: explicit WaveWorld(const std::string &_file)
    : fixture(common::joinPaths(TEST_WORLD_DIR, _file))
  {
    this->fixture.OnPostUpdate(
        [this](const UpdateInfo &_info, const EntityComponentManager &_ecm)
        {
          const Entity world = _ecm.EntityByComponents(components::World());
          if (const auto *c = _ecm.Component<components::WaveCoupling>(world))
            this->coupling = c->Data();
          const Entity box = _ecm.EntityByComponents(
              components::Model(), components::Name("floater"));
          const Entity sphere = _ecm.EntityByComponents(
              components::Model(), components::Name("sinker"));
          if (kNullEntity == box || kNullEntity == sphere)
            return;
          Sample s;
          s.t = std::chrono::duration<double>(_info.simTime).count();
          const auto boxPose = worldPose(box, _ecm);
          s.box = boxPose.Pos().Z();
          s.sphere = worldPose(sphere, _ecm).Pos().Z();
          if (const auto *f = _ecm.Component<components::Wavefield>(world))
          {
            s.eta = waves::SurfaceElevation(f->Data(), boxPose.Pos().X(),
                                            boxPose.Pos().Y(), s.t);
          }
          this->samples.push_back(s);
        });
    this->fixture.Finalize();
  }

  /// \brief Run a span of simulation time.
  /// \param[in] _seconds s.
  /// \return True on success.
  public: bool Run(double _seconds)
  {
    return this->fixture.Server()->Run(
        true, static_cast<std::size_t>(std::lround(_seconds / kStep)), false);
  }

  /// \brief The fixture.
  public: TestFixture fixture;

  /// \brief Every step.
  public: std::vector<Sample> samples;

  /// \brief The water level the world's WaveCoupling holds, if any.
  public: std::optional<double> coupling;
};
}

//////////////////////////////////////////////////
/// On still water a floating box rests at the draft its weight gives, as
/// with the flat buoyancy, and the world is marked as wave coupled.
TEST(WaveBuoyancy, StillWaterRestsAtTheFlatDraft)
{
  WaveWorld world("flat.sdf");
  ASSERT_TRUE(world.Run(4.0));
  ASSERT_FALSE(world.samples.empty());
  EXPECT_NEAR(kRest, world.samples.back().box, 0.01);
  ASSERT_TRUE(world.coupling.has_value());
  EXPECT_DOUBLE_EQ(0.0, *world.coupling);
}

//////////////////////////////////////////////////
/// On a long wave a floating box rises and falls with the surface above
/// it; a sphere wholly under the water does not feel the wave at all.
TEST(WaveBuoyancy, ABoxRidesTheWaveASunkenSphereDoesNot)
{
  WaveWorld world("waves.sdf");
  ASSERT_TRUE(world.Run(16.0));

  // After the wave has ramped in: the box's heave against the surface.
  double sumH{0}, sumE{0}, sumHH{0}, sumEE{0}, sumHE{0};
  double sphereMin{1e9}, sphereMax{-1e9};
  int n{0};
  for (const auto &s : world.samples)
  {
    sphereMin = std::min(sphereMin, s.sphere);
    sphereMax = std::max(sphereMax, s.sphere);
    if (s.t < 8.0)
      continue;
    const double h = s.box - kRest;
    sumH += h;
    sumE += s.eta;
    sumHH += h * h;
    sumEE += s.eta * s.eta;
    sumHE += h * s.eta;
    ++n;
  }
  ASSERT_GT(n, 1000);
  const double varH = sumHH / n - (sumH / n) * (sumH / n);
  const double varE = sumEE / n - (sumE / n) * (sumE / n);
  const double cov = sumHE / n - (sumH / n) * (sumE / n);
  const double correlation = cov / std::sqrt(varH * varE);

  // The surface moves: a wave about 0.3 m high, 0.21 m standard deviation.
  EXPECT_GT(std::sqrt(varE), 0.15);
  // The box follows it, in phase and about as far.
  EXPECT_GT(correlation, 0.9);
  EXPECT_NEAR(1.0, std::sqrt(varH / varE), 0.3);

  // The sphere stays where it was put.
  EXPECT_NEAR(-3.0, sphereMin, 0.02);
  EXPECT_NEAR(-3.0, sphereMax, 0.02);
}
