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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/math/Helpers.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>

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

/// \brief One model at one step.
struct Sample
{
  /// \brief Simulation time, s.
  public: double t{0.0};

  /// \brief The model's world pose.
  public: math::Pose3d pose;

  /// \brief The drawn surface over the model's origin, world z.
  public: double eta{0.0};

  /// \brief The surface's slope along x there.
  public: double slope{0.0};
};

/// \brief Correlation of two series.
/// \param[in] _a One.
/// \param[in] _b The other.
/// \return Pearson's correlation.
double Correlation(const std::vector<double> &_a,
                   const std::vector<double> &_b)
{
  const double n = static_cast<double>(_a.size());
  double sa{0}, sb{0}, saa{0}, sbb{0}, sab{0};
  for (std::size_t i = 0; i < _a.size(); ++i)
  {
    sa += _a[i];
    sb += _b[i];
    saa += _a[i] * _a[i];
    sbb += _b[i] * _b[i];
    sab += _a[i] * _b[i];
  }
  const double va = saa / n - (sa / n) * (sa / n);
  const double vb = sbb / n - (sb / n) * (sb / n);
  return (sab / n - (sa / n) * (sb / n)) / std::sqrt(va * vb);
}

/// \brief Standard deviation of a series.
/// \param[in] _a The series.
/// \return Its standard deviation.
double Deviation(const std::vector<double> &_a)
{
  const double n = static_cast<double>(_a.size());
  double s{0}, ss{0};
  for (double v : _a)
  {
    s += v;
    ss += v * v;
  }
  return std::sqrt(std::max(0.0, ss / n - (s / n) * (s / n)));
}

/// \brief A world under test, its models sampled every step.
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
          const auto *field = _ecm.Component<components::Wavefield>(world);
          const double t =
              std::chrono::duration<double>(_info.simTime).count();
          _ecm.Each<components::Model, components::Name>(
              [&](const Entity &_model, const components::Model *,
                  const components::Name *_name) -> bool
              {
                Sample s;
                s.t = t;
                s.pose = worldPose(_model, _ecm);
                if (field)
                {
                  const double x = s.pose.Pos().X();
                  const double y = s.pose.Pos().Y();
                  s.eta = waves::SurfaceHeight(field->Data(), x, y, t);
                  s.slope =
                      (waves::SurfaceHeight(field->Data(), x + 0.05, y, t) -
                       waves::SurfaceHeight(field->Data(), x - 0.05, y, t)) /
                      0.1;
                }
                this->samples[_name->Data()].push_back(s);
                return true;
              });
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

  /// \brief A model's samples from a time on.
  /// \param[in] _model The model's name.
  /// \param[in] _from s.
  /// \return The samples.
  public: std::vector<Sample> From(const std::string &_model,
                                   double _from) const
  {
    std::vector<Sample> out;
    const auto it = this->samples.find(_model);
    if (it == this->samples.end())
      return out;
    for (const auto &s : it->second)
    {
      if (s.t >= _from)
        out.push_back(s);
    }
    return out;
  }

  /// \brief The fixture.
  public: TestFixture fixture;

  /// \brief Every step, by model.
  public: std::map<std::string, std::vector<Sample>> samples;
};

/// \brief The heave of a floating box against the surface over it.
/// \param[in] _samples The box's samples.
/// \param[out] _heave Its centre's height over its rest.
/// \param[out] _eta The surface's.
void Heave(const std::vector<Sample> &_samples, std::vector<double> &_heave,
           std::vector<double> &_eta)
{
  for (const auto &s : _samples)
  {
    _heave.push_back(s.pose.Pos().Z() - kRest);
    _eta.push_back(s.eta);
  }
}
}  // namespace

//////////////////////////////////////////////////
/// On still water a floating box rests at the draft its weight gives, as
/// with the flat buoyancy, and level.
TEST(WaveBuoyancy, StillWaterRestsAtTheFlatDraft)
{
  WaveWorld world("flat.sdf");
  ASSERT_TRUE(world.Run(4.0));
  const auto box = world.From("floater", 0.0);
  ASSERT_FALSE(box.empty());
  EXPECT_NEAR(kRest, box.back().pose.Pos().Z(), 0.01);
  EXPECT_NEAR(0.0, box.back().pose.Rot().Euler().Y(), 1e-3);
}

//////////////////////////////////////////////////
/// The buoyancy acts where the displaced water is, not at the centre of
/// mass: a box heavy forward trims bow down, about the angle its
/// metacentric height gives, and a box whose shape and centre of mass are
/// both off its link origin floats level, as the first.
TEST(WaveBuoyancy, MomentsAboutTheCentreOfMass)
{
  WaveWorld world("flat.sdf");
  ASSERT_TRUE(world.Run(6.0));

  // Undamped, the trimmed box rocks about its trim: take the mean.
  const auto trimmed = world.From("trimmed", 2.0);
  ASSERT_FALSE(trimmed.empty());
  double pitch{0.0};
  for (const auto &s : trimmed)
    pitch += s.pose.Rot().Euler().Y();
  pitch /= static_cast<double>(trimmed.size());
  // atan(0.03 / 0.325) = 0.092 rad; positive pitch puts +x down.
  EXPECT_NEAR(0.092, pitch, 0.02);

  const auto offset = world.From("offset", 0.0);
  ASSERT_FALSE(offset.empty());
  for (const auto &s : offset)
  {
    ASSERT_NEAR(0.0, s.pose.Rot().Euler().X(), 1e-3) << s.t;
    ASSERT_NEAR(0.0, s.pose.Rot().Euler().Y(), 1e-3) << s.t;
  }
  EXPECT_NEAR(kRest, offset.back().pose.Pos().Z(), 0.01);
}

//////////////////////////////////////////////////
/// The air holds up a body as heavy as the air it displaces.
TEST(WaveBuoyancy, TheAirDisplacedPushesToo)
{
  WaveWorld world("flat.sdf");
  ASSERT_TRUE(world.Run(4.0));
  const auto balloon = world.From("balloon", 0.0);
  ASSERT_FALSE(balloon.empty());
  EXPECT_NEAR(5.0, balloon.back().pose.Pos().Z(), 0.01);
}

//////////////////////////////////////////////////
/// With the flat buoyancy in the world too, the wave buoyancy steps aside:
/// the box floats once, at the flat draft, not on twice the water.
TEST(WaveBuoyancy, StepsAsideForTheFlatBuoyancy)
{
  WaveWorld world("both.sdf");
  ASSERT_TRUE(world.Run(4.0));
  const auto box = world.From("floater", 0.0);
  ASSERT_FALSE(box.empty());
  for (const auto &s : box)
    ASSERT_LT(s.pose.Pos().Z(), kRest + 0.02) << s.t;
  EXPECT_NEAR(kRest, box.back().pose.Pos().Z(), 0.01);
}

//////////////////////////////////////////////////
/// On a long wave a floating box rises and falls with the surface above
/// it, and pitches with its slope.
TEST(WaveBuoyancy, ABoxRidesAndPitchesWithTheWave)
{
  WaveWorld world("waves.sdf");
  ASSERT_TRUE(world.Run(16.0));

  // After the wave has ramped in.
  const auto box = world.From("floater", 8.0);
  ASSERT_GT(box.size(), 1000u);
  std::vector<double> heave, eta, pitch, slope;
  Heave(box, heave, eta);
  for (const auto &s : box)
  {
    pitch.push_back(s.pose.Rot().Euler().Y());
    // The surface rising towards +x lifts the bow: a negative pitch.
    slope.push_back(-std::atan(s.slope));
  }

  // The surface moves: a wave about 0.3 m high, 0.21 m standard deviation.
  EXPECT_GT(Deviation(eta), 0.15);
  // The box follows it, in phase and about as far.
  EXPECT_GT(Correlation(heave, eta), 0.9);
  EXPECT_NEAR(1.0, Deviation(heave) / Deviation(eta), 0.3);

  // And tilts with it: a 25 m wave 0.3 m high slopes 4 degrees at most.
  // The water pushes straight up, so the box heels further than the slope:
  // by BM / GM statically, its centre of mass being above its centre of
  // buoyancy, and a little more for the wave's period against its own.
  const double draft = 200.0 / 1025.0;
  const double bm = (1.0 / 12.0) / draft;
  const double gm = bm - (kRest + draft / 2.0);
  const double natural = 200.0 * 9.80665 * gm / 19.3;
  const double forcing = std::pow(2.0 * GZ_PI / 4.0, 2);
  const double gain = bm / gm / (1.0 - forcing / natural);
  EXPECT_GT(Deviation(slope), 0.03);
  EXPECT_GT(Correlation(pitch, slope), 0.98);
  EXPECT_NEAR(gain, Deviation(pitch) / Deviation(slope), 0.1);
}

//////////////////////////////////////////////////
/// The pressure under a wave is not modelled: a neutral sphere 3 m under
/// the surface stays put, where it would orbit about 0.14 m in a real
/// wave. A limit of the model, held here so a change to it is seen.
TEST(WaveBuoyancy, ASunkenSphereFeelsOnlyStillWater)
{
  WaveWorld world("waves.sdf");
  ASSERT_TRUE(world.Run(16.0));
  const auto sphere = world.From("sinker", 0.0);
  ASSERT_FALSE(sphere.empty());
  for (const auto &s : sphere)
    ASSERT_NEAR(-3.0, s.pose.Pos().Z(), 0.02) << s.t;
}

//////////////////////////////////////////////////
/// On the FFT engine's sea, read at its own update rate, the box rides the
/// drawn surface too.
TEST(WaveBuoyancy, ABoxRidesTheFftSea)
{
  WaveWorld world("fft.sdf");
  ASSERT_TRUE(world.Run(20.0));
  const auto box = world.From("floater", 6.0);
  ASSERT_GT(box.size(), 1000u);
  std::vector<double> heave, eta;
  Heave(box, heave, eta);
  EXPECT_GT(Deviation(eta), 0.05);
  EXPECT_GT(Correlation(heave, eta), 0.8);
}
