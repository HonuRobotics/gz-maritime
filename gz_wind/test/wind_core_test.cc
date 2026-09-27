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
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Angle.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/Windfield.hh"
#include "gz/sim/wind/Windfield.hh"
#include "gz/sim/wind/WindModel.hh"
#include "gz/sim/wind/WindSampler.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief A world with a wind recipe, and optionally a heading.
/// \param[in,out] _ecm Entity component manager to fill.
/// \param[in] _data The recipe.
/// \param[in] _headingDeg Heading of the world frame, if any.
/// \return The world entity.
Entity MakeWorld(EntityComponentManager &_ecm, const wind::WindfieldData &_data,
                 std::optional<double> _headingDeg = std::nullopt)
{
  const Entity world = _ecm.CreateEntity();
  _ecm.CreateComponent(world, components::World());
  _ecm.CreateComponent(world, components::Windfield(_data));
  if (_headingDeg)
  {
    math::SphericalCoordinates sc(
        math::SphericalCoordinates::EARTH_WGS84, GZ_DTOR(36.69),
        GZ_DTOR(-121.93), 0.0, GZ_DTOR(*_headingDeg));
    _ecm.CreateComponent(world, components::SphericalCoordinates(sc));
  }
  return world;
}

/// \brief A wind that blows from the west at a speed.
/// \param[in] _speed Speed, m/s.
/// \return The recipe.
wind::WindfieldData FromTheWest(double _speed)
{
  wind::WindfieldData data;
  data.params.speed = _speed;
  data.params.direction = 270.0;
  data.generation = 1;
  return data;
}

/// \brief A model that returns a fixed wind, to show that a new model needs
/// nothing but registering.
class Steady : public wind::IWindModel
{
  public: void SetParameters(const wind::WindParameters &_p) override
  {
    this->p = _p;
  }
  public: math::Vector3d Velocity(const math::Vector3d &, double) const
      override
  {
    return {0.0, 0.0, this->p.speed};
  }
  public: std::string_view Kind() const override { return "steady"; }
  private: wind::WindParameters p;
};

/// \brief Mean and standard deviation of a series.
/// \param[in] _v The series.
/// \return Mean first, standard deviation second.
std::pair<double, double> Stats(const std::vector<double> &_v)
{
  double mean{0.0};
  for (const double x : _v)
    mean += x;
  mean /= static_cast<double>(_v.size());
  double var{0.0};
  for (const double x : _v)
    var += (x - mean) * (x - mean);
  return {mean, std::sqrt(var / static_cast<double>(_v.size()))};
}

/// \brief Autocorrelation of a series at a lag.
/// \param[in] _v The series.
/// \param[in] _lag Lag in samples.
/// \return The correlation, 1 at lag 0.
double Autocorrelation(const std::vector<double> &_v, std::size_t _lag)
{
  const auto [mean, sd] = Stats(_v);
  double sum{0.0};
  for (std::size_t i = 0; i + _lag < _v.size(); ++i)
    sum += (_v[i] - mean) * (_v[i + _lag] - mean);
  return sum / static_cast<double>(_v.size() - _lag) / (sd * sd);
}

/// \brief A gusty wind: 5 m/s from the west, gusts of 1 m/s and 10 degrees,
/// both with a 0.2 s correlation time.
/// \param[in] _seed Seed.
/// \return The parameters.
wind::WindParameters Gusty(std::uint32_t _seed)
{
  wind::WindParameters p;
  p.speed = 5.0;
  p.direction = 270.0;
  p.speed_gust = 1.0;
  p.speed_gust_time = 0.2;
  p.direction_gust = 10.0;
  p.direction_gust_time = 0.2;
  p.seed = _seed;
  return p;
}
}  // namespace

/////////////////////////////////////////////////
/// The recipe survives a trip through text at full precision, which is how
/// it reaches another process.
TEST(Windfield, RoundTripsAtFullPrecision)
{
  wind::WindfieldData in;
  in.model = "a model with spaces";
  in.generation = 17;
  in.params.seed = 4242424242u;
  in.params.speed = 3.141592653589793;
  in.params.direction = 271.828182845904;
  in.params.vertical = -0.123456789012345;

  std::stringstream ss;
  ss << in;
  wind::WindfieldData out;
  ss >> out;

  EXPECT_EQ(in.model, out.model);
  EXPECT_EQ(in.generation, out.generation);
  EXPECT_EQ(in.params.seed, out.params.seed);
  EXPECT_EQ(in.params.speed, out.params.speed);
  EXPECT_EQ(in.params.direction, out.params.direction);
  EXPECT_EQ(in.params.vertical, out.params.vertical);
}

/////////////////////////////////////////////////
/// Parameters are set by name, and out of range values are refused.
TEST(Windfield, SetParameterByName)
{
  wind::WindParameters p;
  EXPECT_TRUE(wind::SetParameter(p, "speed", 6.0));
  EXPECT_DOUBLE_EQ(6.0, p.speed);
  EXPECT_TRUE(wind::SetParameter(p, "direction", 180.0));
  EXPECT_DOUBLE_EQ(180.0, p.direction);
  EXPECT_TRUE(wind::SetParameter(p, "seed", 7.0));
  EXPECT_EQ(7u, p.seed);

  EXPECT_FALSE(wind::SetParameter(p, "speed", -1.0));
  EXPECT_FALSE(wind::SetParameter(p, "seed", -1.0));
  EXPECT_FALSE(wind::SetParameter(p, "gust", 1.0));
  EXPECT_FALSE(wind::SetParameter(p, "speed_gust", -1.0));
  EXPECT_FALSE(wind::SetParameter(p, "speed_gust_time", 0.0));
  EXPECT_FALSE(wind::SetParameter(p, "direction_gust_time", -2.0));
  EXPECT_TRUE(wind::SetParameter(p, "direction_gust", 15.0));
  EXPECT_FALSE(wind::SetParameter(p, "speed", std::nan("")));
  EXPECT_DOUBLE_EQ(6.0, p.speed) << "a refused value changes nothing";
}

/////////////////////////////////////////////////
/// The standard model is always there; an unknown name gives nothing; a new
/// model is one registration away.
TEST(WindModel, Registry)
{
  wind::WindParameters p;
  p.speed = 2.0;
  auto standard = wind::CreateWindModel("standard", p);
  ASSERT_NE(nullptr, standard);
  EXPECT_EQ("standard", standard->Kind());
  EXPECT_EQ(nullptr, wind::CreateWindModel("no_such_model", p));

  wind::RegisterWindModelFactory("steady",
      [] { return std::make_unique<Steady>(); });
  auto steady = wind::CreateWindModel("steady", p);
  ASSERT_NE(nullptr, steady);
  EXPECT_EQ(math::Vector3d(0, 0, 2), steady->Velocity({}, 0.0));
}

/////////////////////////////////////////////////
/// The direction is where the wind comes from, clockwise from north, in east
/// north up.
TEST(WindModel, StandardDirections)
{
  auto model = wind::CreateWindModel("standard", wind::WindParameters{});
  ASSERT_NE(nullptr, model);
  const auto blow = [&](double _from)
  {
    wind::WindParameters p;
    p.speed = 5.0;
    p.direction = _from;
    model->SetParameters(p);
    return model->Velocity({}, 0.0);
  };
  const auto near = [](const math::Vector3d &_a, const math::Vector3d &_b)
  {
    return (_a - _b).Length() < 1e-9;
  };
  EXPECT_TRUE(near({0, -5, 0}, blow(0.0))) << "a north wind blows south";
  EXPECT_TRUE(near({-5, 0, 0}, blow(90.0))) << "an east wind blows west";
  EXPECT_TRUE(near({0, 5, 0}, blow(180.0))) << "a south wind blows north";
  EXPECT_TRUE(near({5, 0, 0}, blow(270.0))) << "a west wind blows east";
}

/////////////////////////////////////////////////
/// Over a minute of gusts the speed and the direction keep their means, with
/// the standard deviations asked for, and forget themselves over one
/// correlation time: about 1/e after 0.2 s.
TEST(WindModel, GustStatisticsMatchTheParameters)
{
  auto model = wind::CreateWindModel("standard", Gusty(42));
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->TimeVarying());

  // 60 s every 2 ms: 300 correlation times, so the estimates settle.
  std::vector<double> speed;
  std::vector<double> direction;
  for (int i = 0; i < 30000; ++i)
  {
    const math::Vector3d v = model->Velocity({}, i * 0.002);
    speed.push_back(std::hypot(v.X(), v.Y()));
    direction.push_back(
        std::fmod(GZ_RTOD(std::atan2(-v.X(), -v.Y())) + 360.0, 360.0));
  }
  const auto [speedMean, speedSd] = Stats(speed);
  const auto [dirMean, dirSd] = Stats(direction);
  EXPECT_NEAR(5.0, speedMean, 0.2);
  EXPECT_NEAR(1.0, speedSd, 0.1);
  EXPECT_NEAR(270.0, dirMean, 2.0);
  EXPECT_NEAR(10.0, dirSd, 1.0);
  // Lag of one correlation time: 0.2 s is 100 samples of 2 ms.
  EXPECT_NEAR(std::exp(-1.0), Autocorrelation(speed, 100), 0.1);
  EXPECT_NEAR(std::exp(-1.0), Autocorrelation(direction, 100), 0.1);
}

/////////////////////////////////////////////////
/// The seed fixes the gusts: two copies built from the same recipe agree at
/// any time, the way two processes do; another seed gives other gusts.
TEST(WindModel, SeedFixesTheGusts)
{
  auto a = wind::CreateWindModel("standard", Gusty(42));
  auto b = wind::CreateWindModel("standard", Gusty(42));
  auto c = wind::CreateWindModel("standard", Gusty(43));
  for (const double t : {0.0, 1.7, 123.4})
  {
    EXPECT_EQ(a->Velocity({3, 4, 0}, t), b->Velocity({3, 4, 0}, t));
    EXPECT_NE(a->Velocity({3, 4, 0}, t), c->Velocity({3, 4, 0}, t));
  }
}

/////////////////////////////////////////////////
/// The gusts travel with the mean wind: a point 5 m downwind of a 5 m/s
/// wind sees, one second later, the gust the origin saw; a point across the
/// wind sees it at the same time.
TEST(WindModel, GustsTravelWithTheWind)
{
  auto model = wind::CreateWindModel("standard", Gusty(42));
  const math::Vector3d downwind(5, 0, 0);
  const math::Vector3d across(0, 7, 0);
  const auto near = [](const math::Vector3d &_a, const math::Vector3d &_b)
  {
    return (_a - _b).Length() < 1e-9;
  };
  for (const double t : {0.0, 0.3, 2.5})
  {
    EXPECT_TRUE(near(model->Velocity({}, t),
                     model->Velocity(downwind, t + 1.0)));
    EXPECT_TRUE(near(model->Velocity({}, t), model->Velocity(across, t)));
  }
  EXPECT_FALSE(near(model->Velocity({}, 0.3), model->Velocity(downwind, 0.3)));
}

/////////////////////////////////////////////////
/// Without gusts the wind does not change with time.
TEST(WindModel, SteadyWithoutGusts)
{
  auto p = Gusty(42);
  p.speed_gust = 0.0;
  p.direction_gust = 0.0;
  auto model = wind::CreateWindModel("standard", p);
  EXPECT_FALSE(model->TimeVarying());
  EXPECT_EQ(model->Velocity({}, 0.0), model->Velocity({8, 1, 0}, 42.0));
}

/////////////////////////////////////////////////
/// The sampler reads the recipe from the world and answers in the world
/// frame; without a recipe it has no wind.
TEST(WindSampler, AnswersFromTheRecipe)
{
  EntityComponentManager ecm;
  wind::WindSampler sampler;
  EXPECT_FALSE(sampler.Sync(ecm));
  EXPECT_FALSE(sampler.Valid());
  EXPECT_EQ(math::Vector3d::Zero, sampler.At({1, 2, 3}, {}));

  MakeWorld(ecm, FromTheWest(5.0));
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_FALSE(sampler.TimeVarying());
  const math::Vector3d v = sampler.At({10, 20, 3}, std::chrono::seconds(5));
  EXPECT_NEAR(5.0, v.X(), 1e-9);
  EXPECT_NEAR(0.0, v.Y(), 1e-9);
  EXPECT_EQ(v, wind::WindAt(ecm, {10, 20, 3}, std::chrono::seconds(5)));
}

/////////////////////////////////////////////////
/// A new recipe, with a new generation, is picked up on the next Sync.
TEST(WindSampler, FollowsTheGeneration)
{
  EntityComponentManager ecm;
  const Entity world = MakeWorld(ecm, FromTheWest(5.0));
  wind::WindSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_NEAR(5.0, sampler.At({}, {}).X(), 1e-9);

  auto *comp = ecm.Component<components::Windfield>(world);
  comp->Data().params.speed = 8.0;
  comp->Data().generation = 2;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_NEAR(8.0, sampler.At({}, {}).X(), 1e-9);
}

/////////////////////////////////////////////////
/// North is the world's north: in a world turned against it, the sampler
/// turns the wind the same way the world's GPS does.
TEST(WindSampler, NorthIsTheWorldsNorth)
{
  EntityComponentManager ecm;
  const Entity world = MakeWorld(ecm, FromTheWest(5.0), 90.0);
  wind::WindSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  const math::Vector3d v = sampler.At({}, {});

  const auto &sc = ecm.Component<components::SphericalCoordinates>(world)
      ->Data();
  const auto expected = sc.LocalFromGlobalVelocity(
      math::CoordinateVector3::Metric(5.0, 0.0, 0.0));
  ASSERT_TRUE(expected.has_value());
  EXPECT_NEAR(expected->X().value(), v.X(), 1e-9);
  EXPECT_NEAR(expected->Y().value(), v.Y(), 1e-9);
  EXPECT_GT(std::abs(v.Y()), 4.9) << "the heading turned it off +x";
}
