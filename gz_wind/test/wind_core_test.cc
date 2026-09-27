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
