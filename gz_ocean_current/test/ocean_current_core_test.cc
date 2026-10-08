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
#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/OceanCurrentfield.hh"
#include "gz/sim/ocean_current/OceanCurrentModel.hh"
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"
#include "gz/sim/ocean_current/OceanCurrentfield.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief A world with an ocean current recipe, and optionally a heading.
/// \param[in,out] _ecm Entity component manager to fill.
/// \param[in] _data The recipe.
/// \param[in] _headingDeg Heading of the world frame, if any.
/// \return The world entity.
Entity MakeWorld(EntityComponentManager &_ecm,
                 const ocean_current::OceanCurrentfieldData &_data,
                 std::optional<double> _headingDeg = std::nullopt)
{
  const Entity world = _ecm.CreateEntity();
  _ecm.CreateComponent(world, components::World());
  _ecm.CreateComponent(world, components::OceanCurrentfield(_data));
  if (_headingDeg)
  {
    math::SphericalCoordinates sc(
        math::SphericalCoordinates::EARTH_WGS84, GZ_DTOR(36.69),
        GZ_DTOR(-121.93), 0.0, GZ_DTOR(*_headingDeg));
    _ecm.CreateComponent(world, components::SphericalCoordinates(sc));
  }
  return world;
}

/// \brief A current setting east at a speed.
/// \param[in] _speed Speed, m/s.
/// \return The recipe.
ocean_current::OceanCurrentfieldData SettingEast(double _speed)
{
  ocean_current::OceanCurrentfieldData data;
  data.params.speed = _speed;
  data.params.direction = 90.0;
  data.generation = 1;
  return data;
}

/// \brief A model that returns an upward current of the recipe's speed, to
/// show that a new model needs nothing but registering.
class Upwelling : public ocean_current::IOceanCurrentModel
{
  public: void SetParameters(const ocean_current::OceanCurrentParameters &_p)
      override
  {
    this->p = _p;
  }
  public: math::Vector3d Velocity(const math::Vector3d &, double) const
      override
  {
    return {0.0, 0.0, this->p.speed};
  }
  public: std::string_view Kind() const override { return "upwelling"; }
  private: ocean_current::OceanCurrentParameters p;
};

/// \brief A model that places itself on the Earth: it flows north at its
/// world's latitude in degrees, divided by 100, and refuses a negative
/// `depth`, to show what a model gets and what it may refuse.
class Located : public ocean_current::IOceanCurrentModel
{
  public: std::string Validate(
      const ocean_current::OceanCurrentParameters &_p) const override
  {
    const auto it = _p.extra.find("depth");
    if (it != _p.extra.end() && std::stod(it->second) < 0.0)
      return "depth must not be negative";
    return {};
  }
  public: void SetSphericalCoordinates(const math::SphericalCoordinates &_sc)
      override
  {
    this->latitude = _sc.LatitudeReference().Degree();
  }
  public: void SetParameters(const ocean_current::OceanCurrentParameters &)
      override
  {
  }
  public: math::Vector3d Velocity(const math::Vector3d &, double) const
      override
  {
    return {0.0, this->latitude / 100.0, 0.0};
  }
  public: std::string_view Kind() const override { return "located"; }
  private: double latitude{0.0};
};

/// \brief Whether two vectors are the same to a nanometre per second.
/// \param[in] _a One.
/// \param[in] _b The other.
/// \return True if they are.
bool Near(const math::Vector3d &_a, const math::Vector3d &_b)
{
  return (_a - _b).Length() < 1e-9;
}
}  // namespace

/////////////////////////////////////////////////
/// The recipe survives a trip through text at full precision, which is how
/// it reaches another process.
TEST(OceanCurrentfield, RoundTripsAtFullPrecision)
{
  ocean_current::OceanCurrentfieldData in;
  in.model = "a model with spaces";
  in.generation = 17;
  in.params.seed = 4242424242u;
  in.params.speed = 3.141592653589793;
  in.params.direction = 271.828182845904;
  in.params.water_level = -1.234567890123456;
  in.params.extra["source"] = "a file with spaces.nc";
  in.params.extra["a \"quoted\" name"] = "";
  in.params.extra["m2_amplitude"] = "0.73";

  std::stringstream ss;
  ss << in;
  ocean_current::OceanCurrentfieldData out;
  ss >> out;

  EXPECT_EQ(in.model, out.model);
  EXPECT_EQ(in.generation, out.generation);
  EXPECT_EQ(in.params.seed, out.params.seed);
  EXPECT_EQ(in.params.speed, out.params.speed);
  EXPECT_EQ(in.params.direction, out.params.direction);
  EXPECT_EQ(in.params.water_level, out.params.water_level);
  EXPECT_EQ(in.params.extra, out.params.extra);
  EXPECT_TRUE(ss.good() || ss.eof());
}

/////////////////////////////////////////////////
/// No parameters of the model's own round trips too: it is the common case.
TEST(OceanCurrentfield, EmptyExtraRoundTrips)
{
  ocean_current::OceanCurrentfieldData in;
  in.params.speed = 1.0;
  std::stringstream ss;
  ss << in;
  ocean_current::OceanCurrentfieldData out;
  out.params.extra["source"] = "not empty";
  ss >> out;
  EXPECT_TRUE(out.params.extra.empty());
  EXPECT_EQ("standard", out.model);
  EXPECT_EQ(1.0, out.params.speed);
}

/////////////////////////////////////////////////
/// Parameters are set by name, and out of range values are refused.
TEST(OceanCurrentfield, SetParameterByName)
{
  ocean_current::OceanCurrentParameters p;
  EXPECT_TRUE(ocean_current::SetParameter(p, "speed", 0.6));
  EXPECT_DOUBLE_EQ(0.6, p.speed);
  EXPECT_TRUE(ocean_current::SetParameter(p, "direction", 180.0));
  EXPECT_DOUBLE_EQ(180.0, p.direction);
  EXPECT_TRUE(ocean_current::SetParameter(p, "seed", 7.0));
  EXPECT_EQ(7u, p.seed);
  EXPECT_TRUE(ocean_current::SetParameter(p, "water_level", -2.5))
      << "the surface may sit below the world's origin";
  EXPECT_DOUBLE_EQ(-2.5, p.water_level);

  EXPECT_FALSE(ocean_current::SetParameter(p, "speed", -1.0));
  EXPECT_FALSE(ocean_current::SetParameter(p, "seed", -1.0));
  EXPECT_FALSE(ocean_current::SetParameter(p, "vertical", 1.0));
  EXPECT_FALSE(ocean_current::SetParameter(p, "source", 1.0))
      << "the source belongs to the model, not to the typed parameters";
  EXPECT_FALSE(ocean_current::SetParameter(p, "speed", std::nan("")));
  EXPECT_DOUBLE_EQ(0.6, p.speed) << "a refused value changes nothing";
}

/////////////////////////////////////////////////
/// The typed names are the table's and the seed; anything else is the
/// model's.
TEST(OceanCurrentfield, TypedParameters)
{
  for (const char *name : {"speed", "direction", "water_level", "seed"})
    EXPECT_TRUE(ocean_current::IsTypedParameter(name)) << name;
  for (const char *name : {"source", "model", "speeed", ""})
    EXPECT_FALSE(ocean_current::IsTypedParameter(name)) << name;
}

/////////////////////////////////////////////////
/// A direction is kept in [0, 360), whatever turn it was given in.
TEST(OceanCurrentfield, DirectionIsWrapped)
{
  ocean_current::OceanCurrentParameters p;
  EXPECT_TRUE(ocean_current::SetParameter(p, "direction", 450.0));
  EXPECT_DOUBLE_EQ(90.0, p.direction);
  EXPECT_TRUE(ocean_current::SetParameter(p, "direction", -90.0));
  EXPECT_DOUBLE_EQ(270.0, p.direction);
  EXPECT_TRUE(ocean_current::SetParameter(p, "direction", 360.0));
  EXPECT_DOUBLE_EQ(0.0, p.direction);
  EXPECT_TRUE(ocean_current::SetParameter(p, "direction", -1e-20));
  EXPECT_LT(p.direction, 360.0);
  EXPECT_GE(p.direction, 0.0);
}

/////////////////////////////////////////////////
/// The standard model is always there; an unknown name gives nothing; a new
/// model is one registration away.
TEST(OceanCurrentModel, Registry)
{
  ocean_current::OceanCurrentParameters p;
  p.speed = 2.0;
  auto standard = ocean_current::CreateOceanCurrentModel("standard", p);
  ASSERT_NE(nullptr, standard);
  EXPECT_EQ("standard", standard->Kind());
  EXPECT_EQ(nullptr, ocean_current::CreateOceanCurrentModel("no_such", p));

  ocean_current::RegisterOceanCurrentModelFactory("upwelling",
      [] { return std::make_unique<Upwelling>(); });
  auto upwelling = ocean_current::CreateOceanCurrentModel("upwelling", p);
  ASSERT_NE(nullptr, upwelling);
  EXPECT_EQ("upwelling", upwelling->Kind());
  EXPECT_EQ(math::Vector3d(0, 0, 2), upwelling->Velocity({}, 0.0));
}

/////////////////////////////////////////////////
/// A model accepts or refuses its parameters. The standard one reads none of
/// its own, so it refuses any; a refused recipe builds no model.
TEST(OceanCurrentModel, ModelValidatesItsParameters)
{
  ocean_current::OceanCurrentParameters p;
  p.speed = 1.0;
  EXPECT_TRUE(ocean_current::ValidateOceanCurrentModel("standard", p)
      .empty());
  EXPECT_NE(std::string::npos, ocean_current::ValidateOceanCurrentModel(
      "no_such", p).find("no ocean current model named [no_such]"));

  p.extra["source"] = "grid.nc";
  const std::string why =
      ocean_current::ValidateOceanCurrentModel("standard", p);
  EXPECT_NE(std::string::npos, why.find("source")) << why;
  EXPECT_EQ(nullptr, ocean_current::CreateOceanCurrentModel("standard", p));

  ocean_current::RegisterOceanCurrentModelFactory("located",
      [] { return std::make_unique<Located>(); });
  p.extra = {{"depth", "-3"}};
  EXPECT_EQ("depth must not be negative",
            ocean_current::ValidateOceanCurrentModel("located", p));
  EXPECT_EQ(nullptr, ocean_current::CreateOceanCurrentModel("located", p));
  p.extra = {{"depth", "3"}};
  EXPECT_NE(nullptr, ocean_current::CreateOceanCurrentModel("located", p));
}

/////////////////////////////////////////////////
/// The direction is where the current sets towards, clockwise from north,
/// in east north up: the opposite convention from the wind. The standard
/// current is horizontal.
TEST(OceanCurrentModel, StandardDirections)
{
  const auto flow = [](double _towards)
  {
    ocean_current::OceanCurrentParameters p;
    p.speed = 1.0;
    p.direction = _towards;
    return ocean_current::CreateOceanCurrentModel("standard", p)
        ->Velocity({}, 0.0);
  };
  EXPECT_TRUE(Near({0, 1, 0}, flow(0.0))) << "setting north flows north";
  EXPECT_TRUE(Near({1, 0, 0}, flow(90.0))) << "setting east flows east";
  EXPECT_TRUE(Near({0, -1, 0}, flow(180.0))) << "setting south flows south";
  EXPECT_TRUE(Near({-1, 0, 0}, flow(270.0))) << "setting west flows west";
  EXPECT_TRUE(Near({1, 0, 0}, ocean_current::VelocityFromSet(1.0, 90.0)));
  EXPECT_TRUE(Near({0, 0, 0}, ocean_current::VelocityFromSet(0.0, 45.0)))
      << "slack water has no direction worth the name";
}

/////////////////////////////////////////////////
/// The standard current is the same everywhere and at any time, and says
/// so, so nothing caches it in vain.
TEST(OceanCurrentModel, StandardIsConstant)
{
  auto model = ocean_current::CreateOceanCurrentModel("standard",
      SettingEast(0.5).params);
  ASSERT_NE(nullptr, model);
  EXPECT_FALSE(model->TimeVarying());
  EXPECT_TRUE(Near({0.5, 0, 0}, model->Velocity({}, 0.0)));
  EXPECT_TRUE(Near({0.5, 0, 0}, model->Velocity({800, -1, -5}, 3600.0)));
}

/////////////////////////////////////////////////
/// The sampler reads the recipe from the world and answers in the world
/// frame; without a recipe it has no current.
TEST(OceanCurrentSampler, AnswersFromTheRecipe)
{
  EntityComponentManager ecm;
  ocean_current::OceanCurrentSampler sampler;
  EXPECT_FALSE(sampler.Sync(ecm));
  EXPECT_FALSE(sampler.Valid());
  EXPECT_EQ(math::Vector3d::Zero, sampler.At({1, 2, 3}, {}));

  MakeWorld(ecm, SettingEast(1.0));
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_TRUE(sampler.Valid());
  EXPECT_FALSE(sampler.TimeVarying());
  const math::Vector3d v = sampler.At({10, 20, -3}, std::chrono::seconds(5));
  EXPECT_NEAR(1.0, v.X(), 1e-9);
  EXPECT_NEAR(0.0, v.Y(), 1e-9);
  EXPECT_NEAR(0.0, v.Z(), 1e-9);
  EXPECT_EQ(v, ocean_current::OceanCurrentAt(ecm, {10, 20, -3},
                                             std::chrono::seconds(5)));
}

/////////////////////////////////////////////////
/// A new recipe, with a new generation, is picked up on the next Sync.
TEST(OceanCurrentSampler, FollowsTheGeneration)
{
  EntityComponentManager ecm;
  const Entity world = MakeWorld(ecm, SettingEast(1.0));
  ocean_current::OceanCurrentSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_NEAR(1.0, sampler.At({}, {}).X(), 1e-9);

  auto *comp = ecm.Component<components::OceanCurrentfield>(world);
  comp->Data().params.speed = 2.0;
  comp->Data().generation = 2;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_NEAR(2.0, sampler.At({}, {}).X(), 1e-9);
}

/////////////////////////////////////////////////
/// The sampler builds whatever model the recipe names: a model registered
/// by anyone is used without a change to the sampler, and an unknown name
/// is no current rather than a crash.
TEST(OceanCurrentSampler, UsesTheModelTheRecipeNames)
{
  ocean_current::RegisterOceanCurrentModelFactory("upwelling",
      [] { return std::make_unique<Upwelling>(); });

  EntityComponentManager ecm;
  auto data = SettingEast(2.0);
  data.model = "upwelling";
  const Entity world = MakeWorld(ecm, data);
  ocean_current::OceanCurrentSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_TRUE(Near({0, 0, 2}, sampler.At({}, {})));

  auto *comp = ecm.Component<components::OceanCurrentfield>(world);
  comp->Data().model = "no_such_model";
  comp->Data().generation = 2;
  EXPECT_FALSE(sampler.Sync(ecm));
  EXPECT_FALSE(sampler.Valid());
  EXPECT_EQ(math::Vector3d::Zero, sampler.At({}, {}));
}

/////////////////////////////////////////////////
/// North is the world's north: in a world turned against it, the sampler
/// turns the current the same way the world's GPS does. In gz-math a
/// heading of 90 degrees turns the world so its +x points north, so a
/// current setting east flows towards -y. A fixed value, so a change of
/// convention anywhere shows up here.
TEST(OceanCurrentSampler, NorthIsTheWorldsNorth)
{
  EntityComponentManager ecm;
  MakeWorld(ecm, SettingEast(1.0), 90.0);
  ocean_current::OceanCurrentSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  const math::Vector3d v = sampler.At({}, {});
  EXPECT_NEAR(0.0, v.X(), 1e-9);
  EXPECT_NEAR(-1.0, v.Y(), 1e-9) << "geographic east is -y here";
  EXPECT_NEAR(0.0, v.Z(), 1e-9);
}

/////////////////////////////////////////////////
/// Two worlds, two currents: a query on one never answers with the other's,
/// though both number their world entity and their generation alike.
TEST(OceanCurrentSampler, TwoWorldsKeepTheirOwnCurrent)
{
  EntityComponentManager slow;
  EntityComponentManager fast;
  const Entity a = MakeWorld(slow, SettingEast(1.0));
  const Entity b = MakeWorld(fast, SettingEast(2.0));
  ASSERT_EQ(a, b) << "the case that matters: the same world entity";

  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(1.0, ocean_current::OceanCurrentAt(slow, {}, {}).X(), 1e-9);
    EXPECT_NEAR(2.0, ocean_current::OceanCurrentAt(fast, {}, {}).X(), 1e-9);
  }
}

/////////////////////////////////////////////////
/// A model gets the world's spherical coordinates, and is rebuilt when they
/// change, with no new recipe.
TEST(OceanCurrentSampler, ModelGetsTheWorldsSphericalCoordinates)
{
  ocean_current::RegisterOceanCurrentModelFactory("located",
      [] { return std::make_unique<Located>(); });
  EntityComponentManager ecm;
  auto data = SettingEast(0.0);
  data.model = "located";
  const Entity world = MakeWorld(ecm, data, 0.0);
  ocean_current::OceanCurrentSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_NEAR(0.3669, sampler.At({}, {}).Y(), 1e-9)
      << "the latitude MakeWorld sets, 36.69, over 100";

  auto *sc = ecm.Component<components::SphericalCoordinates>(world);
  sc->Data().SetLatitudeReference(GZ_DTOR(10.0));
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_NEAR(0.1, sampler.At({}, {}).Y(), 1e-9);
}
