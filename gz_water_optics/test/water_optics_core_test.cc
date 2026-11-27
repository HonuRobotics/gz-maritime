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

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"
#include "gz/sim/water_optics/Jerlov.hh"
#include "gz/sim/water_optics/WaterOptics.hh"
#include "gz/sim/water_optics/WaterOpticsSampler.hh"

using namespace gz;
using namespace sim;
using water_optics::Rgb;
using water_optics::WaterOpticsData;
using water_optics::WaterOpticsSpec;

namespace
{
constexpr double kTol = 1e-9;

void ExpectRgb(const Rgb &_actual, const Rgb &_expected, double _tol = kTol)
{
  EXPECT_NEAR(_actual.X(), _expected.X(), _tol);
  EXPECT_NEAR(_actual.Y(), _expected.Y(), _tol);
  EXPECT_NEAR(_actual.Z(), _expected.Z(), _tol);
}

Rgb Div(const Rgb &_a, const Rgb &_b)
{
  return Rgb(_a.X() / _b.X(), _a.Y() / _b.Y(), _a.Z() / _b.Z());
}

WaterOpticsData MustResolve(const WaterOpticsSpec &_spec,
                            std::string *_warning = nullptr)
{
  WaterOpticsData out;
  std::string error, warning;
  EXPECT_TRUE(water_optics::Resolve(_spec, out, error, warning)) << error;
  if (_warning)
    *_warning = warning;
  return out;
}

std::string MustFail(const WaterOpticsSpec &_spec)
{
  WaterOpticsData out;
  out.jerlov = "untouched";
  std::string error, warning;
  EXPECT_FALSE(water_optics::Resolve(_spec, out, error, warning));
  EXPECT_EQ(out.jerlov, "untouched") << "a failed Resolve changed its output";
  return error;
}
}  // namespace

/////////////////////////////////////////////////
// The table: every type, in order, with the published values.
TEST(Jerlov, TableHasEveryType)
{
  const auto &types = water_optics::JerlovTypes();
  const std::vector<std::string> names{"I", "IA", "IB", "II", "III",
                                       "1C", "3C", "5C", "7C", "9C"};
  ASSERT_EQ(types.size(), names.size());
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    EXPECT_EQ(types[i].name, names[i]);
    for (const Rgb *v : {&types[i].absorption, &types[i].scattering,
                         &types[i].diffuseAttenuation})
    {
      EXPECT_GT(v->X(), 0.0);
      EXPECT_GT(v->Y(), 0.0);
      EXPECT_GT(v->Z(), 0.0);
    }
  }
  ExpectRgb(water_optics::JerlovWavelengths(), Rgb(625, 525, 450));
}

/////////////////////////////////////////////////
// Spot checks against Solonenko and Mobley (2015), Tables 4-8.
TEST(Jerlov, TableMatchesSource)
{
  struct Row
  {
    const char *name;
    Rgb a, b, kd;
  };
  // Solonenko and Mobley (2015), checked against the paper; 5C red from
  // Williamson and Hollins (2022) and Jerlov (1976). See Jerlov.cc.
  const Row rows[] = {
    {"I", {0.295, 0.046, 0.018}, {0.00104, 0.00205, 0.00381},
     {0.302, 0.049, 0.022}},
    {"IA", {0.295, 0.0468, 0.0221}, {0.00256, 0.00402, 0.00631},
     {0.303, 0.0503, 0.0264}},
    {"IB", {0.296, 0.0469, 0.0235}, {0.0468, 0.0565, 0.068},
     {0.310, 0.0572, 0.0342}},
    {"II", {0.296, 0.0469, 0.0241}, {0.288, 0.387, 0.504},
     {0.337, 0.0845, 0.062}},
    {"III", {0.297, 0.0507, 0.0388}, {0.788, 1.06, 1.38},
     {0.390, 0.129, 0.124}},
    {"1C", {0.304, 0.068, 0.105}, {0.293, 0.395, 0.514},
     {0.395, 0.121, 0.179}},
    {"3C", {0.307, 0.078, 0.154}, {0.855, 1.15, 1.50},
     {0.456, 0.187, 0.319}},
    {"5C", {0.327, 0.127, 0.297}, {1.070, 1.44, 1.87},
     {0.48, 0.277, 0.535}},
    {"7C", {0.367, 0.233, 0.542}, {1.89, 2.54, 3.30},
     {0.600, 0.470, 0.924}},
    {"9C", {0.436, 0.430, 0.943}, {2.51, 3.38, 4.39},
     {0.763, 0.826, 1.56}},
  };
  ASSERT_EQ(water_optics::JerlovTypes().size(), std::size(rows));
  for (const Row &row : rows)
  {
    SCOPED_TRACE(row.name);
    const auto type = water_optics::Jerlov(row.name);
    ASSERT_TRUE(type);
    ExpectRgb(type->absorption, row.a);
    ExpectRgb(type->scattering, row.b);
    ExpectRgb(type->diffuseAttenuation, row.kd);
  }
}

/////////////////////////////////////////////////
// The ordering every reader of the table expects: oceanic waters clearer
// than coastal, and each coastal type murkier than the one before.
TEST(Jerlov, TypesGetMurkier)
{
  const auto &types = water_optics::JerlovTypes();
  for (std::size_t i = 1; i < types.size(); ++i)
  {
    const double prev = types[i - 1].absorption.Y() +
                        types[i - 1].scattering.Y();
    const double cur = types[i].absorption.Y() + types[i].scattering.Y();
    // 1C is clearer than III in the green: Jerlov's series are two.
    if (types[i].name == "1C")
      continue;
    EXPECT_GT(cur, prev) << types[i].name << " after " << types[i - 1].name;
  }
}

/////////////////////////////////////////////////
TEST(Jerlov, LookupIsCaseInsensitive)
{
  ASSERT_TRUE(water_optics::Jerlov("3c"));
  EXPECT_EQ(water_optics::Jerlov("3c")->name, "3C");
  ASSERT_TRUE(water_optics::Jerlov("ib"));
  EXPECT_FALSE(water_optics::Jerlov("4C"));
  EXPECT_FALSE(water_optics::Jerlov(""));
}

/////////////////////////////////////////////////
// From a type: c = a + b, veil = b / c, K_d as tabled.
TEST(Resolve, DerivesFromType)
{
  for (const std::string name : {"I", "II", "3C", "9C"})
  {
    const auto type = water_optics::Jerlov(name);
    ASSERT_TRUE(type);
    WaterOpticsSpec spec;
    spec.jerlov = name;
    const auto out = MustResolve(spec);
    const Rgb c = type->absorption + type->scattering;
    EXPECT_EQ(out.jerlov, name);
    ExpectRgb(out.attenuation, c);
    ExpectRgb(out.backscatter, c);
    ExpectRgb(out.veil, Div(type->scattering, c));
    ExpectRgb(out.diffuseAttenuation, type->diffuseAttenuation);
    EXPECT_NEAR(out.Visibility(), 4.8 / c.Y(), kTol);
  }
}

/////////////////////////////////////////////////
// A hand-computed case: 3C, green, c = 0.078 + 1.15 = 1.228 1/m, so its
// visibility is 4.8 / 1.228 = 3.909 m and its veil 1.15 / 1.228.
TEST(Resolve, HandComputed3C)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  const auto out = MustResolve(spec);
  EXPECT_NEAR(out.attenuation.Y(), 1.228, 1e-12);
  EXPECT_NEAR(out.Visibility(), 3.908794788, 1e-6);
  EXPECT_NEAR(out.veil.Y(), 0.936482085, 1e-6);
}

/////////////////////////////////////////////////
TEST(Resolve, DefaultsToCoastal1C)
{
  const auto out = MustResolve(WaterOpticsSpec{});
  EXPECT_EQ(out.jerlov, "1C");
  WaterOpticsSpec coastal;
  coastal.jerlov = "1C";
  ExpectRgb(out.attenuation, MustResolve(coastal).attenuation);
}

/////////////////////////////////////////////////
// Visibility sets the strength: 4.8 / c_green comes out as asked, for
// several types and visibilities.
TEST(Resolve, VisibilitySetsStrength)
{
  for (const std::string name : {"IB", "3C", "9C"})
  {
    for (const double v : {1.0, 5.0, 20.0})
    {
      WaterOpticsSpec spec;
      spec.jerlov = name;
      spec.visibility = v;
      EXPECT_NEAR(MustResolve(spec).Visibility(), v, 1e-9)
          << name << " at " << v << " m";
    }
  }
}

/////////////////////////////////////////////////
// Visibility scales what the type has above pure water: at the type's own
// visibility nothing changes, and clearer water comes closer to pure water,
// which is bluer.
TEST(Resolve, VisibilityScalesAbovePureWater)
{
  for (const std::string name : {"IB", "II", "3C", "9C"})
  {
    WaterOpticsSpec plain;
    plain.jerlov = name;
    const auto a = MustResolve(plain);
    WaterOpticsSpec same = plain;
    same.visibility = a.Visibility();
    const auto b = MustResolve(same);
    ExpectRgb(b.attenuation, a.attenuation, 1e-9);
    ExpectRgb(b.veil, a.veil, 1e-9);
    ExpectRgb(b.diffuseAttenuation, a.diffuseAttenuation, 1e-9);

    // Clearer: red fades faster than green, relative to the type
    WaterOpticsSpec clearer = plain;
    clearer.visibility = 1.5 * a.Visibility();
    const auto c = MustResolve(clearer);

    // K_d's excess over pure water shrinks by the same factor as c's
    const auto pure = *water_optics::Jerlov("I");
    const Rgb cPure = pure.absorption + pure.scattering;
    const Rgb s = Div(c.attenuation - cPure, a.attenuation - cPure);
    ExpectRgb(Div(c.diffuseAttenuation - pure.diffuseAttenuation,
                  a.diffuseAttenuation - pure.diffuseAttenuation), s, 1e-9);
    EXPECT_LT(s.Y(), 1.0) << name;
    EXPECT_GT(c.attenuation.X() / c.attenuation.Y(),
              a.attenuation.X() / a.attenuation.Y()) << name;
  }

  // As clear as pure water: type I
  WaterOpticsSpec pure;
  pure.jerlov = "I";
  const auto i = MustResolve(pure);
  WaterOpticsSpec clearest;
  clearest.jerlov = "3C";
  clearest.visibility = i.Visibility();
  ExpectRgb(MustResolve(clearest).attenuation, i.attenuation, 1e-9);
}

/////////////////////////////////////////////////
// Nothing is clearer than pure water, and pure water cannot be scaled.
TEST(Resolve, VisibilityNoClearerThanPureWater)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.visibility = 150.0;
  EXPECT_NE(MustFail(spec).find("pure water"), std::string::npos);

  spec.jerlov = "I";
  spec.visibility = 50.0;
  EXPECT_NE(MustFail(spec).find("pure water"), std::string::npos);
}

/////////////////////////////////////////////////
TEST(Resolve, VisibilityAloneScales1C)
{
  WaterOpticsSpec spec;
  spec.visibility = 8.0;
  const auto out = MustResolve(spec);
  EXPECT_EQ(out.jerlov, "1C");
  EXPECT_NEAR(out.Visibility(), 8.0, 1e-9);
}

/////////////////////////////////////////////////
// Each explicit coefficient replaces its derived value, and only that one.
TEST(Resolve, ExplicitOverridesTagByTag)
{
  WaterOpticsSpec base;
  base.jerlov = "3C";
  const auto ref = MustResolve(base);
  const Rgb x(0.5, 0.25, 0.125);

  WaterOpticsSpec s = base;
  s.attenuation = x;
  auto out = MustResolve(s);
  ExpectRgb(out.attenuation, x);
  ExpectRgb(out.backscatter, ref.backscatter);
  ExpectRgb(out.veil, ref.veil);
  ExpectRgb(out.diffuseAttenuation, ref.diffuseAttenuation);

  s = base;
  s.backscatter = x;
  out = MustResolve(s);
  ExpectRgb(out.backscatter, x);
  ExpectRgb(out.attenuation, ref.attenuation);

  s = base;
  s.veil = x;
  out = MustResolve(s);
  ExpectRgb(out.veil, x);
  ExpectRgb(out.attenuation, ref.attenuation);

  s = base;
  s.diffuseAttenuation = x;
  out = MustResolve(s);
  ExpectRgb(out.diffuseAttenuation, x);
  ExpectRgb(out.veil, ref.veil);
}

/////////////////////////////////////////////////
// Visibility with an explicit attenuation: the attenuation wins, with a
// warning, and the visibility comes from it.
TEST(Resolve, ExplicitAttenuationWinsOverVisibility)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.visibility = 5.0;
  spec.attenuation = Rgb(1.0, 0.6, 0.8);
  std::string warning;
  const auto out = MustResolve(spec, &warning);
  ExpectRgb(out.attenuation, Rgb(1.0, 0.6, 0.8));
  EXPECT_NEAR(out.Visibility(), 8.0, 1e-12);
  EXPECT_NE(warning.find("visibility"), std::string::npos) << warning;

  WaterOpticsSpec quiet;
  quiet.jerlov = "3C";
  quiet.visibility = 5.0;
  MustResolve(quiet, &warning);
  EXPECT_TRUE(warning.empty()) << warning;
}

/////////////////////////////////////////////////
TEST(Resolve, RejectsBadInput)
{
  WaterOpticsSpec s;
  s.jerlov = "4C";
  EXPECT_NE(MustFail(s).find("4C"), std::string::npos);

  for (const double v : {0.0, -1.0, std::nan(""),
                         std::numeric_limits<double>::infinity(), 0.05,
                         1e-300, 1e-320})
  {
    s = WaterOpticsSpec{};
    s.visibility = v;
    EXPECT_NE(MustFail(s).find("visibility"), std::string::npos) << v;
  }

  s = WaterOpticsSpec{};
  s.attenuation = Rgb(0.1, -0.1, 0.1);
  EXPECT_NE(MustFail(s).find("attenuation"), std::string::npos);

  s = WaterOpticsSpec{};
  s.veil = Rgb(0.1, std::nan(""), 0.1);
  EXPECT_NE(MustFail(s).find("veil"), std::string::npos);

  s = WaterOpticsSpec{};
  s.diffuseAttenuation = Rgb(0.1, 0.1, -2);
  EXPECT_NE(MustFail(s).find("diffuse_attenuation"), std::string::npos);

  s = WaterOpticsSpec{};
  s.waterLevel = std::nan("");
  EXPECT_NE(MustFail(s).find("water_level"), std::string::npos);
}

/////////////////////////////////////////////////
TEST(Resolve, KeepsGeneration)
{
  WaterOpticsData out;
  out.generation = 7;
  std::string error, warning;
  ASSERT_TRUE(water_optics::Resolve(WaterOpticsSpec{}, out, error, warning));
  EXPECT_EQ(out.generation, 7u);
}

/////////////////////////////////////////////////
TEST(Serialization, RoundTrips)
{
  WaterOpticsSpec spec;
  spec.jerlov = "5C";
  spec.visibility = 3.3;
  spec.waterLevel = -1.25;
  auto in = MustResolve(spec);
  in.generation = 42;
  std::stringstream ss;
  ss << in;
  WaterOpticsData out;
  ss >> out;
  EXPECT_EQ(out.jerlov, in.jerlov);
  EXPECT_EQ(out.generation, in.generation);
  EXPECT_DOUBLE_EQ(out.waterLevel, in.waterLevel);
  for (auto [a, b] : {std::pair{&out.attenuation, &in.attenuation},
                      {&out.backscatter, &in.backscatter},
                      {&out.veil, &in.veil},
                      {&out.diffuseAttenuation, &in.diffuseAttenuation}})
  {
    EXPECT_EQ(*a, *b);
  }
}

/////////////////////////////////////////////////
// The sampler on a world without water optics: clear water.
TEST(Sampler, ClearWithoutOptics)
{
  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());

  water_optics::WaterOpticsSampler sampler;
  EXPECT_FALSE(sampler.Sync(ecm));
  EXPECT_FALSE(sampler.Valid());
  ExpectRgb(sampler.Veil(10.0), Rgb::Zero);
  ExpectRgb(sampler.Data().attenuation, Rgb::Zero);
  EXPECT_DOUBLE_EQ(sampler.Depth({0, 0, -4}, std::chrono::seconds(1)), 4.0);
}

/////////////////////////////////////////////////
// The sampler's closed forms, at 0, 5 and 20 m, on flat water with a level.
TEST(Sampler, ClosedForms)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.waterLevel = 2.0;
  const auto optics = MustResolve(spec);

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::WaterOptics(optics));

  water_optics::WaterOpticsSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_EQ(sampler.Data().jerlov, "3C");

  const auto t = std::chrono::seconds(3);
  EXPECT_DOUBLE_EQ(sampler.Depth({1, 2, -3}, t), 5.0);
  EXPECT_DOUBLE_EQ(sampler.Depth({1, 2, 2.5}, t), -0.5);

  const auto fade = [](const Rgb &_k, double _x)
  {
    return Rgb(std::exp(-_k.X() * _x), std::exp(-_k.Y() * _x),
               std::exp(-_k.Z() * _x));
  };
  for (const double x : {0.0, 5.0, 20.0})
  {
    const Rgb light = fade(optics.diffuseAttenuation, x);
    ExpectRgb(sampler.Veil(x),
              Rgb(optics.veil.X() * light.X(), optics.veil.Y() * light.Y(),
                  optics.veil.Z() * light.Z()));
  }

  // Above the surface: no veil.
  ExpectRgb(sampler.Veil(-1.0), Rgb::Zero);
}

/////////////////////////////////////////////////
// The sampler copies the optics when their generation changes, not on
// every Sync.
TEST(Sampler, CopiesOnANewGeneration)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  auto optics = MustResolve(spec);
  optics.generation = 1;

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::WaterOptics(optics));
  water_optics::WaterOpticsSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));

  // Other water, the same generation: not read
  auto other = optics;
  other.waterLevel = 3.0;
  ecm.SetComponentData<components::WaterOptics>(world, other);
  sampler.Sync(ecm);
  EXPECT_DOUBLE_EQ(sampler.Data().waterLevel, 0.0);

  // A new generation: read
  other.generation = 2;
  ecm.SetComponentData<components::WaterOptics>(world, other);
  sampler.Sync(ecm);
  EXPECT_DOUBLE_EQ(sampler.Data().waterLevel, 3.0);

  // The optics gone: clear water
  ecm.RemoveComponent<components::WaterOptics>(world);
  EXPECT_FALSE(sampler.Sync(ecm));
  ExpectRgb(sampler.Data().attenuation, Rgb::Zero);
}

namespace
{
/// \brief The token of the test's wave engine.
constexpr char kTestEngine[] = "water_optics_test_wave";

/// \brief A wave engine for the test: a sine travelling along x, its
/// amplitude the recipe's gain.
class SineWave : public waves::IWaveField
{
  public: double Elevation(double _x, double, double) const override
  {
    return this->amplitude * std::sin(0.5 * _x - this->time);
  }

  public: math::Vector3d ParticleVelocity(double, double, double) const
      override
  {
    return math::Vector3d::Zero;
  }

  public: math::Vector3d Normal(double, double, double) const override
  {
    return math::Vector3d::UnitZ;
  }

  public: double Jacobian(double, double, double) const override
  {
    return 1.0;
  }

  public: void SetParameters(const waves::WaveParameters &_params) override
  {
    this->amplitude = _params.gain;
  }

  public: void Update(double _simTime) override { this->time = _simTime; }

  public: std::string_view Kind() const override { return kTestEngine; }

  public: const waves::WaveField2D *Field() const override { return nullptr; }

  private: double amplitude{0.0};
  private: double time{0.0};
};
}  // namespace

/////////////////////////////////////////////////
// Where the wave field's recipe is replicated without its engine, as in
// the GUI or a sensor process, the sampler builds an engine of its own:
// depth below the wave, the same as with the source's engine.
TEST(Sampler, DepthBelowTheWaveFromTheRecipeAlone)
{
  waves::RegisterWaveEngineFactory(kTestEngine,
      [] { return std::make_shared<SineWave>(); });
  waves::WavefieldData recipe;
  recipe.algorithm = kTestEngine;
  recipe.params.gain = 0.8;
  recipe.generation = 1;

  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.waterLevel = 1.0;
  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::WaterOptics(MustResolve(spec)));
  ecm.CreateComponent(world, components::Wavefield(recipe));

  // The source's engine, as the server has it
  waves::WavefieldData source = recipe;
  source.simulation =
      waves::CreateWaveSimulation(recipe.algorithm, recipe.params);
  ASSERT_NE(nullptr, source.simulation);

  water_optics::WaterOpticsSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  for (const double t : {0.0, 1.5, 4.0})
  {
    waves::Advance(source, t);
    const auto time = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(t));
    for (const double x : {0.0, 2.0, 5.0})
    {
      const double eta = waves::SurfaceElevation(source, x, 0.0, t);
      EXPECT_NEAR(sampler.Depth({x, 0.0, -2.0}, time), 1.0 + eta + 2.0,
                  1e-12) << "t " << t << ", x " << x;
    }
  }

  // A new recipe: a new engine
  recipe.params.gain = 0.2;
  recipe.generation = 2;
  ecm.SetComponentData<components::Wavefield>(world, recipe);
  sampler.Sync(ecm);
  const double eta = 0.2 * std::sin(0.5 * 2.0 - 1.5);
  EXPECT_NEAR(sampler.Depth({2.0, 0.0, -2.0}, std::chrono::milliseconds(1500)),
              1.0 + eta + 2.0, 1e-12);
}

/////////////////////////////////////////////////
// Sync on one thread, as a system's PreUpdate, while another asks, as a
// renderer: the other thread never reads the source's engine, which the
// first advances, but an engine of its own, and gets the wave at the time
// it asks for. Run under the thread sanitizer, this is where a race shows.
TEST(Sampler, AskedFromAnotherThread)
{
  std::atomic<int> built{0};
  waves::RegisterWaveEngineFactory(kTestEngine,
      [&built] { ++built; return std::make_shared<SineWave>(); });
  waves::WavefieldData recipe;
  recipe.algorithm = kTestEngine;
  recipe.params.gain = 0.8;
  recipe.generation = 1;
  // The source's engine is in the component, as on the server
  recipe.simulation =
      waves::CreateWaveSimulation(recipe.algorithm, recipe.params);
  built = 0;

  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.waterLevel = 1.0;
  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::WaterOptics(MustResolve(spec)));
  ecm.CreateComponent(world, components::Wavefield(recipe));

  water_optics::WaterOpticsSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  EXPECT_EQ(0, built) << "an engine built before a depth was asked for";

  std::atomic<bool> stop{false};
  std::atomic<int> asked{0};
  std::thread renderer([&]
  {
    for (int i = 0; !stop; ++i)
    {
      const double t = 0.01 * (i % 300);
      const auto time = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(t));
      const double depth = sampler.Depth({2.0, 0.0, -2.0}, time);
      EXPECT_NEAR(depth, 1.0 + 0.8 * std::sin(1.0 - t) + 2.0, 1e-8)
          << "t " << t;
      EXPECT_EQ(sampler.Data().jerlov, "3C");
      EXPECT_TRUE(sampler.Valid());
      EXPECT_GT(sampler.Veil(depth).Y(), 0.0);
      ++asked;
    }
  });

  // The simulation: the source advances, the optics change
  auto optics = MustResolve(spec);
  for (int step = 0; step < 2000 || asked < 100; ++step)
  {
    waves::Advance(ecm.Component<components::Wavefield>(world)->Data(),
                   0.001 * step);
    if (step % 100 == 0)
    {
      optics.generation = 2 + step;
      ecm.SetComponentData<components::WaterOptics>(world, optics);
    }
    sampler.Sync(ecm);
    // The Sync thread reads the source's engine, at its own time
    const double t = 0.001 * step;
    EXPECT_NEAR(sampler.Depth({2.0, 0.0, -2.0},
                    std::chrono::duration_cast<
                        std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(t))),
                1.0 + 0.8 * std::sin(1.0 - t) + 2.0, 1e-9);
  }
  stop = true;
  renderer.join();
  EXPECT_GT(asked, 100);
  EXPECT_EQ(1, built) << "the other thread builds one engine of its own";
}

/////////////////////////////////////////////////
// An engine whose registrar loads after the first depth is asked for is
// picked up when it comes.
TEST(Sampler, EngineRegisteredLater)
{
  constexpr char kLate[] = "water_optics_test_late_wave";
  waves::WavefieldData recipe;
  recipe.algorithm = kLate;
  recipe.params.gain = 0.5;
  recipe.generation = 1;

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::WaterOptics(MustResolve(WaterOpticsSpec{})));
  ecm.CreateComponent(world, components::Wavefield(recipe));

  water_optics::WaterOpticsSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  const auto t = std::chrono::seconds(1);
  const math::Vector3d at(3.0, 0.0, -2.0);
  EXPECT_DOUBLE_EQ(sampler.Depth(at, t), 2.0)
      << "no engine yet: below the still water";

  waves::RegisterWaveEngineFactory(kLate,
      [] { return std::make_shared<SineWave>(); });
  double depth = 2.0;
  for (int i = 0; i < 30 && depth == 2.0; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    depth = sampler.Depth(at, t);
  }
  EXPECT_NEAR(depth, 0.5 * std::sin(1.5 - 1.0) + 2.0, 1e-12);
}

/////////////////////////////////////////////////
// All four coefficients explicit: no type is used, and none is reported.
TEST(Resolve, FullyExplicitUsesNoType)
{
  WaterOpticsSpec spec;
  spec.attenuation = Rgb(1.0, 0.5, 0.25);
  spec.backscatter = Rgb(1.0, 0.5, 0.25);
  spec.veil = Rgb(0.1, 0.2, 0.3);
  spec.diffuseAttenuation = Rgb(0.2, 0.1, 0.05);
  std::string warning;
  auto out = MustResolve(spec, &warning);
  EXPECT_EQ(out.jerlov, "");
  EXPECT_TRUE(warning.empty()) << warning;

  // A type asked for, even an unknown one, is ignored, with a warning
  spec.jerlov = "4C";
  out = MustResolve(spec, &warning);
  EXPECT_EQ(out.jerlov, "");
  ExpectRgb(out.attenuation, Rgb(1.0, 0.5, 0.25));
  EXPECT_NE(warning.find("jerlov"), std::string::npos) << warning;

  // So is a visibility, even one out of range
  spec.jerlov.clear();
  for (const double v : {8.0, 150.0, -1.0})
  {
    spec.visibility = v;
    out = MustResolve(spec, &warning);
    ExpectRgb(out.attenuation, Rgb(1.0, 0.5, 0.25));
    EXPECT_NE(warning.find("visibility"), std::string::npos) << warning;
  }
}

/////////////////////////////////////////////////
TEST(Params, SameWater)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  auto a = MustResolve(spec);
  auto b = a;
  b.generation = a.generation + 5;
  EXPECT_TRUE(water_optics::SameWater(a, b));
  for (const auto &param : water_optics::ColourParams())
  {
    auto c = a;
    c.*param.data += Rgb(0.0, 1e-9, 0.0);
    EXPECT_FALSE(water_optics::SameWater(a, c)) << param.name;
  }
  b.waterLevel += 0.1;
  EXPECT_FALSE(water_optics::SameWater(a, b));
}

/////////////////////////////////////////////////
// Every element the plugin reads, and every preset, is in the reference
// docs and the README, so the docs cannot fall behind the plugin.
TEST(Docs, EveryElementAndPresetDocumented)
{
  const auto read = [](const std::string &_path)
  {
    std::ifstream in(_path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
  };
  const std::string reference = read(DOCS_REFERENCE);
  const std::string readme = read(PACKAGE_README);
  ASSERT_FALSE(reference.empty()) << DOCS_REFERENCE;
  ASSERT_FALSE(readme.empty()) << PACKAGE_README;
  std::vector<std::string> tags{"jerlov", "publish_rate"};
  for (const auto &param : water_optics::NumberParams())
    tags.push_back(param.name);
  for (const auto &param : water_optics::ColourParams())
    tags.push_back(param.name);
  // The set topic's keys, in the reference's "Water optics" section
  const auto section = reference.find("## Water optics\n");
  ASSERT_NE(section, std::string::npos);
  const std::string topic = reference.substr(section,
      reference.find("\n## ", section + 1) - section);
  for (const auto &tag : tags)
  {
    if (tag != "publish_rate")
    {
      EXPECT_NE(topic.find("`" + tag + "`"), std::string::npos)
          << "'" << tag << "' is not among the set topic's keys in "
          << DOCS_REFERENCE;
    }
  }
  for (const auto &tag : tags)
  {
    EXPECT_NE(reference.find("`<" + tag + ">`"), std::string::npos)
        << "<" << tag << "> is not in " << DOCS_REFERENCE;
    EXPECT_NE(readme.find("`<" + tag + ">`"), std::string::npos)
        << "<" << tag << "> is not in " << PACKAGE_README;
  }
  for (const auto &type : water_optics::JerlovTypes())
  {
    EXPECT_NE(reference.find("| `" + type.name + "` |"), std::string::npos)
        << "type " << type.name << " is not in the reference's table";
  }
  for (const std::string topic : {"water_optics/set", "water_optics_info"})
  {
    EXPECT_NE(reference.find(topic), std::string::npos) << topic;
    EXPECT_NE(readme.find(topic), std::string::npos) << topic;
  }
}

/////////////////////////////////////////////////
// Daylight scales the veil, and only the veil; night has none.
TEST(Sampler, DaylightScalesTheVeil)
{
  WaterOpticsSpec spec;
  spec.jerlov = "3C";
  spec.daylight = 0.25;
  const auto optics = MustResolve(spec);
  EXPECT_DOUBLE_EQ(optics.daylight, 0.25);

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::WaterOptics(optics));
  water_optics::WaterOpticsSampler sampler;
  ASSERT_TRUE(sampler.Sync(ecm));
  const Rgb &k = optics.diffuseAttenuation;
  ExpectRgb(sampler.Veil(5.0),
            Rgb(optics.veil.X() * std::exp(-k.X() * 5.0) * 0.25,
                optics.veil.Y() * std::exp(-k.Y() * 5.0) * 0.25,
                optics.veil.Z() * std::exp(-k.Z() * 5.0) * 0.25));

  spec.daylight = 0.0;
  auto night = MustResolve(spec);
  night.generation = optics.generation + 1;
  ecm.SetComponentData<components::WaterOptics>(world, night);
  sampler.Sync(ecm);
  ExpectRgb(sampler.Veil(5.0), Rgb::Zero);

  spec.daylight = -1.0;
  EXPECT_NE(MustFail(spec).find("daylight"), std::string::npos);
}
