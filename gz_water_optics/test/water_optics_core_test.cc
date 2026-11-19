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
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/WaterOptics.hh"
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
  const auto ii = water_optics::Jerlov("II");
  ASSERT_TRUE(ii);
  ExpectRgb(ii->absorption, Rgb(0.296, 0.0469, 0.0241));
  ExpectRgb(ii->scattering, Rgb(0.288, 0.387, 0.504));
  ExpectRgb(ii->diffuseAttenuation, Rgb(0.337, 0.0845, 0.062));

  const auto c3 = water_optics::Jerlov("3C");
  ASSERT_TRUE(c3);
  ExpectRgb(c3->absorption, Rgb(0.307, 0.078, 0.154));
  ExpectRgb(c3->scattering, Rgb(0.855, 1.15, 1.50));
  ExpectRgb(c3->diffuseAttenuation, Rgb(0.456, 0.187, 0.319));

  const auto c9 = water_optics::Jerlov("9C");
  ASSERT_TRUE(c9);
  ExpectRgb(c9->absorption, Rgb(0.436, 0.430, 0.943));
  ExpectRgb(c9->scattering, Rgb(2.51, 3.38, 4.39));
  ExpectRgb(c9->diffuseAttenuation, Rgb(0.763, 0.826, 1.56));
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
TEST(Resolve, DefaultsToOceanicII)
{
  const auto out = MustResolve(WaterOpticsSpec{});
  EXPECT_EQ(out.jerlov, "II");
  WaterOpticsSpec ii;
  ii.jerlov = "II";
  ExpectRgb(out.attenuation, MustResolve(ii).attenuation);
}

/////////////////////////////////////////////////
// Visibility sets the strength: 4.8 / c_green comes out as asked, for
// several types and visibilities.
TEST(Resolve, VisibilitySetsStrength)
{
  for (const std::string name : {"I", "3C", "9C"})
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
// Visibility keeps the colour: the ratios between channels, and the veil.
TEST(Resolve, VisibilityKeepsColour)
{
  for (const std::string name : {"I", "II", "3C", "9C"})
  {
    WaterOpticsSpec plain;
    plain.jerlov = name;
    WaterOpticsSpec scaled = plain;
    scaled.visibility = 2.5;
    const auto a = MustResolve(plain);
    const auto b = MustResolve(scaled);
    EXPECT_NEAR(b.attenuation.X() / b.attenuation.Y(),
                a.attenuation.X() / a.attenuation.Y(), 1e-12) << name;
    EXPECT_NEAR(b.attenuation.Z() / b.attenuation.Y(),
                a.attenuation.Z() / a.attenuation.Y(), 1e-12) << name;
    ExpectRgb(b.veil, a.veil, 1e-12);
    EXPECT_NEAR(b.diffuseAttenuation.X() / b.attenuation.X(),
                a.diffuseAttenuation.X() / a.attenuation.X(), 1e-12) << name;
  }
}

/////////////////////////////////////////////////
TEST(Resolve, VisibilityAloneScalesII)
{
  WaterOpticsSpec spec;
  spec.visibility = 8.0;
  const auto out = MustResolve(spec);
  EXPECT_EQ(out.jerlov, "II");
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
                         std::numeric_limits<double>::infinity()})
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
  ExpectRgb(sampler.Transmittance(10.0), Rgb::One);
  ExpectRgb(sampler.VeilBuildUp(10.0), Rgb::Zero);
  ExpectRgb(sampler.Daylight(10.0), Rgb::One);
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
    ExpectRgb(sampler.Daylight(x), fade(optics.diffuseAttenuation, x));
    const Rgb light = fade(optics.diffuseAttenuation, x);
    ExpectRgb(sampler.Veil(x),
              Rgb(optics.veil.X() * light.X(), optics.veil.Y() * light.Y(),
                  optics.veil.Z() * light.Z()));
    ExpectRgb(sampler.Transmittance(x), fade(optics.attenuation, x));
    ExpectRgb(sampler.VeilBuildUp(x),
              Rgb::One - fade(optics.backscatter, x));
  }

  // Above the surface: all the daylight, and no veil.
  ExpectRgb(sampler.Daylight(-1.0), Rgb::One);
  ExpectRgb(sampler.Veil(-1.0), Rgb::Zero);
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
  for (const std::string tag : {"jerlov", "visibility", "attenuation",
                                "backscatter", "veil", "diffuse_attenuation",
                                "water_level", "publish_rate", "daylight"})
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
  const Rgb light = sampler.Daylight(5.0);
  ExpectRgb(sampler.Veil(5.0),
            Rgb(optics.veil.X() * light.X() * 0.25,
                optics.veil.Y() * light.Y() * 0.25,
                optics.veil.Z() * light.Z() * 0.25));
  ExpectRgb(sampler.Daylight(0.0), Rgb::One);

  spec.daylight = 0.0;
  ecm.SetComponentData<components::WaterOptics>(world, MustResolve(spec));
  sampler.Sync(ecm);
  ExpectRgb(sampler.Veil(5.0), Rgb::Zero);

  spec.daylight = -1.0;
  EXPECT_NE(MustFail(spec).find("daylight"), std::string::npos);
}
