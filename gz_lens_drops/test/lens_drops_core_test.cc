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

// The drop model without a world or a renderer: the seeded draws, the
// parameters made safe, the rate against the rain, the lens never holding
// more than it can draw, drying, sliding, merging, a paused step, a seed
// repeating, clearing, and the packing for the shader.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gz/sim/lens_drops/Drops.hh"

using namespace gz::sim::lens_drops;

namespace
{
/// \brief Parameters with a rain rate and a drying time.
/// \param[in] _rain mm/h.
/// \param[in] _dry Seconds a drop lasts.
/// \return The parameters.
Params Rain(double _rain, double _dry = 20.0)
{
  Params p;
  p.rainRate = _rain;
  p.dryTime = _dry;
  return p;
}

/// \brief Step a lens a number of times.
/// \param[in,out] _drops The lens.
/// \param[in] _n How many steps.
/// \param[in] _dt Seconds per step.
void Advance(Drops &_drops, int _n, double _dt)
{
  for (int i = 0; i < _n; ++i)
    _drops.Step(_dt);
}

/// \brief Expect two lists of drops to be equal, field by field.
/// \param[in] _a One list.
/// \param[in] _b The other.
void ExpectSame(const std::vector<Drop> &_a, const std::vector<Drop> &_b)
{
  ASSERT_EQ(_a.size(), _b.size());
  for (std::size_t i = 0; i < _a.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(_a[i].x, _b[i].x);
    EXPECT_DOUBLE_EQ(_a[i].y, _b[i].y);
    EXPECT_DOUBLE_EQ(_a[i].radius, _b[i].radius);
    EXPECT_DOUBLE_EQ(_a[i].age, _b[i].age);
  }
}
}  // namespace

//////////////////////////////////////////////////
TEST(LensDropsCore, HashIsFnv1a)
{
  EXPECT_EQ(Hash(""), 2166136261u);
  EXPECT_EQ(Hash("a"), 0xE40C292Cu);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, UniformIsOpenUnit)
{
  Generator rng(7);
  double sum = 0.0;
  const int n = 10000;
  for (int i = 0; i < n; ++i)
  {
    const double u = Uniform(rng);
    ASSERT_GT(u, 0.0);
    ASSERT_LT(u, 1.0);
    sum += u;
  }
  EXPECT_NEAR(sum / n, 0.5, 0.02);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, SanitizeClampsAndReplaces)
{
  Params p;
  p.rainRate = -5.0;
  p.exposure = 2.0;
  p.focus = std::numeric_limits<double>::quiet_NaN();
  p.dryTime = 0.0;
  p.aspect = std::numeric_limits<double>::infinity();
  const Params s = Sanitize(p);
  EXPECT_DOUBLE_EQ(s.rainRate, 0.0);
  EXPECT_DOUBLE_EQ(s.exposure, 1.0);
  EXPECT_DOUBLE_EQ(s.focus, 0.5);
  EXPECT_DOUBLE_EQ(s.dryTime, 0.1);
  EXPECT_DOUBLE_EQ(s.aspect, 4.0 / 3.0);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, NoRainNoDrops)
{
  Drops drops(7, Rain(0.0));
  Advance(drops, 1000, 0.1);
  EXPECT_TRUE(drops.List().empty());
  EXPECT_EQ(drops.Spawned(), 0u);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, RateFollowsRain)
{
  // 2.5 mm/h at flux 1, with the splash floor, is 1.1 drops a second.
  Drops open(7, Rain(2.5));
  Advance(open, 10000, 0.1);
  EXPECT_GE(open.Spawned(), 990u);
  EXPECT_LE(open.Spawned(), 1210u);

  Params half = Rain(2.5);
  half.exposure = 0.5;
  Drops sheltered(7, half);
  Advance(sheltered, 10000, 0.1);
  EXPECT_GE(sheltered.Spawned(), 470u);
  EXPECT_LE(sheltered.Spawned(), 630u);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, NeverMoreThanMax)
{
  Drops drops(7, Rain(1000.0, 3600.0));
  for (int i = 0; i < 4000; ++i)
  {
    drops.Step(0.05);
    ASSERT_LE(drops.List().size(), kMaxDrops);
  }
  EXPECT_GT(drops.Spawned(), kMaxDrops);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, DropsDry)
{
  Drops drops(7, Rain(20.0, 5.0));
  Advance(drops, 200, 0.1);
  EXPECT_GT(drops.Spawned(), 0u);
  drops.SetParams(Rain(0.0, 5.0));
  Advance(drops, 60, 0.1);
  EXPECT_TRUE(drops.List().empty());
}

//////////////////////////////////////////////////
TEST(LensDropsCore, ShownRadiusShrinks)
{
  Drop d{0.0, 0.0, 0.04, 0.0};
  EXPECT_DOUBLE_EQ(ShownRadius(d, 20.0), 0.04);
  d.age = 15.0;
  EXPECT_NEAR(ShownRadius(d, 20.0), 0.02, 1e-12);
  d.age = 20.0;
  EXPECT_DOUBLE_EQ(ShownRadius(d, 20.0), 0.0);
  d.age = 25.0;
  EXPECT_DOUBLE_EQ(ShownRadius(d, 20.0), 0.0);
  d.age = 0.0;
  EXPECT_DOUBLE_EQ(ShownRadius(d, 0.0), 0.0);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, BigDropsSlideOut)
{
  const double kSlideRadius = Params().slideRadius;
  Drops drops(7, Rain(200.0, 3600.0));
  bool big = false;
  for (int i = 0; i < 400; ++i)
  {
    drops.Step(0.1);
    for (const Drop &d : drops.List())
      big = big || d.radius >= kSlideRadius;
  }
  EXPECT_TRUE(big) << "the rain made no drop big enough to slide";

  drops.SetParams(Rain(0.0, 3600.0));
  Advance(drops, 4000, 0.1);
  for (const Drop &d : drops.List())
    EXPECT_LT(d.radius, kSlideRadius);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, OverlappingDropsMerge)
{
  Drops drops(7, Rain(1000.0, 3600.0));
  const double aspect = drops.Parameters().aspect;
  for (int step = 0; step < 2000; ++step)
  {
    drops.Step(0.05);
    // Every pair is O(n^2) on up to kMaxDrops drops: check now and then.
    if (step % 50 != 49)
      continue;
    const std::vector<Drop> &list = drops.List();
    for (std::size_t i = 0; i < list.size(); ++i)
    {
      ASSERT_LE(list[i].radius, kMaxMergedRadius);
      for (std::size_t j = i + 1; j < list.size(); ++j)
      {
        const double dx = (list[i].x - list[j].x) * aspect;
        const double dy = list[i].y - list[j].y;
        ASSERT_GE(std::sqrt(dx * dx + dy * dy),
                  std::max(list[i].radius, list[j].radius))
            << "drops " << i << " and " << j << " overlap at step " << step;
      }
    }
  }
}

//////////////////////////////////////////////////
TEST(LensDropsCore, PausedStepsDoNothing)
{
  Drops drops(7, Rain(100.0));
  Advance(drops, 100, 0.1);
  ASSERT_FALSE(drops.List().empty());
  const std::vector<Drop> before = drops.List();
  const double time = drops.Time();
  const auto spawned = drops.Spawned();

  drops.Step(0.0);
  drops.Step(-1.0);
  drops.Step(std::numeric_limits<double>::quiet_NaN());

  ExpectSame(drops.List(), before);
  EXPECT_DOUBLE_EQ(drops.Time(), time);
  EXPECT_EQ(drops.Spawned(), spawned);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, SeedRepeats)
{
  Drops a(42, Rain(100.0));
  Drops b(42, Rain(100.0));
  Drops c(43, Rain(100.0));
  Advance(a, 300, 0.1);
  Advance(b, 300, 0.1);
  Advance(c, 300, 0.1);
  ASSERT_FALSE(a.List().empty());
  ExpectSame(a.List(), b.List());

  bool differs = a.List().size() != c.List().size();
  for (std::size_t i = 0; !differs && i < a.List().size(); ++i)
  {
    differs = a.List()[i].x != c.List()[i].x ||
              a.List()[i].y != c.List()[i].y ||
              a.List()[i].radius != c.List()[i].radius;
  }
  EXPECT_TRUE(differs);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, ClearAndReset)
{
  Drops reference(42, Rain(100.0));
  Advance(reference, 300, 0.1);

  Drops drops(9, Rain(100.0));
  Advance(drops, 100, 0.1);
  ASSERT_FALSE(drops.List().empty());
  const double time = drops.Time();
  const auto spawned = drops.Spawned();

  drops.Clear();
  EXPECT_TRUE(drops.List().empty());
  EXPECT_DOUBLE_EQ(drops.Time(), time);
  EXPECT_EQ(drops.Spawned(), spawned);

  drops.Reset(42);
  EXPECT_DOUBLE_EQ(drops.Time(), 0.0);
  EXPECT_EQ(drops.Spawned(), 0u);
  Advance(drops, 300, 0.1);
  ExpectSame(drops.List(), reference.List());
}

//////////////////////////////////////////////////
TEST(LensDropsCore, PackWritesFourFloats)
{
  std::array<float, kMaxDrops * kDropFloats> buf;
  buf.fill(-1.0f);

  Drops drops(7, Rain(100.0));
  EXPECT_EQ(drops.Pack(nullptr, kMaxDrops), 0u);
  Advance(drops, 100, 0.1);
  ASSERT_GE(drops.List().size(), 1u);

  EXPECT_EQ(drops.Pack(buf.data(), 0), 0u);
  EXPECT_FLOAT_EQ(buf[0], -1.0f);

  const std::size_t n = drops.Pack(buf.data(), 2);
  EXPECT_EQ(n, std::min<std::size_t>(2, drops.List().size()));
  for (std::size_t i = 0; i < n; ++i)
  {
    const Drop &d = drops.List()[i];
    EXPECT_FLOAT_EQ(buf[kDropFloats * i + 0], static_cast<float>(d.x));
    EXPECT_FLOAT_EQ(buf[kDropFloats * i + 1], static_cast<float>(d.y));
    EXPECT_FLOAT_EQ(buf[kDropFloats * i + 2],
        static_cast<float>(ShownRadius(d, drops.Parameters().dryTime)));
    EXPECT_FLOAT_EQ(buf[kDropFloats * i + 3], 0.0f);
  }
  for (std::size_t k = kDropFloats * n; k < buf.size(); ++k)
    EXPECT_FLOAT_EQ(buf[k], -1.0f);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, LongStepIsCapped)
{
  Drops drops(7, Rain(10.0));
  drops.Step(0.5);
  const double before = drops.Time();
  drops.Step(100.0);
  EXPECT_DOUBLE_EQ(drops.Time() - before, kMaxStep);
}

namespace
{
/// \brief An environment.
/// \param[in] _gx Gravity to the image's right, g.
/// \param[in] _gy Gravity down the image, g.
/// \param[in] _ax Air to the image's right, m/s.
/// \param[in] _flux Rain flux factor.
/// \return The environment.
Environment Env(double _gx, double _gy, double _ax = 0.0, double _flux = 1.0)
{
  Environment e;
  e.gravityX = _gx;
  e.gravityY = _gy;
  e.airX = _ax;
  e.flux = _flux;
  return e;
}

/// \brief A drop to Add.
/// \param[in] _x Centre, width fraction.
/// \param[in] _y Centre, height fraction.
/// \param[in] _r Radius, image heights.
/// \return The drop.
Drop At(double _x, double _y, double _r)
{
  Drop d;
  d.x = _x;
  d.y = _y;
  d.radius = _r;
  return d;
}

/// \brief The water on a lens.
/// \param[in] _drops The lens.
/// \return The sum of the drops' volumes.
double Water(const Drops &_drops)
{
  double sum = 0.0;
  for (const Drop &d : _drops.List())
    sum += Volume(d);
  return sum;
}
}  // namespace

//////////////////////////////////////////////////
TEST(LensDropsPhysics, PinnedBelowThreshold)
{
  Drops drops(7, Rain(0.0));
  drops.Add(At(0.5, 0.5, 0.5 * Params().slideRadius));
  Advance(drops, 100, 0.1);
  ASSERT_EQ(drops.List().size(), 1u);
  EXPECT_DOUBLE_EQ(drops.List()[0].x, 0.5);
  EXPECT_DOUBLE_EQ(drops.List()[0].y, 0.5);
  EXPECT_FALSE(drops.List()[0].sliding);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, SlidesDownTheWorld)
{
  // The camera rolled a quarter turn: down is the image's right.
  Drops drops(7, Rain(0.0));
  drops.SetEnvironment(Env(1.0, 0.0));
  drops.Add(At(0.3, 0.5, 2.0 * Params().slideRadius));
  Advance(drops, 10, 0.1);
  ASSERT_FALSE(drops.List().empty());
  EXPECT_GT(drops.List()[0].x, 0.3);
  EXPECT_LT(std::abs(drops.List()[0].y - 0.5), 0.02);
  EXPECT_GE(drops.Sliding(), 1u);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, FacingUpStaysPut)
{
  Drops drops(7, Rain(0.0));
  drops.SetEnvironment(Env(0.0, 0.0));
  drops.Add(At(0.5, 0.5, 2.0 * Params().slideRadius));
  Advance(drops, 10, 0.1);
  ASSERT_EQ(drops.List().size(), 1u);
  EXPECT_DOUBLE_EQ(drops.List()[0].x, 0.5);
  EXPECT_DOUBLE_EQ(drops.List()[0].y, 0.5);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, AirPushes)
{
  Drops drops(7, Rain(0.0));
  drops.SetEnvironment(Env(0.0, 0.0, 30.0));
  drops.Add(At(0.3, 0.5, 2.0 * Params().slideRadius));
  Advance(drops, 10, 0.1);
  ASSERT_FALSE(drops.List().empty());
  EXPECT_GT(drops.List()[0].x, 0.3);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, VolumeIsConserved)
{
  Drops drops(7, Rain(0.0, 3600.0));
  drops.SetEnvironment(Env(0.0, 1.0));
  for (const double x : {0.3, 0.33, 0.36, 0.6, 0.8})
    drops.Add(At(x, 0.2, 0.06));
  const double water = Water(drops);
  for (int i = 0; i < 20; ++i)
  {
    drops.Step(0.1);
    ASSERT_NEAR(Water(drops), water, 1e-9 * water) << "step " << i;
    // The three that touch are one after the first step; beads come later.
    if (i == 0)
    {
      EXPECT_EQ(drops.List().size(), 3u) << "the touching drops did not merge";
    }
  }
  EXPECT_GE(drops.Sliding(), 1u);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, BeadsAndTrails)
{
  Drops drops(7, Rain(0.0));
  drops.SetEnvironment(Env(0.0, 1.0));
  drops.Add(At(0.5, 0.1, 0.06));
  Advance(drops, 20, 0.1);
  EXPECT_GT(drops.List().size(), 1u) << "no bead";
  EXPECT_FALSE(drops.Trails().empty());

  // A little over half the drying time with nothing sliding: the film is
  // gone.
  drops.SetEnvironment(Env(0.0, 0.0));
  Advance(drops, 105, 0.1);
  EXPECT_TRUE(drops.Trails().empty());
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, FluxScalesRain)
{
  Drops dry(7, Rain(10.0));
  dry.SetEnvironment(Env(0.0, 1.0, 0.0, 0.0));
  Advance(dry, 10000, 0.1);
  const double floor = 10.0 * kDropsPerMmHour * kSplashFloor * 1000.0;
  EXPECT_NEAR(static_cast<double>(dry.Spawned()), floor, 0.1 * floor);

  Drops wet(7, Rain(10.0));
  wet.SetEnvironment(Env(0.0, 1.0, 0.0, 2.0));
  Advance(wet, 10000, 0.1);
  const double heavy = 10.0 * kDropsPerMmHour * (2.0 + kSplashFloor) * 1000.0;
  EXPECT_NEAR(static_cast<double>(wet.Spawned()), heavy, 0.1 * heavy);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, GenerationMoves)
{
  Drops drops(7, Rain(0.0));
  const auto g0 = drops.Generation();
  drops.Step(0.0);
  EXPECT_EQ(drops.Generation(), g0);
  drops.Add(At(0.5, 0.5, 0.01));
  const auto g1 = drops.Generation();
  EXPECT_GT(g1, g0);
  drops.Clear();
  EXPECT_GT(drops.Generation(), g1);
}

//////////////////////////////////////////////////
TEST(LensDropsPhysics, SanitizeEnvironment)
{
  Environment e;
  e.gravityX = std::numeric_limits<double>::quiet_NaN();
  e.flux = std::numeric_limits<double>::infinity();
  const Environment a = Sanitize(e);
  EXPECT_DOUBLE_EQ(a.gravityX, 0.0);
  EXPECT_DOUBLE_EQ(a.gravityY, 1.0);
  EXPECT_DOUBLE_EQ(a.flux, 1.0);

  const Environment b = Sanitize(Env(3.0, 4.0));
  EXPECT_DOUBLE_EQ(b.gravityX, 0.6);
  EXPECT_DOUBLE_EQ(b.gravityY, 0.8);
}
