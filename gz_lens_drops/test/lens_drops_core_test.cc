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
  // 20 mm/h is one drop a second.
  Drops open(7, Rain(20.0));
  Advance(open, 10000, 0.1);
  EXPECT_GE(open.Spawned(), 900u);
  EXPECT_LE(open.Spawned(), 1100u);

  Params half = Rain(20.0);
  half.exposure = 0.5;
  Drops sheltered(7, half);
  Advance(sheltered, 10000, 0.1);
  EXPECT_GE(sheltered.Spawned(), 420u);
  EXPECT_LE(sheltered.Spawned(), 580u);
}

//////////////////////////////////////////////////
TEST(LensDropsCore, NeverMoreThanMax)
{
  Drops drops(7, Rain(1000.0, 3600.0));
  for (int i = 0; i < 2000; ++i)
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
