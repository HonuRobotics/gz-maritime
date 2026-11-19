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

// The drop map without a renderer: the cap's profile, one drop's area and
// peak, wobbly rims, the blur keeping the water, sliding drops stretched,
// wet trails, the mask, and the time it takes for a full lens.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "gz/sim/lens_drops/DropMap.hh"
#include "gz/sim/lens_drops/Drops.hh"

using namespace gz::sim::lens_drops;

namespace
{
/// \brief Pi.
constexpr double kPi = 3.141592653589793;

/// \brief A lens with a square image and no rain.
/// \param[in] _focus Focus.
/// \return The lens.
Drops Square(double _focus)
{
  Params p;
  p.aspect = 1.0;
  p.focus = _focus;
  p.dryTime = 3600.0;
  return Drops(7, p);
}

/// \brief A drop to Add.
/// \param[in] _x Centre.
/// \param[in] _y Centre.
/// \param[in] _r Radius.
/// \param[in] _wobble Rim wobble.
/// \return The drop.
Drop At(double _x, double _y, double _r, float _wobble = 0.0f)
{
  Drop d;
  d.x = _x;
  d.y = _y;
  d.radius = _r;
  d.wobble = _wobble;
  d.phaseA = 1.0f;
  d.phaseB = 2.0f;
  return d;
}

/// \brief One channel of a texel.
/// \param[in] _map The map.
/// \param[in] _i Column.
/// \param[in] _j Row.
/// \param[in] _c Channel.
/// \return The value.
float Texel(const DropMap &_map, int _i, int _j, int _c)
{
  return _map.Data()[(static_cast<std::size_t>(_j) * _map.Width() + _i) * 4 +
      static_cast<std::size_t>(_c)];
}

/// \brief The sum of one channel.
/// \param[in] _map The map.
/// \param[in] _c Channel.
/// \return The sum.
double Sum(const DropMap &_map, int _c)
{
  double sum = 0.0;
  for (std::size_t k = static_cast<std::size_t>(_c); k < _map.Data().size();
       k += 4)
  {
    sum += _map.Data()[k];
  }
  return sum;
}
}  // namespace

//////////////////////////////////////////////////
TEST(DropMap, RejectsBadSize)
{
  EXPECT_THROW(DropMap(0, 10), std::invalid_argument);
  EXPECT_THROW(DropMap(10, 0), std::invalid_argument);
  EXPECT_THROW(DropMap(5000, 10), std::invalid_argument);
  EXPECT_NO_THROW(DropMap(1, 1));
}

//////////////////////////////////////////////////
TEST(DropMap, CapProfile)
{
  EXPECT_NEAR(CapHeight(0.0, 0.05, kPi / 2.0), 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(CapHeight(0.05, 0.05, 0.96), 0.0);
  EXPECT_DOUBLE_EQ(CapHeight(0.0, 0.0, 0.96), 0.0);
  const double big = 0.05 / std::sin(0.96);
  EXPECT_NEAR(CapHeight(0.0, 0.05, 0.96), big - big * std::cos(0.96), 1e-12);
}

//////////////////////////////////////////////////
TEST(DropMap, EmptyIsZero)
{
  const Drops drops = Square(1.0);
  DropMap map(64, 48);
  map.Render(drops);
  EXPECT_EQ(map.Covered(), 0u);
  EXPECT_DOUBLE_EQ(Sum(map, 0), 0.0);
  EXPECT_DOUBLE_EQ(Sum(map, 1), 0.0);
}

//////////////////////////////////////////////////
TEST(DropMap, OneDropArea)
{
  Drops drops = Square(1.0);
  drops.Add(At(0.5, 0.5, 0.1));
  DropMap map(360, 360);
  map.Render(drops);
  const double area = kPi * 36.0 * 36.0;
  EXPECT_NEAR(static_cast<double>(map.Covered()), area, 0.03 * area);

  int bi = 0;
  int bj = 0;
  float best = -1.0f;
  for (int j = 0; j < 360; ++j)
  {
    for (int i = 0; i < 360; ++i)
    {
      if (Texel(map, i, j, 0) > best)
      {
        best = Texel(map, i, j, 0);
        bi = i;
        bj = j;
      }
    }
  }
  EXPECT_LE(std::abs(bi - 180), 1);
  EXPECT_LE(std::abs(bj - 180), 1);
}

//////////////////////////////////////////////////
TEST(DropMap, WobbleKeepsCentre)
{
  Drops drops = Square(1.0);
  drops.Add(At(0.5, 0.5, 0.1, 0.15f));
  DropMap map(360, 360);
  map.Render(drops);
  const double area = kPi * 36.0 * 36.0;
  EXPECT_NEAR(static_cast<double>(map.Covered()), area, 0.25 * area);
  const float centre = Texel(map, 180, 180, 0);
  const auto &data = map.Data();
  float best = 0.0f;
  for (std::size_t k = 0; k < data.size(); k += 4)
    best = std::max(best, data[k]);
  EXPECT_NEAR(centre, best, 1e-3f);
}

//////////////////////////////////////////////////
TEST(DropMap, BlurConservesWater)
{
  Drops sharp = Square(1.0);
  sharp.Add(At(0.5, 0.5, 0.1));
  Drops blurred = Square(0.0);
  blurred.Add(At(0.5, 0.5, 0.1));
  DropMap a(360, 360);
  DropMap b(360, 360);
  a.Render(sharp);
  b.Render(blurred);
  EXPECT_NEAR(Sum(b, 0), Sum(a, 0), 0.01 * Sum(a, 0));
  // And the edge is soft: more texels hold some water.
  EXPECT_GT(b.Covered(), a.Covered());
}

//////////////////////////////////////////////////
TEST(DropMap, SlidingStretches)
{
  Drops drops = Square(1.0);
  drops.SetEnvironment(Environment());   // gravity down the image
  drops.Add(At(0.5, 0.3, 0.06));
  drops.Step(0.1);
  ASSERT_EQ(drops.List().size(), 1u);
  ASSERT_TRUE(drops.List()[0].sliding);
  DropMap map(360, 360);
  map.Render(drops);
  int i0 = 360, i1 = -1, j0 = 360, j1 = -1;
  for (int j = 0; j < 360; ++j)
  {
    for (int i = 0; i < 360; ++i)
    {
      if (Texel(map, i, j, 0) > 1e-4f)
      {
        i0 = std::min(i0, i);
        i1 = std::max(i1, i);
        j0 = std::min(j0, j);
        j1 = std::max(j1, j);
      }
    }
  }
  EXPECT_GT(j1 - j0, i1 - i0) << "a drop sliding down is not longer than wide";
}

//////////////////////////////////////////////////
TEST(DropMap, TrailIsWet)
{
  Drops drops = Square(1.0);
  drops.Add(At(0.5, 0.1, 0.06));
  for (int k = 0; k < 20 && drops.Trails().empty(); ++k)
    drops.Step(0.1);
  ASSERT_FALSE(drops.Trails().empty());
  const Trail t = drops.Trails().front();
  DropMap map(360, 360);
  map.Render(drops);

  const int mi = static_cast<int>(0.5 * (t.x0 + t.x1) * 360);
  const int mj = static_cast<int>(0.5 * (t.y0 + t.y1) * 360);
  EXPECT_GT(Texel(map, mi, mj, 1), 0.3f);
  const int far = mi + static_cast<int>(std::ceil(3.0 * t.halfWidth * 360));
  ASSERT_LT(far, 360);
  EXPECT_FLOAT_EQ(Texel(map, far, mj, 1), 0.0f);
}

//////////////////////////////////////////////////
TEST(DropMap, MaskMatchesCovered)
{
  Drops drops = Square(0.5);
  drops.Add(At(0.3, 0.3, 0.05));
  drops.Add(At(0.7, 0.6, 0.08, 0.1f));
  DropMap map(200, 200);
  map.Render(drops);
  std::vector<std::uint8_t> mask;
  map.Mask(mask);
  ASSERT_EQ(mask.size(), 200u * 200u);
  const auto set = std::count(mask.begin(), mask.end(), std::uint8_t{255});
  EXPECT_EQ(static_cast<std::size_t>(set), map.Covered());
  EXPECT_EQ(static_cast<std::size_t>(set) +
      static_cast<std::size_t>(std::count(mask.begin(), mask.end(), 0)),
      mask.size());
}

//////////////////////////////////////////////////
TEST(DropMap, RenderBudget)
{
  Params p;
  p.rainRate = 1000.0;
  p.dryTime = 3600.0;
  p.focus = 0.0;
  p.aspect = 640.0 / 360.0;
  // A full lens, the worst case: 2048 drops of the sizes rain brings,
  // added so that none has merged yet.
  Drops drops(1, p);
  Generator rng(1);
  for (std::size_t k = 0; k < kMaxDrops; ++k)
  {
    const double u = Uniform(rng);
    drops.Add(At(Uniform(rng), Uniform(rng),
        kMinRadius + (kMaxRadius - kMinRadius) * u * u * u,
        static_cast<float>(0.15 * Uniform(rng))));
  }
  ASSERT_EQ(drops.List().size(), kMaxDrops);

  DropMap map(640, 360);
  const int runs = 20;
  const auto start = std::chrono::steady_clock::now();
  for (int k = 0; k < runs; ++k)
    map.Render(drops);
  const double mean = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count() / runs;
  std::cout << "[ DropMap  ] " << drops.List().size() << " drops, "
            << drops.Trails().size() << " trails, 640x360, focus 0: "
            << mean << " ms a map\n";
  EXPECT_LT(mean, 25.0);
}
