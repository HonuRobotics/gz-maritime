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
// The surface seen from below (gz_waves_rendering's water shader): Snell's
// window at asin(1 / 1.333) = 48.6 degrees from the vertical.
#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "OwnPartition.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief Step a world until _n frames of a camera from _from s on have
/// come. The test steps it: a server on a thread of its own can hang when
/// it is destroyed (gz-sim 10.5, in its Sensors system).
/// \param[in] _world The world's file, in the test worlds.
/// \param[in] _topic The camera's topic.
/// \param[in] _n How many frames.
/// \param[in] _from From this simulation time on, s.
/// \param[out] _rendered Whether any frame came at all: none means no GPU.
/// \return The frames.
std::vector<msgs::Image> Frames(const std::string &_world,
    const std::string &_topic, std::size_t _n, int _from, bool &_rendered)
{
  std::mutex mutex;
  std::vector<msgs::Image> frames;
  bool rendered = false;
  transport::Node node;
  std::function<void(const msgs::Image &)> cb = [&](const msgs::Image &_m)
  {
    const std::lock_guard<std::mutex> lock(mutex);
    rendered = true;
    if (_m.header().stamp().sec() >= _from)
      frames.push_back(_m);
  };
  node.Subscribe(_topic, cb);
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
  auto server = std::make_unique<Server>(config);
  for (int i = 0; i < 4000; ++i)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (frames.size() >= _n || (i == 100 && !rendered))
        break;
    }
    server->Run(true, 5, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  server.reset();
  node.Unsubscribe(_topic);
  const std::lock_guard<std::mutex> lock(mutex);
  _rendered = rendered;
  return frames;
}

/// \brief Pixels green as the test worlds' veil, which only the surface
/// seen from below fades into.
/// \param[in] _m The frame.
/// \return How many.
std::size_t Green(const msgs::Image &_m)
{
  std::size_t n = 0;
  for (std::uint32_t v = 0; v < _m.height(); ++v)
  {
    for (std::uint32_t u = 0; u < _m.width(); ++u)
    {
      const auto *p = reinterpret_cast<const std::uint8_t *>(
          _m.data().data() + v * _m.step() + 3 * u);
      n += p[1] > 40 && p[1] > p[0] + 30 && p[1] > p[2] + 30;
    }
  }
  return n;
}
}  // namespace

//////////////////////////////////////////////////
TEST(Snell, WindowAt48Point6Degrees)
{
  bool rendered = false;
  const auto frames = Frames("snell.sdf", "/uwsnell/cam", 1, 2, rendered);
  if (!rendered)
    GTEST_SKIP() << "nothing rendered: no GPU?";
  ASSERT_FALSE(frames.empty()) << "no frame";
  const msgs::Image *frame = &frames.back();

  const int w = frame->width(), h = frame->height();
  const double fx = w / (2 * std::tan(2.6 / 2));
  const auto lum = [&](int _u, int _v)
  {
    const std::size_t i = (_v * frame->step()) + 3 * _u;
    double s = 0;
    for (int c = 0; c < 3; ++c)
      s += static_cast<std::uint8_t>(frame->data()[i + c]);
    return s / 3;
  };
  // The window's edge along four directions from the centre: where the
  // image reaches the mirror's constant colour (the sky inside the window is
  // textured, so its brightness varies; the mirror outside does not).
  const double outside = (lum(2, h / 2) + lum(w - 3, h / 2)) / 2;
  const double inside = lum(w / 2 + 40, h / 2);
  std::cout << "inside " << inside << ", outside " << outside << "\n";
  ASSERT_GT(inside - outside, 40) << "no window: is the surface drawn?";
  double sum = 0;
  int n = 0;
  for (auto [du, dv] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
  {
    for (int r = 1; r < w / 2 - 1; ++r)
    {
      if (std::abs(lum(w / 2 + du * r, h / 2 + dv * r) - outside) < 3)
      {
        sum += std::atan((r - 0.5) / fx);
        ++n;
        break;
      }
    }
  }
  ASSERT_EQ(n, 4);
  const double angle = sum / n * 180.0 / GZ_PI;
  std::cout << "Snell's window half-angle: " << angle << " degrees\n";
  EXPECT_NEAR(angle, std::asin(1 / 1.333) * 180 / GZ_PI, 1.0);
}

//////////////////////////////////////////////////
// A camera above the still water, below the crests: the crests higher than
// the camera face it and are seen from above, reflecting the sky. Drawn as
// if from below, with the water drawn, they would fade into the veil, here
// pure green.
TEST(Snell, CrestsAboveACameraOnABoatAreSeenFromAbove)
{
  // A whole wave period at 5 Hz, from 4 s: in its first seconds the
  // surface is still settling, and two runs differ
  bool rendered = false;
  const auto frames = Frames("crest.sdf", "/uwcrest/cam", 30, 4, rendered);
  if (!rendered)
    GTEST_SKIP() << "nothing rendered: no GPU?";
  ASSERT_GE(frames.size(), 30u);
  // Frames with the camera in air: the top row is the sky, blue (under a
  // crest it is the surface seen from below, faded into the green veil),
  // and the bottom row is the surface below the camera, not the background
  // (at the instant a crest's flank passes through the camera, the surface
  // is edge on, and nothing is drawn above or below it).
  std::size_t dark = 0, above = 0, inAir = 0;
  for (const auto &m : frames)
  {
    const auto *corner = reinterpret_cast<const std::uint8_t *>(
        m.data().data());
    int sky = 0, surface = 0;
    for (std::uint32_t u = 0; u < m.width(); ++u)
    {
      const auto *p = reinterpret_cast<const std::uint8_t *>(
          m.data().data() + 3 * u);
      sky += p[2] > p[1] && p[2] > 150;
      const auto *q = reinterpret_cast<const std::uint8_t *>(
          m.data().data() + (m.height() - 1) * m.step() + 3 * u);
      int off = 0;
      for (int c = 0; c < 3; ++c)
        off = std::max(off, std::abs(int(q[c]) - int(corner[c])));
      surface += off > 10;
    }
    if (sky < static_cast<int>(m.width()) * 9 / 10 ||
        surface < static_cast<int>(m.width()) * 9 / 10)
    {
      continue;
    }
    ++inAir;
    // Green: the veil, which only the surface seen from below fades into.
    // From above, the surface is the sky's blue reflected, or the deep blue
    // at grazing angles.
    dark += Green(m);
    above += m.width() * m.height();
  }
  std::cout << "camera in air in " << inAir << " frames of " << frames.size()
            << "\n";
  ASSERT_GE(inAir, 5u) << "the camera was never above the surface";
  std::cout << "green pixels: " << dark << " of " << above << "\n";
  EXPECT_LT(dark, above / 250) << "crests above the camera drawn from below";
}

//////////////////////////////////////////////////
// A camera 2 m above a rippled sea, looking at the horizon: all of the
// surface is below it, and none of it is shaded as seen from below, even
// far away, where the view grazes it and a normal interpolated from the
// corners of a triangle disagrees with the triangle drawn.
TEST(Snell, NoSpecklesAtTheHorizon)
{
  bool rendered = false;
  const auto frames =
      Frames("horizon.sdf", "/uwhorizon/cam", 10, 4, rendered);
  if (!rendered)
    GTEST_SKIP() << "nothing rendered: no GPU?";
  ASSERT_GE(frames.size(), 10u);
  std::size_t green = 0, all = 0;
  for (const auto &m : frames)
  {
    green += Green(m);
    all += m.width() * m.height();
  }
  std::cout << "seen from below at the horizon: " << green << " pixels of "
            << all << "\n";
  EXPECT_LT(green, all / 2000) << "speckles of the surface seen from below";
}
