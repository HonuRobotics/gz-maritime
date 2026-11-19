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
// Caustics and marine snow, in the GPU path.
#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/underwater_camera/UnderwaterImage.hh"

using namespace gz;
using namespace sim;
namespace uc = underwater_camera;

namespace
{
/// \brief Frames of some topics from one running world, newest kept.
class Capture
{
  public: explicit Capture(const std::string &_world,
                           const std::vector<std::string> &_topics)
  {
    for (const auto &t : _topics)
    {
      std::function<void(const msgs::Image &)> cb =
          [this, t](const msgs::Image &_m)
          {
            if (_m.header().stamp().sec() < 2)
              return;
            const std::lock_guard<std::mutex> lock(this->mutex);
            this->frames[t].push_back(_m);
          };
      this->node.Subscribe(t, cb);
    }
    ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
    this->server = std::make_unique<Server>(config);
    this->server->Run(false, 0, false);
  }

  /// \brief Wait for _n more frames on a topic; return the last _n.
  public: std::vector<msgs::Image> Next(const std::string &_topic,
                                        std::size_t _n)
  {
    std::size_t have = 0;
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      have = this->frames[_topic].size();
    }
    for (int i = 0; i < 600; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(this->mutex);
        auto &f = this->frames[_topic];
        if (f.size() >= have + _n)
          return std::vector<msgs::Image>(f.end() - _n, f.end());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ADD_FAILURE() << "no frames on " << _topic << ": is there a GPU?";
    return {};
  }

  public: transport::Node node;
  public: std::unique_ptr<Server> server;
  private: std::mutex mutex;
  private: std::map<std::string, std::vector<msgs::Image>> frames;
};

/// \brief Linear green of a pixel.
double G(const msgs::Image &_m, std::uint32_t _u, std::uint32_t _v)
{
  return uc::SrgbToLinear(static_cast<std::uint8_t>(
      _m.data()[_v * _m.step() + 3 * _u + 1]));
}

/// \brief Relative contrast, std / mean, of the green in the central half.
double Contrast(const msgs::Image &_m)
{
  double sum = 0, sum2 = 0;
  int n = 0;
  for (std::uint32_t v = _m.height() / 4; v < 3 * _m.height() / 4; ++v)
    for (std::uint32_t u = _m.width() / 4; u < 3 * _m.width() / 4; ++u)
    {
      const double g = G(_m, u, v);
      sum += g;
      sum2 += g * g;
      ++n;
    }
  const double mean = sum / n;
  return std::sqrt(std::max(0.0, sum2 / n - mean * mean)) / mean;
}
}  // namespace

/////////////////////////////////////////////////
// With waves, the caustics' contrast (averaged over time) on a seabed 8 m
// deep is
// exp(-K_d (8 - 3)) of the contrast 3 m deep, within 10 %, and the pattern
// moves; on flat water there are none.
TEST(Caustics, FadeWithDepthAndNeedWaves)
{
  double ratio = 0;
  {
    Capture c("caustics_waves.sdf", {"/uwcaustics/a", "/uwcaustics/b"});
    // Let the wave height estimate settle.
    c.Next("/uwcaustics/a", 20);
    // Averaged over 30 frames each, while the pattern moves: a camera sees
    // only a few of its periods at once.
    const auto a = c.Next("/uwcaustics/a", 30);
    const auto b = c.Next("/uwcaustics/b", 30);
    c.server.reset();
    ASSERT_EQ(a.size(), 30u);
    ASSERT_EQ(b.size(), 30u);
    double ca = 0, cb = 0;
    for (int i = 0; i < 30; ++i)
    {
      ca += Contrast(a[i]) / 30;
      cb += Contrast(b[i]) / 30;
    }
    ratio = cb / ca;
    std::cout << "caustics contrast: 3 m " << ca << ", 8 m " << cb
              << ", ratio " << ratio << ", expected " << std::exp(-0.2 * 5)
              << "\n";
    EXPECT_GT(ca, 0.05) << "no caustics on the waves";
    EXPECT_NEAR(ratio, std::exp(-0.2 * 5), 0.1 * std::exp(-0.2 * 5));
    EXPECT_NE(a[0].data(), a[1].data()) << "the caustics do not move";
  }
  {
    Capture c("caustics_flat.sdf", {"/uwflat/a"});
    c.Next("/uwflat/a", 20);
    const auto a = c.Next("/uwflat/a", 1);
    c.server.reset();
    ASSERT_EQ(a.size(), 1u);
    std::cout << "flat water contrast " << Contrast(a[0]) << "\n";
    EXPECT_LT(Contrast(a[0]), 0.01) << "caustics on flat water";
  }
}

/////////////////////////////////////////////////
// The water's surface is drawn above the seabed and the sun casts shadows:
// the seabed is as bright as without the surface, which lets the sunlight
// through.
TEST(Caustics, TheSurfaceDoesNotShadeTheSeabed)
{
  const auto mean = [](const msgs::Image &_m)
  {
    double sum = 0;
    for (std::uint32_t v = 0; v < _m.height(); ++v)
      for (std::uint32_t u = 0; u < _m.width(); ++u)
        sum += G(_m, u, v);
    return sum / (_m.width() * _m.height());
  };
  double open = 0, covered = 0;
  for (const auto &[world, value] :
       {std::pair<std::string, double *>{"caustics_waves.sdf", &open},
        {"caustics_surface.sdf", &covered}})
  {
    Capture c(world, {"/uwcaustics/a"});
    c.Next("/uwcaustics/a", 20);
    const auto a = c.Next("/uwcaustics/a", 10);
    c.server.reset();
    ASSERT_EQ(a.size(), 10u);
    for (const auto &m : a)
      *value += mean(m) / 10;
  }
  std::cout << "seabed, linear green: " << open << " without the surface, "
            << covered << " under it\n";
  EXPECT_NEAR(covered, open, 0.1 * open);
}

/////////////////////////////////////////////////
// Open water, 6 m down, nothing in view but the surface above: every pixel
// is the water's own colour, the veil at the camera's depth, whether the
// ray ends on the surface or on nothing at all. Above the water the
// background is untouched.
TEST(OpenWater, TheWatersColourUnderItTheBackgroundAboveIt)
{
  Capture c("open_water.sdf", {"/uwopen/under", "/uwopen/above"});
  const auto under = c.Next("/uwopen/under", 10);
  const auto above = c.Next("/uwopen/above", 1);
  c.server.reset();
  ASSERT_EQ(under.size(), 10u);
  ASSERT_EQ(above.size(), 1u);
  const auto &m = under.back();
  // The veil just below the surface, 0.3 in the green, faded by the
  // daylight's attenuation over the camera's depth.
  const double expected = 0.3 * std::exp(-0.1 * 6);
  double low = 1e9, high = -1e9;
  for (std::uint32_t v = 0; v < m.height(); ++v)
  {
    for (std::uint32_t u = 0; u < m.width(); ++u)
    {
      low = std::min(low, G(m, u, v));
      high = std::max(high, G(m, u, v));
    }
  }
  std::cout << "open water, linear green: " << low << " to " << high
            << ", the veil is " << expected << "\n";
  EXPECT_NEAR(low, expected, 0.1 * expected);
  EXPECT_NEAR(high, expected, 0.1 * expected);

  // The top of the image above the water: the scene's background, 0.9.
  const auto &t = above[0];
  const auto top = static_cast<std::uint8_t>(
      t.data()[2 * t.step() + 3 * (t.width() / 2) + 1]);
  std::cout << "above the water, the background is " << int(top) << "\n";
  EXPECT_GT(top, 200);
}

/////////////////////////////////////////////////
// Marine snow at night: lamps on, specks, fixed in the world (the same in
// two frames of a still camera); lamps off, nothing to see; the depth image
// the same with and without snow.
TEST(MarineSnow, LitByLampsFixedInTheWorld)
{
  Capture c("snow.sdf", {"/uwsnow/cam/image", "/uwsnow/cam/depth_image"});
  const auto on = c.Next("/uwsnow/cam/image", 2);
  const auto depthSnow = c.Next("/uwsnow/cam/depth_image", 1);
  ASSERT_EQ(on.size(), 2u);

  // Specks: pixels much brighter than their row neighbours.
  const auto specks = [](const msgs::Image &_m)
  {
    std::vector<std::size_t> out;
    for (std::uint32_t v = 0; v < _m.height(); ++v)
      for (std::uint32_t u = 2; u + 2 < _m.width(); ++u)
      {
        const double g = G(_m, u, v);
        const double around = 0.5 * (G(_m, u - 2, v) + G(_m, u + 2, v));
        if (g > around + 0.02 && g > 1.5 * around)
          out.push_back(v * _m.width() + u);
      }
    return out;
  };
  const auto s0 = specks(on[0]), s1 = specks(on[1]);
  std::size_t same = 0;
  for (auto i : s0)
    same += std::binary_search(s1.begin(), s1.end(), i);
  std::cout << "specks " << s0.size() << ", " << same << " in both frames\n";
  EXPECT_GT(s0.size(), 20u) << "no marine snow in the lamps' light";
  EXPECT_GE(same, s0.size() * 9 / 10) << "the snow moves";

  // Lamps off.
  auto pub = c.node.Advertise<msgs::Param>(
      "/world/uwsnow/water_optics/set");
  msgs::Param noSnow;
  (*noSnow.mutable_params())["marine_snow"].set_type(msgs::Any::DOUBLE);
  (*noSnow.mutable_params())["marine_snow"].set_double_value(0.0);
  for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  pub.Publish(noSnow);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  const auto off = c.Next("/uwsnow/cam/image", 1);
  const auto depthNone = c.Next("/uwsnow/cam/depth_image", 1);
  c.server.reset();
  ASSERT_EQ(off.size(), 1u);
  EXPECT_LT(specks(off[0]).size(), s0.size() / 10) << "specks without snow";
  ASSERT_EQ(depthSnow.size(), 1u);
  ASSERT_EQ(depthNone.size(), 1u);
  EXPECT_EQ(depthSnow[0].data(), depthNone[0].data())
      << "the snow changed the depth image";
}

/////////////////////////////////////////////////
// A lamp too dim to matter is left out of the shader (its beam's march is
// the costliest part). In the dark, a seabed under such a lamp stays black;
// lit by it, it would be about 30 of 255.
TEST(Lamps, OneThatGivesNoLightIsLeftOut)
{
  Capture c("flat_idle_lamp.sdf", {"/uwflat/a"});
  const auto a = c.Next("/uwflat/a", 1);
  c.server.reset();
  ASSERT_EQ(a.size(), 1u);
  int brightest = 0;
  for (const char b : a[0].data())
    brightest = std::max(brightest, static_cast<int>(
        static_cast<std::uint8_t>(b)));
  std::cout << "under the idle lamp, the brightest pixel is " << brightest
            << " of 255\n";
  EXPECT_LE(brightest, 1);
}

/////////////////////////////////////////////////
// <quality>low</quality>: fewer steps in the beam and the snow drawn half
// as far. The lit scene is as bright, within 10 %, and the snow is there.
TEST(Quality, LowIsCloseToMedium)
{
  double mean[2] = {0, 0};
  std::size_t bright[2] = {0, 0};
  int k = 0;
  for (const auto &[world, topic] :
       {std::pair<std::string, std::string>{"snow.sdf", "/uwsnow/cam/image"},
        {"snow_low.sdf", "/uwsnow/cam/image"}})
  {
    Capture c(world, {topic});
    const auto f = c.Next(topic, 1);
    c.server.reset();
    ASSERT_EQ(f.size(), 1u);
    const auto &m = f[0];
    for (std::uint32_t v = 0; v < m.height(); ++v)
    {
      for (std::uint32_t u = 2; u + 2 < m.width(); ++u)
      {
        const double g = G(m, u, v);
        mean[k] += g / (m.width() * m.height());
        const double around = 0.5 * (G(m, u - 2, v) + G(m, u + 2, v));
        bright[k] += g > around + 0.02 && g > 1.5 * around;
      }
    }
    ++k;
  }
  std::cout << "linear green, medium " << mean[0] << ", low " << mean[1]
            << "; specks " << bright[0] << " and " << bright[1] << "\n";
  EXPECT_NEAR(mean[1], mean[0], 0.1 * mean[0]);
  EXPECT_GT(bright[1], 5u) << "no marine snow at low quality";
}

/////////////////////////////////////////////////
// At night with the lamps off there is no light for the snow to show.
TEST(MarineSnow, DarkWithoutLamps)
{
  Capture c("snow_dark.sdf", {"/uwsnowdark/cam/image"});
  const auto f = c.Next("/uwsnowdark/cam/image", 1);
  c.server.reset();
  ASSERT_EQ(f.size(), 1u);
  int bright = 0;
  for (std::size_t i = 0; i < f[0].data().size(); ++i)
    bright += static_cast<std::uint8_t>(f[0].data()[i]) > 2;
  EXPECT_EQ(bright, 0) << "something glows with no light";
}
