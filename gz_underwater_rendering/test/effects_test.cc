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

#include <gz/msgs/annotated_axis_aligned_2d_box_v.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/param.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
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
            const std::lock_guard<std::mutex> lock(this->mutex);
            this->rendered = true;
            if (_m.header().stamp().sec() < 2)
              return;
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
        // Nothing at all in 10 s: nothing renders here; the test skips
        if (i == 200 && !this->rendered)
          return {};
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ADD_FAILURE() << "no frames on " << _topic;
    return {};
  }

  /// \brief The last _n frames of a topic, or as many as there are.
  public: std::vector<msgs::Image> Last(const std::string &_topic,
                                        std::size_t _n)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    auto &f = this->frames[_topic];
    return std::vector<msgs::Image>(f.end() - std::min(_n, f.size()),
                                    f.end());
  }

  /// \brief Whether any frame came: none means no GPU.
  public: bool Rendered()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->rendered;
  }

  /// \brief Set one number of the world's water, and wait until the water
  /// optics system publishes it.
  /// \return Whether it did, within 10 s.
  public: bool SetNumber(const std::string &_world, const std::string &_key,
                         double _value)
  {
    std::mutex m;
    std::optional<double> now;
    std::function<void(const msgs::Param &)> onInfo =
        [&m, &now, _key](const msgs::Param &_msg)
        {
          const auto it = _msg.params().find(_key);
          if (it == _msg.params().end())
            return;
          const std::lock_guard<std::mutex> lock(m);
          now = it->second.double_value();
        };
    const std::string info = "/world/" + _world + "/water_optics_info";
    this->node.Subscribe(info, onInfo);
    msgs::Param msg;
    auto &p = (*msg.mutable_params())[_key];
    p.set_type(msgs::Any::DOUBLE);
    p.set_double_value(_value);
    auto pub = this->node.Advertise<msgs::Param>(
        "/world/" + _world + "/water_optics/set");
    for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    pub.Publish(msg);
    bool done = false;
    for (int i = 0; i < 500 && !done; ++i)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      const std::lock_guard<std::mutex> lock(m);
      done = now && *now == _value;
    }
    this->node.Unsubscribe(info);
    return done;
  }

  public: transport::Node node;
  public: std::unique_ptr<Server> server;
  private: std::mutex mutex;
  private: std::map<std::string, std::vector<msgs::Image>> frames;
  private: bool rendered{false};
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
    if (!c.Rendered())
      GTEST_SKIP() << "nothing rendered: no GPU?";
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
    if (!c.Rendered())
      GTEST_SKIP() << "nothing rendered: no GPU?";
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
    // Without caustics: the two runs cannot be held to the same instant of
    // the pattern, and what is checked is only that the surface drawn above
    // does not darken the seabed
    Capture c(world, {"/uwcaustics/a"});
    c.Next("/uwcaustics/a", 1);
    if (!c.Rendered())
      GTEST_SKIP() << "nothing rendered: no GPU?";
    ASSERT_TRUE(c.SetNumber("uwcaustics", "caustics", 0.0)) << world;
    c.Next("/uwcaustics/a", 5);
    // One whole period of the wave (4 s at 10 Hz): the camera's depth
    // under it, and so the veil, rises and falls with the wave
    const auto a = c.Next("/uwcaustics/a", 40);
    if (!c.Rendered())
      GTEST_SKIP() << "nothing rendered: no GPU?";
    c.server.reset();
    ASSERT_EQ(a.size(), 40u);
    for (const auto &m : a)
      *value += mean(m) / 40;
  }
  std::cout << "seabed, linear green: " << open << " without the surface, "
            << covered << " under it\n";
  EXPECT_NEAR(covered, open, 0.02 * open);
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
  if (!c.Rendered())
    GTEST_SKIP() << "nothing rendered: no GPU?";
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
  if (!c.Rendered())
    GTEST_SKIP() << "nothing rendered: no GPU?";
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
  if (!c.Rendered())
    GTEST_SKIP() << "nothing rendered: no GPU?";
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
// <quality>: low and high are close to medium, the default, each with its
// marine snow; a value it does not know is medium, the same image.
TEST(Quality, LowAndHighCloseToMediumUnknownIsMedium)
{
  const std::vector<std::string> worlds{"snow.sdf", "snow_low.sdf",
                                        "snow_high.sdf", "snow_ultra.sdf"};
  std::vector<double> mean(worlds.size(), 0.0);
  std::vector<std::size_t> bright(worlds.size(), 0);
  std::vector<std::string> pixels(worlds.size());
  for (std::size_t k = 0; k < worlds.size(); ++k)
  {
    const std::string topic = "/uwsnow/cam/image";
    Capture c(worlds[k], {topic});
    const auto f = c.Next(topic, 1);
    if (!c.Rendered())
      GTEST_SKIP() << "nothing rendered: no GPU?";
    c.server.reset();
    ASSERT_EQ(f.size(), 1u) << worlds[k];
    const auto &m = f[0];
    pixels[k] = m.data();
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
  }
  std::cout << "linear green, medium " << mean[0] << ", low " << mean[1]
            << ", high " << mean[2] << "; specks " << bright[0] << ", "
            << bright[1] << " and " << bright[2] << "\n";
  for (std::size_t k : {1u, 2u})
  {
    EXPECT_NEAR(mean[k], mean[0], 0.1 * mean[0]) << worlds[k];
    EXPECT_GT(bright[k], 5u) << "no marine snow: " << worlds[k];
  }
  EXPECT_EQ(pixels[3], pixels[0]) << "an unknown quality is not medium";
}

/////////////////////////////////////////////////
// At night with the lamps off there is no light for the snow to show.
TEST(MarineSnow, DarkWithoutLamps)
{
  Capture c("snow_dark.sdf", {"/uwsnowdark/cam/image"});
  const auto f = c.Next("/uwsnowdark/cam/image", 1);
  if (!c.Rendered())
    GTEST_SKIP() << "nothing rendered: no GPU?";
  c.server.reset();
  ASSERT_EQ(f.size(), 1u);
  int bright = 0;
  for (std::size_t i = 0; i < f[0].data().size(); ++i)
    bright += static_cast<std::uint8_t>(f[0].data()[i]) > 2;
  EXPECT_EQ(bright, 0) << "something glows with no light";
}

/////////////////////////////////////////////////
// Depth is measured below the wave over the camera, not below the still
// water: under a wave of amplitude 0.5 m a camera 0.15 m above the still
// level is under water for part of each period (40 % of it) and one 0.15 m
// below is in air for as long. Under water the panel 2 m away is dim.
TEST(WaveDepth, TheCameraIsUnderTheWaveNotTheStillLevel)
{
  Capture c("wave_depth.sdf", {"/uwwavedepth/above", "/uwwavedepth/below"});
  // Let the wave grow.
  c.Next("/uwwavedepth/above", 30);
  // Eight seconds, two periods.
  c.Next("/uwwavedepth/above", 80);
  c.Next("/uwwavedepth/below", 1);
  c.server.reset();
  const auto above = c.Last("/uwwavedepth/above", 80);
  const auto below = c.Last("/uwwavedepth/below", 80);
  ASSERT_EQ(above.size(), 80u);
  ASSERT_EQ(below.size(), 80u);
  // The bottom rows: rays that go down, so all their range is in water.
  const auto brightness = [](const msgs::Image &_m)
  {
    double sum = 0;
    int n = 0;
    for (std::uint32_t v = 3 * _m.height() / 4; v < _m.height(); ++v)
      for (std::uint32_t u = 0; u < _m.width(); ++u, ++n)
        sum += G(_m, u, v);
    return sum / n;
  };
  const auto stamp = [](const msgs::Image &_m)
  {
    return _m.header().stamp().sec() * 1000000000LL +
           _m.header().stamp().nsec();
  };
  double air = 0;
  for (const auto &m : above)
    air = std::max(air, brightness(m));
  ASSERT_GT(air, 0.1) << "the panel is not in view";
  std::map<long long, bool> aboveUnder;
  int nAbove = 0, nBelow = 0, both = 0, wrong = 0;
  for (const auto &m : above)
  {
    const bool under = brightness(m) < 0.5 * air;
    aboveUnder[stamp(m)] = under;
    nAbove += under;
  }
  for (const auto &m : below)
  {
    const bool under = brightness(m) < 0.5 * air;
    nBelow += under;
    const auto it = aboveUnder.find(stamp(m));
    if (it == aboveUnder.end())
      continue;
    ++both;
    // The higher camera is never under water while the lower is in air.
    wrong += it->second && !under;
  }
  std::cout << "under water: the camera above the still level in " << nAbove
            << " of 80 frames (32 expected), the one below in " << nBelow << " (48)"
            << "; frames of the same instant " << both << "\n";
  EXPECT_GE(nAbove, 24) << "never under the crest";
  EXPECT_LE(nAbove, 40);
  EXPECT_GE(nBelow, 40);
  EXPECT_LE(nBelow, 56) << "never above the trough";
  EXPECT_GT(both, 60);
  EXPECT_EQ(wrong, 0);
}

/////////////////////////////////////////////////
// What nothing lights (Ogre's unlit shaders: a particle emitter's particles)
// is seen through the water too: particles 2 m away have the colour they
// have in air, faded over the 2 m, plus the veil built over them.
TEST(Unlit, FadesAndTakesTheVeil)
{
  Capture c("unlit.sdf", {"/uwunlit/air", "/uwunlit/water"});
  // The middle of the image: 2 m of water, within 1 %.
  const auto centre = [](const msgs::Image &_m)
  {
    double sum = 0;
    int n = 0;
    for (std::uint32_t v = _m.height() / 2 - 4; v < _m.height() / 2 + 4; ++v)
      for (std::uint32_t u = _m.width() / 2 - 4; u < _m.width() / 2 + 4;
           ++u, ++n)
        sum += G(_m, u, v);
    return sum / n;
  };
  // The emitter starts after a while of the clock's time, not the
  // simulation's: wait for the first particle, up to 30 s.
  std::vector<msgs::Image> air, water;
  for (int i = 0; i < 300; ++i)
  {
    air = c.Next("/uwunlit/air", 1);
    ASSERT_EQ(air.size(), 1u);
    if (centre(air[0]) > 0.1)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  air = c.Next("/uwunlit/air", 2);
  water = c.Next("/uwunlit/water", 2);
  c.server.reset();
  ASSERT_EQ(air.size(), 2u);
  ASSERT_EQ(water.size(), 2u);
  const double inAir = centre(air[1]);
  const double inWater = centre(water[1]);
  const double expected = inAir * std::exp(-0.5 * 2) +
                          0.2 * (1 - std::exp(-0.5 * 2));
  std::cout << "particles, linear green: in air " << inAir << ", in water "
            << inWater << ", expected " << expected << "\n";
  ASSERT_GT(inAir, 0.1) << "no particles in view";
  EXPECT_NEAR(inWater, expected, 0.02);
}

/////////////////////////////////////////////////
// Sensors that are not colour cameras do not see the water, nor the
// backdrop drawn around the world: a lidar's scan (open water out of range),
// a thermal image and a bounding box camera's boxes are the same with the
// GPU path and without it.
TEST(Backdrop, LidarThermalAndBoxesUntouched)
{
  struct Seen
  {
    msgs::LaserScan scan;
    msgs::Image thermal;
    msgs::AnnotatedAxisAligned2DBox_V boxes;
    bool rendered{false};
  };
  const auto see = [](const std::string &_world)
  {
    Seen seen;
    std::mutex m;
    int got = 0;
    const auto late = [](const msgs::Header &_h)
    { return _h.stamp().sec() >= 1; };
    transport::Node node;
    std::function<void(const msgs::LaserScan &)> onScan =
        [&](const msgs::LaserScan &_msg)
        {
          const std::lock_guard<std::mutex> lock(m);
          seen.rendered = true;
          if (late(_msg.header()) && !seen.scan.ranges_size())
          {
            seen.scan = _msg;
            ++got;
          }
        };
    std::function<void(const msgs::Image &)> onThermal =
        [&](const msgs::Image &_msg)
        {
          const std::lock_guard<std::mutex> lock(m);
          seen.rendered = true;
          if (late(_msg.header()) && !seen.thermal.width())
          {
            seen.thermal = _msg;
            ++got;
          }
        };
    std::function<void(const msgs::AnnotatedAxisAligned2DBox_V &)> onBoxes =
        [&](const msgs::AnnotatedAxisAligned2DBox_V &_msg)
        {
          const std::lock_guard<std::mutex> lock(m);
          if (late(_msg.header()) && !seen.boxes.has_header())
          {
            seen.boxes = _msg;
            ++got;
          }
        };
    node.Subscribe("/uwsensors/lidar", onScan);
    node.Subscribe("/uwsensors/thermal", onThermal);
    node.Subscribe("/uwsensors/boxes", onBoxes);
    ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
    auto server = std::make_unique<Server>(config);
    server->Run(false, 0, false);
    for (int i = 0; i < 600; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(m);
        if (got == 3 || (i == 200 && !seen.rendered))
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    server.reset();
    node.Unsubscribe("/uwsensors/lidar");
    node.Unsubscribe("/uwsensors/thermal");
    node.Unsubscribe("/uwsensors/boxes");
    const std::lock_guard<std::mutex> lock(m);
    EXPECT_TRUE(!seen.rendered || got == 3) << _world << ": topics missing";
    return seen;
  };
  const Seen off = see("sensors_off.sdf");
  if (!off.rendered)
    GTEST_SKIP() << "nothing rendered: no GPU?";
  const Seen on = see("sensors_on.sdf");

  // The lidar: the box in the middle, open water out of range elsewhere
  ASSERT_GT(off.scan.ranges_size(), 0);
  ASSERT_EQ(on.scan.ranges_size(), off.scan.ranges_size());
  int hits = 0, misses = 0, differ = 0;
  for (int i = 0; i < off.scan.ranges_size(); ++i)
  {
    const double a = off.scan.ranges(i), b = on.scan.ranges(i);
    std::isfinite(a) && a < 50 ? ++hits : ++misses;
    differ += !(a == b || (!std::isfinite(a) && !std::isfinite(b)));
  }
  std::cout << "lidar: " << hits << " hits, " << misses << " out of range, "
            << differ << " different\n";
  EXPECT_GT(hits, 0);
  EXPECT_GT(misses, 0) << "the lidar hit something in open water";
  EXPECT_EQ(differ, 0);

  // The thermal image, byte for byte
  ASSERT_GT(off.thermal.width(), 0u);
  EXPECT_EQ(on.thermal.data(), off.thermal.data());

  // The boxes: the target's alone, in the same place
  ASSERT_EQ(off.boxes.annotated_box_size(), 1);
  ASSERT_EQ(on.boxes.annotated_box_size(), 1);
  const auto &a = off.boxes.annotated_box(0), &b = on.boxes.annotated_box(0);
  EXPECT_EQ(b.label(), a.label());
  EXPECT_EQ(b.box().min_corner().x(), a.box().min_corner().x());
  EXPECT_EQ(b.box().max_corner().y(), a.box().max_corner().y());
}
