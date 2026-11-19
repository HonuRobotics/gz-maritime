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
// Vehicle lamps under water, at night (scene D of the plan): the lamp's
// light on a plate, faded both ways, and its beam, against the model
// integrated here independently of the shader.
#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/underwater_camera/UnderwaterImage.hh"
#include "gz/sim/water_optics/WaterOptics.hh"

using namespace gz;
using namespace sim;
using math::Vector3d;
namespace uc = underwater_camera;

namespace
{
// The lamp of lamps.sdf, in the camera's frame (x ahead, y left, z up).
const Vector3d kLampPos(0.0, 0.35, -0.15);
const Vector3d kLampDir = Vector3d(1, -0.12, 0.02).Normalized();
// gz gives Ogre the lamp's power as its <intensity> times pi.
const Vector3d kLampColour = Vector3d(40, 40, 40) * GZ_PI;
constexpr double kRange = 40.0, kLinear = 0.0, kQuadratic = 0.02;
constexpr double kInner = 0.6, kOuter = 1.2, kFalloff = 1.0;
constexpr double kG = 0.9;

/// \brief Ogre's spot cone, as the shader has it.
double SpotAtten(double _cos)
{
  const double cOuter = std::cos(kOuter / 2), cInner = std::cos(kInner / 2);
  if (_cos < cOuter)
    return 0.0;
  return std::pow(std::clamp((_cos - cOuter) / (cInner - cOuter), 0.0, 1.0),
                  kFalloff);
}

double Atten(double _d)
{
  return _d <= kRange ? 1.0 / (0.5 + (kLinear + kQuadratic * _d) * _d) : 0.0;
}

/// \brief The beam along a ray, single scatter, integrated finely.
uc::Rgb Beam(const Vector3d &_ray, double _tMax,
             const water_optics::WaterOpticsData &_w)
{
  uc::Rgb sum(0, 0, 0);
  const int n = 4000;
  const double dt = std::min(_tMax, 20.0) / n;
  for (int k = 0; k < n; ++k)
  {
    const double t = (k + 0.5) * dt;
    const Vector3d from = _ray * t - kLampPos;
    const double d = from.Length();
    const Vector3d prop = from / d;
    const double a = Atten(d) * SpotAtten(prop.Dot(kLampDir));
    if (a <= 0)
      continue;
    const double cosT = prop.Dot(-_ray);
    const double phase = (1 - kG * kG) /
        (4 * GZ_PI * std::pow(1 + kG * kG - 2 * kG * cosT, 1.5));
    for (int c = 0; c < 3; ++c)
    {
      const double b = _w.veil[c] * _w.attenuation[c];
      sum[c] += b * phase * kLampColour[c] * a *
                std::exp(-_w.attenuation[c] * (d + t)) * dt;
    }
  }
  return sum;
}

/// \brief One colour frame and its depth, of one stamp.
struct Frame
{
  std::vector<std::uint8_t> rgb;
  std::vector<float> depth;
  std::uint32_t w{0}, h{0};
};

class LampsTest : public ::testing::Test
{
  protected: static void SetUpTestSuite()
  {
    ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "lamps.sdf"));
    server = std::make_unique<Server>(config);
    node = std::make_unique<transport::Node>();
    std::function<void(const msgs::Image &)> onImage =
        [](const msgs::Image &_m) { Keep(_m, false); };
    std::function<void(const msgs::Image &)> onDepth =
        [](const msgs::Image &_m) { Keep(_m, true); };
    node->Subscribe("/uwlamps/cam/image", onImage);
    node->Subscribe("/uwlamps/cam/depth_image", onDepth);
    server->Run(false, 0, false);

    // Clear water, then the three waters, each after it has settled.
    frames["clear"] = Wait();
    for (const std::string type : {"3C", "IB", "7C"})
    {
      msgs::Param msg;
      auto &p = *msg.mutable_params();
      for (const char *k : {"attenuation", "backscatter", "veil",
                            "diffuse_attenuation"})
      {
        p[k].set_type(msgs::Any::STRING);
        p[k].set_string_value("");
      }
      p["jerlov"].set_type(msgs::Any::STRING);
      p["jerlov"].set_string_value(type);
      auto pub = node->Advertise<msgs::Param>("/world/uwlamps/water_optics/set");
      for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      pub.Publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      frames[type] = Wait();
    }
  }

  protected: static void TearDownTestSuite()
  {
    server.reset();
    node.reset();
  }

  /// \brief Keep the latest colour and depth by stamp.
  private: static void Keep(const msgs::Image &_m, bool _depth)
  {
    const std::lock_guard<std::mutex> lock(mutex);
    const auto stamp = _m.header().stamp().sec() * 1000000000LL +
                       _m.header().stamp().nsec();
    (_depth ? depths : images)[stamp] = _m;
  }

  /// \brief The newest stamp with both, after the call.
  private: static Frame Wait()
  {
    std::int64_t after = 0;
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (!images.empty())
        after = images.rbegin()->first;
    }
    for (int i = 0; i < 400; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(mutex);
        for (auto it = images.rbegin(); it != images.rend(); ++it)
        {
          if (it->first > after + 300000000LL && it->first > 1000000000LL &&
              depths.count(it->first))
          {
            Frame f;
            const auto &im = it->second;
            const auto &dp = depths[it->first];
            f.w = im.width();
            f.h = im.height();
            f.rgb.assign(im.data().begin(), im.data().end());
            f.depth.resize(f.w * f.h);
            std::memcpy(f.depth.data(), dp.data().data(), f.depth.size() * 4);
            return f;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ADD_FAILURE() << "no frames: is there a GPU?";
    return {};
  }

  /// \brief The camera ray through a pixel (hfov 1.2, square pixels).
  protected: static Vector3d Ray(const Frame &_f, std::uint32_t _u,
                                 std::uint32_t _v, double *_factor = nullptr)
  {
    const double fx = _f.w / (2 * std::tan(0.6));
    const double x = (_u + 0.5 - _f.w / 2.0) / fx;
    const double y = (_v + 0.5 - _f.h / 2.0) / fx;
    const Vector3d d(1, -x, -y);
    if (_factor)
      *_factor = d.Length();
    return d.Normalized();
  }

  protected: static water_optics::WaterOpticsData Water(const std::string &_t)
  {
    water_optics::WaterOpticsSpec spec;
    spec.jerlov = _t;
    spec.daylight = 0;
    water_optics::WaterOpticsData out;
    std::string error, warning;
    EXPECT_TRUE(water_optics::Resolve(spec, out, error, warning));
    return out;
  }

  protected: static double Lin(const Frame &_f, std::size_t _i, int _c)
  { return uc::SrgbToLinear(_f.rgb[3 * _i + _c]); }

  protected: static std::unique_ptr<Server> server;
  protected: static std::unique_ptr<transport::Node> node;
  protected: static std::map<std::string, Frame> frames;
  private: static std::mutex mutex;
  private: static std::map<std::int64_t, msgs::Image> images, depths;
};
std::unique_ptr<Server> LampsTest::server;
std::unique_ptr<transport::Node> LampsTest::node;
std::map<std::string, Frame> LampsTest::frames;
std::mutex LampsTest::mutex;
std::map<std::int64_t, msgs::Image> LampsTest::images, LampsTest::depths;
}  // namespace

/////////////////////////////////////////////////
// The plate, lit by the lamp: its clear-water colour faded both ways, lamp
// to plate and plate to camera, plus the beam in front of it.
TEST_F(LampsTest, LitPlateFadesBothWays)
{
  const auto &air = frames["clear"];
  const auto &uw = frames["3C"];
  ASSERT_FALSE(air.rgb.empty());
  ASSERT_FALSE(uw.rgb.empty());
  const auto w = Water("3C");
  int checked = 0, worst = 0;
  for (std::uint32_t v = air.h / 3; v < 2 * air.h / 3; v += 2)
  {
    for (std::uint32_t u = 0; u < air.w / 3; u += 2)
    {
      const std::size_t i = v * air.w + u;
      if (!(air.depth[i] > 0 && air.depth[i] < 4))
        continue;  // not the plate
      double factor = 1;
      const Vector3d ray = Ray(air, u, v, &factor);
      const double r = air.depth[i] * factor;
      const double toLamp = (ray * r - kLampPos).Length();
      const auto beam = Beam(ray, r, w);
      for (int c = 0; c < 3; ++c)
      {
        const double e = Lin(air, i, c) *
            std::exp(-w.attenuation[c] * (toLamp + r)) + beam[c];
        worst = std::max(worst, std::abs(int(uc::LinearToSrgb(e)) -
                                         int(uw.rgb[3 * i + c])));
      }
      ++checked;
    }
  }
  std::cout << "plate pixels " << checked << ", worst " << worst
            << " steps\n";
  EXPECT_GT(checked, 50) << "the plate is not where it should be";
  EXPECT_LE(worst, 2);
}

/////////////////////////////////////////////////
// The beam against the black backdrop, along the centre row, against the
// fine integration, in each water: its profile within 5 % of its peak.
TEST_F(LampsTest, BeamProfile)
{
  for (const std::string type : {"IB", "3C", "7C"})
  {
    const auto &uw = frames[type];
    ASSERT_FALSE(uw.rgb.empty()) << type;
    const auto w = Water(type);
    const std::uint32_t v = uw.h / 2;
    std::vector<double> got, want;
    for (std::uint32_t u = 0; u < uw.w; u += 2)
    {
      const std::size_t i = v * uw.w + u;
      if (!(uw.depth[i] > 4))
        continue;  // the plate, not the backdrop
      double factor = 1;
      const Vector3d ray = Ray(uw, u, v, &factor);
      got.push_back(Lin(uw, i, 1));
      want.push_back(Beam(ray, uw.depth[i] * factor, w)[1]);
    }
    const double peak = *std::max_element(want.begin(), want.end());
    double worst = 0;
    for (std::size_t k = 0; k < got.size(); ++k)
      worst = std::max(worst, std::abs(got[k] - want[k]) / peak);
    std::cout << type << ": beam peak " << peak << ", worst " << 100 * worst
              << " % of it, over " << got.size() << " pixels\n";
    EXPECT_GT(peak, 0.005) << type << ": no beam in view";
    EXPECT_LT(worst, 0.05) << type;
  }
}

/////////////////////////////////////////////////
// Clear water carries the beam farther across the view; murkier water
// keeps it nearer the lamp. (Brighter near the lens is not one of them:
// 35 cm from the lens, the murkiest water fades the lamp's light before it
// reaches the view.)
TEST_F(LampsTest, ClearReachesFartherMurkyStaysNear)
{
  const auto glow = [](const Frame &_f, std::uint32_t _u)
  {
    return Lin(_f, (_f.h / 2) * _f.w + _u, 1);
  };
  ASSERT_FALSE(frames["IB"].rgb.empty());
  const std::uint32_t w = frames["IB"].w;
  const std::uint32_t far = 3 * w / 4, nearU = w / 16;
  EXPECT_GT(glow(frames["IB"], far), glow(frames["3C"], far));
  EXPECT_GT(glow(frames["3C"], far), glow(frames["7C"], far));
  const auto ratio = [&](const char *_t)
  { return glow(frames[_t], nearU) / std::max(glow(frames[_t], far), 1e-4); };
  EXPECT_GT(ratio("7C"), ratio("3C"));
  EXPECT_GT(ratio("3C"), ratio("IB"));
  // In clear water there is nothing to scatter: no beam.
  EXPECT_LT(glow(frames["clear"], far), 0.002);
}
