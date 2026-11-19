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

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <cstring>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/underwater_camera/UnderwaterImage.hh"
#include "gz/sim/water_optics/WaterOptics.hh"

using namespace gz;
using namespace sim;
namespace uc = underwater_camera;

namespace
{
/// \brief Frames of one camera, by stamp: in air, depth, under water.
struct Frames
{
  std::mutex mutex;
  std::map<std::int64_t, msgs::Image> image, depth, under;
  std::optional<msgs::CameraInfo> info;

  static std::int64_t Key(const msgs::Image &_m)
  {
    return _m.header().stamp().sec() * 1000000000LL +
           _m.header().stamp().nsec();
  }

  void Listen(transport::Node &_node, const std::string &_base)
  {
    using ImageCb = std::function<void(const msgs::Image &)>;
    const auto into = [this](std::map<std::int64_t, msgs::Image> &_map)
    {
      return ImageCb([this, &_map](const msgs::Image &_m)
          { const std::lock_guard<std::mutex> l(mutex); _map[Key(_m)] = _m; });
    };
    _node.Subscribe(_base + "/image", into(image));
    _node.Subscribe(_base + "/depth_image", into(depth));
    _node.Subscribe(_base + "/underwater/image", into(under));
    std::function<void(const msgs::CameraInfo &)> onInfo =
        [this](const msgs::CameraInfo &_m)
        { const std::lock_guard<std::mutex> l(mutex); info = _m; };
    _node.Subscribe(_base + "/camera_info", onInfo);
  }

  /// \brief A stamp with all three frames, past the first second.
  std::optional<std::int64_t> Matched()
  {
    const std::lock_guard<std::mutex> l(mutex);
    for (const auto &[stamp, m] : under)
    {
      if (stamp > 1000000000LL && image.count(stamp) && depth.count(stamp))
        return stamp;
    }
    return std::nullopt;
  }
};

class UnderwaterCameraTest : public ::testing::Test
{
  protected: void SetUp() override
  {
    ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "camera.sdf"));
    this->server = std::make_unique<Server>(config);
    this->under.Listen(this->node, "/uwcam/under");
    this->above.Listen(this->node, "/uwcam/above");
    this->server->Run(false, 0, false);
  }

  protected: void TearDown() override
  {
    this->server.reset();
  }

  /// \brief Wait for a stamp with all three frames from a camera.
  protected: std::int64_t WaitFor(Frames &_f)
  {
    for (int i = 0; i < 600; ++i)
    {
      if (auto s = _f.Matched())
        return *s;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ADD_FAILURE() << "no matched frames in 30 s: is there a GPU?";
    return -1;
  }

  protected: transport::Node node;
  protected: std::unique_ptr<Server> server;
  protected: Frames under, above;
};
}  // namespace

/////////////////////////////////////////////////
// The published image, pixel by pixel, against the model computed here from
// the in-air frame of the same stamp, its depth, and Jerlov 3C at 3 m.
TEST_F(UnderwaterCameraTest, MatchesTheModelPerPixel)
{
  const auto stamp = this->WaitFor(this->under);
  ASSERT_GE(stamp, 0);
  const std::lock_guard<std::mutex> lock(this->under.mutex);
  const auto &img = this->under.image.at(stamp);
  const auto &dep = this->under.depth.at(stamp);
  const auto &out = this->under.under.at(stamp);
  ASSERT_TRUE(this->under.info);
  const auto &K = this->under.info->intrinsics().k();
  const double fx = K[0], cx = K[2], fy = K[4], cy = K[5];
  ASSERT_EQ(out.width(), img.width());
  ASSERT_EQ(out.height(), img.height());

  water_optics::WaterOpticsSpec spec;
  spec.jerlov = "3C";
  water_optics::WaterOpticsData optics;
  std::string error, warning;
  ASSERT_TRUE(water_optics::Resolve(spec, optics, error, warning));
  const double camDepth = 3.0;

  std::vector<float> z(img.width() * img.height());
  std::memcpy(z.data(), dep.data().data(), z.size() * sizeof(float));

  int worst = 0, hits = 0, misses = 0;
  double sumAbs = 0.0;
  for (std::uint32_t v = 0; v < img.height(); ++v)
  {
    for (std::uint32_t u = 0; u < img.width(); ++u)
    {
      const std::size_t i = v * img.width() + u;
      const double x = (u + 0.5 - cx) / fx;
      const double y = (v + 0.5 - cy) / fy;
      const double factor = std::sqrt(1 + x * x + y * y);
      const double rayUp = -y / factor;
      const bool hit = std::isfinite(z[i]) && z[i] > 0;
      hit ? ++hits : ++misses;
      for (int c = 0; c < 3; ++c)
      {
        const double veil = optics.veil[c] *
            std::exp(-optics.diffuseAttenuation[c] * camDepth);
        double expected;
        if (hit)
        {
          const double range = z[i] * factor;
          const double objDepth = std::max(0.0, camDepth - rayUp * range);
          // Only the water up to the surface, if the ray climbs out first.
          const double r = rayUp > 1e-9 ? std::min(range, camDepth / rayUp)
                                        : range;
          const double j = uc::SrgbToLinear(static_cast<std::uint8_t>(
              img.data()[v * img.step() + 3 * u + c]));
          const double t = std::exp(-optics.attenuation[c] * r);
          expected = j * std::exp(-optics.diffuseAttenuation[c] * objDepth) *
                     t + veil * (1 - std::exp(-optics.backscatter[c] * r));
        }
        else
        {
          // Nothing hit: water to the surface for a climbing ray, else all.
          const double r = rayUp > 1e-9 ? camDepth / rayUp : INFINITY;
          const double j = std::isfinite(r) ? uc::SrgbToLinear(
              static_cast<std::uint8_t>(
                  img.data()[v * img.step() + 3 * u + c])) : 0.0;
          const double t = std::isfinite(r)
              ? std::exp(-optics.attenuation[c] * r) : 0.0;
          expected = j * t + veil * (std::isfinite(r)
              ? 1 - std::exp(-optics.backscatter[c] * r) : 1.0);
        }
        const int e = uc::LinearToSrgb(expected);
        const int got = static_cast<std::uint8_t>(out.data()[3 * i + c]);
        worst = std::max(worst, std::abs(e - got));
        sumAbs += std::abs(e - got);
      }
    }
  }
  const double mean = sumAbs / (3.0 * img.width() * img.height());
  std::cout << "hits " << hits << ", misses " << misses << "; mean |diff| "
            << mean << ", worst " << worst << " steps\n";
  EXPECT_GT(hits, 1000) << "the camera saw no wall or plate";
  EXPECT_LE(mean, 0.05) << "mean difference, 8-bit steps";
  EXPECT_LE(worst, 1) << "worst pixel, 8-bit steps";
}

/////////////////////////////////////////////////
// The far wall fades into the veil more than the near plate: how far each
// is from the veil's colour, under water, against in air, in linear light.
TEST_F(UnderwaterCameraTest, FarFadesIntoTheVeil)
{
  const auto stamp = this->WaitFor(this->under);
  ASSERT_GE(stamp, 0);
  const std::lock_guard<std::mutex> lock(this->under.mutex);
  const auto &img = this->under.image.at(stamp);
  const auto &out = this->under.under.at(stamp);
  const std::uint32_t w = img.width(), h = img.height();

  water_optics::WaterOpticsSpec spec;
  spec.jerlov = "3C";
  water_optics::WaterOpticsData optics;
  std::string error, warning;
  ASSERT_TRUE(water_optics::Resolve(spec, optics, error, warning));
  uc::Rgb veil;
  for (int c = 0; c < 3; ++c)
    veil[c] = optics.veil[c] * std::exp(-optics.diffuseAttenuation[c] * 3.0);

  const auto colour = [&](const msgs::Image &_m, std::uint32_t _u)
  {
    const std::size_t i = (h / 2) * _m.step() + 3 * _u;
    return uc::Rgb(
        uc::SrgbToLinear(static_cast<std::uint8_t>(_m.data()[i])),
        uc::SrgbToLinear(static_cast<std::uint8_t>(_m.data()[i + 1])),
        uc::SrgbToLinear(static_cast<std::uint8_t>(_m.data()[i + 2])));
  };
  // The plate is left of centre (2.5 m), the wall right of it (6 m).
  const double plateKept = (colour(out, w / 4) - veil).Length() /
                           (colour(img, w / 4) - veil).Length();
  const double wallKept = (colour(out, 3 * w / 4) - veil).Length() /
                          (colour(img, 3 * w / 4) - veil).Length();
  std::cout << "kept apart from the veil: plate " << plateKept << ", wall "
            << wallKept << "\n";
  EXPECT_GT(plateKept, wallKept) << "nearer keeps more of itself";
  EXPECT_LT(wallKept, 0.1) << "at 6 m in 3C water (visibility 3.9 m) the "
                              "wall is all but gone";
  EXPECT_GT(plateKept, 0.03);
}

/////////////////////////////////////////////////
// Above the water the image is the plain camera's, byte for byte.
TEST_F(UnderwaterCameraTest, AboveWaterUnchanged)
{
  const auto stamp = this->WaitFor(this->above);
  ASSERT_GE(stamp, 0);
  const std::lock_guard<std::mutex> lock(this->above.mutex);
  EXPECT_EQ(this->above.under.at(stamp).data(),
            this->above.image.at(stamp).data());
}

/////////////////////////////////////////////////
// The depth image is untouched: the plugin publishes beside it, never on
// it, so a depth camera or a lidar sees what it saw.
TEST_F(UnderwaterCameraTest, DepthUntouched)
{
  const auto stamp = this->WaitFor(this->under);
  ASSERT_GE(stamp, 0);
  const std::lock_guard<std::mutex> lock(this->under.mutex);
  const auto &dep = this->under.depth.at(stamp);
  EXPECT_EQ(dep.pixel_format_type(), msgs::PixelFormatType::R_FLOAT32);
  std::vector<float> z(dep.width() * dep.height());
  std::memcpy(z.data(), dep.data().data(), z.size() * sizeof(float));
  // The wall, straight ahead and right of the plate, is 6 - 0.1 m away.
  const float centreRight = z[(dep.height() / 2) * dep.width() +
                              3 * dep.width() / 4];
  EXPECT_NEAR(centreRight, 5.9f, 0.01f);
}
