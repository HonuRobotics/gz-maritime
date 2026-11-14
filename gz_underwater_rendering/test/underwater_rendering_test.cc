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

#include <gz/msgs/image.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief One frame of every topic the test reads, from one run.
struct Capture
{
  std::string underImage, underWater, aboveImage, labels;
  std::vector<float> depth;
  std::uint32_t width{0}, height{0};
};

/// \brief Run a world until every topic has published past 1 s, and keep
/// one frame of each.
Capture CaptureWorld(const std::string &_world)
{
  Capture c;
  std::mutex mutex;
  std::map<std::string, msgs::Image> latest;
  transport::Node node;
  const auto keep = [&](const std::string &_topic)
  {
    std::function<void(const msgs::Image &)> cb =
        [&mutex, &latest, _topic](const msgs::Image &_m)
        {
          if (_m.header().stamp().sec() < 1)
            return;
          const std::lock_guard<std::mutex> lock(mutex);
          latest[_topic] = _m;
        };
    node.Subscribe(_topic, cb);
  };
  const std::vector<std::string> topics{
      "/uwgpu/under/image", "/uwgpu/under/underwater/image",
      "/uwgpu/under/depth_image", "/uwgpu/above/image",
      "/uwgpu/seg/labels_map"};
  for (const auto &t : topics)
    keep(t);

  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
  auto server = std::make_unique<Server>(config);
  server->Run(false, 0, false);
  for (int i = 0; i < 600; ++i)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (latest.size() == topics.size())
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  server.reset();
  const std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(latest.size(), topics.size()) << _world << ": topics missing";
  if (latest.size() != topics.size())
    return c;
  c.underImage = latest["/uwgpu/under/image"].data();
  c.underWater = latest["/uwgpu/under/underwater/image"].data();
  c.aboveImage = latest["/uwgpu/above/image"].data();
  c.labels = latest["/uwgpu/seg/labels_map"].data();
  const auto &d = latest["/uwgpu/under/depth_image"];
  c.width = d.width();
  c.height = d.height();
  c.depth.resize(c.width * c.height);
  std::memcpy(c.depth.data(), d.data().data(), c.depth.size() * 4);
  return c;
}

class UnderwaterRenderingTest : public ::testing::Test
{
  /// \brief The same scene, without and then with the GPU path, once.
  protected: static void SetUpTestSuite()
  {
    off = CaptureWorld("gpu_off.sdf");
    on = CaptureWorld("gpu_on.sdf");
  }
  protected: static Capture off, on;
};
Capture UnderwaterRenderingTest::off;
Capture UnderwaterRenderingTest::on;
}  // namespace

/////////////////////////////////////////////////
// The GPU path's image against the reference camera's, per pixel: the
// renderer's own colour output with the GPU path, against the reference
// camera's underwater image without it.
TEST_F(UnderwaterRenderingTest, MatchesTheReferenceCamera)
{
  ASSERT_FALSE(on.underImage.empty());
  ASSERT_EQ(on.underImage.size(), off.underWater.size());
  std::vector<int> diffs;
  diffs.reserve(on.underImage.size());
  for (std::size_t i = 0; i < on.underImage.size(); ++i)
  {
    diffs.push_back(std::abs(
        static_cast<int>(static_cast<std::uint8_t>(on.underImage[i])) -
        static_cast<int>(static_cast<std::uint8_t>(off.underWater[i]))));
  }
  double mean = 0;
  for (int d : diffs)
    mean += d;
  mean /= diffs.size();
  std::sort(diffs.begin(), diffs.end());
  const int p99 = diffs[diffs.size() * 99 / 100];
  std::cout << "GPU against reference: mean " << mean << ", 99th percentile "
            << p99 << ", worst " << diffs.back() << " steps\n";
  EXPECT_LE(mean, 1.0);
  EXPECT_LE(p99, 3);
  // And the GPU image is really under water: not the in-air image.
  EXPECT_NE(on.underImage, off.underImage);
}

/////////////////////////////////////////////////
// The optics change colour, never depth.
TEST_F(UnderwaterRenderingTest, DepthUntouched)
{
  ASSERT_FALSE(on.depth.empty());
  EXPECT_EQ(on.depth, off.depth);
}

/////////////////////////////////////////////////
// A camera above the water is untouched.
TEST_F(UnderwaterRenderingTest, AboveWaterUntouched)
{
  ASSERT_FALSE(on.aboveImage.empty());
  EXPECT_EQ(on.aboveImage, off.aboveImage);
}

/////////////////////////////////////////////////
// Segmentation labels keep their exact values.
TEST_F(UnderwaterRenderingTest, SegmentationUntouched)
{
  ASSERT_FALSE(on.labels.empty());
  EXPECT_EQ(on.labels, off.labels);
}

/////////////////////////////////////////////////
// With the GPU path on, the reference camera passes its image through,
// rather than apply the water a second time.
TEST_F(UnderwaterRenderingTest, ReferenceCameraPassesThrough)
{
  ASSERT_FALSE(on.underWater.empty());
  EXPECT_EQ(on.underWater, on.underImage);
}

/////////////////////////////////////////////////
// The model is in the reference docs.
TEST(Docs, Documented)
{
  std::ifstream in(DOCS_REFERENCE);
  std::stringstream ss;
  ss << in.rdbuf();
  EXPECT_NE(ss.str().find("underwater_optics"), std::string::npos);
}
