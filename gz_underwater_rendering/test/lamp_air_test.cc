/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

// A lamp seen from above the water lights as Ogre lights it: a metal plate
// and a glossy one, specular included, with and without the GPU path. A
// process of its own: once installed, the renderer stays installed.
#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
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
/// \brief A colour frame and its depth.
struct Frame
{
  std::string rgb;
  std::vector<float> depth;
  std::uint32_t w{0}, h{0};
};

/// \brief Step a world until a colour and depth frame of one stamp past
/// 1 s have come.
Frame Capture(const std::string &_world)
{
  std::mutex mutex;
  std::map<std::int64_t, msgs::Image> images, depths;
  transport::Node node;
  const auto keep = [&](bool _depth)
  {
    return std::function<void(const msgs::Image &)>(
        [&mutex, &images, &depths, _depth](const msgs::Image &_m)
        {
          const std::lock_guard<std::mutex> lock(mutex);
          (_depth ? depths : images)[_m.header().stamp().sec() * 1000000000LL +
                                     _m.header().stamp().nsec()] = _m;
        });
  };
  node.Subscribe("/uwair/cam/image", keep(false));
  node.Subscribe("/uwair/cam/depth_image", keep(true));

  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
  auto server = std::make_unique<Server>(config);
  Frame f;
  for (int i = 0; i < 2000 && f.rgb.empty(); ++i)
  {
    server->Run(true, 5, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const std::lock_guard<std::mutex> lock(mutex);
    if (i == 100 && images.empty())
      break;
    for (auto it = images.rbegin(); it != images.rend(); ++it)
    {
      if (it->first > 1000000000LL && depths.count(it->first))
      {
        f.w = it->second.width();
        f.h = it->second.height();
        f.rgb = it->second.data();
        f.depth.resize(f.w * f.h);
        std::memcpy(f.depth.data(), depths[it->first].data().data(),
                    f.depth.size() * 4);
        break;
      }
    }
  }
  server.reset();
  return f;
}

/// \brief One plate's pixels, each channel, from one half of the image.
std::vector<int> Plate(const Frame &_f, bool _left)
{
  std::vector<int> out;
  for (std::uint32_t v = 0; v < _f.h; ++v)
  {
    for (std::uint32_t u = _left ? 0 : _f.w / 2; u < (_left ? _f.w / 2 : _f.w);
         ++u)
    {
      const std::size_t i = v * _f.w + u;
      if (!(_f.depth[i] > 0.0f && _f.depth[i] < 4.0f))
        continue;
      for (int c = 0; c < 3; ++c)
        out.push_back(static_cast<std::uint8_t>(_f.rgb[3 * i + c]));
    }
  }
  return out;
}
}  // namespace

/////////////////////////////////////////////////
TEST(LampInAir, LightsAsOgreDoesSpecularIncluded)
{
  const Frame ogre = Capture("lamp_air_off.sdf");
  if (ogre.rgb.empty())
    GTEST_SKIP() << "nothing rendered: no GPU?";
  const Frame ours = Capture("lamp_air.sdf");
  ASSERT_FALSE(ours.rgb.empty());
  ASSERT_EQ(ogre.depth, ours.depth);

  for (const bool left : {true, false})
  {
    const std::string name = left ? "metal" : "glossy";
    const auto a = Plate(ogre, left), b = Plate(ours, left);
    ASSERT_GT(a.size(), 300u) << name << ": no plate in view";
    ASSERT_EQ(a.size(), b.size());
    double meanA = 0, meanB = 0;
    std::vector<int> diffs;
    for (std::size_t i = 0; i < a.size(); ++i)
    {
      meanA += a[i];
      meanB += b[i];
      diffs.push_back(std::abs(a[i] - b[i]));
    }
    meanA /= a.size();
    meanB /= b.size();
    std::sort(diffs.begin(), diffs.end());
    const int highlight = *std::max_element(a.begin(), a.end());
    std::cout << name << ": Ogre " << meanA << ", the shader " << meanB
              << "; 99th percentile " << diffs[diffs.size() * 99 / 100]
              << ", worst " << diffs.back() << " steps; brightest "
              << highlight << "\n";
    EXPECT_GT(meanA, 5.0) << name << ": the lamp lit nothing";
    EXPECT_NEAR(meanB, meanA, std::max(0.5, 0.02 * meanA)) << name;
    EXPECT_LE(diffs[diffs.size() * 99 / 100], 3) << name;
  }
}
