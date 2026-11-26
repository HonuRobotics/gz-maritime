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
#ifndef GZ_UNDERWATER_RENDERING_TEST_CAPTUREWORLD_HH_
#define GZ_UNDERWATER_RENDERING_TEST_CAPTUREWORLD_HH_

#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
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

// The scene of test/worlds/gpu_on.sdf and gpu_off.sdf, captured: shared by
// the tests that run it in one process and those that need a process of
// their own.
namespace gz::sim::underwater_test
{
/// \brief One frame of every topic the test reads, from one run.
struct Capture
{
  std::string underImage, underWater, aboveImage, labels;
  std::vector<float> depth;
  std::uint32_t width{0}, height{0};

  /// \brief Whether any frame came at all: none means no GPU.
  bool rendered{false};
};

/// \brief Run a world until every topic has published past 1 s, and keep
/// one frame of each.
/// \param[in] _world The world file, in the test worlds.
/// \param[in] _action If given, called once every topic has published past
/// 1 s; then only frames 1 s after the latest of those count.
inline Capture CaptureWorld(const std::string &_world,
    const std::function<void(transport::Node &)> &_action = {})
{
  Capture c;
  std::mutex mutex;
  std::map<std::string, msgs::Image> latest;
  // Frames before this sim time, in s, do not count
  double from = 1.0;
  bool any = false;
  transport::Node node;
  const auto keep = [&](const std::string &_topic)
  {
    std::function<void(const msgs::Image &)> cb =
        [&mutex, &latest, &from, &any, _topic](const msgs::Image &_m)
        {
          const double t = _m.header().stamp().sec() +
                           1e-9 * _m.header().stamp().nsec();
          const std::lock_guard<std::mutex> lock(mutex);
          any = true;
          if (t >= from)
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
  // Until every topic has a frame that counts; true if they all do
  const auto wait = [&]()
  {
    for (int i = 0; i < 600; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(mutex);
        if (latest.size() == topics.size())
          return true;
        // Nothing at all in 10 s: nothing renders here
        if (i == 200 && !any)
          return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  };
  if (wait() && _action)
  {
    _action(node);
    const std::lock_guard<std::mutex> lock(mutex);
    double last = 0.0;
    for (const auto &[topic, m] : latest)
    {
      last = std::max(last, m.header().stamp().sec() +
                            1e-9 * m.header().stamp().nsec());
    }
    from = last + 1.0;
    latest.clear();
  }
  if (_action)
    wait();
  server.reset();
  const std::lock_guard<std::mutex> lock(mutex);
  c.rendered = any;
  if (!any)
    return c;
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
}  // namespace gz::sim::underwater_test

#endif
