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

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;

TEST(Snell, WindowAt48Point6Degrees)
{
  std::mutex mutex;
  std::optional<msgs::Image> frame;
  transport::Node node;
  std::function<void(const msgs::Image &)> cb = [&](const msgs::Image &_m)
  {
    if (_m.header().stamp().sec() < 2)
      return;
    const std::lock_guard<std::mutex> lock(mutex);
    frame = _m;
  };
  node.Subscribe("/uwsnell/cam", cb);
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "snell.sdf"));
  auto server = std::make_unique<Server>(config);
  server->Run(false, 0, false);
  for (int i = 0; i < 600; ++i)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (frame)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  server.reset();
  const std::lock_guard<std::mutex> lock(mutex);
  ASSERT_TRUE(frame) << "no frame: is there a GPU?";

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
