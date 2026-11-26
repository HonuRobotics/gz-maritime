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

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>

#include "CaptureWorld.hh"

using namespace gz;
using sim::underwater_test::Capture;
using sim::underwater_test::CaptureWorld;

namespace
{
/// \brief The mean difference of two images, in 8-bit steps.
double MeanDiff(const std::string &_a, const std::string &_b)
{
  if (_a.size() != _b.size() || _a.empty())
    return 255.0;
  double sum = 0.0;
  for (std::size_t i = 0; i < _a.size(); ++i)
  {
    sum += std::abs(static_cast<int>(static_cast<std::uint8_t>(_a[i])) -
                    static_cast<int>(static_cast<std::uint8_t>(_b[i])));
  }
  return sum / _a.size();
}

/// \brief Once the renderer stops drawing the water, the scene looks as
/// without it: the in-air image as in a world without the GPU path, and the
/// reference camera drawing the water instead, to a step or so.
void ExpectAsWithout(const Capture &_off, const Capture &_now,
                     const std::string &_what)
{
  ASSERT_FALSE(_now.underImage.empty()) << _what;
  EXPECT_EQ(_now.underImage, _off.underImage)
      << _what << ": the renderer still draws the water";
  const double mean = MeanDiff(_now.underWater, _off.underWater);
  std::cout << _what << ": reference camera against the run without the "
            << "GPU path, mean " << mean << " steps\n";
  EXPECT_LE(mean, 1.0) << _what << ": the reference camera did not take over";
  EXPECT_NE(_now.underWater, _now.underImage) << _what;
}
}  // namespace

/////////////////////////////////////////////////
// The GPU path turned off and on while running, and its model removed. One
// process, in order: once installed, the renderer stays installed for the
// rest of the process, drawing nothing when told so.
TEST(UnderwaterRenderingRuntime, OffOnAndRemoved)
{
  const Capture off = CaptureWorld("gpu_off.sdf");
  if (!off.rendered)
    GTEST_SKIP() << "nothing rendered: no GPU?";

  // Off on its topic
  const auto enable = [](bool _on)
  {
    return [_on](transport::Node &_node)
    {
      auto pub = _node.Advertise<msgs::Boolean>(
          "/world/uwgpu/underwater_rendering/enable");
      msgs::Boolean msg;
      msg.set_data(_on);
      for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      pub.Publish(msg);
    };
  };
  ExpectAsWithout(off, CaptureWorld("gpu_on.sdf", enable(false)), "off");

  // And on again: drawn by the renderer, the reference camera passing it
  const Capture on = CaptureWorld("gpu_on.sdf", [&](transport::Node &_node)
  {
    enable(false)(_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    enable(true)(_node);
  });
  ASSERT_FALSE(on.underImage.empty());
  EXPECT_NE(on.underImage, off.underImage) << "on again: no water drawn";
  EXPECT_EQ(on.underWater, on.underImage) << "on again: drawn twice";

  // Its model removed: no plugin is left to take the world's mark off
  ExpectAsWithout(off, CaptureWorld("gpu_on.sdf", [](transport::Node &_node)
  {
    msgs::Entity req;
    req.set_name("underwater_optics");
    req.set_type(msgs::Entity::MODEL);
    msgs::Boolean rep;
    bool result = false;
    EXPECT_TRUE(_node.Request("/world/uwgpu/remove", req, 5000, rep, result));
    EXPECT_TRUE(result && rep.data());
  }), "removed");
}
