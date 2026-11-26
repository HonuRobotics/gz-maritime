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
// The camera plugin fed frames by the test, in worlds with no Sensors
// system: nothing is rendered, so no GPU is needed, and each frame is known.
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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
constexpr std::uint32_t kWidth = 8, kHeight = 6;

/// \brief A world, stepped by the test.
std::unique_ptr<Server> Start(const std::string &_world)
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
  auto server = std::make_unique<Server>(config);
  server->Run(true, 1, false);
  return server;
}

/// \brief A colour frame of one grey level, stamped.
msgs::Image Colour(std::uint8_t _level, double _stamp,
                   std::uint32_t _width = kWidth,
                   msgs::PixelFormatType _format =
                       msgs::PixelFormatType::RGB_INT8)
{
  msgs::Image m;
  m.mutable_header()->mutable_stamp()->set_sec(
      static_cast<std::int64_t>(_stamp));
  m.mutable_header()->mutable_stamp()->set_nsec(static_cast<std::int32_t>(
      std::lround((_stamp - std::floor(_stamp)) * 1e9)));
  m.set_width(_width);
  m.set_height(kHeight);
  m.set_step(_width * 3);
  m.set_pixel_format_type(_format);
  std::vector<std::uint8_t> data(_width * kHeight * 3, _level);
  m.set_data(data.data(), data.size());
  return m;
}

/// \brief A depth frame, everything 2 m away, stamped like a colour frame.
msgs::Image Depth(const msgs::Image &_colour)
{
  msgs::Image m;
  *m.mutable_header() = _colour.header();
  m.set_width(kWidth);
  m.set_height(kHeight);
  m.set_step(kWidth * 4);
  m.set_pixel_format_type(msgs::PixelFormatType::R_FLOAT32);
  std::vector<float> z(kWidth * kHeight, 2.0f);
  m.set_data(z.data(), z.size() * sizeof(float));
  return m;
}

/// \brief The camera's intrinsics.
msgs::CameraInfo Info()
{
  msgs::CameraInfo m;
  m.set_width(kWidth);
  m.set_height(kHeight);
  for (const double v : {4.0, 0.0, 4.0, 0.0, 4.0, 3.0, 0.0, 0.0, 1.0})
    m.mutable_intrinsics()->add_k(v);
  return m;
}

/// \brief One camera as the test sees it: it publishes the camera's frames
/// and keeps the underwater images by stamp.
class Camera
{
  public: Camera(transport::Node &_node, const std::string &_base)
    : image(_node.Advertise<msgs::Image>(_base + "/image")),
      depth(_node.Advertise<msgs::Image>(_base + "/depth_image")),
      info(_node.Advertise<msgs::CameraInfo>(_base + "/camera_info")),
      base(_base), node(_node)
  {
  }

  /// \brief Subscribe to the underwater image.
  public: void Watch()
  {
    std::function<void(const msgs::Image &)> cb =
        [this](const msgs::Image &_m)
        {
          const std::lock_guard<std::mutex> lock(this->mutex);
          const double stamp = _m.header().stamp().sec() +
                               1e-9 * _m.header().stamp().nsec();
          this->out[std::lround(stamp * 1000)] = _m.data();
        };
    this->node.Subscribe(this->base + "/underwater/image", cb);
  }

  /// \brief Whether the plugin listens to the camera.
  public: bool Heard() { return this->image.HasConnections(); }

  /// \brief Publish a frame pair, with the intrinsics.
  public: void Send(const msgs::Image &_colour,
                    std::optional<msgs::Image> _depth = std::nullopt)
  {
    this->info.Publish(Info());
    this->image.Publish(_colour);
    this->depth.Publish(_depth ? *_depth : Depth(_colour));
  }

  /// \brief The underwater image of a stamp, waited for.
  public: std::optional<std::string> Out(double _stamp, int _waitMs = 3000)
  {
    for (int i = 0; i < _waitMs / 20; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(this->mutex);
        const auto it = this->out.find(std::lround(_stamp * 1000));
        if (it != this->out.end())
          return it->second;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return std::nullopt;
  }

  public: transport::Node::Publisher image, depth, info;
  public: std::string base;
  public: transport::Node &node;
  public: std::mutex mutex;
  public: std::map<long, std::string> out;
};

/// \brief Wait until the publisher has a subscriber, stepping the world.
bool WaitHeard(Server &_server, Camera &_cam)
{
  for (int i = 0; i < 100 && !_cam.Heard(); ++i)
  {
    _server.Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return _cam.Heard();
}
}  // namespace

/////////////////////////////////////////////////
// With no water optics the camera is in clear water: the image passes
// through, the background below the horizon included.
TEST(Frames, PassThroughWithoutWaterOptics)
{
  auto server = Start("frames_dry.sdf");
  transport::Node node;
  Camera cam(node, "/fdry/cam");
  cam.Watch();
  ASSERT_TRUE(WaitHeard(*server, cam));
  server->Run(true, 10, false);
  const auto in = Colour(120, 0.05);
  // Nothing hit anywhere: the depth image is empty
  msgs::Image empty = Depth(in);
  std::vector<float> none(kWidth * kHeight,
                          std::numeric_limits<float>::infinity());
  empty.set_data(none.data(), none.size() * sizeof(float));
  cam.Send(in, empty);
  const auto out = cam.Out(0.05);
  ASSERT_TRUE(out);
  EXPECT_EQ(*out, in.data());
}

/////////////////////////////////////////////////
// The plugin listens to the camera only while someone listens to it, so
// the camera is not rendered for it alone.
TEST(Frames, ListensOnlyWhileListenedTo)
{
  auto server = Start("frames.sdf");
  transport::Node node;
  Camera cam(node, "/fcam/cam");
  server->Run(true, 20, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_FALSE(cam.Heard()) << "listening with nobody listening to it";

  {
    transport::Node watcher;
    Camera watching(watcher, "/fcam/cam");
    watching.Watch();
    EXPECT_TRUE(WaitHeard(*server, cam));
  }
  // The watcher gone: it stops
  for (int i = 0; i < 100 && cam.Heard(); ++i)
  {
    server->Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_FALSE(cam.Heard());
}

/////////////////////////////////////////////////
// A frame is made with the water of the step it was rendered at, not of
// the latest step.
TEST(Frames, TheWaterOfTheFramesOwnStep)
{
  auto server = Start("frames.sdf");
  transport::Node node;
  Camera cam(node, "/fcam/cam");
  cam.Watch();
  ASSERT_TRUE(WaitHeard(*server, cam));
  server->Run(true, 10, false);  // to about 0.1 s, in 3C
  cam.Send(Colour(200, 0.05));
  const auto before = cam.Out(0.05);
  ASSERT_TRUE(before);

  // Murkier water from now on
  auto pub = node.Advertise<msgs::Param>("/world/frames/water_optics/set");
  for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  msgs::Param murky;
  (*murky.mutable_params())["jerlov"].set_type(msgs::Any::STRING);
  (*murky.mutable_params())["jerlov"].set_string_value("9C");
  pub.Publish(murky);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  server->Run(true, 40, false);  // to about 0.5 s, in 9C

  // A frame of 0.05 s again: the water of then
  cam.Send(Colour(200, 0.051));
  const auto again = cam.Out(0.051);
  ASSERT_TRUE(again);
  EXPECT_EQ(*again, *before);
  // A frame of now: the murkier water
  cam.Send(Colour(200, 0.45));
  const auto now = cam.Out(0.45);
  ASSERT_TRUE(now);
  EXPECT_NE(*now, *before);
}

/////////////////////////////////////////////////
// Frames that do not fit the camera are skipped, and the next good one is
// made.
TEST(Frames, FramesThatDoNotFitAreSkipped)
{
  auto server = Start("frames.sdf");
  transport::Node node;
  Camera cam(node, "/fcam/cam");
  cam.Watch();
  ASSERT_TRUE(WaitHeard(*server, cam));
  server->Run(true, 10, false);

  // Another size, another format, a colour image shorter than its size
  cam.Send(Colour(100, 0.01, kWidth + 2));
  cam.Send(Colour(100, 0.02, kWidth, msgs::PixelFormatType::L_INT8));
  msgs::Image shortFrame = Colour(100, 0.03);
  shortFrame.set_data(shortFrame.data().substr(0, 10));
  cam.Send(shortFrame);
  for (const double stamp : {0.01, 0.02, 0.03})
    EXPECT_FALSE(cam.Out(stamp, 300)) << stamp;

  cam.Send(Colour(100, 0.04));
  EXPECT_TRUE(cam.Out(0.04));
}

/////////////////////////////////////////////////
// Each camera of a stereo pair gets its own noise, the same every run.
TEST(Frames, EachCameraItsOwnRepeatableNoise)
{
  std::map<std::string, std::string> first;
  for (int run = 0; run < 2; ++run)
  {
    auto server = Start("frames.sdf");
    transport::Node node;
    Camera left(node, "/fcam/left"), right(node, "/fcam/right");
    left.Watch();
    right.Watch();
    ASSERT_TRUE(WaitHeard(*server, left));
    ASSERT_TRUE(WaitHeard(*server, right));
    server->Run(true, 10, false);
    left.Send(Colour(128, 0.05));
    right.Send(Colour(128, 0.05));
    const auto l = left.Out(0.05), r = right.Out(0.05);
    ASSERT_TRUE(l);
    ASSERT_TRUE(r);
    EXPECT_NE(*l, *r) << "the same noise in both cameras";
    if (run == 0)
    {
      first["left"] = *l;
      first["right"] = *r;
    }
    else
    {
      EXPECT_EQ(*l, first["left"]) << "not the same as the run before";
      EXPECT_EQ(*r, first["right"]) << "not the same as the run before";
    }
  }
}

/////////////////////////////////////////////////
// After a world reset, time goes back, and frames are still made.
TEST(Frames, AfterAReset)
{
  auto server = Start("frames.sdf");
  transport::Node node;
  Camera cam(node, "/fcam/cam");
  cam.Watch();
  ASSERT_TRUE(WaitHeard(*server, cam));
  server->Run(true, 100, false);

  msgs::WorldControl control;
  control.mutable_reset()->set_all(true);
  msgs::Boolean reply;
  bool result{false};
  std::thread request([&]
  {
    node.Request("/world/frames/control", control, 5000, reply, result);
  });
  for (int i = 0; i < 20; ++i)
  {
    server->Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  request.join();
  ASSERT_TRUE(result);
  server->Run(true, 10, false);
  cam.Send(Colour(90, 0.05));
  EXPECT_TRUE(cam.Out(0.05));
}

/////////////////////////////////////////////////
// Where it does not belong it does nothing; parameters that make no sense
// fall back to their defaults, and the camera works.
TEST(Frames, ConfigureErrors)
{
  auto server = Start("frames_errors.sdf");
  transport::Node node;
  std::vector<std::string> topics;
  node.TopicList(topics);
  for (const auto &t : topics)
    EXPECT_EQ(t.find("/ferr/plain/underwater"), std::string::npos) << t;

  Camera odd(node, "/ferr/odd");
  odd.Watch();
  ASSERT_TRUE(WaitHeard(*server, odd));
  server->Run(true, 10, false);
  odd.Send(Colour(150, 0.05));
  EXPECT_TRUE(odd.Out(0.05));
}

/////////////////////////////////////////////////
// The system destroyed while frames flow: nothing touches it after. Run
// under AddressSanitizer this caught a use after free.
TEST(Frames, DestroyedWhileFramesFlow)
{
  for (int round = 0; round < 5; ++round)
  {
    auto server = Start("frames.sdf");
    transport::Node node;
    Camera cam(node, "/fcam/cam");
    cam.Watch();
    ASSERT_TRUE(WaitHeard(*server, cam));
    server->Run(true, 5, false);
    std::atomic<bool> stop{false};
    std::thread flood([&]
    {
      for (int i = 0; !stop; ++i)
        cam.Send(Colour(static_cast<std::uint8_t>(i), 0.01 * (i % 5)));
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    server.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    stop = true;
    flood.join();
  }
  SUCCEED();
}
