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

// The drops show in a camera's images. A server renders camera sensors
// (test/worlds/lens_drops_camera.sdf, lens_drops_horizon.sdf) with the lens
// drops system, stepped from the test: the render pass must go on the
// camera; a dry lens, or one with <enabled> false, must leave the image as
// it is; rain must change it and a wipe give the dry image back; a drop is
// a lens and its mask is stamped as a frame; two cameras, one with noise
// and distortion, get their own drops; the map is drawn per frame only
// when the drops changed; removing a camera while it renders is safe; and
// a shader that does not compile turns the pass off, not the camera.
//
// It needs a GPU; without one no image comes and it skips, unless
// GZ_LENS_DROPS_EXPECT_GPU is set, when it fails.

#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief A topic of a camera's lens drops in the camera world.
/// \param[in] _model The camera's model.
/// \param[in] _leaf `set`, `state` or `mask`.
/// \return The topic.
std::string Topic(const std::string &_model, const std::string &_leaf)
{
  return "/world/lens_drops_camera/model/" + _model +
      "/link/link/sensor/camera/lens_drops/" + _leaf;
}

/// \brief Stamp of a message's header, in nanoseconds.
/// \param[in] _header The header.
/// \return The stamp.
std::int64_t Stamp(const msgs::Header &_header)
{
  return _header.stamp().sec() * 1000000000LL + _header.stamp().nsec();
}

/// \brief A camera's images.
class Images
{
  /// \brief Subscribe.
  /// \param[in] _topic Topic.
  public: explicit Images(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &Images::OnImage, this);
  }

  /// \brief How many images came.
  /// \return The count.
  public: std::uint64_t Count()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->count;
  }

  /// \brief The last image's pixels.
  /// \return Them.
  public: std::string Last()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->last;
  }

  /// \brief Whether an image came with a stamp.
  /// \param[in] _stamp Nanoseconds.
  /// \return True if one did.
  public: bool HadStamp(std::int64_t _stamp)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->stamps.count(_stamp) > 0;
  }

  /// \brief An image came.
  /// \param[in] _msg The image.
  private: void OnImage(const msgs::Image &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->last = _msg.data();
    this->stamps.insert(Stamp(_msg.header()));
    ++this->count;
  }

  /// \brief Guards the rest.
  private: std::mutex mutex;

  /// \brief The last image's pixels.
  private: std::string last;

  /// \brief The stamps of every image.
  private: std::set<std::int64_t> stamps;

  /// \brief How many images came.
  private: std::uint64_t count{0};

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief Bytes that differ between two images by more than a little.
/// \param[in] _a One image.
/// \param[in] _b The other.
/// \return The count of bytes that differ by more than 24.
int Changed(const std::string &_a, const std::string &_b)
{
  if (_a.size() != _b.size())
    return static_cast<int>(std::max(_a.size(), _b.size()));
  int n{0};
  for (std::size_t i = 0; i < _a.size(); ++i)
  {
    const int d = static_cast<unsigned char>(_a[i]) -
        static_cast<unsigned char>(_b[i]);
    if (std::abs(d) > 24)
      ++n;
  }
  return n;
}

/// \brief The last state a lens published.
class State
{
  /// \brief Subscribe.
  /// \param[in] _topic The state topic.
  public: explicit State(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &State::OnState, this);
  }

  /// \brief A value of the last state.
  /// \param[in] _key Its key.
  /// \return It, as a double, or NaN before it came.
  public: double Value(const std::string &_key)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    const auto it = this->values.find(_key);
    return it == this->values.end() ?
        std::numeric_limits<double>::quiet_NaN() : it->second;
  }

  /// \brief How many states came.
  /// \return The count.
  public: std::uint64_t Seen()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->seen;
  }

  /// \brief A state came.
  /// \param[in] _msg The state.
  private: void OnState(const msgs::Param &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    ++this->seen;
    for (const auto &[key, value] : _msg.params())
    {
      switch (value.type())
      {
        case msgs::Any::DOUBLE:
          this->values[key] = value.double_value();
          break;
        case msgs::Any::INT32:
          this->values[key] = value.int_value();
          break;
        case msgs::Any::BOOLEAN:
          this->values[key] = value.bool_value() ? 1.0 : 0.0;
          break;
        default:
          break;
      }
    }
  }

  /// \brief Guards the rest.
  private: std::mutex mutex;

  /// \brief How many states came.
  private: std::uint64_t seen{0};

  /// \brief The last state's values.
  private: std::map<std::string, double> values;

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief Publish one set message.
/// \param[in] _topic The set topic.
/// \param[in] _values Keys and numbers.
/// \return True if someone was subscribed.
bool Set(const std::string &_topic,
         const std::map<std::string, double> &_values)
{
  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(_topic);
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!pub.HasConnections())
    return false;
  msgs::Param msg;
  for (const auto &[key, value] : _values)
  {
    msgs::Any &any = (*msg.mutable_params())[key];
    any.set_type(msgs::Any::DOUBLE);
    any.set_double_value(value);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pub.Publish(msg);
  return true;
}

/// \brief A server on a world, rendered headless and stepped from the test.
/// \param[in] _world The world's file in TEST_WORLD_DIR.
/// \return The server.
std::unique_ptr<Server> Start(const std::string &_world)
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
  config.SetHeadlessRendering(true);
  return std::make_unique<Server>(config);
}

/// \brief Step until a condition holds, a camera frame (100 steps) at a
/// time, letting transport deliver between.
/// \param[in,out] _server The server.
/// \param[in] _ok The condition.
/// \param[in] _frames At most this many frames.
/// \return True if it held.
bool StepUntil(Server &_server, const std::function<bool()> &_ok,
               int _frames)
{
  for (int i = 0; i < _frames; ++i)
  {
    if (_ok())
      return true;
    _server.Run(true, 100, false);
    // Images go out on the render thread; let them land
    for (int k = 0; k < 25 && !_ok(); ++k)
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  return _ok();
}

/// \brief Step a few frames and return the last image after them.
/// \param[in,out] _server The server.
/// \param[in,out] _images The camera's images.
/// \param[in] _frames At least this many new images.
/// \return The image, or none if they did not come.
std::optional<std::string> Frame(Server &_server, Images &_images,
                                 int _frames = 2)
{
  const std::uint64_t seen = _images.Count();
  if (!StepUntil(_server, [&]() { return _images.Count() >=
          seen + static_cast<std::uint64_t>(_frames); }, 10 * _frames))
  {
    return std::nullopt;
  }
  return _images.Last();
}

/// \brief Whether this machine must render.
/// \return True if GZ_LENS_DROPS_EXPECT_GPU is set and not empty.
bool ExpectGpu()
{
  const char *env = std::getenv("GZ_LENS_DROPS_EXPECT_GPU");
  return env != nullptr && *env != '\0';
}
}  // namespace

/// \brief Skip, or fail where a GPU is expected, when no image came.
#define REQUIRE_RENDERING(_image)                                          \
  if (!(_image))                                                           \
  {                                                                        \
    if (ExpectGpu())                                                       \
      FAIL() << "no camera image, and GZ_LENS_DROPS_EXPECT_GPU is set";    \
    GTEST_SKIP() << "no camera image: this machine cannot render";         \
  }

//////////////////////////////////////////////////
TEST(LensDropsCamera, DropsShowAndClear)
{
  Images images("/lens_drops_camera/image");
  State state(Topic("camera", "state"));
  auto server = Start("lens_drops_camera.sdf");

  const auto a = Frame(*server, images, 3);
  REQUIRE_RENDERING(a);
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("attached") == 1.0; }, 20))
      << "the render pass never went on the camera";

  // A dry lens on a still scene changes nothing, and draws nothing after
  // the first pull: the pass is off.
  const double drawn = state.Value("draws");
  EXPECT_LE(drawn, 1.0);
  const auto b = Frame(*server, images, 5);
  ASSERT_TRUE(b);
  EXPECT_LT(Changed(*a, *b), 10);
  StepUntil(*server, []() { return false; }, 2);
  EXPECT_EQ(state.Value("draws"), drawn);

  // Rain puts drops on the image.
  ASSERT_TRUE(Set(Topic("camera", "set"), {{"rain_rate", 1000.0}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("count") >= 5.0; }, 100))
      << "the rain left no drops";
  const auto c = Frame(*server, images);
  ASSERT_TRUE(c);
  EXPECT_GT(Changed(*a, *c), 300);

  // A wipe gives the dry image back.
  ASSERT_TRUE(Set(Topic("camera", "set"), {{"rain_rate", 0.0}, {"clear", 1.0}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("count") == 0.0; }, 30))
      << "the wipe left drops";
  const auto d = Frame(*server, images);
  ASSERT_TRUE(d);
  EXPECT_LT(Changed(*a, *d), 10);
}

//////////////////////////////////////////////////
// <enabled> false keeps the drops but leaves the image as it is; true
// shows them again.
TEST(LensDropsCamera, DisabledLeavesTheImage)
{
  Images images("/lens_drops_camera/image");
  State state(Topic("camera", "state"));
  auto server = Start("lens_drops_camera.sdf");
  const auto dry = Frame(*server, images, 3);
  REQUIRE_RENDERING(dry);
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("attached") == 1.0; }, 20));

  ASSERT_TRUE(Set(Topic("camera", "set"), {{"enabled", 0.0},
      {"add_x", 0.5}, {"add_y", 0.45}, {"add_radius", 0.1}}));
  ASSERT_TRUE(StepUntil(*server, [&]() {
      return state.Value("count") == 1.0 && state.Value("enabled") == 0.0; },
      30));
  const auto off = Frame(*server, images, 3);
  ASSERT_TRUE(off);
  EXPECT_LT(Changed(*dry, *off), 10) << "drops drawn while disabled";

  ASSERT_TRUE(Set(Topic("camera", "set"), {{"enabled", 1.0}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("enabled") == 1.0; }, 30));
  const auto on = Frame(*server, images, 3);
  ASSERT_TRUE(on);
  EXPECT_GT(Changed(*dry, *on), 300);
}

//////////////////////////////////////////////////
// The map is drawn for the frames the camera renders, and only when the
// drops changed: pinned drops that never dry are drawn once.
TEST(LensDropsCamera, DrawsOnlyWhenChanged)
{
  Images images("/lens_drops_camera/image");
  State state(Topic("camera", "state"));
  auto server = Start("lens_drops_camera.sdf");
  REQUIRE_RENDERING(Frame(*server, images, 3));

  // Small enough to stay pinned on a level camera: 0.01 is under the slide
  // radius
  ASSERT_TRUE(Set(Topic("camera", "set"),
      {{"add_x", 0.3}, {"add_y", 0.5}, {"add_radius", 0.01}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("draws") >= 1.0; }, 30));
  ASSERT_TRUE(Frame(*server, images, 2));
  const double draws = state.Value("draws");
  const std::uint64_t frames = images.Count();
  ASSERT_TRUE(Frame(*server, images, 20));
  // The state lags a frame or two behind the images
  ASSERT_TRUE(Frame(*server, images, 2));
  std::cout << "[ camera   ] " << images.Count() - frames << " frames, "
            << state.Value("draws") - draws << " maps drawn\n";
  EXPECT_LE(state.Value("draws") - draws, 1.0);

  // Rain: at most one map per frame, however many steps
  ASSERT_TRUE(Set(Topic("camera", "set"), {{"rain_rate", 1000.0}}));
  ASSERT_TRUE(Frame(*server, images, 3));
  const double before = state.Value("draws");
  const std::uint64_t start = images.Count();
  ASSERT_TRUE(Frame(*server, images, 20));
  // Two more frames for the state to catch up
  StepUntil(*server, []() { return false; }, 2);
  const double drawn = state.Value("draws") - before;
  const double rendered = static_cast<double>(images.Count() - start);
  std::cout << "[ camera   ] raining: " << rendered << " frames, " << drawn
            << " maps drawn\n";
  EXPECT_GT(drawn, 0.0);
  EXPECT_LE(drawn, rendered + 2.0);
}

//////////////////////////////////////////////////
// Two cameras in one world each get their own drops; the second has noise
// and distortion, and its drops are drawn over them.
TEST(LensDropsCamera, TwoCamerasOneWithNoise)
{
  Images first("/lens_drops_camera/image");
  Images second("/lens_drops_camera/image2");
  State one(Topic("camera", "state"));
  State two(Topic("camera2", "state"));
  auto server = Start("lens_drops_camera.sdf");
  const auto dryOne = Frame(*server, first, 3);
  REQUIRE_RENDERING(dryOne);
  const auto dryTwo = Frame(*server, second, 1);
  ASSERT_TRUE(dryTwo);
  ASSERT_TRUE(StepUntil(*server, [&]() {
      return one.Value("attached") == 1.0 && two.Value("attached") == 1.0; },
      20));

  for (const char *model : {"camera", "camera2"})
    ASSERT_TRUE(Set(Topic(model, "set"), {{"rain_rate", 1000.0}}));
  ASSERT_TRUE(StepUntil(*server, [&]() {
      return one.Value("count") >= 5.0 && two.Value("count") >= 5.0; }, 100));
  const auto wetOne = Frame(*server, first);
  const auto wetTwo = Frame(*server, second);
  ASSERT_TRUE(wetOne && wetTwo);
  EXPECT_GT(Changed(*dryOne, *wetOne), 300);
  EXPECT_GT(Changed(*dryTwo, *wetTwo), 300);
  // Their own seeds: not the same drops
  EXPECT_NE(one.Value("covered"), two.Value("covered"));
}

//////////////////////////////////////////////////
// A camera removed while it renders with drops: the world goes on, and the
// other camera too.
TEST(LensDropsCamera, RemovedWhileRendering)
{
  Images first("/lens_drops_camera/image");
  Images second("/lens_drops_camera/image2");
  State one(Topic("camera", "state"));
  auto server = Start("lens_drops_camera.sdf");
  REQUIRE_RENDERING(Frame(*server, first, 3));
  ASSERT_TRUE(Set(Topic("camera", "set"), {{"rain_rate", 1000.0}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return one.Value("draws") >= 1.0; }, 100));

  ASSERT_TRUE(server->RequestRemoveEntity("camera"));
  // A few steps at a time, so the removal lands between frames and while
  // one renders
  for (int i = 0; i < 20; ++i)
    server->Run(true, 7, false);
  const std::uint64_t states = one.Seen();
  const auto after = Frame(*server, second, 5);
  ASSERT_TRUE(after) << "the other camera stopped";
  EXPECT_LE(one.Seen(), states + 1) << "the removed camera's lens lives on";
}

//////////////////////////////////////////////////
TEST(LensDropsCamera, RefractionInvertsHorizon)
{
  // A drop is a lens: a drop centred on the horizon shows the sea in its
  // upper part and the sky in its lower part, the other way round from the
  // scene beside it. With a contact angle of 55 degrees, the water's
  // surface 0.8 radius from the centre is tilted 41 degrees; the ray there,
  // 3.5 degrees above the horizon in this camera, enters the flat water and
  // leaves its surface about 13 degrees below the horizon, and its mirror
  // image below the centre about 13 above.
  const std::string base =
      "/world/lens_drops_horizon/model/camera/link/link/sensor/camera/"
      "lens_drops";
  Images images("/lens_drops_horizon/image");
  State state(base + "/state");

  // The mask's last image.
  std::mutex maskMutex;
  std::string maskData;
  std::int64_t maskStamp{-1};
  unsigned int maskWidth{0};
  unsigned int maskHeight{0};
  transport::Node node;
  std::function<void(const msgs::Image &)> onMask =
      [&](const msgs::Image &_msg)
  {
    const std::lock_guard<std::mutex> lock(maskMutex);
    maskData = _msg.data();
    maskStamp = Stamp(_msg.header());
    maskWidth = _msg.width();
    maskHeight = _msg.height();
  };
  node.Subscribe(base + "/mask", onMask);

  auto server = Start("lens_drops_horizon.sdf");
  REQUIRE_RENDERING(Frame(*server, images, 3));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("attached") == 1.0; }, 20));

  const double x = 0.5;
  const double y = 0.5;
  const double r = 0.1;
  ASSERT_TRUE(Set(base + "/set",
      {{"add_x", x}, {"add_y", y}, {"add_radius", r}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("covered") > 0.0; }, 30))
      << "the drop never reached the map";
  const auto image = Frame(*server, images);
  ASSERT_TRUE(image);

  const int width = 320;
  const int height = 180;
  ASSERT_EQ(image->size(), static_cast<std::size_t>(width * height * 3));
  // Grey level at an image point, x across the width, y down the height.
  const auto grey = [&](double _x, double _y)
  {
    const int i = static_cast<int>(_x * width);
    const int j = static_cast<int>(_y * height);
    const std::size_t k = (static_cast<std::size_t>(j) * width + i) * 3;
    return (static_cast<unsigned char>((*image)[k]) +
            static_cast<unsigned char>((*image)[k + 1]) +
            static_cast<unsigned char>((*image)[k + 2])) / 3;
  };
  const double aspect = static_cast<double>(width) / height;
  const double side = x + 3.0 * r / aspect;
  const int upper = grey(x, y - 0.8 * r);
  const int lower = grey(x, y + 0.8 * r);
  const int upperBeside = grey(side, y - 0.8 * r);
  const int lowerBeside = grey(side, y + 0.8 * r);
  EXPECT_GE(lower - upper, 40)
      << "upper " << upper << ", lower " << lower << ": the drop does not "
      << "show the sea up and the sky down";
  EXPECT_GE(upperBeside - upper, 40)
      << "upper " << upper << ", beside it " << upperBeside;
  EXPECT_GE(lower - lowerBeside, 40)
      << "lower " << lower << ", beside it " << lowerBeside;

  // The mask has the drop, at the map's size, stamped as a frame.
  const auto hasDrop = [&]()
  {
    const std::lock_guard<std::mutex> lock(maskMutex);
    return !maskData.empty() &&
        std::count(maskData.begin(), maskData.end(), '\xff') > 0;
  };
  ASSERT_TRUE(StepUntil(*server, hasDrop, 20)) << "no mask with the drop came";
  // Its frame's image may land after it
  ASSERT_TRUE(Frame(*server, images, 2));
  {
    const std::lock_guard<std::mutex> lock(maskMutex);
    ASSERT_EQ(maskData.size(), static_cast<std::size_t>(maskWidth) * maskHeight);
    const double set = static_cast<double>(
        std::count(maskData.begin(), maskData.end(), '\xff'));
    const double expected = 3.141592653589793 *
        (r * maskHeight) * (r * maskHeight);
    EXPECT_NEAR(set, expected, 0.15 * expected);
    EXPECT_TRUE(images.HadStamp(maskStamp))
        << "the mask's stamp " << maskStamp << " is no frame's";
  }
}

//////////////////////////////////////////////////
// A shader that does not compile turns the pass off: the camera goes on
// publishing the image as it is.
TEST(LensDropsCamera, BrokenShaderLeavesTheImage)
{
  ::setenv("GZ_LENS_DROPS_SHADER_PATH", TEST_BROKEN_SHADER_DIR, 1);
  struct Restore
  {
    ~Restore() { ::setenv("GZ_LENS_DROPS_SHADER_PATH", TEST_SHADER_DIR, 1); }
  } restore;

  Images images("/lens_drops_horizon/image");
  const std::string base =
      "/world/lens_drops_horizon/model/camera/link/link/sensor/camera/"
      "lens_drops";
  State state(base + "/state");
  auto server = Start("lens_drops_horizon.sdf");
  const auto dry = Frame(*server, images, 3);
  REQUIRE_RENDERING(dry);
  ASSERT_TRUE(Set(base + "/set",
      {{"add_x", 0.5}, {"add_y", 0.5}, {"add_radius", 0.1}}));
  ASSERT_TRUE(StepUntil(*server,
      [&]() { return state.Value("count") == 1.0; }, 30));
  const std::uint64_t before = images.Count();
  const auto wet = Frame(*server, images, 5);
  ASSERT_TRUE(wet) << "the camera stopped with a broken shader";
  EXPECT_GE(images.Count(), before + 5);
  EXPECT_LT(Changed(*dry, *wet), 10);
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  const std::string partition =
      "gz_lens_drops_camera_test_" + std::to_string(::getpid());
  ::setenv("GZ_PARTITION", partition.c_str(), 1);
  return RUN_ALL_TESTS();
}
