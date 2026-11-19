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

// The drops show in a camera's images and go when the lens is wiped. A
// server renders a camera sensor (test/worlds/lens_drops_camera.sdf) with
// the lens drops system: the render pass must go on the camera, a dry lens
// must leave the image as it is, rain must change it, and a wipe must give
// the dry image back. It needs a GPU; without one, no image comes and it
// skips.

#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <mutex>
#include <optional>
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
/// \brief The base of the camera's lens drops topics.
const char kBase[] =
    "/world/lens_drops_camera/model/camera/link/link/sensor/camera/lens_drops";

/// \brief The camera's images.
class Images
{
  /// \brief Subscribe.
  /// \param[in] _topic Topic.
  public: explicit Images(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &Images::OnImage, this);
  }

  /// \brief An image taken after now, or none within a timeout.
  /// \param[in] _timeout How long to wait.
  /// \return Its pixels.
  public: std::optional<std::string> Next(std::chrono::seconds _timeout)
  {
    std::uint64_t seen{0};
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      seen = this->count;
    }
    const auto end = std::chrono::steady_clock::now() + _timeout;
    while (std::chrono::steady_clock::now() < end)
    {
      {
        const std::lock_guard<std::mutex> lock(this->mutex);
        // Two images on, so the one returned was rendered after the call.
        if (this->count >= seen + 2)
          return this->last;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return std::nullopt;
  }

  /// \brief An image came.
  /// \param[in] _msg The image.
  private: void OnImage(const msgs::Image &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->last = _msg.data();
    ++this->count;
  }

  /// \brief Guards the rest.
  private: std::mutex mutex;

  /// \brief The last image's pixels.
  private: std::string last;

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

/// \brief The last state the lens published.
class State
{
  /// \brief Subscribe.
  /// \param[in] _topic The state topic.
  public: explicit State(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &State::OnState, this);
  }

  /// \brief Drops on the lens.
  /// \return The count, or -1 before a state came.
  public: int Count()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->seen ? this->count : -1;
  }

  /// \brief Share of the drop map with water on it.
  /// \return The share, or -1 before a state came.
  public: double Covered()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->seen ? this->covered : -1.0;
  }

  /// \brief Whether the render pass is on the camera.
  /// \return The flag.
  public: bool Attached()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->attached;
  }

  /// \brief Wait for a condition on the state.
  /// \param[in] _ok The condition.
  /// \param[in] _timeout How long to wait.
  /// \return True if it held in time.
  public: template <typename F>
  bool WaitUntil(F _ok, std::chrono::seconds _timeout)
  {
    const auto end = std::chrono::steady_clock::now() + _timeout;
    while (std::chrono::steady_clock::now() < end)
    {
      if (_ok())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return _ok();
  }

  /// \brief A state came.
  /// \param[in] _msg The state.
  private: void OnState(const msgs::Param &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->seen = true;
    for (const auto &[key, value] : _msg.params())
    {
      if (key == "count")
        this->count = value.int_value();
      else if (key == "covered")
        this->covered = value.double_value();
      else if (key == "attached")
        this->attached = value.bool_value();
    }
  }

  /// \brief Guards the rest.
  private: std::mutex mutex;

  /// \brief True once a state came.
  private: bool seen{false};

  /// \brief The last state's count.
  private: int count{0};

  /// \brief The last state's covered share.
  private: double covered{0.0};

  /// \brief The last state's attached flag.
  private: bool attached{false};

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
}  // namespace

//////////////////////////////////////////////////
TEST(LensDropsCamera, DropsShowAndClear)
{
  const std::string base = kBase;
  Images images("/lens_drops_camera/image");
  State state(base + "/state");

  ServerConfig config;
  config.SetSdfFile(
      common::joinPaths(TEST_WORLD_DIR, "lens_drops_camera.sdf"));
  config.SetHeadlessRendering(true);
  Server server(config);
  server.Run(false, 0, false);

  const auto a = images.Next(std::chrono::seconds(20));
  if (!a)
  {
    server.Stop();
    GTEST_SKIP() << "no camera image: this machine cannot render";
  }

  ASSERT_TRUE(state.WaitUntil([&]() { return state.Attached(); },
      std::chrono::seconds(10))) << "the render pass never went on the camera";

  // A dry lens on a still scene changes nothing.
  const auto b = images.Next(std::chrono::seconds(10));
  ASSERT_TRUE(b);
  EXPECT_LT(Changed(*a, *b), 10);

  // Rain puts drops on the image.
  ASSERT_TRUE(Set(base + "/set", {{"rain_rate", 1000.0}}));
  ASSERT_TRUE(state.WaitUntil([&]() { return state.Count() >= 5; },
      std::chrono::seconds(20))) << "the rain left no drops";
  const auto c = images.Next(std::chrono::seconds(10));
  ASSERT_TRUE(c);
  EXPECT_GT(Changed(*a, *c), 300);

  // A wipe gives the dry image back.
  ASSERT_TRUE(Set(base + "/set", {{"rain_rate", 0.0}, {"clear", 1.0}}));
  ASSERT_TRUE(state.WaitUntil([&]() { return state.Count() == 0; },
      std::chrono::seconds(10))) << "the wipe left drops";
  const auto d = images.Next(std::chrono::seconds(10));
  ASSERT_TRUE(d);
  EXPECT_LT(Changed(*a, *d), 10);

  server.Stop();
}

//////////////////////////////////////////////////
TEST(LensDropsCamera, RefractionInvertsHorizon)
{
  // A drop is a lens: a drop centred on the horizon shows the sea in its
  // upper part and the sky in its lower part, the other way round from the
  // scene beside it. With a contact angle of 55 degrees, the water's
  // surface 0.8 radius from the centre is tilted 41 degrees; the ray there,
  // 3.5 degrees above the horizon in this camera, leaves the water about 13
  // degrees below it, and its mirror image below the centre about 13 above.
  const std::string base =
      "/world/lens_drops_horizon/model/camera/link/link/sensor/camera/"
      "lens_drops";
  Images images("/lens_drops_horizon/image");
  State state(base + "/state");

  // The mask's last image.
  std::mutex maskMutex;
  std::string maskData;
  unsigned int maskWidth{0};
  unsigned int maskHeight{0};
  transport::Node node;
  std::function<void(const msgs::Image &)> onMask =
      [&](const msgs::Image &_msg)
  {
    const std::lock_guard<std::mutex> lock(maskMutex);
    maskData = _msg.data();
    maskWidth = _msg.width();
    maskHeight = _msg.height();
  };
  node.Subscribe(base + "/mask", onMask);

  ServerConfig config;
  config.SetSdfFile(
      common::joinPaths(TEST_WORLD_DIR, "lens_drops_horizon.sdf"));
  config.SetHeadlessRendering(true);
  Server server(config);
  server.Run(false, 0, false);

  if (!images.Next(std::chrono::seconds(20)))
  {
    server.Stop();
    GTEST_SKIP() << "no camera image: this machine cannot render";
  }
  ASSERT_TRUE(state.WaitUntil([&]() { return state.Attached(); },
      std::chrono::seconds(10)));

  const double x = 0.5;
  const double y = 0.5;
  const double r = 0.1;
  ASSERT_TRUE(Set(base + "/set",
      {{"add_x", x}, {"add_y", y}, {"add_radius", r}}));
  ASSERT_TRUE(state.WaitUntil([&]() { return state.Covered() > 0.0; },
      std::chrono::seconds(10))) << "the drop never reached the map";
  const auto image = images.Next(std::chrono::seconds(10));
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

  // The mask has the drop, at the map's size.
  bool gotMask = false;
  for (int k = 0; k < 250 && !gotMask; ++k)
  {
    {
      const std::lock_guard<std::mutex> lock(maskMutex);
      gotMask = !maskData.empty() &&
          std::count(maskData.begin(), maskData.end(), '\xff') > 0;
    }
    if (!gotMask)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(gotMask) << "no mask with the drop came";
  {
    const std::lock_guard<std::mutex> lock(maskMutex);
    ASSERT_EQ(maskData.size(), static_cast<std::size_t>(maskWidth) * maskHeight);
    const double set = static_cast<double>(
        std::count(maskData.begin(), maskData.end(), '\xff'));
    const double expected = 3.141592653589793 *
        (r * maskHeight) * (r * maskHeight);
    EXPECT_NEAR(set, expected, 0.15 * expected);
  }

  server.Stop();
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
