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
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "OwnPartition.hh"

#include "gz/sim/underwater_camera/UnderwaterImage.hh"

using namespace gz;
using namespace sim;
namespace uc = underwater_camera;

namespace
{
/// \brief A world running, its camera's colour and depth by stamp.
class WorldRun
{
  public: WorldRun(const std::string &_world, const std::string &_topic)
  {
    const auto keep = [this](bool _depth)
    {
      return std::function<void(const msgs::Image &)>(
          [this, _depth](const msgs::Image &_m)
          {
            const std::lock_guard<std::mutex> lock(this->mutex);
            (_depth ? this->depths : this->images)[
                _m.header().stamp().sec() * 1000000000LL +
                _m.header().stamp().nsec()] = _m;
          });
    };
    this->node.Subscribe(_topic + "/image", keep(false));
    this->node.Subscribe(_topic + "/depth_image", keep(true));
    ServerConfig config;
    config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, _world));
    this->server = std::make_unique<Server>(config);
  }

  /// \brief A few steps of the world. The test steps it: a server on a
  /// thread of its own can hang when it is destroyed (gz-sim 10.5, in its
  /// Sensors system).
  public: void Step()
  {
    this->server->Run(true, 5, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  /// \brief Whether any frame came: none means no GPU.
  public: bool Rendered()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return !this->images.empty();
  }

  /// \brief The newest stamp so far, ns.
  public: std::int64_t Latest()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->images.empty() ? 0 : this->images.rbegin()->first;
  }

  /// \brief Of the first frame with both images after a stamp: the plate's
  /// mean and the backdrop's brightest, linear green, middle rows.
  /// \return Whether there was one, within 20 s.
  public: bool Measure(std::int64_t _after, double &_plate, double &_backdrop)
  {
    for (int i = 0; i < 2000; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(this->mutex);
        for (const auto &[stamp, im] : this->images)
        {
          if (stamp <= _after || !this->depths.count(stamp))
            continue;
          const auto &dp = this->depths[stamp];
          std::vector<float> z(dp.width() * dp.height());
          std::memcpy(z.data(), dp.data().data(), z.size() * 4);
          int n = 0;
          _plate = 0;
          _backdrop = 0;
          for (std::uint32_t v = im.height() / 3; v < 2 * im.height() / 3; ++v)
          {
            for (std::uint32_t u = 0; u < im.width(); ++u)
            {
              const std::size_t k = v * im.width() + u;
              const double g = uc::SrgbToLinear(static_cast<std::uint8_t>(
                  im.data()[v * im.step() + 3 * u + 1]));
              if (z[k] > 0 && z[k] < 4)
              {
                _plate += g;
                ++n;
              }
              else if (z[k] > 4)
              {
                _backdrop = std::max(_backdrop, g);
              }
            }
          }
          _plate /= std::max(n, 1);
          return n > 50;
        }
      }
      this->Step();
    }
    return false;
  }

  /// \brief Of the first frame with both images after a stamp: the mean
  /// linear green of what is nearer than 4 m in each quarter of the image's
  /// width, middle rows.
  /// \param[in] _after Stamp, ns: the frame must come after it.
  /// \param[out] _mean The four means, left to right.
  /// \return Whether there was one.
  public: bool Quarters(std::int64_t _after, std::array<double, 4> &_mean)
  {
    for (int i = 0; i < 2000; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(this->mutex);
        for (const auto &[stamp, im] : this->images)
        {
          if (stamp <= _after || !this->depths.count(stamp))
            continue;
          const auto &dp = this->depths[stamp];
          std::vector<float> z(dp.width() * dp.height());
          std::memcpy(z.data(), dp.data().data(), z.size() * 4);
          std::array<int, 4> n{};
          _mean = {};
          for (std::uint32_t v = im.height() / 3; v < 2 * im.height() / 3; ++v)
          {
            for (std::uint32_t u = 0; u < im.width(); ++u)
            {
              const std::size_t k = v * im.width() + u;
              if (!(z[k] > 0 && z[k] < 4))
                continue;
              const std::size_t q = 4 * u / im.width();
              _mean[q] += uc::SrgbToLinear(static_cast<std::uint8_t>(
                  im.data()[v * im.step() + 3 * u + 1]));
              ++n[q];
            }
          }
          for (int q = 0; q < 4; ++q)
            _mean[q] /= std::max(n[q], 1);
          return true;
        }
      }
      this->Step();
    }
    return false;
  }

  public: std::unique_ptr<Server> server;
  private: std::mutex mutex;
  private: std::map<std::int64_t, msgs::Image> images, depths;
  /// \brief Last, so it goes first: nothing is delivered after the maps go.
  public: transport::Node node;
};

/// \brief Wait for the first frame; false if nothing renders here.
bool Started(WorldRun &_run)
{
  for (int i = 0; i < 100 && !_run.Rendered(); ++i)
    _run.Step();
  return _run.Rendered();
}
}  // namespace

//////////////////////////////////////////////////
// Five lamps, one more than the shader takes. The four it takes light
// nothing in view: one whose range lies wholly behind the camera (its
// interval along each view ray is reversed) and three aimed behind it; none
// draws a beam. The fifth, past the limit, is aimed at the plate and is lit
// by Ogre, as in air: the plate is lit, and with no beam, since Ogre draws
// none.
TEST(LampEdges, MoreLampsThanTheLimitAndLampsOutOfView)
{
  WorldRun run("lamp_edges.sdf", "/uwlampedge/cam");
  if (!Started(run))
    GTEST_SKIP() << "nothing rendered: no GPU?";
  double plate = 0, backdrop = 0;
  ASSERT_TRUE(run.Measure(1000000000LL, plate, backdrop));
  run.server.reset();
  std::cout << "five lamps: plate " << plate << ", brightest backdrop "
            << backdrop << "\n";
  EXPECT_GT(plate, 0.02) << "the lamp past the limit lit nothing";
  EXPECT_LT(backdrop, 0.002) << "a beam from a lamp that lights no view";
}

//////////////////////////////////////////////////
// A lamp on a vehicle: its beam shows; the vehicle is removed while the
// world runs, and the lamp, its light and its beam are gone, nothing left
// pointing at it; then the world shuts down.
TEST(LampEdges, LampRemovedThenShutdown)
{
  WorldRun run("lamp_removed.sdf", "/uwlampgone/cam");
  if (!Started(run))
    GTEST_SKIP() << "nothing rendered: no GPU?";
  double plate = 0, backdrop = 0;
  ASSERT_TRUE(run.Measure(1000000000LL, plate, backdrop));
  std::cout << "with the lamp: plate " << plate << ", brightest backdrop "
            << backdrop << "\n";
  EXPECT_GT(plate, 0.02);
  EXPECT_GT(backdrop, 0.005) << "no beam to begin with";

  msgs::Entity req;
  req.set_name("rov");
  req.set_type(msgs::Entity::MODEL);
  msgs::Boolean rep;
  bool result = false, sent = false;
  std::atomic<bool> done{false};
  std::thread request([&]
  {
    sent = run.node.Request("/world/uwlampgone/remove", req, 5000, rep,
                            result);
    done = true;
  });
  while (!done)
    run.Step();
  request.join();
  ASSERT_TRUE(sent);
  ASSERT_TRUE(result && rep.data());
  // A second of simulation later
  ASSERT_TRUE(run.Measure(run.Latest() + 1000000000LL, plate, backdrop));
  std::cout << "lamp removed: plate " << plate << ", brightest backdrop "
            << backdrop << "\n";
  EXPECT_LT(plate, 0.002) << "the removed lamp still lights";
  EXPECT_LT(backdrop, 0.002) << "the removed lamp's beam is still drawn";
  run.server.reset();
}

//////////////////////////////////////////////////
// The renderer's model removed while a lamp shines: the lamp goes back to
// Ogre, which lights the plate with it as in air, and the beam is gone.
TEST(LampEdges, LampsGoBackToOgreWhenTheRendererGoes)
{
  WorldRun run("lamp_removed.sdf", "/uwlampgone/cam");
  if (!Started(run))
    GTEST_SKIP() << "nothing rendered: no GPU?";
  double plate = 0, backdrop = 0;
  ASSERT_TRUE(run.Measure(1000000000LL, plate, backdrop));
  EXPECT_GT(backdrop, 0.005) << "no beam to begin with";

  msgs::Entity req;
  req.set_name("underwater_optics");
  req.set_type(msgs::Entity::MODEL);
  msgs::Boolean rep;
  bool result = false, sent = false;
  std::atomic<bool> done{false};
  std::thread request([&]
  {
    sent = run.node.Request("/world/uwlampgone/remove", req, 5000, rep,
                            result);
    done = true;
  });
  while (!done)
    run.Step();
  request.join();
  ASSERT_TRUE(sent);
  ASSERT_TRUE(result && rep.data());

  const double withRenderer = plate;
  ASSERT_TRUE(run.Measure(run.Latest() + 1000000000LL, plate, backdrop));
  std::cout << "renderer removed: plate " << plate << " (" << withRenderer
            << " with it), brightest backdrop " << backdrop << "\n";
  EXPECT_GT(plate, withRenderer) << "the lamp stays hidden from Ogre: "
                                    "nobody lights with it";
  EXPECT_LT(backdrop, 0.002) << "a beam with no renderer";
  run.server.reset();
}

//////////////////////////////////////////////////
// Four lamps at <quality>low</quality>, which lights two in the shader:
// the other two are left to Ogre, not hidden from it, so each of the four
// plates is lit by its lamp.
TEST(LampEdges, LowQualityLeavesTheRestToOgre)
{
  WorldRun run("lamps_low.sdf", "/uwlamplow/cam");
  if (!Started(run))
    GTEST_SKIP() << "nothing rendered: no GPU?";
  std::array<double, 4> plate{};
  ASSERT_TRUE(run.Quarters(1000000000LL, plate));
  run.server.reset();
  std::cout << "four lamps at low quality, plates:";
  for (double p : plate)
    std::cout << " " << p;
  std::cout << "\n";
  for (int q = 0; q < 4; ++q)
    EXPECT_GT(plate[q], 0.02) << "plate " << q << " is dark";
}
