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
// A vehicle's camera never sees what the panels draw, because the panels
// draw only through the GUI's marker service. A server renders a camera
// sensor looking down at an empty scene. The panels' markers, sent where the
// panels send them, the GUI's `/marker_array`, which the server does not
// serve, leave the image as it was. Sent to the sensors' own service,
// `/sensors/marker_array`, the same markers show in it, `visibility: GUI`
// notwithstanding: the flag alone protects nothing, and a panel must never
// draw there. A marker visible to all, there, shows too, so the test can see
// a marker at all. It needs a GPU; without one, no image comes and it skips.

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/marker_v.pb.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/maritime_gui/Acoustics.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/maritime_gui/Flow.hh"
#include "gz/sim/maritime_gui/MarkerLayer.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

namespace
{
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

  /// \brief The last image's pixels and how many came.
  private: std::string last;
  private: std::uint64_t count{0};

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief Pixels that differ between two images by more than a little.
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

/// \brief Send markers to a marker array service and wait for the answer.
/// \param[in] _service The service.
/// \param[in] _markers The markers.
/// \return True if something served it and took them.
bool Send(const std::string &_service, const std::vector<msgs::Marker> &_markers)
{
  transport::Node node;
  msgs::Marker_V req;
  for (const auto &m : _markers)
    *req.add_marker() = m;
  msgs::Boolean rep;
  bool result{false};
  return node.Request(_service, req, 2000, rep, result) && result;
}

/// \brief A white square under the camera, the way the panels build a
/// marker (SolidMarker: unlit, emissive, visible to the GUI only), or with
/// another visibility.
/// \param[in] _visibility Who sees it.
/// \return The marker.
msgs::Marker Square(msgs::Marker::Visibility _visibility)
{
  const double h = 3.0;
  const std::vector<math::Vector3d> tris{
      {-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, -h, 0}, {h, h, 0}, {-h, h, 0},
      {-h, -h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}, {-h, h, 0}, {h, h, 0}};
  auto m = SolidMarker("test/square", 1, msgs::Marker::TRIANGLE_LIST,
                       math::Color::White, tris);
  m.set_visibility(_visibility);
  return m;
}

/// \brief Whether this machine must render, as on CI's GPU runners:
/// GZ_MARITIME_REQUIRE_RENDERING is set to anything but empty or 0.
/// \return True when a missing camera image fails the test instead of
/// skipping it.
bool RenderingRequired()
{
  const char *value = std::getenv("GZ_MARITIME_REQUIRE_RENDERING");
  if (value == nullptr)
    return false;
  const std::string required(value);
  return !required.empty() && required != "0";
}
}

//////////////////////////////////////////////////
TEST(MarkersCamera, PanelMarkersStayOutOfCameras)
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "camera.sdf"));
  config.SetHeadlessRendering(true);
  Server server(config);
  Images images("/marker_test/image");
  server.Run(false, 0, false);

  const auto before = images.Next(std::chrono::seconds(20));
  if (!before)
  {
    if (RenderingRequired())
    {
      FAIL() << "no camera image, and GZ_MARITIME_REQUIRE_RENDERING is set: "
                "this machine must render";
    }
    GTEST_SKIP() << "no camera image: this machine cannot render";
  }

  // The panels' drawings: the acoustics panel's reach and a solid square,
  // both built as every panel builds its markers.
  PingerView pinger;
  pinger.data.reach = 4.0;
  pinger.position = math::Vector3d(0, 0, -1);
  auto panel = ReachMarkers({pinger}, 0.0, "test/reach");
  panel.push_back(Square(msgs::Marker::GUI));

  // Where the panels send them by default, kMarkerService, which every
  // panel and every layer takes its default from: not the sensors' service,
  // and one the server does not serve. That is the whole protection, so it
  // is the default that is checked, not a name written here; the unchanged
  // image then follows.
  const std::string panels = std::string(kMarkerService) + "_array";
  const std::string sensorsService = "/sensors/marker_array";
  ASSERT_NE(sensorsService, panels);
  EXPECT_FALSE(Send(panels, panel));
  const auto gui = images.Next(std::chrono::seconds(5));
  ASSERT_TRUE(gui.has_value());
  EXPECT_EQ(0, Changed(*before, *gui));

  // Sent to the sensors' own service, the same markers show: the GUI
  // visibility flag does not keep them out of a camera. Only drawing through
  // the GUI's service does.
  EXPECT_TRUE(Send(sensorsService, panel));
  const auto sensors = images.Next(std::chrono::seconds(5));
  ASSERT_TRUE(sensors.has_value());
  EXPECT_GT(Changed(*before, *sensors), 100);

  // Taken off again, the image is the first one: what follows is seen on
  // an empty scene, not on top of the markers above.
  std::vector<msgs::Marker> off;
  for (const auto &m : panel)
    off.push_back(DeleteMarker(m.ns(), m.id()));
  EXPECT_TRUE(Send(sensorsService, off));
  const auto cleared = images.Next(std::chrono::seconds(5));
  ASSERT_TRUE(cleared.has_value());
  EXPECT_EQ(0, Changed(*before, *cleared));

  // A marker visible to all, there, shows as well: the camera sees markers.
  auto all = Square(msgs::Marker::ALL);
  all.set_ns("test/control");
  EXPECT_TRUE(Send(sensorsService, {all}));
  const auto control = images.Next(std::chrono::seconds(5));
  ASSERT_TRUE(control.has_value());
  EXPECT_GT(Changed(*before, *control), 100);

  server.Stop();
}
