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

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/image.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gsw = gz::sim::waves;

namespace
{
/// \brief The token the test's wave field is registered under.
constexpr char kEngine[] = "drawn_surface_test";

/// \brief The tile of the test's wave field [m] and its cells per axis. The
/// grid is coarse on purpose, 4 m cells, so a surface drawn a fraction of a
/// cell off its field stands out.
constexpr double kTile = 128.0;
constexpr std::size_t kCells = 32;

/// \brief The depth camera of test/worlds/drawn_surface.sdf: its height over
/// the still water [m], straight down over the world origin.
constexpr double kCameraHeight = 60.0;

/// \brief Its horizontal field of view [rad].
constexpr double kHorizontalFov = 0.8;

/// \brief Its image topic.
constexpr char kDepthTopic[] = "/drawn_surface/depth";

/// \brief The sea is compared at the first frame from this time on [s].
constexpr double kCompareFrom = 2.0;

//////////////////////////////////////////////////
/// \brief A wave field for the test, so the renderer is tested without any
/// wave engine: two trains of sine waves that travel, with the sideways
/// displacement of a choppy sea, sampled on a grid. Like the FFT engine, its
/// elevation is the bilinear sample of that grid.
class GridWaveField : public gsw::IWaveField
{
  // Documentation inherited
  public: double Elevation(double _x, double _y, double) const override
  {
    const double cell = this->field.tile / static_cast<double>(this->field.n);
    const double fi = _x / cell;
    const double fj = _y / cell;
    const double fx = fi - std::floor(fi);
    const double fy = fj - std::floor(fj);
    const int n = static_cast<int>(this->field.n);
    const int i0 = ((static_cast<int>(std::floor(fi)) % n) + n) % n;
    const int j0 = ((static_cast<int>(std::floor(fj)) % n) + n) % n;
    const int i1 = (i0 + 1) % n;
    const int j1 = (j0 + 1) % n;
    auto at = [&](int _i, int _j) { return this->dz[_i + _j * n]; };
    return at(i0, j0) * (1 - fx) * (1 - fy) + at(i1, j0) * fx * (1 - fy) +
           at(i0, j1) * (1 - fx) * fy + at(i1, j1) * fx * fy;
  }

  // Documentation inherited
  public: gz::math::Vector3d ParticleVelocity(
      double, double, double) const override
  {
    return gz::math::Vector3d::Zero;
  }

  // Documentation inherited
  public: gz::math::Vector3d Normal(double, double, double) const override
  {
    return gz::math::Vector3d::UnitZ;
  }

  // Documentation inherited
  public: double Jacobian(double, double, double) const override
  {
    return 1.0;
  }

  // Documentation inherited
  public: void SetParameters(const gsw::WaveParameters &_params) override
  {
    const std::size_t n = _params.gridSize;
    this->dz.assign(n * n, 0.0);
    this->dx.assign(n * n, 0.0);
    this->dy.assign(n * n, 0.0);
    this->field.n = n;
    this->field.tile = _params.tileSize;
    this->field.dz = this->dz.data();
    this->field.dx = this->dx.data();
    this->field.dy = this->dy.data();
    this->Update(0.0);
  }

  // Documentation inherited
  public: void Update(double _simTime) override
  {
    // Whole numbers of waves over the tile, so the field is periodic. Each
    // train displaces the surface towards its crests, a quarter wave from
    // the height.
    struct Train
    {
      public: double amplitude;
      public: int wavesAlongX;
      public: int wavesAlongY;
      public: double omega;
    };
    static const Train kTrains[] = {{0.35, 2, 0, 1.1}, {0.20, 1, 3, 1.7}};
    const std::size_t n = this->field.n;
    const double step = 2.0 * M_PI / static_cast<double>(n);
    for (std::size_t j = 0; j < n; ++j)
    {
      for (std::size_t i = 0; i < n; ++i)
      {
        double height = 0.0;
        double alongX = 0.0;
        double alongY = 0.0;
        for (const Train &train : kTrains)
        {
          const double phase =
              step * (train.wavesAlongX * static_cast<double>(i) +
                      train.wavesAlongY * static_cast<double>(j)) -
              train.omega * _simTime;
          const double waves =
              std::hypot(train.wavesAlongX, train.wavesAlongY);
          height += train.amplitude * std::sin(phase);
          alongX -= train.amplitude * std::cos(phase) *
              train.wavesAlongX / waves;
          alongY -= train.amplitude * std::cos(phase) *
              train.wavesAlongY / waves;
        }
        this->dz[i + j * n] = height;
        this->dx[i + j * n] = alongX;
        this->dy[i + j * n] = alongY;
      }
    }
  }

  // Documentation inherited
  public: std::string_view Kind() const override { return kEngine; }

  // Documentation inherited
  public: const gsw::WaveField2D *Field() const override
  {
    return &this->field;
  }

  /// \brief Height, and displacement along x and along y [m], column-major.
  private: std::vector<double> dz;
  private: std::vector<double> dx;
  private: std::vector<double> dy;

  /// \brief The grids as the renderer takes them.
  private: gsw::WaveField2D field;
};

/// \brief One depth frame.
struct Frame
{
  /// \brief Simulation time of the frame [s].
  double time{0.0};

  /// \brief Image width [px].
  int width{0};

  /// \brief Image height [px].
  int height{0};

  /// \brief Depth along the camera's axis per pixel, row by row [m].
  std::vector<float> depth;
};

/// \brief The frame the camera's callback keeps, shared with it: gz-transport
/// may still deliver a message after an unsubscribe.
struct Capture
{
  /// \brief Guards the frame.
  std::mutex mutex;

  /// \brief The first frame of the risen sea, once it has come.
  std::optional<Frame> frame;
};

/// \brief What a frame says against the wave field.
struct Comparison
{
  /// \brief RMS height of the surface the camera saw [m].
  double seaRms{0.0};

  /// \brief RMS of that surface minus the field's drawn surface [m].
  double differenceRms{0.0};

  /// \brief Pixels compared.
  int pixels{0};
};

//////////////////////////////////////////////////
/// \brief The surface a frame saw against the field's drawn surface.
/// \param[in] _frame The depth frame.
/// \param[in] _field The wave field, at the frame's time.
/// \param[in] _shift Where the field is asked, off the point seen [m], in x
/// and in y: zero for the field itself.
/// \return The comparison.
Comparison Compare(const Frame &_frame, const gsw::WavefieldData &_field,
                   double _shift)
{
  // The camera's axis points down. Its image's right is world -y and its
  // image's down is world -x; a pixel's depth is along the axis.
  const double focal =
      0.5 * _frame.width / std::tan(0.5 * kHorizontalFov);
  Comparison out;
  double sea = 0.0;
  double difference = 0.0;
  for (int v = 0; v < _frame.height; v += 3)
  {
    for (int u = 0; u < _frame.width; u += 3)
    {
      const double depth = _frame.depth[v * _frame.width + u];
      if (!std::isfinite(depth))
        continue;
      const double x = -(v + 0.5 - 0.5 * _frame.height) / focal * depth;
      const double y = -(u + 0.5 - 0.5 * _frame.width) / focal * depth;
      const double seen = kCameraHeight - depth;
      const double drawn = gsw::SurfaceHeight(
          _field, x - _shift, y - _shift, _frame.time);
      sea += seen * seen;
      difference += (seen - drawn) * (seen - drawn);
      ++out.pixels;
    }
  }
  if (out.pixels > 0)
  {
    out.seaRms = std::sqrt(sea / out.pixels);
    out.differenceRms = std::sqrt(difference / out.pixels);
  }
  return out;
}
}  // namespace

//////////////////////////////////////////////////
/// The surface the renderer draws is the one the wave field says is drawn:
/// what a boat floats on is what the camera sees. The shader samples the
/// height texture at texel centres; half a texel off, the drawn sea sits
/// half a grid cell from the field, which on this sea is 9 cm of height.
TEST(WaterVisual, DrawsTheSurfaceOfTheWaveField)
{
  gsw::RegisterWaveEngineFactory(kEngine,
      [] { return std::make_shared<GridWaveField>(); });
  gsw::WavefieldData field;
  field.algorithm = kEngine;
  field.params.tileSize = kTile;
  field.params.gridSize = kCells;
  field.generation = 1;

  // The world has no wave system: the recipe goes on the world here, and
  // the water visual builds its engine from it as it does from any other.
  gz::sim::ServerConfig config;
  config.SetSdfFile(std::string(TEST_WORLD_DIR) + "/drawn_surface.sdf");
  config.SetHeadlessRendering(true);
  gz::sim::TestFixture fixture(config);
  bool recipeSet = false;
  fixture.OnConfigure(
    [&](const gz::sim::Entity &_world,
        const std::shared_ptr<const sdf::Element> &,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &)
    {
      _ecm.CreateComponent(_world, gz::sim::components::Wavefield(field));
      recipeSet = true;
    }).Finalize();
  ASSERT_TRUE(recipeSet) << "the world did not load";

  auto capture = std::make_shared<Capture>();
  gz::transport::Node node;
  std::function<void(const gz::msgs::Image &)> onImage =
    [capture](const gz::msgs::Image &_msg)
    {
      const double time = static_cast<double>(_msg.header().stamp().sec()) +
          1e-9 * static_cast<double>(_msg.header().stamp().nsec());
      std::lock_guard<std::mutex> lock(capture->mutex);
      if (capture->frame || time < kCompareFrom)
        return;
      Frame f;
      f.time = time;
      f.width = static_cast<int>(_msg.width());
      f.height = static_cast<int>(_msg.height());
      f.depth.resize(_msg.data().size() / sizeof(float));
      std::memcpy(f.depth.data(), _msg.data().data(),
                  f.depth.size() * sizeof(float));
      capture->frame = std::move(f);
    };
  ASSERT_TRUE(node.Subscribe(kDepthTopic, onImage));
  const auto frameCame = [&capture]
  {
    std::lock_guard<std::mutex> lock(capture->mutex);
    return capture->frame.has_value();
  };

  // A second of simulation at a time, each run blocking. A server stopped
  // while it runs on a thread of its own hung about once in thirty runs,
  // with a PostUpdate of gz-sim's sensors system waiting on its render
  // thread.
  for (int second = 0; second < 20 && !frameCame(); ++second)
    ASSERT_TRUE(fixture.Server()->Run(true, 100, false));
  // The frame of the last step may still be on its way.
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!frameCame() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  node.Unsubscribe(kDepthTopic);

  std::optional<Frame> frame;
  {
    std::lock_guard<std::mutex> lock(capture->mutex);
    frame = capture->frame;
  }
  if (!frame)
    GTEST_SKIP() << "no depth image: this machine cannot render";

  // The engine the renderer builds from the same recipe, at the same time.
  field.simulation = gsw::CreateWaveSimulation(field.algorithm, field.params);
  ASSERT_NE(field.simulation, nullptr);
  gsw::Advance(field, frame->time);

  const Comparison same = Compare(*frame, field, 0.0);
  const Comparison halfCellOff =
      Compare(*frame, field, 0.5 * kTile / static_cast<double>(kCells));
  std::cout << "sea rms " << same.seaRms << " m over " << same.pixels
            << " pixels at t = " << frame->time << " s; drawn minus field rms "
            << same.differenceRms << " m, against " << halfCellOff.differenceRms
            << " m for a field half a cell off\n";

  ASSERT_GT(same.pixels, 1000);
  // A sea, not flat water.
  EXPECT_GT(same.seaRms, 0.15);
  // Millimetres. The renderer may draw the sea of the step after the
  // frame's: WaterVisual takes its time from the simulation thread, which
  // can be a step ahead of a frame being rendered.
  EXPECT_LT(same.differenceRms, 0.02);
  // The comparison tells half a cell.
  EXPECT_GT(halfCellOff.differenceRms, 3.0 * same.differenceRms);
}

//////////////////////////////////////////////////
/// The test uses a fixed topic, so each process takes a partition of its
/// own: two copies run at the same time do not hear each other.
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  const std::string partition =
      "gz_waves_rendering_test_" + std::to_string(::getpid());
  ::setenv("GZ_PARTITION", partition.c_str(), 1);
  return RUN_ALL_TESTS();
}
