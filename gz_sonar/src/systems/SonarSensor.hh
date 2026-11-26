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
#ifndef GZ_MARITIME_SONAR_SYSTEMS_SONARSENSOR_HH_
#define GZ_MARITIME_SONAR_SYSTEMS_SONARSENSOR_HH_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Sensor.hh>

#include "gz/maritime/sonar/CameraSampler.hh"
#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Engine.hh"
#include "gz/maritime/sonar/Readout.hh"
#include "gz/maritime/sonar/Water.hh"

namespace gz::sim::maritime
{
/// \brief The acoustic surfaces of the world, by label id: what the
/// segmentation camera's labels index. Shared, read-only, by the pings that
/// were captured while it was current.
struct LabelTable
{
  /// \brief Properties by id; entry 0 (nothing, or an unlabeled visual)
  /// holds the world's default.
  std::vector<gz::maritime::sonar::AcousticProperties> properties;

  /// \brief Scoped name of the model each id stands for; entry 0 is empty.
  std::vector<std::string> names;
};

/// \brief What the rendering thread captured for one ping.
struct PingFrame
{
  std::uint64_t ping{0};
  std::chrono::nanoseconds stamp{0};
  /// \brief Time since this sensor's previous ping, s.
  double dt{0.0};
  std::vector<std::vector<float>> depth;
  std::vector<std::vector<std::uint8_t>> labels;
  unsigned int channels{1};
  std::shared_ptr<const LabelTable> table;
};

/// \brief One sonar sensor. The sonar plugin (the world system, Sonar.cc)
/// creates one SonarSensor for each <sensor type="custom"> in the world
/// whose gz:type is one of  {imaging_sonar, sidescan_sonar or profiling_sonar}.
///
/// Each SonarSensor owns the engine and the camera sampler for its sonar's
/// configuration, and a worker thread of its own. The world system draws
/// this sonar's cameras on the rendering thread and hands each rendered
/// ping to Push. The worker thread turns the ping's images into ray hits,
/// the hits into intensities, and the intensities into this sonar's message
/// (a SonarImage, or a SonarDetections for a profiler), and publishes it.
/// The queue between them is two pings deep and Push blocks when it is
/// full, so a slow engine slows the simulation instead of dropping or
/// reordering pings.
class SonarSensor : public sensors::Sensor
{
  /// \brief Destructor: stops the worker.
  public: ~SonarSensor() override;

  /// \brief Load the sensor; Load(sdf::Sensor) is the base class's and
  /// reads topic, frame and rate. This reads the <gz:sonar> block.
  /// \param[in] _sdf The sensor.
  /// \param[in] _type Its gz:type.
  /// \param[in] _water The world's water.
  /// \param[in] _seed The world's seed, mixed with the sensor's name.
  /// \return True if the sensor loaded.
  public: bool Load(const sdf::Sensor &_sdf, const std::string &_type,
              const gz::maritime::sonar::Water &_water, std::uint64_t _seed);

  using sensors::Sensor::Load;
  using sensors::Sensor::Update;

  // Documentation inherited
  // Unused: pings come from the rendering thread through Push.
  public: bool Update(const std::chrono::steady_clock::duration &) override;

  /// \brief Hand over one capture; blocks while two are waiting.
  /// \param[in] _frame The capture.
  public: void Push(PingFrame &&_frame);

  /// \brief The camera layout to draw.
  /// \return The layout.
  public: const gz::maritime::sonar::CameraLayout &Layout() const;

  /// \brief Pings per second.
  /// \return The rate, Hz.
  public: double Rate() const;

  /// \brief The sensor's gz:type.
  /// \return The type.
  public: const std::string &Type() const;

  /// \brief Whether the ground-truth labels topic is on.
  /// \return True if the labels topic is on.
  public: bool Labels() const;

  // Documentation inherited
  // Anyone subscribed to this sonar's data, labels or water column. With
  // no one listening the world system skips the sonar's pings, as gz-sim
  // does for its own rendering sensors.
  public: bool HasConnections() const override;

  /// \brief Whether the sonar pings with no subscriber too, its
  /// <always_ping>. HasConnections trusts gz-transport, which can miss a
  /// subscriber; this is the way around it.
  /// \return True if it pings regardless of subscribers.
  public: bool AlwaysPing() const;

  /// \brief Start over after a world reset: the automatic gain forgets
  /// the pings it has seen.
  public: void ResetState();

  /// \brief The <topic>/enable service: true turns the sonar on, false off.
  /// \param[in] _enable The request.
  private: void OnEnable(const msgs::Boolean &_enable);

  private: void Work();
  private: void Process(const PingFrame &_frame);

  private: std::string type;
  private: gz::maritime::sonar::SonarConfig config;
  private: gz::maritime::sonar::Water water;
  private: bool labels{false};

  /// \brief Ping with no subscriber too (<always_ping>).
  private: bool alwaysPing{false};
  private: std::unique_ptr<gz::maritime::sonar::Engine> engine;
  private: std::unique_ptr<gz::maritime::sonar::CameraSampler> sampler;
  private: gz::maritime::sonar::GainTracker gain;

  /// \brief Set by ResetState, read by the worker before its next ping.
  private: std::atomic<bool> resetGain{false};

  private: transport::Node node;
  private: transport::Node::Publisher imagePub;
  private: transport::Node::Publisher labelsPub;

  /// \brief A profiler's intensity before detection, when published.
  private: transport::Node::Publisher waterColumnPub;

  /// \brief Publish a profiler's water column (<water_column>).
  private: bool waterColumn{false};

  private: std::mutex mutex;
  private: std::condition_variable cv;
  private: std::deque<PingFrame> queue;
  private: bool stop{false};
  private: std::thread worker;
};
}  // namespace gz::sim::maritime

#endif  // GZ_MARITIME_SONAR_SYSTEMS_SONARSENSOR_HH_
