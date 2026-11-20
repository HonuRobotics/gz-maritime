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

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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

/// \brief One sonar: a gz-sensors custom sensor,
/// <sensor type="custom" gz:type="imaging_sonar"> (or sidescan_sonar), the
/// pattern of gz_wind's anemometer. It owns the engine and the camera
/// sampler for its configuration. The world system draws its cameras on the
/// rendering thread and hands each capture to Push; a worker thread turns it
/// into ray hits, a ping and a readout, and publishes it. The queue is two
/// deep and Push blocks when it is full, so a slow engine slows the
/// simulation instead of dropping or reordering pings.
class SonarSensor : public sensors::Sensor
{
  public: ~SonarSensor() override;

  /// \brief Load the sensor; Load(sdf::Sensor) is the base class's and
  /// reads topic, frame and rate. This reads the <gz:sonar> block.
  /// \param[in] _sdf The sensor.
  /// \param[in] _type Its gz:type.
  /// \param[in] _water The world's water.
  /// \param[in] _seed The world's seed, mixed with the sensor's name.
  public: bool Load(const sdf::Sensor &_sdf, const std::string &_type,
              const gz::maritime::sonar::Water &_water, std::uint64_t _seed);

  using sensors::Sensor::Load;
  using sensors::Sensor::Update;

  /// \brief Unused: pings come from the rendering thread through Push.
  public: bool Update(const std::chrono::steady_clock::duration &) override;

  /// \brief Hand over one capture; blocks while two are waiting.
  public: void Push(PingFrame &&_frame);

  /// \brief The camera layout to draw.
  public: const gz::maritime::sonar::CameraLayout &Layout() const;

  /// \brief Pings per second.
  public: double Rate() const;

  /// \brief The sensor's gz:type.
  public: const std::string &Type() const;

  /// \brief Whether the ground-truth labels topic is on.
  public: bool Labels() const;

  private: void Work();
  private: void Process(const PingFrame &_frame);

  private: std::string type;
  private: gz::maritime::sonar::SonarConfig config;
  private: gz::maritime::sonar::Water water;
  private: bool labels{false};
  private: std::unique_ptr<gz::maritime::sonar::Engine> engine;
  private: std::unique_ptr<gz::maritime::sonar::CameraSampler> sampler;
  private: gz::maritime::sonar::GainTracker gain;

  private: transport::Node node;
  private: transport::Node::Publisher imagePub;
  private: transport::Node::Publisher labelsPub;

  private: std::mutex mutex;
  private: std::condition_variable cv;
  private: std::deque<PingFrame> queue;
  private: bool stop{false};
  private: std::thread worker;
};
}  // namespace gz::sim::maritime

#endif  // GZ_MARITIME_SONAR_SYSTEMS_SONARSENSOR_HH_
