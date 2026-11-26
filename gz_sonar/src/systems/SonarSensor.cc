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
#include "SonarSensor.hh"

#include <algorithm>
#include <cstring>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>

#include "SonarSdf.hh"
#include "gz/maritime/sonar/sonar_detections.pb.h"
#include "gz/maritime/sonar/sonar_image.pb.h"
#include "gz/maritime/sonar/sonar_labels.pb.h"

namespace gz::sim::maritime
{
namespace sonar = gz::maritime::sonar;

namespace
{
/// \brief Queue depth between the rendering thread and the worker.
constexpr std::size_t kQueueDepth = 2;

/// \brief FNV-1a: a hash of the sensor's name that is the same on every
/// platform, unlike std::hash, so a seed means the same pings everywhere.
std::uint64_t Fnv1a(const std::string &_s)
{
  std::uint64_t h = 0xcbf29ce484222325ULL;
  for (const unsigned char ch : _s)
  {
    h ^= ch;
    h *= 0x100000001b3ULL;
  }
  return h;
}

void Stamp(msgs::Header *_header, std::chrono::nanoseconds _t,
  const std::string &_frame)
{
  const auto ns = _t.count();
  _header->mutable_stamp()->set_sec(ns / 1000000000);
  _header->mutable_stamp()->set_nsec(static_cast<int>(ns % 1000000000));
  auto *frame = _header->add_data();
  frame->set_key("frame_id");
  frame->add_value(_frame);
}
}  // namespace

SonarSensor::~SonarSensor()
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->stop = true;
  }
  this->cv.notify_all();
  if (this->worker.joinable())
    this->worker.join();
}

bool SonarSensor::Load(const sdf::Sensor &_sdf, const std::string &_type,
  const sonar::Water &_water, std::uint64_t _seed)
{
  if (!sensors::Sensor::Load(_sdf))
    return false;
  this->type = _type;
  this->water = _water;

  const auto elem = _sdf.Element();
  const sdf::ElementPtr block = elem && elem->HasElement("gz:sonar") ?
    elem->FindElement("gz:sonar") : nullptr;
  std::string error;
  const auto config = LoadSonarConfig(_type, block, this->labels, error);
  if (!config)
  {
    gzerr << "Sonar [" << this->Name() << "]: " << error << "\n";
    return false;
  }
  this->config = *config;
  this->config.seed = _seed ^ Fnv1a(this->Name());

  this->engine = std::make_unique<sonar::Engine>(this->config, this->water);
  this->sampler = std::make_unique<sonar::CameraSampler>(this->config);
  if (this->type == kProfilingSonar)
  {
    // Soundings on the topic itself; the intensity before detection, as an
    // image of levels in dB, only when asked for.
    this->imagePub =
      this->node.Advertise<gz::maritime::sonar::SonarDetections>(
        this->Topic());
    this->waterColumn = block && block->HasElement("water_column") &&
      block->Get<bool>("water_column");
    if (this->waterColumn)
    {
      this->waterColumnPub =
        this->node.Advertise<gz::maritime::sonar::SonarImage>(
          this->Topic() + "/water_column");
    }
  }
  else
  {
    this->imagePub = this->node.Advertise<gz::maritime::sonar::SonarImage>(
      this->Topic());
  }
  if (this->labels)
  {
    this->labelsPub = this->node.Advertise<gz::maritime::sonar::SonarLabels>(
      this->Topic() + "/labels");
  }
  // Turn the sonar off and on at run time; off, it costs nothing.
  const std::string enableTopic = this->Topic() + "/enable";
  if (!this->node.Advertise(enableTopic, &SonarSensor::OnEnable, this))
    gzerr << "Sonar [" << this->Name() << "]: cannot advertise [" << enableTopic << "]\n";
  this->worker = std::thread([this]() { this->Work(); });

  const auto &l = this->sampler->Layout();
  gzmsg << "Sonar [" << this->Name() << "] (" << _type << ") on ["
        << this->Topic() << "]: " << this->config.beams << " beams, "
        << this->config.samples << " samples to " << this->config.rangeMax
        << " m, " << l.yaws.size() << " camera(s) of " << l.width << "x"
        << l.height << "\n";
  if (this->UpdateRate() > this->Rate())
  {
    gzwarn << "Sonar [" << this->Name() << "]: <update_rate> "
           << this->UpdateRate() << " Hz is faster than echoes from "
           << this->config.rangeMax << " m can return; pinging at "
           << this->Rate() << " Hz\n";
  }
  return true;
}

bool SonarSensor::Update(const std::chrono::steady_clock::duration &)
{
  return true;
}

void SonarSensor::Push(PingFrame &&_frame)
{
  std::unique_lock<std::mutex> lock(this->mutex);
  this->cv.wait(lock, [this]() {
    return this->stop || this->queue.size() < kQueueDepth;
  });
  if (this->stop)
    return;
  this->queue.push_back(std::move(_frame));
  lock.unlock();
  this->cv.notify_all();
}

const sonar::CameraLayout &SonarSensor::Layout() const
{
  return this->sampler->Layout();
}

double SonarSensor::Rate() const
{
  const double asked = this->UpdateRate() > 0.0 ? this->UpdateRate() : 10.0;
  // One ping's echoes must return before the next: no faster than c / 2R.
  const double cap = this->water.soundSpeed / (2.0 * this->config.rangeMax);
  return std::min(asked, cap);
}

const std::string &SonarSensor::Type() const
{
  return this->type;
}

bool SonarSensor::HasConnections() const
{
  return (this->imagePub && this->imagePub.HasConnections()) ||
    (this->labelsPub && this->labelsPub.HasConnections()) ||
    (this->waterColumnPub && this->waterColumnPub.HasConnections());
}

//////////////////////////////////////////////////
void SonarSensor::OnEnable(const msgs::Boolean &_enable)
{
  this->SetActive(_enable.data());
  gzmsg << "Sonar [" << this->Name() << "] " << (_enable.data() ? "on" : "off")
        << "\n";
}

//////////////////////////////////////////////////
bool SonarSensor::Labels() const
{
  return this->labels;
}

void SonarSensor::ResetState()
{
  // The worker owns the gain tracker; it starts over at its next ping.
  this->resetGain = true;
}

void SonarSensor::Work()
{
  while (true)
  {
    PingFrame frame;
    {
      std::unique_lock<std::mutex> lock(this->mutex);
      this->cv.wait(lock, [this]() {
        return this->stop || !this->queue.empty();
      });
      if (this->stop)
        return;
      frame = std::move(this->queue.front());
    }
    // Keep the frame queued while it is processed, so the queue's depth
    // bounds the pings in flight, not only those waiting.
    this->Process(frame);
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->queue.pop_front();
    }
    this->cv.notify_all();
  }
}

void SonarSensor::Process(const PingFrame &_frame)
{
  if (this->resetGain.exchange(false))
    this->gain = gz::maritime::sonar::GainTracker();
  std::vector<const float *> depth;
  std::vector<const std::uint8_t *> labelImages;
  for (const auto &d : _frame.depth)
    depth.push_back(d.data());
  for (const auto &l : _frame.labels)
    labelImages.push_back(l.empty() ? nullptr : l.data());

  const auto hits = this->sampler->Sample(depth, labelImages,
    _frame.channels, _frame.table->properties);
  const auto intensity = this->engine->Ping(hits, _frame.ping);
  const auto levels = sonar::ReadLevels(intensity, this->config,
    this->engine->Absorption(), _frame.ping, &this->gain, _frame.dt);

  gz::maritime::sonar::SonarImage msg;
  Stamp(msg.mutable_header(), _frame.stamp, this->FrameId());
  msg.set_ping(_frame.ping);
  msg.set_frequency(this->config.frequency);
  msg.set_sound_speed(
    this->config.AssumedSoundSpeed(this->water.soundSpeed));
  msg.set_beam_width_horizontal(static_cast<float>(this->config.beamWidth));
  msg.set_beam_width_vertical(static_cast<float>(this->config.verticalFov));
  msg.set_beam_count(static_cast<std::uint32_t>(levels.beams));
  msg.set_sample_count(static_cast<std::uint32_t>(levels.samples));
  for (double a : levels.azimuths)
    msg.add_azimuths(static_cast<float>(a));
  for (double r : levels.ranges)
    msg.add_ranges(static_cast<float>(r));
  for (double g : levels.tvgDb)
    msg.add_tvg(static_cast<float>(g));
  msg.set_gain(levels.gainDb);

  if (this->type == kImagingSonar)
  {
    const auto frame = sonar::ReadImaging(levels, this->config);
    msg.set_data_type(gz::maritime::sonar::SonarImage::UINT8);
    msg.set_data(frame.data.data(), frame.data.size());
    this->imagePub.Publish(msg);
  }
  else if (this->type == kProfilingSonar)
  {
    // Bottom detection: the soundings are what the device delivers.
    const double c = this->config.AssumedSoundSpeed(this->water.soundSpeed);
    const auto soundings = sonar::ReadProfiling(levels, this->config, c,
      _frame.ping);
    gz::maritime::sonar::SonarDetections dm;
    Stamp(dm.mutable_header(), _frame.stamp, this->FrameId());
    dm.set_ping(_frame.ping);
    dm.set_frequency(this->config.frequency);
    dm.set_sound_speed(c);
    dm.set_beam_width_across(static_cast<float>(this->config.beamWidth));
    dm.set_beam_width_along(static_cast<float>(this->config.verticalFov));
    for (const auto &sd : soundings)
    {
      dm.add_angles(static_cast<float>(sd.angle));
      dm.add_travel_times(static_cast<float>(sd.travelTime));
      dm.add_intensities(static_cast<float>(sd.levelDb));
      dm.add_flags(static_cast<std::uint32_t>(sd.flag));
    }
    this->imagePub.Publish(dm);

    if (this->waterColumn)
    {
      std::vector<float> db(levels.db.begin(), levels.db.end());
      msg.set_data_type(gz::maritime::sonar::SonarImage::FLOAT32);
      msg.set_data(db.data(), db.size() * sizeof(float));
      this->waterColumnPub.Publish(msg);
    }
  }
  else
  {
    // Sidescan: one beam, levels in dB as float32.
    std::vector<float> db(levels.db.begin(), levels.db.end());
    msg.set_data_type(gz::maritime::sonar::SonarImage::FLOAT32);
    msg.set_data(db.data(), db.size() * sizeof(float));
    this->imagePub.Publish(msg);
  }

  if (this->labels)
  {
    gz::maritime::sonar::SonarLabels lm;
    Stamp(lm.mutable_header(), _frame.stamp, this->FrameId());
    lm.set_ping(_frame.ping);
    lm.set_beam_count(static_cast<std::uint32_t>(intensity.beams));
    lm.set_sample_count(static_cast<std::uint32_t>(intensity.samples));
    std::string bytes(intensity.labels.size(), '\0');
    for (std::size_t i = 0; i < intensity.labels.size(); ++i)
      bytes[i] = static_cast<char>(intensity.labels[i] & 0xff);
    lm.set_labels(bytes);
    for (const auto &n : _frame.table->names)
      lm.add_names(n);
    this->labelsPub.Publish(lm);
  }
}
}  // namespace gz::sim::maritime
