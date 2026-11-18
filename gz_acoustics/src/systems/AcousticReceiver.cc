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
#include "AcousticReceiver.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/Utility.hh>
#include <sdf/Noise.hh>

#include "gz/sim/acoustics/Hearing.hh"

using namespace gz;
using namespace sim;
using namespace maritime;

namespace
{
  /// \brief The name of the receiver's own block in the sensor.
  const std::string kBlock{"gz:acoustic_receiver"};

  /// \brief Add a double to a message.
  /// \param[in,out] _msg Message.
  /// \param[in] _key Key.
  /// \param[in] _value Value.
  void AddDouble(msgs::Param &_msg, const std::string &_key, double _value)
  {
    auto &any = (*_msg.mutable_params())[_key];
    any.set_type(msgs::Any::DOUBLE);
    any.set_double_value(_value);
  }

  /// \brief Seconds of a duration.
  /// \param[in] _d Duration.
  /// \return Seconds.
  double Seconds(const std::chrono::steady_clock::duration &_d)
  {
    return std::chrono::duration<double>(_d).count();
  }
}

//////////////////////////////////////////////////
bool AcousticReceiver::Load(const sdf::Sensor &_sdf)
{
  if (!sensors::Sensor::Load(_sdf))
    return false;
  this->pub = this->node.Advertise<msgs::Param>(this->Topic());
  const std::string setTopic = this->Topic() + "/set";
  if (!this->node.Subscribe(setTopic, &AcousticReceiver::OnSet, this))
    gzerr << "AcousticReceiver: cannot subscribe to " << setTopic << "\n";

  const auto elem = _sdf.Element();
  if (!elem || !elem->HasElement(kBlock))
    return true;
  const auto block = elem->FindElement(kBlock);

  auto &s = this->settings;
  s.frequency = block->Get<double>("frequency", s.frequency).first;
  s.bandwidth = block->Get<double>("bandwidth", s.bandwidth).first;
  s.range = block->Get<bool>("range", s.range).first;
  s.emitTime = block->Get<bool>("emit_time", s.emitTime).first;
  s.minElevation = block->Get<double>("min_elevation", s.minElevation).first;
  s.maxElevation = block->Get<double>("max_elevation", s.maxElevation).first;
  if (s.minElevation > s.maxElevation)
  {
    gzerr << "AcousticReceiver [" << this->Name() << "]: <min_elevation> is "
          << "above <max_elevation>, so it hears nothing\n";
  }
  s.outlierProbability = std::clamp(
      block->Get<double>("outlier_probability", 0.0).first, 0.0, 1.0);
  s.dropoutStart = std::clamp(
      block->Get<double>("dropout_start", 1.0).first, 0.0, 1.0);

  // The same <noise> as any Gazebo sensor, one block per channel, but
  // drawn from the receiver's generator: gz-sensors' noise models draw from
  // gz-math's global one, which a seed here must not touch.
  const char *channels[] = {"bearing_noise", "elevation_noise", "range_noise",
                            "amplitude_noise"};
  for (std::size_t i = 0; i < this->noise.size(); ++i)
  {
    if (!block->HasElement(channels[i]))
      continue;
    const auto wrapper = block->FindElement(channels[i]);
    if (!wrapper->HasElement("noise"))
    {
      gzerr << "AcousticReceiver [" << this->Name() << "]: <" << channels[i]
            << "> needs a <noise> block inside\n";
      return false;
    }
    sdf::Noise noiseSdf;
    const auto errors = noiseSdf.Load(wrapper->FindElement("noise"));
    if (!errors.empty())
    {
      gzerr << "AcousticReceiver [" << this->Name() << "]: cannot load its <"
            << channels[i] << ">\n";
      return false;
    }
    if (noiseSdf.Type() == sdf::NoiseType::NONE)
      continue;
    this->noise[i].mean = noiseSdf.Mean();
    this->noise[i].stddev = std::max(noiseSdf.StdDev(), 0.0);
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN_QUANTIZED)
      this->noise[i].precision = std::max(noiseSdf.Precision(), 0.0);
    this->bias[i] = {noiseSdf.BiasMean(),
                     std::max(noiseSdf.BiasStdDev(), 0.0)};
    if (noiseSdf.DynamicBiasStdDev() > 0.0)
    {
      gzwarn << "AcousticReceiver [" << this->Name() << "]: <" << channels[i]
             << "> has a dynamic bias, which is not modelled\n";
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool AcousticReceiver::Update(const std::chrono::steady_clock::duration &)
{
  return true;
}

//////////////////////////////////////////////////
bool AcousticReceiver::HasConnections() const
{
  return this->pub && this->pub.HasConnections();
}

//////////////////////////////////////////////////
ReceiverSettings AcousticReceiver::Settings() const
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  return this->settings;
}

//////////////////////////////////////////////////
void AcousticReceiver::DrawBias(acoustics::Generator &_rng)
{
  for (std::size_t i = 0; i < this->noise.size(); ++i)
  {
    const auto &[mean, stddev] = this->bias[i];
    this->noise[i].bias = stddev > 0.0 ?
        acoustics::Normal(_rng, mean, stddev) : mean;
  }
}

//////////////////////////////////////////////////
double AcousticReceiver::ApplyNoise(Channel _channel, double _value,
    acoustics::Generator &_rng) const
{
  const double value = acoustics::ApplyNoise(
      this->noise[static_cast<std::size_t>(_channel)], _value, _rng);
  switch (_channel)
  {
    case Channel::kBearing:
      return acoustics::WrapAngle(value);
    case Channel::kElevation:
      return std::clamp(value, -0.5 * GZ_PI, 0.5 * GZ_PI);
    case Channel::kRange:
      return std::max(value, 0.0);
    case Channel::kAmplitude:
      return value;
  }
  return value;
}

//////////////////////////////////////////////////
bool AcousticReceiver::Publish(const ReceiverReading &_reading)
{
  msgs::Param msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_reading.arrived);
  auto *frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());
  this->AddSequence(msg.mutable_header());

  AddDouble(msg, "bearing", _reading.bearing);
  AddDouble(msg, "elevation", _reading.elevation);
  AddDouble(msg, "frequency", _reading.frequency);
  AddDouble(msg, "amplitude", _reading.amplitude);
  // The range, and the time the ping was sent, which gives the range, only
  // when asked for: a passive receiver knows neither.
  const ReceiverSettings now = this->Settings();
  if (now.range)
    AddDouble(msg, "range", _reading.range);
  if (now.emitTime)
    AddDouble(msg, "emit_time", Seconds(_reading.emitted));
  auto &valid = (*msg.mutable_params())["valid"];
  valid.set_type(msgs::Any::BOOLEAN);
  valid.set_bool_value(_reading.valid);
  return this->pub.Publish(msg);
}

//////////////////////////////////////////////////
void AcousticReceiver::OnSet(const msgs::Param &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &[key, value] : _msg.params())
  {
    double d{0.0};
    if (value.type() == msgs::Any::DOUBLE)
      d = value.double_value();
    else if (value.type() == msgs::Any::INT32)
      d = value.int_value();
    else
    {
      gzwarn << "AcousticReceiver [" << this->Name() << "]: key '" << key
             << "' is not a number, ignored\n";
      continue;
    }
    if (!std::isfinite(d) || d < 0.0)
    {
      gzwarn << "AcousticReceiver [" << this->Name() << "]: key '" << key
             << "' out of range, ignored\n";
    }
    else if (key == "frequency")
    {
      this->settings.frequency = d;
    }
    else if (key == "bandwidth")
    {
      this->settings.bandwidth = d;
    }
    else
    {
      gzwarn << "AcousticReceiver [" << this->Name() << "]: key '" << key
             << "' is unknown, ignored\n";
    }
  }
}
