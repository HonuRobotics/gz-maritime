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
#include "Anemometer.hh"

#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <sdf/Noise.hh>

using namespace gz;
using namespace sim;
using namespace maritime;

//////////////////////////////////////////////////
bool Anemometer::Load(const sdf::Sensor &_sdf)
{
  if (!sensors::Sensor::Load(_sdf))
    return false;
  this->pub = this->node.Advertise<msgs::Twist>(this->Topic());

  // The same <noise> as any Gazebo sensor, under the sensor's own element,
  // as gz-sensors' custom sensors keep it. Each axis draws its own noise.
  const auto elem = _sdf.Element();
  if (!elem || !elem->HasElement("gz:anemometer"))
    return true;
  const auto custom = elem->FindElement("gz:anemometer");
  if (!custom->HasElement("noise"))
    return true;
  sdf::Noise noiseSdf;
  noiseSdf.Load(custom->FindElement("noise"));
  for (auto &axis : this->noise)
  {
    axis = sensors::NoiseFactory::NewNoiseModel(noiseSdf);
    if (nullptr == axis)
    {
      gzerr << "Anemometer [" << this->Name() << "]: cannot load its noise\n";
      return false;
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool Anemometer::Update(const std::chrono::steady_clock::duration &_now)
{
  math::Vector3d reading = this->apparent;
  for (std::size_t i = 0; i < this->noise.size(); ++i)
  {
    if (this->noise[i])
      reading[i] = this->noise[i]->Apply(reading[i]);
  }

  msgs::Twist msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto *frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());
  this->AddSequence(msg.mutable_header());
  msgs::Set(msg.mutable_linear(), reading);
  return this->pub.Publish(msg);
}

//////////////////////////////////////////////////
bool Anemometer::HasConnections() const
{
  return this->pub && this->pub.HasConnections();
}

//////////////////////////////////////////////////
void Anemometer::SetApparentWind(const math::Vector3d &_wind)
{
  this->apparent = _wind;
}
