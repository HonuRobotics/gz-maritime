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
#include "gz/sim/water_optics/WaterOpticsSampler.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include <gz/sim/Util.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/waves/Eval.hh"

namespace gz::sim::water_optics
{
class WaterOpticsSamplerPrivate
{
  /// \brief The optics, clear water until a Sync finds some.
  public: WaterOpticsData data;

  /// \brief Whether the world has water optics.
  public: bool valid{false};

  /// \brief The world's wave field, when it has one.
  public: std::optional<waves::WavefieldData> field;
};

namespace
{
/// \brief exp(-k x) per channel.
Rgb Fade(const Rgb &_k, double _x)
{
  return Rgb(std::exp(-_k.X() * _x), std::exp(-_k.Y() * _x),
             std::exp(-_k.Z() * _x));
}
}  // namespace

//////////////////////////////////////////////////
WaterOpticsSampler::WaterOpticsSampler()
  : dataPtr(std::make_unique<WaterOpticsSamplerPrivate>())
{
}

//////////////////////////////////////////////////
WaterOpticsSampler::~WaterOpticsSampler() = default;

//////////////////////////////////////////////////
bool WaterOpticsSampler::Sync(const EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  const Entity world = worldEntity(_ecm);
  const auto *optics = _ecm.Component<components::WaterOptics>(world);
  d.valid = nullptr != optics;
  d.data = optics ? optics->Data() : WaterOpticsData{};

  const auto *field = _ecm.Component<components::Wavefield>(world);
  if (field && field->Data().simulation)
    d.field = field->Data();
  else
    d.field.reset();
  return d.valid;
}

//////////////////////////////////////////////////
bool WaterOpticsSampler::Valid() const
{
  return this->dataPtr->valid;
}

//////////////////////////////////////////////////
const WaterOpticsData &WaterOpticsSampler::Data() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
double WaterOpticsSampler::Depth(const math::Vector3d &_position,
    const std::chrono::steady_clock::duration &_time) const
{
  const auto &d = *this->dataPtr;
  double surface = d.data.waterLevel;
  if (d.field)
  {
    surface += waves::SurfaceElevation(*d.field, _position.X(),
        _position.Y(), std::chrono::duration<double>(_time).count());
  }
  return surface - _position.Z();
}

//////////////////////////////////////////////////
Rgb WaterOpticsSampler::Daylight(double _depth) const
{
  return Fade(this->dataPtr->data.diffuseAttenuation,
              std::max(0.0, _depth));
}

//////////////////////////////////////////////////
Rgb WaterOpticsSampler::Veil(double _depth) const
{
  if (_depth < 0.0)
    return Rgb::Zero;
  const Rgb light = this->Daylight(_depth) * this->dataPtr->data.daylight;
  const Rgb &v = this->dataPtr->data.veil;
  return Rgb(v.X() * light.X(), v.Y() * light.Y(), v.Z() * light.Z());
}

//////////////////////////////////////////////////
Rgb WaterOpticsSampler::Transmittance(double _range) const
{
  return Fade(this->dataPtr->data.attenuation, std::max(0.0, _range));
}

//////////////////////////////////////////////////
Rgb WaterOpticsSampler::VeilBuildUp(double _range) const
{
  return Rgb::One - Fade(this->dataPtr->data.backscatter,
                         std::max(0.0, _range));
}
}  // namespace gz::sim::water_optics
