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
#include <memory>
#include <mutex>
#include <optional>

#include <gz/common/Console.hh>
#include <gz/sim/Util.hh>

#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/WaveSimulation.hh"

namespace gz::sim::water_optics
{
class WaterOpticsSamplerPrivate
{
  /// \brief The optics, clear water until a Sync finds some.
  public: WaterOpticsData data;

  /// \brief Whether the world has water optics.
  public: bool valid{false};

  /// \brief The world's wave field, when it has one: the source's own
  /// engine, or one of our own built from the recipe.
  public: std::optional<waves::WavefieldData> field;

  /// \brief Whether the engine is our own, which we advance; the source's
  /// is advanced by the source.
  public: bool ownEngine{false};

  /// \brief The simulation time our own engine is at, s.
  public: double engineTime{-1.0};

  /// \brief Whether we warned that no engine could be built.
  public: bool warnedNoEngine{false};

  /// \brief Guards our own engine, which Depth advances.
  public: std::mutex mutex;
};

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
  if (!optics)
  {
    d.valid = false;
    d.data = WaterOpticsData{};
  }
  else if (!d.valid || optics->Data().generation != d.data.generation)
  {
    d.valid = true;
    d.data = optics->Data();
  }

  const std::lock_guard<std::mutex> lock(d.mutex);
  const auto *field = _ecm.Component<components::Wavefield>(world);
  if (!field)
  {
    d.field.reset();
    return d.valid;
  }
  const auto &recipe = field->Data();
  if (recipe.simulation)
  {
    // The source's own engine, advanced by the source
    if (!d.field || d.field->simulation != recipe.simulation ||
        d.field->generation != recipe.generation)
    {
      d.field = recipe;
      d.ownEngine = false;
    }
    return d.valid;
  }
  // The recipe only: an engine of our own, built again when it changes
  if (d.field && d.ownEngine && d.field->generation == recipe.generation &&
      d.field->algorithm == recipe.algorithm)
  {
    return d.valid;
  }
  d.field = recipe;
  d.field->simulation =
      waves::CreateWaveSimulation(recipe.algorithm, recipe.params);
  d.ownEngine = true;
  d.engineTime = -1.0;
  if (!d.field->simulation && !d.warnedNoEngine)
  {
    gzwarn << "WaterOpticsSampler: no '" << recipe.algorithm << "' wave "
           << "engine in this process; depth is below the still water "
           << "level\n";
    d.warnedNoEngine = true;
  }
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
  auto &d = *this->dataPtr;
  double surface = d.data.waterLevel;
  const double t = std::chrono::duration<double>(_time).count();
  const std::lock_guard<std::mutex> lock(d.mutex);
  if (d.field && d.field->simulation)
  {
    if (d.ownEngine && t != d.engineTime)
    {
      waves::Advance(*d.field, t);
      d.engineTime = t;
    }
    surface += waves::SurfaceElevation(*d.field, _position.X(),
                                       _position.Y(), t);
  }
  return surface - _position.Z();
}

//////////////////////////////////////////////////
Rgb WaterOpticsSampler::Veil(double _depth) const
{
  if (_depth < 0.0)
    return Rgb::Zero;
  const Rgb &k = this->dataPtr->data.diffuseAttenuation;
  const Rgb &v = this->dataPtr->data.veil;
  return Rgb(v.X() * std::exp(-k.X() * _depth),
             v.Y() * std::exp(-k.Y() * _depth),
             v.Z() * std::exp(-k.Z() * _depth));
}
}  // namespace gz::sim::water_optics
