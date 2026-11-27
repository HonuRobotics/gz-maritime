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
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

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
  /// \brief Guards everything Sync writes.
  public: mutable std::mutex mutex;

  /// \brief The optics, clear water until a Sync finds some.
  public: WaterOpticsData data;

  /// \brief Whether the world has water optics.
  public: bool valid{false};

  /// \brief The world's wave recipe, without an engine, when it has one.
  public: std::shared_ptr<const waves::WavefieldData> recipe;

  /// \brief The source's own wave field, engine included, when this
  /// process holds it. Read on the Sync thread only.
  public: std::shared_ptr<const waves::WavefieldData> source;

  /// \brief The thread that calls Sync.
  public: std::thread::id syncThread;

  /// \brief Guards our own engine, which Depth builds and advances.
  public: std::mutex engineMutex;

  /// \brief Our own wave field, built from the recipe on the first depth
  /// asked for off the Sync thread.
  public: std::optional<waves::WavefieldData> own;

  /// \brief The simulation time our own engine is at, s.
  public: double engineTime{-1.0};

  /// \brief The engine registry's version at our last try at building
  /// our own engine: we try again only when it changes, as when the
  /// engine's registrar loads after us.
  public: std::optional<std::uint64_t> triedVersion;

  /// \brief Whether we said there is no engine for this recipe.
  public: bool warned{false};
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
  const auto *field = _ecm.Component<components::Wavefield>(world);

  const std::lock_guard<std::mutex> lock(d.mutex);
  d.syncThread = std::this_thread::get_id();
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

  if (!field)
  {
    d.recipe.reset();
    d.source.reset();
    return d.valid;
  }
  const auto &recipe = field->Data();
  if (!d.recipe || d.recipe->generation != recipe.generation ||
      d.recipe->algorithm != recipe.algorithm)
  {
    auto copy = std::make_shared<waves::WavefieldData>(recipe);
    copy->simulation.reset();
    d.recipe = copy;
  }
  if (!recipe.simulation)
    d.source.reset();
  else if (!d.source || d.source->simulation != recipe.simulation ||
           d.source->generation != recipe.generation)
    d.source = std::make_shared<const waves::WavefieldData>(recipe);
  return d.valid;
}

//////////////////////////////////////////////////
bool WaterOpticsSampler::Valid() const
{
  const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->valid;
}

//////////////////////////////////////////////////
WaterOpticsData WaterOpticsSampler::Data() const
{
  const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
double WaterOpticsSampler::Depth(const math::Vector3d &_position,
    const std::chrono::steady_clock::duration &_time) const
{
  auto &d = *this->dataPtr;
  const double t = std::chrono::duration<double>(_time).count();
  double surface{0.0};
  std::shared_ptr<const waves::WavefieldData> source, recipe;
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    surface = d.data.waterLevel;
    recipe = d.recipe;
    if (std::this_thread::get_id() == d.syncThread)
      source = d.source;
  }

  // The source's engine, on the thread that also sees it advance
  if (source)
  {
    return surface + waves::SurfaceElevation(*source, _position.X(),
                                             _position.Y(), t) -
           _position.Z();
  }
  if (!recipe)
    return surface - _position.Z();

  // Our own engine, built from the recipe. Only a newer recipe, or
  // another engine for the same one, replaces it: a reader holding a recipe
  // from before the last Sync uses the newer engine rather than building
  // the old one back, so a change builds one engine however many read.
  const std::lock_guard<std::mutex> lock(d.engineMutex);
  if (d.own && (recipe->generation > d.own->generation ||
                (recipe->generation == d.own->generation &&
                 recipe->algorithm != d.own->algorithm)))
  {
    d.own.reset();
    d.triedVersion.reset();
    d.warned = false;
  }
  const std::uint64_t version = waves::WaveEngineRegistryVersion();
  if ((!d.own || !d.own->simulation) && d.triedVersion != version)
  {
    d.triedVersion = version;
    const waves::WavefieldData &want = d.own ? *d.own : *recipe;
    if (waves::HasWaveEngineFactory(want.algorithm))
    {
      waves::WavefieldData built = want;
      built.simulation =
          waves::CreateWaveSimulation(want.algorithm, want.params);
      d.own = std::move(built);
      d.engineTime = -1.0;
    }
    else
    {
      if (!d.own)
      {
        d.own = want;
        d.own->simulation.reset();
      }
      if (!d.warned)
      {
        gzwarn << "WaterOpticsSampler: no '" << want.algorithm << "' wave "
               << "engine in this process yet; depth is below the still "
               << "water level until one is registered\n";
        d.warned = true;
      }
    }
  }
  if (d.own && d.own->simulation)
  {
    if (t != d.engineTime)
    {
      waves::Advance(*d.own, t);
      d.engineTime = t;
    }
    surface += waves::SurfaceElevation(*d.own, _position.X(), _position.Y(),
                                       t);
  }
  return surface - _position.Z();
}

//////////////////////////////////////////////////
Rgb WaterOpticsSampler::Veil(double _depth) const
{
  if (_depth < 0.0)
    return Rgb::Zero;
  const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const Rgb &k = this->dataPtr->data.diffuseAttenuation;
  const double day = this->dataPtr->data.daylight;
  const Rgb &v = this->dataPtr->data.veil;
  return Rgb(day * v.X() * std::exp(-k.X() * _depth),
             day * v.Y() * std::exp(-k.Y() * _depth),
             day * v.Z() * std::exp(-k.Z() * _depth));
}
}  // namespace gz::sim::water_optics
