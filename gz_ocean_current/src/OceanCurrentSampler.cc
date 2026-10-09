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
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/OceanCurrentfield.hh"
#include "gz/sim/ocean_current/OceanCurrentModel.hh"

namespace gz::sim::ocean_current
{
/// \brief Private data of the sampler.
class OceanCurrentSamplerPrivate
{
  /// \brief Rotate a vector between the world frame and east north up,
  /// through the world's spherical coordinates, the frame its GPS uses.
  /// Without them north is +y and this is the identity.
  /// \param[in] _v Vector.
  /// \param[in] _toEnu True from the world frame, false towards it.
  /// \return The rotated vector.
  public: math::Vector3d Rotate(const math::Vector3d &_v, bool _toEnu) const
  {
    if (!this->sc)
      return _v;
    const auto out = this->sc->VelocityTransform(
        math::CoordinateVector3::Metric(_v),
        _toEnu ? math::SphericalCoordinates::LOCAL :
                 math::SphericalCoordinates::GLOBAL,
        _toEnu ? math::SphericalCoordinates::GLOBAL :
                 math::SphericalCoordinates::LOCAL);
    if (!out || !out->IsMetric())
      return _v;
    return out->AsMetricVector().value_or(_v);
  }

  /// \brief The world entity, found once.
  public: Entity world{kNullEntity};

  /// \brief The private copy of the model.
  public: std::unique_ptr<IOceanCurrentModel> model;

  /// \brief Model name of that copy.
  public: std::string modelName;

  /// \brief Generation of that copy.
  public: std::uint64_t generation{0};

  /// \brief The world's spherical coordinates, if it has them.
  public: std::optional<math::SphericalCoordinates> sc;

  /// \brief Logged about a model it could not build already.
  public: bool logged{false};
};

//////////////////////////////////////////////////
OceanCurrentSampler::OceanCurrentSampler()
  : dataPtr(std::make_unique<OceanCurrentSamplerPrivate>())
{
}

//////////////////////////////////////////////////
OceanCurrentSampler::~OceanCurrentSampler() = default;

//////////////////////////////////////////////////
bool OceanCurrentSampler::Sync(const EntityComponentManager &_ecm)
{
  // The world entity never changes; look it up again only when the sampler
  // is handed a world it has not seen.
  Entity &world = this->dataPtr->world;
  if (nullptr == _ecm.Component<components::World>(world))
    world = _ecm.EntityByComponents(components::World());
  const auto *field = _ecm.Component<components::OceanCurrentfield>(world);
  if (nullptr == field)
  {
    this->dataPtr->model.reset();
    return false;
  }

  // Copied only when they change: a copy allocates, and Sync runs every
  // step. A model that places itself on the Earth is rebuilt with them.
  auto &cached = this->dataPtr->sc;
  bool scChanged{false};
  const auto *sc = _ecm.Component<components::SphericalCoordinates>(world);
  if (nullptr != sc && (!cached || !(*cached == sc->Data())))
  {
    cached = sc->Data();
    scChanged = true;
  }
  else if (nullptr == sc && cached)
  {
    cached.reset();
    scChanged = true;
  }

  const auto &data = field->Data();
  // Without a model it tries again every step: a plugin may register the
  // model after the world started.
  if (!this->dataPtr->model || scChanged ||
      data.generation != this->dataPtr->generation ||
      data.model != this->dataPtr->modelName)
  {
    this->dataPtr->model = CreateOceanCurrentModel(data.model, data.params,
        cached ? &*cached : nullptr);
    this->dataPtr->modelName = data.model;
    this->dataPtr->generation = data.generation;
    // The ocean current system reports a model it cannot build, once; a
    // consumer only notes it, so each process does not repeat the error.
    if (!this->dataPtr->model && !this->dataPtr->logged)
    {
      gzdbg << "OceanCurrent: cannot build ocean current model ["
            << data.model << "], no current\n";
      this->dataPtr->logged = true;
    }
  }
  return nullptr != this->dataPtr->model;
}

//////////////////////////////////////////////////
bool OceanCurrentSampler::Valid() const
{
  return nullptr != this->dataPtr->model;
}

//////////////////////////////////////////////////
bool OceanCurrentSampler::TimeVarying() const
{
  return this->dataPtr->model && this->dataPtr->model->TimeVarying();
}

//////////////////////////////////////////////////
math::Vector3d OceanCurrentSampler::At(const math::Vector3d &_position,
    const std::chrono::steady_clock::duration &_time) const
{
  if (!this->dataPtr->model)
    return math::Vector3d::Zero;
  const double t = std::chrono::duration<double>(_time).count();
  const math::Vector3d enu = this->dataPtr->model->Velocity(
      this->dataPtr->Rotate(_position, true), t);
  return this->dataPtr->Rotate(enu, false);
}

//////////////////////////////////////////////////
math::Vector3d OceanCurrentAt(const EntityComponentManager &_ecm,
    const math::Vector3d &_position,
    const std::chrono::steady_clock::duration &_time)
{
  // A sampler of its own, every call: one kept between calls could answer
  // for another world, since every world numbers its entity and its
  // generations alike, and would outlive a model a plugin registered. A
  // system that asks every step keeps its own OceanCurrentSampler.
  OceanCurrentSampler sampler;
  sampler.Sync(_ecm);
  return sampler.At(_position, _time);
}
}  // namespace gz::sim::ocean_current
