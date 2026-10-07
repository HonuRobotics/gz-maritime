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

  /// \brief Warned about an unknown model already.
  public: bool warned{false};
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

  const auto *sc = _ecm.Component<components::SphericalCoordinates>(world);
  if (nullptr != sc)
    this->dataPtr->sc = sc->Data();
  else
    this->dataPtr->sc.reset();

  const auto &data = field->Data();
  if (!this->dataPtr->model || data.generation != this->dataPtr->generation ||
      data.model != this->dataPtr->modelName)
  {
    this->dataPtr->model = CreateOceanCurrentModel(data.model, data.params);
    this->dataPtr->modelName = data.model;
    this->dataPtr->generation = data.generation;
    if (!this->dataPtr->model && !this->dataPtr->warned)
    {
      gzerr << "OceanCurrent: no ocean current model named [" << data.model
            << "]\n";
      this->dataPtr->warned = true;
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
  // One sampler per thread, kept between calls, so the model is rebuilt only
  // when the recipe changes.
  thread_local OceanCurrentSampler sampler;
  sampler.Sync(_ecm);
  return sampler.At(_position, _time);
}
}  // namespace gz::sim::ocean_current
