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
#ifndef GZ_SIM_OCEAN_CURRENT_OCEANCURRENTMODEL_HH_
#define GZ_SIM_OCEAN_CURRENT_OCEANCURRENTMODEL_HH_

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include <gz/math/Vector3.hh>

#include "gz/sim/ocean_current/OceanCurrentfield.hh"

namespace gz::sim::ocean_current
{
/// \brief An ocean current model: the water's velocity at any point and
/// time, from its parameters.
///
/// A model is a pure function of position and time, like a wave engine: two
/// copies built from the same parameters give the same current at the same
/// point and time, whichever process holds them. Positions and velocities
/// are east, north and up, in metres and metres per second, with the origin
/// at the world's origin; the caller converts to and from the world frame.
/// The query is a point query and nothing more: a consumer that spans a
/// gradient integrates over its own extent with repeated queries.
///
/// A new model, simpler or richer, is one class and one registration: the
/// system that owns the recipe and the systems that ask for the current do
/// not change.
class IOceanCurrentModel
{
  /// \brief Destructor.
  public: virtual ~IOceanCurrentModel() = default;

  /// \brief Set the parameters.
  /// \param[in] _params Parameters.
  public: virtual void SetParameters(const OceanCurrentParameters &_params)
      = 0;

  /// \brief The current at a point.
  /// \param[in] _enu Point, east north up, m.
  /// \param[in] _time Simulation time, s.
  /// \return The velocity of the water, east north up, m/s.
  public: virtual math::Vector3d Velocity(const math::Vector3d &_enu,
                                          double _time) const = 0;

  /// \brief Whether the current changes with time at a fixed point, so a
  /// producer knows it must refresh anything that caches it.
  /// \return True if it varies in time.
  public: virtual bool TimeVarying() const { return false; }

  /// \brief The name the model is registered under.
  /// \return Its name.
  public: virtual std::string_view Kind() const = 0;
};

/// \brief Builds a model.
using OceanCurrentModelFactory =
    std::function<std::unique_ptr<IOceanCurrentModel>()>;

/// \brief Register a model under a name. The model built in is always
/// registered; a plugin can add its own.
/// \param[in] _name Name, as it appears in the recipe.
/// \param[in] _factory Builds the model.
void RegisterOceanCurrentModelFactory(const std::string &_name,
                                      OceanCurrentModelFactory _factory);

/// \brief Build a registered model and set its parameters.
/// \param[in] _name Name the model was registered under.
/// \param[in] _params Parameters.
/// \return The model, or null if no model has that name.
std::unique_ptr<IOceanCurrentModel> CreateOceanCurrentModel(
    const std::string &_name, const OceanCurrentParameters &_params);

/// \brief The velocity a speed and a direction the current sets towards
/// make, east north up and horizontal. The one place the convention lives: a
/// direction of 90, clockwise from north, sets east.
/// \param[in] _speed Speed, m/s.
/// \param[in] _directionDeg Direction the current sets towards, degrees
/// clockwise from true north.
/// \return The velocity, east north up, m/s.
math::Vector3d SetVector(double _speed, double _directionDeg);
}  // namespace gz::sim::ocean_current

#endif  // GZ_SIM_OCEAN_CURRENT_OCEANCURRENTMODEL_HH_
