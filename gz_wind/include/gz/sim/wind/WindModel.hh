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
#ifndef GZ_SIM_WIND_WINDMODEL_HH_
#define GZ_SIM_WIND_WINDMODEL_HH_

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include <gz/math/Vector3.hh>

#include "gz/sim/wind/Windfield.hh"

namespace gz::sim::wind
{
/// \brief A wind model: the wind at any point and time, from its parameters.
///
/// A model is a pure function of position and time, like a wave engine: two
/// copies built from the same parameters give the same wind at the same
/// point and time, whichever process holds them. Positions and velocities are
/// east, north and up, in metres and metres per second, with the origin at the
/// world's origin; the caller converts to and from the world frame.
class IWindModel
{
  /// \brief Destructor.
  public: virtual ~IWindModel() = default;

  /// \brief Set the parameters.
  /// \param[in] _params Parameters.
  public: virtual void SetParameters(const WindParameters &_params) = 0;

  /// \brief The wind at a point.
  /// \param[in] _enu Point, east north up, m.
  /// \param[in] _time Simulation time, s.
  /// \return The velocity of the air, east north up, m/s.
  public: virtual math::Vector3d Velocity(const math::Vector3d &_enu,
                                          double _time) const = 0;

  /// \brief Whether the wind changes with time at a fixed point, so a
  /// producer knows it must refresh anything that caches it.
  /// \return True if it varies in time.
  public: virtual bool TimeVarying() const { return false; }

  /// \brief The name the model is registered under.
  /// \return Its name.
  public: virtual std::string_view Kind() const = 0;
};

/// \brief Builds a model.
using WindModelFactory = std::function<std::unique_ptr<IWindModel>()>;

/// \brief Register a model under a name. The built in models are always
/// registered; a plugin can add its own.
/// \param[in] _name Name, as it appears in the recipe.
/// \param[in] _factory Builds the model.
void RegisterWindModelFactory(const std::string &_name,
                              WindModelFactory _factory);

/// \brief Build a registered model and set its parameters.
/// \param[in] _name Name the model was registered under.
/// \param[in] _params Parameters.
/// \return The model, or null if no model has that name.
std::unique_ptr<IWindModel> CreateWindModel(const std::string &_name,
                                            const WindParameters &_params);
}  // namespace gz::sim::wind

#endif  // GZ_SIM_WIND_WINDMODEL_HH_
