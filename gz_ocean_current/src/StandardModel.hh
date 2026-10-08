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
#ifndef GZ_SIM_OCEAN_CURRENT_STANDARDMODEL_HH_
#define GZ_SIM_OCEAN_CURRENT_STANDARDMODEL_HH_

#include <string>
#include <string_view>

#include <gz/math/Vector3.hh>

#include "gz/sim/ocean_current/OceanCurrentModel.hh"

namespace gz::sim::ocean_current
{
/// \brief The standard ocean current: a speed and the direction it sets
/// towards, uniform over the world and horizontal. On the
/// kilometre and hour scales this simulation works at, that is the water.
class StandardModel : public IOceanCurrentModel
{
  // Documentation inherited. The standard model reads no parameter of its
  // own, so it refuses any, rather than ignore a typo or a source meant for
  // another model.
  public: std::string Validate(const OceanCurrentParameters &_params) const
      override;

  // Documentation inherited.
  public: void SetParameters(const OceanCurrentParameters &_params) override;

  // Documentation inherited.
  public: math::Vector3d Velocity(const math::Vector3d &_enu,
                                  double _time) const override;

  // Documentation inherited.
  public: std::string_view Kind() const override { return "standard"; }

  /// \brief The current, east north up, m/s.
  private: math::Vector3d velocity;
};
}  // namespace gz::sim::ocean_current

#endif  // GZ_SIM_OCEAN_CURRENT_STANDARDMODEL_HH_
