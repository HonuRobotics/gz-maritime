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
#ifndef GZ_SIM_WIND_STANDARDMODEL_HH_
#define GZ_SIM_WIND_STANDARDMODEL_HH_

#include <string_view>

#include "gz/sim/wind/WindModel.hh"

namespace gz::sim::wind
{
/// \brief The standard wind: a mean speed and direction.
class StandardModel : public IWindModel
{
  // Documentation inherited.
  public: void SetParameters(const WindParameters &_params) override;

  // Documentation inherited.
  public: math::Vector3d Velocity(const math::Vector3d &_enu,
                                  double _time) const override;

  // Documentation inherited.
  public: std::string_view Kind() const override { return "standard"; }

  /// \brief Parameters.
  private: WindParameters params;
};
}  // namespace gz::sim::wind

#endif  // GZ_SIM_WIND_STANDARDMODEL_HH_
