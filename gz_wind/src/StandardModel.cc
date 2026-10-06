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
#include "StandardModel.hh"

#include <cmath>

#include <gz/math/Angle.hh>

namespace gz::sim::wind
{
//////////////////////////////////////////////////
void StandardModel::SetParameters(const WindParameters &_params)
{
  this->params = _params;
}

//////////////////////////////////////////////////
math::Vector3d StandardModel::Velocity(const math::Vector3d &/*_enu*/,
    double /*_time*/) const
{
  // A wind from a direction blows towards the opposite one.
  const double a = GZ_DTOR(this->params.direction);
  return {-this->params.speed * std::sin(a),
          -this->params.speed * std::cos(a),
          this->params.vertical};
}
}  // namespace gz::sim::wind
