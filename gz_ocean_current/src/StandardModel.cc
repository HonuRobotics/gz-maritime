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

#include <string>

namespace gz::sim::ocean_current
{
//////////////////////////////////////////////////
std::string StandardModel::Validate(
    const OceanCurrentParameters &_params) const
{
  if (_params.extra.empty())
    return {};
  std::string names;
  for (const auto &[name, value] : _params.extra)
    names += (names.empty() ? "" : ", ") + name;
  return "the standard model takes no parameter [" + names + "]";
}

//////////////////////////////////////////////////
void StandardModel::SetParameters(const OceanCurrentParameters &_params)
{
  this->velocity = VelocityFromSet(_params.speed, _params.direction);
}

//////////////////////////////////////////////////
math::Vector3d StandardModel::Velocity(const math::Vector3d &/*_enu*/,
    double /*_time*/) const
{
  return this->velocity;
}
}  // namespace gz::sim::ocean_current
