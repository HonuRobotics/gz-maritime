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
#include "gz/sim/ocean_current/OceanCurrentfield.hh"

#include <cmath>
#include <string>

namespace gz::sim::ocean_current
{
//////////////////////////////////////////////////
bool SetParameter(OceanCurrentParameters &_p, const std::string &_name,
                  double _value)
{
  if (!std::isfinite(_value))
    return false;
  if (_name == "seed")
  {
    if (_value < 0.0 || _value > 4294967295.0)
      return false;
    _p.seed = static_cast<std::uint32_t>(_value);
    return true;
  }
  if (_name == "speed" && _value < 0.0)
    return false;
  // A direction is kept in [0, 360), whatever turn it was given in.
  if (_name == "direction")
  {
    _value = std::fmod(_value, 360.0);
    if (_value < 0.0)
      _value += 360.0;
    if (_value >= 360.0)
      _value = 0.0;
  }
#define GZ_OCEAN_CURRENT_SET(m, name) \
  if (_name == name) { _p.m = _value; return true; }
  GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_SET)
#undef GZ_OCEAN_CURRENT_SET
  return false;
}
}  // namespace gz::sim::ocean_current
