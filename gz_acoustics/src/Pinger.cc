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
#include "gz/sim/acoustics/Pinger.hh"

#include <cmath>
#include <string>

namespace gz::sim::acoustics
{
//////////////////////////////////////////////////
bool SetParameter(PingerData &_p, const std::string &_name, double _value)
{
  if (!std::isfinite(_value))
    return false;
  if (_name == "active")
  {
    _p.active = 0.0 != _value;
    return true;
  }
  if ((_name == "frequency" || _name == "period") && _value <= 0.0)
    return false;
  if (_value < 0.0)
    return false;
#define GZ_ACOUSTICS_SET(m, name) \
  if (_name == name) { _p.m = _value; return true; }
  GZ_ACOUSTICS_PINGER_TABLE(GZ_ACOUSTICS_SET)
#undef GZ_ACOUSTICS_SET
  return false;
}
}  // namespace gz::sim::acoustics
