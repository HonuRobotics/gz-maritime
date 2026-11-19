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
#include "gz/sim/water_optics/WaterOptics.hh"

#include <cmath>
#include <string>

#include "gz/sim/water_optics/Jerlov.hh"

namespace gz::sim::water_optics
{
namespace
{
/// \brief Whether every channel is finite and not negative.
bool NonNegative(const Rgb &_v)
{
  for (double c : {_v.X(), _v.Y(), _v.Z()})
  {
    if (!std::isfinite(c) || c < 0.0)
      return false;
  }
  return true;
}
}  // namespace

//////////////////////////////////////////////////
bool Resolve(const WaterOpticsSpec &_spec, WaterOpticsData &_out,
             std::string &_error, std::string &_warning)
{
  _error.clear();
  _warning.clear();

  const std::string name = _spec.jerlov.empty() ? "II" : _spec.jerlov;
  const auto type = Jerlov(name);
  if (!type)
  {
    _error = "unknown Jerlov type '" + _spec.jerlov + "'";
    return false;
  }
  if (!std::isfinite(_spec.waterLevel))
  {
    _error = "<water_level> must be a finite number";
    return false;
  }
  if (!std::isfinite(_spec.daylight) || _spec.daylight < 0.0)
  {
    _error = "<daylight> must be a number, 0 or more";
    return false;
  }
  if (!(_spec.caustics >= 0.0 && _spec.caustics <= 1.0))
  {
    _error = "<caustics> must be between 0 and 1";
    return false;
  }
  if (!(_spec.marineSnow >= 0.0 && _spec.marineSnow <= 1.0))
  {
    _error = "<marine_snow> must be between 0 and 1";
    return false;
  }

  Rgb a = type->absorption;
  Rgb b = type->scattering;
  Rgb kd = type->diffuseAttenuation;

  if (_spec.visibility)
  {
    const double v = *_spec.visibility;
    if (!std::isfinite(v) || v <= 0.0)
    {
      _error = "<visibility> must be a positive number of metres";
      return false;
    }
    // Scale the whole water, every channel alike, so its colour stays and
    // 4.8 / c_green comes out at the visibility asked for.
    const double cGreen = a.Y() + b.Y();
    const double s = kVisibilityConstant / (v * cGreen);
    a *= s;
    b *= s;
    kd *= s;
    if (_spec.attenuation)
    {
      _warning = "<visibility> is ignored for <attenuation>, given "
                 "explicitly; the visibility comes from it instead";
    }
  }

  const Rgb c = a + b;
  WaterOpticsData out;
  out.jerlov = type->name;
  out.attenuation = c;
  out.backscatter = c;
  out.veil = Rgb(c.X() > 0 ? b.X() / c.X() : 0.0,
                 c.Y() > 0 ? b.Y() / c.Y() : 0.0,
                 c.Z() > 0 ? b.Z() / c.Z() : 0.0);
  out.diffuseAttenuation = kd;
  out.waterLevel = _spec.waterLevel;
  out.daylight = _spec.daylight;
  out.caustics = _spec.caustics;
  out.marineSnow = _spec.marineSnow;

  struct Override
  {
    const std::optional<Rgb> &value;
    Rgb &target;
    const char *tag;
  };
  for (const Override &o : {
           Override{_spec.attenuation, out.attenuation, "attenuation"},
           Override{_spec.backscatter, out.backscatter, "backscatter"},
           Override{_spec.veil, out.veil, "veil"},
           Override{_spec.diffuseAttenuation, out.diffuseAttenuation,
                    "diffuse_attenuation"}})
  {
    if (!o.value)
      continue;
    if (!NonNegative(*o.value))
    {
      _error = std::string("<") + o.tag +
               "> needs three finite numbers, none negative";
      return false;
    }
    o.target = *o.value;
  }

  out.generation = _out.generation;
  _out = out;
  return true;
}
}  // namespace gz::sim::water_optics
