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

  // Every coefficient explicit: no type is used.
  const bool explicitWater = _spec.attenuation && _spec.backscatter &&
      _spec.veil && _spec.diffuseAttenuation;
  // The default is clear coastal water, the water of most worlds.
  const std::string name = _spec.jerlov.empty() ? "1C" : _spec.jerlov;
  auto type = Jerlov(name);
  if (explicitWater)
  {
    if (!_spec.jerlov.empty())
    {
      _warning = "<jerlov> is ignored: every coefficient is given "
                 "explicitly";
    }
    type = Jerlov("1C");
  }
  else if (!type)
  {
    _error = "unknown Jerlov type '" + _spec.jerlov + "'";
    return false;
  }
  if (!std::isfinite(_spec.waterLevel))
  {
    _error = "<water_level> must be a finite number";
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
    // Scale what the type has above pure water, which type I stands for,
    // so that 4.8 / c_green comes out at the visibility asked for. The
    // water itself stays: clearer water is bluer, and nothing is clearer
    // than pure water.
    const JerlovType pure = *Jerlov("I");
    const double cPure = pure.absorption.Y() + pure.scattering.Y();
    const double cExcess = a.Y() + b.Y() - cPure;
    const double cWanted = kVisibilityConstant / v;
    if (cWanted < cPure)
    {
      _error = "<visibility> is clearer than pure water, which can be seen "
               "for " + std::to_string(kVisibilityConstant / cPure) + " m";
      return false;
    }
    if (cExcess <= 0.0)
    {
      _error = "<visibility> cannot change type " + type->name +
               ", which is pure water";
      return false;
    }
    const double s = (cWanted - cPure) / cExcess;
    a = pure.absorption + (a - pure.absorption) * s;
    b = pure.scattering + (b - pure.scattering) * s;
    kd = pure.diffuseAttenuation + (kd - pure.diffuseAttenuation) * s;
    if (_spec.attenuation)
    {
      _warning += std::string(_warning.empty() ? "" : "; ") +
          "<visibility> is ignored for <attenuation>, given explicitly; "
          "the visibility comes from it instead";
    }
  }

  const Rgb c = a + b;
  WaterOpticsData out;
  out.jerlov = explicitWater ? std::string() : type->name;
  out.attenuation = c;
  out.backscatter = c;
  out.veil = Rgb(c.X() > 0 ? b.X() / c.X() : 0.0,
                 c.Y() > 0 ? b.Y() / c.Y() : 0.0,
                 c.Z() > 0 ? b.Z() / c.Z() : 0.0);
  out.diffuseAttenuation = kd;
  out.waterLevel = _spec.waterLevel;

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
//////////////////////////////////////////////////
const std::vector<NumberParam> &NumberParams()
{
  static const std::vector<NumberParam> params{
    {"visibility",
     [](WaterOpticsSpec &_s, std::optional<double> _v) { _s.visibility = _v; },
     true,
     [](const WaterOpticsData &_d) { return _d.Visibility(); }},
    {"water_level",
     [](WaterOpticsSpec &_s, std::optional<double> _v)
     { _s.waterLevel = _v.value_or(0.0); },
     false,
     [](const WaterOpticsData &_d) { return _d.waterLevel; }},
  };
  return params;
}

//////////////////////////////////////////////////
const std::vector<ColourParam> &ColourParams()
{
  static const std::vector<ColourParam> params{
    {"attenuation", &WaterOpticsSpec::attenuation,
     &WaterOpticsData::attenuation},
    {"backscatter", &WaterOpticsSpec::backscatter,
     &WaterOpticsData::backscatter},
    {"veil", &WaterOpticsSpec::veil, &WaterOpticsData::veil},
    {"diffuse_attenuation", &WaterOpticsSpec::diffuseAttenuation,
     &WaterOpticsData::diffuseAttenuation},
  };
  return params;
}

//////////////////////////////////////////////////
bool SameWater(const WaterOpticsData &_a, const WaterOpticsData &_b)
{
  if (_a.jerlov != _b.jerlov)
    return false;
  for (const auto &p : NumberParams())
  {
    const double x = p.get(_a), y = p.get(_b);
    if (!(x == y || (std::isinf(x) && std::isinf(y))))
      return false;
  }
  for (const auto &p : ColourParams())
  {
    // Exactly: math::Vector3's == allows a tolerance
    const Rgb &x = _a.*p.data, &y = _b.*p.data;
    if (x.X() != y.X() || x.Y() != y.Y() || x.Z() != y.Z())
      return false;
  }
  return true;
}

}  // namespace gz::sim::water_optics
