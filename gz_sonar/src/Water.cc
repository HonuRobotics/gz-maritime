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
#include "gz/maritime/sonar/Water.hh"

#include <cmath>

namespace gz::maritime::sonar
{
//////////////////////////////////////////////////
double AbsorptionDbPerMeter(double _frequency, const Water &_water)
{
  const double t = _water.temperature;
  const double s = _water.salinity;
  const double d = _water.depth;
  const double f = _frequency / 1000.0;  // the equations take kHz
  const double f2 = f * f;

  // The sound speed the equations use internally.
  const double c = 1412.0 + 3.21 * t + 1.19 * s + 0.0167 * d;

  // Boric acid.
  const double a1 = 8.86 / c * std::pow(10.0, 0.78 * _water.ph - 5.0);
  const double f1 = 2.8 * std::sqrt(s / 35.0) *
    std::pow(10.0, 4.0 - 1245.0 / (t + 273.0));
  const double boric = a1 * f1 * f2 / (f1 * f1 + f2);

  // Magnesium sulfate.
  const double a2 = 21.44 * s / c * (1.0 + 0.025 * t);
  const double p2 = 1.0 - 1.37e-4 * d + 6.2e-9 * d * d;
  const double fm = 8.17 * std::pow(10.0, 8.0 - 1990.0 / (t + 273.0)) /
    (1.0 + 0.0018 * (s - 35.0));
  const double magnesium = a2 * p2 * fm * f2 / (fm * fm + f2);

  // Pure water.
  const double p3 = 1.0 - 3.83e-5 * d + 4.9e-10 * d * d;
  const double a3 = t <= 20.0 ?
    4.937e-4 - 2.59e-5 * t + 9.11e-7 * t * t - 1.50e-8 * t * t * t :
    3.964e-4 - 1.146e-5 * t + 1.45e-7 * t * t - 6.5e-10 * t * t * t;
  const double water = a3 * p3 * f2;

  // dB/km to dB/m.
  return (boric + magnesium + water) / 1000.0;
}
}  // namespace gz::maritime::sonar
