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
#ifndef GZ_MARITIME_SONAR_WATER_HH_
#define GZ_MARITIME_SONAR_WATER_HH_

namespace gz::maritime::sonar
{
/// \brief The water a sonar works in: what sets absorption and the true
/// speed of sound. One value each for the whole water column.
struct Water
{
  /// \brief Temperature, degrees Celsius.
  double temperature{10.0};

  /// \brief Salinity, PSU.
  double salinity{35.0};

  /// \brief Depth at which absorption is evaluated, m.
  double depth{0.0};

  /// \brief Acidity, which sets the boric acid relaxation.
  double ph{8.0};

  /// \brief The true speed of sound, m/s: echoes return at 2R over this.
  double soundSpeed{1500.0};
};

/// \brief Absorption of sound in seawater from the Francois and Garrison
/// equations (JASA 72(6), 1982): boric acid and magnesium sulfate
/// relaxations plus pure water.
/// \param[in] _frequency Frequency, Hz.
/// \param[in] _water The water.
/// \return Absorption, dB per meter, one way.
double AbsorptionDbPerMeter(double _frequency, const Water &_water);
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_WATER_HH_
