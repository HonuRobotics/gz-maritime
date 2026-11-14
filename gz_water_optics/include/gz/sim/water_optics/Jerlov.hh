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
#ifndef GZ_SIM_WATER_OPTICS_JERLOV_HH_
#define GZ_SIM_WATER_OPTICS_JERLOV_HH_

#include <optional>
#include <string>
#include <vector>

#include "gz/sim/water_optics/WaterOptics.hh"

namespace gz::sim::water_optics
{
/// \brief A Jerlov water type's inherent optical properties, per colour
/// channel (X red, Y green, Z blue), at the wavelength standing for each
/// channel (JerlovWavelengths). The sources are cited in Jerlov.cc.
struct JerlovType
{
  /// \brief Its name: I, IA, IB, II, III, 1C, 3C, 5C, 7C, 9C.
  std::string name;

  /// \brief Absorption a, 1/m.
  Rgb absorption;

  /// \brief Scattering b, 1/m.
  Rgb scattering;

  /// \brief Diffuse attenuation of downwelling light K_d, 1/m.
  Rgb diffuseAttenuation;
};

/// \brief A type by name, case insensitive.
/// \param[in] _name The type's name.
/// \return The type, or nothing if there is none of that name.
std::optional<JerlovType> Jerlov(const std::string &_name);

/// \brief Every type, oceanic first, clearest first.
/// \return The types.
const std::vector<JerlovType> &JerlovTypes();

/// \brief The wavelength standing for each channel in the table, nm.
/// \return Red, green, blue.
Rgb JerlovWavelengths();
}  // namespace gz::sim::water_optics

#endif  // GZ_SIM_WATER_OPTICS_JERLOV_HH_
