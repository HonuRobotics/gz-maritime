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
#ifndef GZ_MARITIME_SONAR_SYSTEMS_SONARSDF_HH_
#define GZ_MARITIME_SONAR_SYSTEMS_SONARSDF_HH_

#include <cstdint>
#include <optional>
#include <string>

#include <sdf/Element.hh>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Water.hh"

namespace gz::sim::maritime
{
/// \brief The custom sensor types the sonar system drives. What each type
/// of sonar is, and how this project uses the terms (imaging, profiling,
/// sidescan, multibeam and their synonyms), is in the design's Glossary:
/// docs/design/sonar.md, section "Sonars in scope", "Glossary".

inline constexpr const char *kImagingSonar = "imaging_sonar";
inline constexpr const char *kSidescanSonar = "sidescan_sonar";
inline constexpr const char *kProfilingSonar = "profiling_sonar";

/// \brief Whether a gz:type is one of the sonars.
/// \param[in] _type The gz:type.
/// \return True for a sonar's type.
bool IsSonarType(const std::string &_type);

/// \brief A sonar's configuration: its type's exemplar, with whatever its
/// <gz:sonar> block sets on top. Angles in radians, as SDF gives them.
/// \param[in] _type The sensor's gz:type.
/// \param[in] _block The <gz:sonar> element; null for the exemplar as it is.
/// \param[out] _labels Whether the ground-truth labels topic is on.
/// \param[out] _error Why it failed, when it returns empty.
/// \return The configuration, or empty if the block is invalid.
std::optional<gz::maritime::sonar::SonarConfig> LoadSonarConfig(
  const std::string &_type, const sdf::ElementPtr &_block, bool &_labels,
  std::string &_error);

/// \brief The acoustic marks on one <visual> or <model>: each set only where
/// the element is present, so the levels of the precedence can be layered.
struct AcousticMarks
{
  std::optional<double> lambertDb;
  std::optional<double> specularDb;
  std::optional<double> specularWidth;
  bool transparent{false};
};

/// \brief Read the acoustic marks of a <visual> or <model>: its attributes
/// gz:acoustic_lambert, gz:acoustic_specular, gz:acoustic_specular_width
/// (numbers) and gz:acoustic_transparent (true or false), as in
/// <visual name="glass" gz:acoustic_lambert="-30">. A malformed value is
/// reported and left unset.
/// \param[in] _element The element.
/// \return The marks it sets.
AcousticMarks ReadAcousticMarks(const sdf::ElementPtr &_element);

/// \brief Read a block of acoustic values written without the prefix, as in
/// the world system's <seabed> and <default>: <lambert>, <specular>,
/// <specular_width>.
/// \param[in] _element The block.
/// \return The values it sets.
AcousticMarks ReadAcousticValues(const sdf::ElementPtr &_element);

/// \brief Read the world system's water and seed: <temperature> (-2 to
/// 40 C), <salinity> (0 to 45 PSU), <sound_speed> (1300 to 1700 m/s) and
/// <seed> (a whole number), each strictly; absent ones keep their value.
/// \param[in] _plugin The sonar system's <plugin> element.
/// \param[in,out] _water The water, with defaults on entry.
/// \param[in,out] _seed The seed, with its default on entry.
/// \param[out] _error Why it failed, when it returns false.
/// \return Whether every value given was valid.
bool ReadWater(const sdf::ElementPtr &_plugin,
  gz::maritime::sonar::Water &_water, std::uint64_t &_seed,
  std::string &_error);

/// \brief Apply marks on top of properties: what the marks set wins.
/// \param[in,out] _properties The properties.
/// \param[in] _marks The marks.
void ApplyMarks(gz::maritime::sonar::AcousticProperties &_properties,
  const AcousticMarks &_marks);
}  // namespace gz::sim::maritime

#endif  // GZ_MARITIME_SONAR_SYSTEMS_SONARSDF_HH_
