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
#ifndef GZ_SIM_OCEAN_CURRENT_OCEANCURRENTFIELD_HH_
#define GZ_SIM_OCEAN_CURRENT_OCEANCURRENTFIELD_HH_

#include <cstdint>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <string>

namespace gz::sim::ocean_current
{
/// \brief Every numeric parameter a world sets, as (member, name). The name
/// is the tag in the world file; the same table drives serialization and
/// parsing, so there is no second list to keep in sync. All are doubles.
///
/// * speed: speed of the current, m/s.
/// * direction: direction the current sets towards, degrees clockwise from
///   true north, as charts draw it: 90 sets east.
/// * water_level: world z of the water's surface, m, so a model that varies
///   with depth knows where the surface is.
#define GZ_OCEAN_CURRENT_PARAM_TABLE(X)   \
  X(speed,       "speed")                 \
  X(direction,   "direction")             \
  X(water_level, "water_level")

/// \brief What an ocean current model is built from.
struct OceanCurrentParameters
{
  /// \brief Speed of the current, m/s.
  double speed{0.0};

  /// \brief Direction the current sets towards, degrees clockwise from true
  /// north.
  double direction{0.0};

  /// \brief World z of the water's surface, m. The standard model is the
  /// same at every depth and does not read it; a model that varies with depth
  /// measures depth from it.
  double water_level{0.0};

  /// \brief An external source for a model that reads one, such as the file
  /// of a gridded current. Opaque to the ocean current system, which only
  /// stores and replicates it: its meaning belongs to the model that reads
  /// it. Empty for the model built in; here so that such a model later
  /// changes neither the recipe nor the query consumers are written against.
  std::string source;

  /// \brief Seed of anything random in a model. The ocean current system
  /// resolves a requested 0 into a drawn seed before it writes the recipe,
  /// so every process that rebuilds the model gets the same current.
  std::uint32_t seed{1};
};

/// \brief The ocean current of a world as a recipe: which model, with which
/// parameters. It carries no engine; every consumer rebuilds its own from
/// the recipe, the way the wave field and the wind work, so a new model is a
/// new registered model and no change to the system that owns the recipe or
/// to the systems that ask for the current.
struct OceanCurrentfieldData
{
  /// \brief Name the model was registered under.
  std::string model{"standard"};

  /// \brief The model's parameters.
  OceanCurrentParameters params;

  /// \brief Goes up on every change of the recipe, so a consumer knows when
  /// to rebuild its engine.
  std::uint64_t generation{0};
};

/// \brief Write the recipe, at full precision, so a copy in another process
/// rebuilds exactly the same current.
/// \param[in] _os Stream.
/// \param[in] _d Recipe.
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_os,
                                const OceanCurrentfieldData &_d)
{
  const auto old = _os.precision(std::numeric_limits<double>::max_digits10);
  _os << std::quoted(_d.model) << ' ' << _d.generation << ' '
      << _d.params.seed << ' ' << std::quoted(_d.params.source);
#define GZ_OCEAN_CURRENT_WR(m, name) _os << ' ' << _d.params.m;
  GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_WR)
#undef GZ_OCEAN_CURRENT_WR
  _os.precision(old);
  return _os;
}

/// \brief Read a recipe written by operator<<.
/// \param[in] _is Stream.
/// \param[out] _d Recipe.
/// \return The stream.
inline std::istream &operator>>(std::istream &_is, OceanCurrentfieldData &_d)
{
  _is >> std::quoted(_d.model) >> _d.generation >> _d.params.seed
      >> std::quoted(_d.params.source);
#define GZ_OCEAN_CURRENT_RD(m, name) _is >> _d.params.m;
  GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_RD)
#undef GZ_OCEAN_CURRENT_RD
  return _is;
}

/// \brief Set one numeric parameter by its name. A direction is wrapped into
/// [0, 360).
/// \param[in,out] _p Parameters.
/// \param[in] _name Name from the table, or "seed".
/// \param[in] _value Value.
/// \return False if the name is unknown or the value out of range.
bool SetParameter(OceanCurrentParameters &_p, const std::string &_name,
                  double _value);
}  // namespace gz::sim::ocean_current

#endif  // GZ_SIM_OCEAN_CURRENT_OCEANCURRENTFIELD_HH_
