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
#include <map>
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

  /// \brief Seed of anything random in a model. The ocean current system
  /// resolves a requested 0 into a drawn seed before it writes the recipe,
  /// so every process that rebuilds the model gets the same current.
  std::uint32_t seed{1};

  /// \brief Parameters the model owns, by name, as text: the file of a
  /// gridded current (`source`), the constituents of a tide, a depth table.
  /// Opaque to the ocean current system, which only stores and replicates
  /// them; their meaning belongs to the model, which accepts or rejects them
  /// (IOceanCurrentModel::Validate). Empty for the model built in; here so
  /// that a richer model changes neither the recipe nor the query consumers
  /// are written against.
  std::map<std::string, std::string> extra;
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
      << _d.params.seed;
#define GZ_OCEAN_CURRENT_WR(m, name) _os << ' ' << _d.params.m;
  GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_WR)
#undef GZ_OCEAN_CURRENT_WR
  // The model's own parameters: a count, then quoted name and value pairs.
  _os << ' ' << _d.params.extra.size();
  for (const auto &[name, value] : _d.params.extra)
    _os << ' ' << std::quoted(name) << ' ' << std::quoted(value);
  _os.precision(old);
  return _os;
}

/// \brief Read a recipe written by operator<<.
/// \param[in] _is Stream.
/// \param[out] _d Recipe.
/// \return The stream.
inline std::istream &operator>>(std::istream &_is, OceanCurrentfieldData &_d)
{
  _is >> std::quoted(_d.model) >> _d.generation >> _d.params.seed;
#define GZ_OCEAN_CURRENT_RD(m, name) _is >> _d.params.m;
  GZ_OCEAN_CURRENT_PARAM_TABLE(GZ_OCEAN_CURRENT_RD)
#undef GZ_OCEAN_CURRENT_RD
  std::size_t count{0};
  _is >> count;
  _d.params.extra.clear();
  for (std::size_t i = 0; i < count && _is; ++i)
  {
    std::string name;
    std::string value;
    _is >> std::quoted(name) >> std::quoted(value);
    _d.params.extra[name] = value;
  }
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

/// \brief Whether a name is one of the numeric parameters the recipe types,
/// the table's or "seed"; any other name belongs to the model.
/// \param[in] _name Parameter name.
/// \return True if SetParameter takes it.
bool IsTypedParameter(const std::string &_name);
}  // namespace gz::sim::ocean_current

#endif  // GZ_SIM_OCEAN_CURRENT_OCEANCURRENTFIELD_HH_
