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
#ifndef GZ_SIM_WIND_WINDFIELD_HH_
#define GZ_SIM_WIND_WINDFIELD_HH_

#include <cstdint>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <string>

namespace gz::sim::wind
{
/// \brief Every wind parameter, as (member, name). The name is the key in the
/// world file and on the wind topic; the same table drives serialization and
/// parsing, so there is no second list to keep in sync. All are doubles.
///
/// * speed: mean wind speed, m/s.
/// * direction: direction the wind comes from, degrees clockwise from true
///   north, as in weather reports.
/// * vertical: vertical component, m/s, positive up.
#define GZ_WIND_PARAM_TABLE(X) \
  X(speed,     "speed")        \
  X(direction, "direction")    \
  X(vertical,  "vertical")

/// \brief What a wind model is built from.
struct WindParameters
{
  /// \brief Mean wind speed, m/s.
  double speed{0.0};

  /// \brief Direction the wind comes from, degrees clockwise from true north.
  double direction{0.0};

  /// \brief Vertical component, m/s, positive up.
  double vertical{0.0};

  /// \brief Seed of anything random in the model. The wind system resolves a
  /// requested 0 into a drawn seed before it writes the recipe, so every
  /// process that rebuilds the model gets the same wind.
  std::uint32_t seed{1};
};

/// \brief The wind of a world as a recipe: which model, with which
/// parameters. It carries no engine; every consumer rebuilds its own from the
/// recipe, the way the wave field works, so a new model is a new engine and
/// no change to the systems that ask for the wind.
struct WindfieldData
{
  /// \brief Name the model was registered under.
  std::string model{"standard"};

  /// \brief The model's parameters.
  WindParameters params;

  /// \brief Goes up on every change of the recipe, so a consumer knows when
  /// to rebuild its engine.
  std::uint64_t generation{0};
};

/// \brief Write the recipe, at full precision, so a copy in another process
/// rebuilds exactly the same wind.
/// \param[in] _os Stream.
/// \param[in] _d Recipe.
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_os, const WindfieldData &_d)
{
  const auto old = _os.precision(std::numeric_limits<double>::max_digits10);
  _os << std::quoted(_d.model) << ' ' << _d.generation << ' ' << _d.params.seed;
#define GZ_WIND_WR(m, name) _os << ' ' << _d.params.m;
  GZ_WIND_PARAM_TABLE(GZ_WIND_WR)
#undef GZ_WIND_WR
  _os.precision(old);
  return _os;
}

/// \brief Read a recipe written by operator<<.
/// \param[in] _is Stream.
/// \param[out] _d Recipe.
/// \return The stream.
inline std::istream &operator>>(std::istream &_is, WindfieldData &_d)
{
  _is >> std::quoted(_d.model) >> _d.generation >> _d.params.seed;
#define GZ_WIND_RD(m, name) _is >> _d.params.m;
  GZ_WIND_PARAM_TABLE(GZ_WIND_RD)
#undef GZ_WIND_RD
  return _is;
}

/// \brief Set one parameter by its name.
/// \param[in,out] _p Parameters.
/// \param[in] _name Name from the table, or "seed".
/// \param[in] _value Value.
/// \return False if the name is unknown or the value out of range.
bool SetParameter(WindParameters &_p, const std::string &_name, double _value);
}  // namespace gz::sim::wind

#endif  // GZ_SIM_WIND_WINDFIELD_HH_
