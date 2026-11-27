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
#ifndef GZ_SIM_WATER_OPTICS_WATEROPTICS_HH_
#define GZ_SIM_WATER_OPTICS_WATEROPTICS_HH_

#include <cstdint>
#include <iomanip>
#include <istream>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>

namespace gz::sim::water_optics
{
/// \brief Colour channels, as the X, Y and Z of a math::Vector3d: X red, Y
/// green, Z blue. Every per-channel quantity here is stored that way.
using Rgb = math::Vector3d;

/// \brief The distance at which a black target can no longer be seen
/// horizontally, from the beam attenuation in the green, m: Zaneveld and
/// Pegau (2003), visibility = 4.8 / c. Green, because that is the band a
/// camera, and an eye, is most sensitive in.
constexpr double kVisibilityConstant{4.8};

/// \brief The shortest visibility a world can ask for, m. Below it the
/// water is mud, and the numbers lose their meaning.
constexpr double kMinVisibility{0.1};

/// \brief What a world asks for: a water type, a visibility, explicit
/// coefficients, each optional. Resolve turns it into the water's optics.
struct WaterOpticsSpec
{
  /// \brief Jerlov water type: I, IA, IB, II, III, 1C, 3C, 5C, 7C, 9C.
  /// Empty: coastal 1C.
  std::string jerlov;

  /// \brief Horizontal visibility of a black target, m. Scales the type's
  /// absorption and scattering above pure water's.
  std::optional<double> visibility;

  /// \brief Attenuation of an object's light, beta^D, 1/m, per channel.
  std::optional<Rgb> attenuation;

  /// \brief Attenuation of the veil's build-up, beta^B, 1/m, per channel.
  std::optional<Rgb> backscatter;

  /// \brief The veil, B-infinity, just below the surface, as a fraction of
  /// the daylight there, per channel.
  std::optional<Rgb> veil;

  /// \brief Diffuse attenuation of the daylight with depth, K_d, 1/m.
  std::optional<Rgb> diffuseAttenuation;

  /// \brief World z of the still water, m.
  double waterLevel{0.0};
};

/// \brief The water's optics, resolved: what every consumer reads.
///
/// The image formation model they feed, per channel, at range r along a ray
/// from a camera at depth z below the surface (Akkaynak and Treibitz 2018):
///
///   I = J exp(-attenuation r) + veil(z) (1 - exp(-backscatter r)),
///   veil(z) = veil exp(-diffuseAttenuation z).
struct WaterOpticsData
{
  /// \brief The Jerlov type the optics came from, or empty if explicit.
  std::string jerlov;

  /// \brief beta^D, 1/m, per channel.
  Rgb attenuation{0, 0, 0};

  /// \brief beta^B, 1/m, per channel.
  Rgb backscatter{0, 0, 0};

  /// \brief B-infinity just below the surface, per channel, as a fraction
  /// of the daylight there.
  Rgb veil{0, 0, 0};

  /// \brief K_d, 1/m, per channel.
  Rgb diffuseAttenuation{0, 0, 0};

  /// \brief World z of the still water, m.
  double waterLevel{0.0};

  /// \brief Goes up on every change, so a consumer knows to re-read.
  std::uint64_t generation{0};

  /// \brief Horizontal visibility of a black target, m: 4.8 / attenuation
  /// in the green. Infinite in water that does not attenuate.
  /// \return The visibility.
  double Visibility() const
  {
    return this->attenuation.Y() > 0.0
        ? kVisibilityConstant / this->attenuation.Y()
        : std::numeric_limits<double>::infinity();
  }
};

/// \brief A number of the water: its world file element and set topic
/// key, how it sets a spec, and where the resolved optics hold it. Every
/// consumer of the parameters (the system's world file and topic, its
/// ground truth, the docs test) goes through this table, so a new one is
/// added in one place.
struct NumberParam
{
  /// \brief Element and key.
  const char *name;

  /// \brief Set it in a spec; std::nullopt removes it, back to what the
  /// type gives, where it can be removed.
  void (*set)(WaterOpticsSpec &, std::optional<double>);

  /// \brief Whether an empty value removes it.
  bool removable;

  /// \brief Its value in the resolved optics, for the ground truth.
  double (*get)(const WaterOpticsData &);
};

/// \brief A per-channel value of the water, "r g b": its element and key,
/// and its members in the spec and in the resolved optics.
struct ColourParam
{
  /// \brief Element and key.
  const char *name;

  /// \brief In the spec; unset, the type gives it.
  std::optional<Rgb> WaterOpticsSpec::*spec;

  /// \brief In the resolved optics.
  Rgb WaterOpticsData::*data;
};

/// \brief The water's numbers.
/// \return The table.
const std::vector<NumberParam> &NumberParams();

/// \brief The water's per-channel values.
/// \return The table.
const std::vector<ColourParam> &ColourParams();

/// \brief Whether two optics are the same water, their generation aside.
/// \param[in] _a One.
/// \param[in] _b The other.
/// \return True if they are.
bool SameWater(const WaterOpticsData &_a, const WaterOpticsData &_b);

/// \brief Turn what a world asks for into the water's optics.
///
/// * The base is a Jerlov type (coastal 1C if none is named), from its
///   published absorption a, scattering b and diffuse attenuation K_d per
///   channel: attenuation = backscatter = c = a + b, veil = b / c (the
///   single-scattering albedo), diffuse attenuation = K_d.
/// * A visibility scales what a, b and K_d have above pure water's (type
///   I), every channel by the same factor, so that 4.8 / c_green comes out
///   at the visibility: clearer water turns bluer, as real water does, and
///   it can be no clearer than pure water (about 100 m). Type I itself
///   cannot be scaled.
/// * Each explicit coefficient then replaces its derived value. When all
///   four are explicit, no type is used: `jerlov` comes out empty, and a
///   type asked for, known or not, is ignored with a warning.
/// \param[in] _spec What the world asks for.
/// \param[out] _out The optics, when the spec is valid; its generation is
/// left as it was.
/// \param[out] _error Why not, when the spec is invalid: an unknown type, a
/// visibility that is not positive, a negative or non-finite coefficient.
/// \param[out] _warning Set when the spec is valid but not all of it is
/// used: a visibility given with an explicit attenuation.
/// \return True if the spec is valid.
bool Resolve(const WaterOpticsSpec &_spec, WaterOpticsData &_out,
             std::string &_error, std::string &_warning);

/// \brief Write the optics at full precision.
/// \param[in] _os Stream.
/// \param[in] _d Optics.
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_os, const WaterOpticsData &_d)
{
  const auto old = _os.precision(std::numeric_limits<double>::max_digits10);
  _os << std::quoted(_d.jerlov) << ' ' << _d.generation << ' '
      << _d.waterLevel;
  for (const Rgb *v : {&_d.attenuation, &_d.backscatter, &_d.veil,
                       &_d.diffuseAttenuation})
  {
    _os << ' ' << v->X() << ' ' << v->Y() << ' ' << v->Z();
  }
  _os.precision(old);
  return _os;
}

/// \brief Read optics written by operator<<.
/// \param[in] _is Stream.
/// \param[out] _d Optics.
/// \return The stream.
inline std::istream &operator>>(std::istream &_is, WaterOpticsData &_d)
{
  _is >> std::quoted(_d.jerlov) >> _d.generation >> _d.waterLevel;
  for (Rgb *v : {&_d.attenuation, &_d.backscatter, &_d.veil,
                 &_d.diffuseAttenuation})
  {
    double x{0}, y{0}, z{0};
    _is >> x >> y >> z;
    v->Set(x, y, z);
  }
  return _is;
}
}  // namespace gz::sim::water_optics

#endif  // GZ_SIM_WATER_OPTICS_WATEROPTICS_HH_
