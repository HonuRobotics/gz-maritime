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
#ifndef GZ_SIM_LENS_DROPS_DROPS_HH_
#define GZ_SIM_LENS_DROPS_DROPS_HH_

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

/// \brief Water drops on a camera's lens, as behaviour, not optics: drops
/// land at a rate set by the rain, dry out, slide down when big and merge
/// when they touch. Positions are in the image: x and y in [0, 1] from the
/// top left, a radius as a fraction of the image height. No Gazebo header,
/// nothing thread safe: the caller locks.
namespace gz::sim::lens_drops
{
/// \brief The most drops a lens holds. Equal to the render pass's limit.
inline constexpr std::size_t kMaxDrops = 64;

/// \brief Floats per packed drop: x, y, shown radius, 0.
inline constexpr std::size_t kDropFloats = 4;

/// \brief Smallest radius of a new drop, as a fraction of the image height.
inline constexpr double kMinRadius = 0.015;

/// \brief Largest radius of a new drop, as a fraction of the image height.
inline constexpr double kMaxRadius = 0.06;

/// \brief A drop never grows past this radius by merging.
inline constexpr double kMaxMergedRadius = 0.12;

/// \brief Drops of at least this radius slide down.
inline constexpr double kSlideRadius = 0.05;

/// \brief Speed of a drop of kSlideRadius, in image heights per second.
inline constexpr double kSlideSpeed = 0.04;

/// \brief New drops per second, per mm/h of rain, at exposure 1.
inline constexpr double kDropsPerMmHour = 0.05;

/// \brief The longest step taken at once, in seconds.
inline constexpr double kMaxStep = 1.0;

/// \brief The generator every draw comes from.
using Generator = std::mt19937;

/// \brief One drop on the lens.
struct Drop
{
  /// \brief Centre, in [0, 1] from the image's left.
  double x{0.0};

  /// \brief Centre, in [0, 1] from the image's top; may pass 1 while the
  /// drop slides out.
  double y{0.0};

  /// \brief Radius when fresh, as a fraction of the image height.
  double radius{0.0};

  /// \brief Seconds since it landed, or since the youngest drop it merged
  /// with landed.
  double age{0.0};
};

/// \brief What drives the drops.
struct Params
{
  /// \brief Rain, in mm/h.
  double rainRate{0.0};

  /// \brief How open the lens is to the rain: 0 fully sheltered to 1 open.
  double exposure{1.0};

  /// \brief 0 for drops on the lens, blurred, to 1 for drops on a port
  /// ahead of it, sharp. The model only carries it to the renderer.
  double focus{0.5};

  /// \brief Seconds a drop lasts.
  double dryTime{20.0};

  /// \brief Image width over height.
  double aspect{4.0 / 3.0};
};

/// \brief FNV-1a, 32 bit, of a text: written out so a seed made from a name
/// repeats on every standard library.
/// \param[in] _text The text.
/// \return The hash.
std::uint32_t Hash(const std::string &_text);

/// \brief A uniform draw in (0, 1), written out so a seed repeats on every
/// standard library.
/// \param[in] _rng The generator.
/// \return (_rng() + 0.5) / 2^32.
double Uniform(Generator &_rng);

/// \brief Parameters made safe: a non-finite field takes its default, then
/// rainRate is clamped to [0, 1000], exposure and focus to [0, 1], dryTime
/// to [0.1, 3600] and aspect to [0.1, 10].
/// \param[in] _params The parameters.
/// \return The safe parameters.
Params Sanitize(const Params &_params);

/// \brief How many drops land per second.
/// \param[in] _params The parameters, sanitized first.
/// \return rainRate * exposure * kDropsPerMmHour.
double SpawnRate(const Params &_params);

/// \brief The radius a drop shows as it dries.
/// \param[in] _drop The drop.
/// \param[in] _dryTime Seconds a drop lasts.
/// \return radius * sqrt(max(0, 1 - age / dryTime)), or 0 if _dryTime is
/// not positive.
double ShownRadius(const Drop &_drop, double _dryTime);

/// \brief The drops on one lens.
class Drops
{
  /// \brief Constructor.
  /// \param[in] _seed Seed of the generator.
  /// \param[in] _params What drives the drops; sanitized.
  public: explicit Drops(std::uint32_t _seed = 0u,
                         const Params &_params = Params());

  /// \brief Change what drives the drops. The drops on the lens stay; the
  /// next arrival is drawn again only if the spawn rate changed.
  /// \param[in] _params The parameters; sanitized.
  public: void SetParams(const Params &_params);

  /// \brief What drives the drops.
  /// \return The sanitized parameters.
  public: const Params &Parameters() const;

  /// \brief Age, slide, merge and spawn drops over a step.
  /// \param[in] _dt Seconds; capped at kMaxStep. Zero, negative or not
  /// finite does nothing, as on a paused world.
  public: void Step(double _dt);

  /// \brief Wipe the lens. Time, the schedule, the generator and the count
  /// of drops spawned stay.
  public: void Clear();

  /// \brief Start again: seed the generator, wipe the lens, time and count
  /// back to zero. The parameters stay.
  /// \param[in] _seed Seed of the generator.
  public: void Reset(std::uint32_t _seed);

  /// \brief The drops on the lens.
  /// \return The drops, oldest first.
  public: const std::vector<Drop> &List() const;

  /// \brief Seconds stepped since construction or the last Reset.
  /// \return The time.
  public: double Time() const;

  /// \brief How many drops arrived, kept or not for a full lens.
  /// \return The count.
  public: std::uint64_t Spawned() const;

  /// \brief Write the drops for the renderer: x, y, shown radius and 0 per
  /// drop, as floats.
  /// \param[out] _out At least kDropFloats * _maxDrops floats. Null writes
  /// nothing.
  /// \param[in] _maxDrops The most drops to write.
  /// \return How many drops were written: the least of the drops on the
  /// lens, _maxDrops and kMaxDrops; 0 for a null _out.
  public: std::size_t Pack(float *_out, std::size_t _maxDrops) const;

  /// \brief What drives the drops, sanitized.
  private: Params params;

  /// \brief Every draw comes from here.
  private: Generator rng;

  /// \brief The drops on the lens, oldest first.
  private: std::vector<Drop> drops;

  /// \brief Seconds stepped.
  private: double time{0.0};

  /// \brief Time of the next arrival; +infinity when none is scheduled.
  private: double nextArrival;

  /// \brief Drops that arrived.
  private: std::uint64_t spawned{0};
};
}  // namespace gz::sim::lens_drops

#endif  // GZ_SIM_LENS_DROPS_DROPS_HH_
