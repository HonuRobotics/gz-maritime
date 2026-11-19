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

/// \brief Water drops on a camera's lens, as behaviour, not optics. Drops
/// land at a rate set by the rain flux on the lens, stay pinned until
/// gravity along the lens and the apparent wind overcome a threshold that
/// grows with their size, then slide, meander, shed beads and leave wet
/// trails; they merge keeping their water, and dry. Positions are in the
/// image: x in [0, 1] of the width, y in [0, 1] of the height from the top,
/// sizes in image heights. No Gazebo header, nothing thread safe: the caller
/// locks.
namespace gz::sim::lens_drops
{
/// \brief The most drops a lens holds, mist included.
inline constexpr std::size_t kMaxDrops = 2048;

/// \brief The most wet trails kept; the oldest goes first.
inline constexpr std::size_t kMaxTrails = 512;

/// \brief Floats per packed drop: x, y, shown radius, 0.
inline constexpr std::size_t kDropFloats = 4;

/// \brief Smallest radius of a new drop, mist, in image heights.
inline constexpr double kMinRadius = 0.002;

/// \brief Largest radius of a new drop, in image heights.
inline constexpr double kMaxRadius = 0.05;

/// \brief A drop never grows past this radius by merging.
inline constexpr double kMaxMergedRadius = 0.12;

/// \brief Sliding speed at a push of 2, in image heights per second.
inline constexpr double kSlideSpeed = 0.06;

/// \brief Apparent air speed, m/s, that alone moves a drop of the slide
/// radius.
inline constexpr double kAirPush = 15.0;

/// \brief Largest turn of a sliding drop, rad per second of sliding.
inline constexpr double kMeander = 0.6;

/// \brief Image heights slid between two beads left behind, on average;
/// each spacing is drawn from 0.5 to 1.5 of it.
inline constexpr double kShedDistance = 0.03;

/// \brief Largest radius of a bead over the radius of the drop that leaves
/// it; each is drawn from 0.5 to 1 of it.
inline constexpr double kBeadFraction = 0.3;

/// \brief New drops per second, per mm/h of rain, at flux 1 and exposure 1.
inline constexpr double kDropsPerMmHour = 0.4;

/// \brief Flux added for splashes off the camera's housing.
inline constexpr double kSplashFloor = 0.1;

/// \brief Fall speed of a rain drop, m/s.
inline constexpr double kRainTerminalSpeed = 6.5;

/// \brief The longest step taken at once, in seconds.
inline constexpr double kMaxStep = 1.0;

/// \brief Refractive index of water.
inline constexpr double kWaterIor = 1.333;

/// \brief The generator every draw comes from.
using Generator = std::mt19937;

/// \brief One drop on the lens.
struct Drop
{
  /// \brief Centre, in [0, 1] of the width from the left.
  double x{0.0};

  /// \brief Centre, in [0, 1] of the height from the top.
  double y{0.0};

  /// \brief Radius when fresh, in image heights.
  double radius{0.0};

  /// \brief Seconds since it landed, or since the youngest drop it merged
  /// with landed.
  double age{0.0};

  /// \brief Velocity to the image's right, image heights per second.
  double vx{0.0};

  /// \brief Velocity down the image, image heights per second.
  double vy{0.0};

  /// \brief Image heights slid since the last bead.
  double travelled{0.0};

  /// \brief Amplitude of the rim's wobble, in [0, 0.15].
  float wobble{0.0f};

  /// \brief First phase of the rim's wobble, rad.
  float phaseA{0.0f};

  /// \brief Second phase of the rim's wobble, rad.
  float phaseB{0.0f};

  /// \brief Where the wet trail being laid started, image coordinates;
  /// set when the drop starts to slide and at each bead.
  double markX{0.0};

  /// \brief Where the wet trail being laid started, image coordinates.
  double markY{0.0};

  /// \brief True while the drop slides.
  bool sliding{false};
};

/// \brief A wet film a sliding drop left behind, a segment.
struct Trail
{
  /// \brief Start, image coordinates as Drop.
  double x0{0.0};

  /// \brief Start, image coordinates as Drop.
  double y0{0.0};

  /// \brief End, image coordinates as Drop.
  double x1{0.0};

  /// \brief End, image coordinates as Drop.
  double y1{0.0};

  /// \brief Half its width, image heights.
  double halfWidth{0.0};

  /// \brief Seconds since it was left.
  double age{0.0};
};

/// \brief What drives the drops, from the SDF and the set topic.
struct Params
{
  /// \brief Rain, mm/h, in [0, 1000].
  double rainRate{0.0};

  /// \brief How open the lens is to the rain, in [0, 1].
  double exposure{1.0};

  /// \brief 0 for drops on the lens, blurred, to 1 for drops on a port
  /// ahead of it, sharp.
  double focus{0.5};

  /// \brief Seconds a drop lasts, in [0.1, 3600].
  double dryTime{20.0};

  /// \brief Image width over height, in [0.1, 10].
  double aspect{4.0 / 3.0};

  /// \brief Contact angle of water on the lens, rad, in [0.17, 2.6]; 0.96,
  /// 55 degrees, is plain glass.
  double contactAngle{0.96};

  /// \brief Radius, image heights, at which gravity alone along the lens
  /// moves a drop, in [0.005, 0.2]; smaller on a hydrophobic coating.
  double slideRadius{0.03};
};

/// \brief What the world does to the lens, from the system, every step.
struct Environment
{
  /// \brief Gravity along the lens, to the image's right, in g. The length
  /// of (gravityX, gravityY) is at most 1.
  double gravityX{0.0};

  /// \brief Gravity along the lens, down the image, in g.
  double gravityY{1.0};

  /// \brief Apparent air along the lens, to the image's right, m/s.
  double airX{0.0};

  /// \brief Apparent air along the lens, down the image, m/s.
  double airY{0.0};

  /// \brief Rain flux factor, 1 for a lens facing up in still air; >= 0.
  double flux{1.0};
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
/// each is clamped to the range its member documents.
/// \param[in] _params The parameters.
/// \return The safe parameters.
Params Sanitize(const Params &_params);

/// \brief Environment made safe: a non-finite field takes its default;
/// gravity longer than 1 is scaled to 1; air is clamped to [-100, 100] m/s
/// per axis and flux to [0, 10].
/// \param[in] _env The environment.
/// \return The safe environment.
Environment Sanitize(const Environment &_env);

/// \brief How many drops land per second.
/// \param[in] _params The parameters, sanitized first.
/// \param[in] _env The environment, sanitized first.
/// \return rainRate * exposure * kDropsPerMmHour * (flux + kSplashFloor).
double SpawnRate(const Params &_params, const Environment &_env);

/// \brief How many drops land per second in the default environment.
/// \param[in] _params The parameters, sanitized first.
/// \return SpawnRate(_params, Environment()).
double SpawnRate(const Params &_params);

/// \brief How hard gravity and the air push a drop along the lens, in units
/// of the push that just moves it: it slides above 1.
/// \param[in] _drop The drop.
/// \param[in] _params The parameters, sanitized.
/// \param[in] _env The environment, sanitized.
/// \param[out] _dirX Unit direction of the push, to the image's right; 0
/// without a push.
/// \param[out] _dirY Unit direction of the push, down the image.
/// \return The push.
double Push(const Drop &_drop, const Params &_params, const Environment &_env,
            double &_dirX, double &_dirY);

/// \brief The water in a drop, up to a constant: a cap of fixed contact
/// angle holds radius^3.
/// \param[in] _drop The drop.
/// \return radius^3.
double Volume(const Drop &_drop);

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

  /// \brief Change what the world does to the lens. The next arrival is
  /// drawn again only if the spawn rate changed.
  /// \param[in] _env The environment; sanitized.
  public: void SetEnvironment(const Environment &_env);

  /// \brief What the world does to the lens.
  /// \return The sanitized environment.
  public: const Environment &Env() const;

  /// \brief Age and dry, slide, spawn and merge drops over a step.
  /// \param[in] _dt Seconds; capped at kMaxStep. Zero, negative or not
  /// finite does nothing, as on a paused world.
  public: void Step(double _dt);

  /// \brief Put a drop on the lens, if there is room. Its age, velocity
  /// and distance slid start at zero and it is pinned.
  /// \param[in] _drop The drop.
  public: void Add(const Drop &_drop);

  /// \brief Wipe the lens, trails included. Time, the schedule, the
  /// generator and the count of drops spawned stay.
  public: void Clear();

  /// \brief Start again: seed the generator, wipe the lens, time and count
  /// back to zero. The parameters and the environment stay.
  /// \param[in] _seed Seed of the generator.
  public: void Reset(std::uint32_t _seed);

  /// \brief The drops on the lens.
  /// \return The drops, oldest first.
  public: const std::vector<Drop> &List() const;

  /// \brief The wet trails on the lens.
  /// \return The trails, oldest first.
  public: const std::vector<Trail> &Trails() const;

  /// \brief Seconds stepped since construction or the last Reset.
  /// \return The time.
  public: double Time() const;

  /// \brief How many drops arrived, kept or not for a full lens.
  /// \return The count.
  public: std::uint64_t Spawned() const;

  /// \brief How many drops slide.
  /// \return The count.
  public: std::size_t Sliding() const;

  /// \brief A counter that changes whenever what the lens shows may have
  /// changed: a step that changed something, Add, Clear, Reset, SetParams.
  /// \return The counter.
  public: std::uint64_t Generation() const;

  /// \brief Write the drops for a renderer: x, y, shown radius and 0 per
  /// drop, as floats.
  /// \param[out] _out At least kDropFloats * _maxDrops floats. Null writes
  /// nothing.
  /// \param[in] _maxDrops The most drops to write.
  /// \return How many drops were written: the least of the drops on the
  /// lens, _maxDrops and kMaxDrops; 0 for a null _out.
  public: std::size_t Pack(float *_out, std::size_t _maxDrops) const;

  /// \brief Draw the next arrival time if none is scheduled.
  /// \param[in] _from Time to draw from.
  /// \param[in] _rate Arrivals per second, positive.
  private: void Schedule(double _from, double _rate);

  /// \brief What drives the drops, sanitized.
  private: Params params;

  /// \brief What the world does to the lens, sanitized.
  private: Environment env;

  /// \brief Every draw comes from here.
  private: Generator rng;

  /// \brief The drops on the lens, oldest first.
  private: std::vector<Drop> drops;

  /// \brief The wet trails, oldest first.
  private: std::vector<Trail> trails;

  /// \brief Seconds stepped.
  private: double time{0.0};

  /// \brief Time of the next arrival; +infinity when none is scheduled.
  private: double nextArrival;

  /// \brief Drops that arrived.
  private: std::uint64_t spawned{0};

  /// \brief Changes whenever what the lens shows may have changed.
  private: std::uint64_t generation{0};

  /// \brief True if Add put a drop on the lens since the last step, so the
  /// next merge checks every drop.
  private: bool added{false};
};
}  // namespace gz::sim::lens_drops

#endif  // GZ_SIM_LENS_DROPS_DROPS_HH_
