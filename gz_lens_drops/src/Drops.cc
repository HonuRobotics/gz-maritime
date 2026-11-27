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
#include "gz/sim/lens_drops/Drops.hh"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gz::sim::lens_drops
{
namespace
{
/// \brief No arrival scheduled.
constexpr double kNever = std::numeric_limits<double>::infinity();

/// \brief Two pi.
constexpr double kTwoPi = 6.283185307179586;

/// \brief A value made safe: its fallback if not finite, then clamped.
/// \param[in] _value The value.
/// \param[in] _fallback Used when _value is not finite.
/// \param[in] _lo Lower bound.
/// \param[in] _hi Upper bound.
/// \return The safe value.
double Fix(double _value, double _fallback, double _lo, double _hi)
{
  return std::clamp(std::isfinite(_value) ? _value : _fallback, _lo, _hi);
}
}  // namespace

//////////////////////////////////////////////////
std::uint32_t Hash(const std::string &_text)
{
  std::uint32_t h = 2166136261u;
  for (const char c : _text)
  {
    h ^= static_cast<unsigned char>(c);
    h *= 16777619u;
  }
  return h;
}

//////////////////////////////////////////////////
double Uniform(Generator &_rng)
{
  return (static_cast<double>(_rng()) + 0.5) / 4294967296.0;
}

//////////////////////////////////////////////////
Params Sanitize(const Params &_params)
{
  const Params d;
  Params out;
  out.rainRate = Fix(_params.rainRate, d.rainRate, 0.0, 1000.0);
  out.exposure = Fix(_params.exposure, d.exposure, 0.0, 1.0);
  out.focus = Fix(_params.focus, d.focus, 0.0, 1.0);
  out.dryTime = Fix(_params.dryTime, d.dryTime, 0.1, 3600.0);
  out.aspect = Fix(_params.aspect, d.aspect, 0.1, 10.0);
  out.contactAngle = Fix(_params.contactAngle, d.contactAngle, 0.17,
                         kMaxContactAngle);
  out.slideRadius = Fix(_params.slideRadius, d.slideRadius, 0.005, 0.2);
  out.minRadius = Fix(_params.minRadius, d.minRadius, kMinRadius, kMaxRadius);
  return out;
}

//////////////////////////////////////////////////
Environment Sanitize(const Environment &_env)
{
  const Environment d;
  Environment out;
  out.gravityX = std::isfinite(_env.gravityX) ? _env.gravityX : d.gravityX;
  out.gravityY = std::isfinite(_env.gravityY) ? _env.gravityY : d.gravityY;
  const double g = std::hypot(out.gravityX, out.gravityY);
  if (g > 1.0)
  {
    out.gravityX /= g;
    out.gravityY /= g;
  }
  out.airX = Fix(_env.airX, d.airX, -100.0, 100.0);
  out.airY = Fix(_env.airY, d.airY, -100.0, 100.0);
  out.flux = Fix(_env.flux, d.flux, 0.0, 10.0);
  return out;
}

//////////////////////////////////////////////////
double SpawnRate(const Params &_params, const Environment &_env)
{
  const Params p = Sanitize(_params);
  const Environment e = Sanitize(_env);
  return p.rainRate * p.exposure * kDropsPerMmHour * (e.flux + kSplashFloor);
}

//////////////////////////////////////////////////
double SpawnRate(const Params &_params)
{
  return SpawnRate(_params, Environment());
}

//////////////////////////////////////////////////
double Push(const Drop &_drop, const Params &_params, const Environment &_env,
            double &_dirX, double &_dirY)
{
  // Gravity on the water over the contact line that holds it: a fresh
  // drop's (r / r_slide)^2; the air on what stands up from the glass
  const double s = _drop.radius / _params.slideRadius;
  const double f = std::clamp(_drop.fill, 0.0, 1.0);
  const double a = std::hypot(_env.airX, _env.airY);
  const double air = (a / kAirPush) * (a / kAirPush) * s * f;
  const double g = s * s * f;
  const double px = _env.gravityX * g + (a > 0.0 ? _env.airX / a : 0.0) * air;
  const double py = _env.gravityY * g + (a > 0.0 ? _env.airY / a : 0.0) * air;
  const double p = std::hypot(px, py);
  _dirX = p > 0.0 ? px / p : 0.0;
  _dirY = p > 0.0 ? py / p : 0.0;
  return p;
}

//////////////////////////////////////////////////
double Volume(const Drop &_drop)
{
  return _drop.fill * _drop.radius * _drop.radius * _drop.radius;
}

//////////////////////////////////////////////////
Drops::Drops(std::uint32_t _seed, const Params &_params)
  : params(Sanitize(_params)), rng(_seed), nextArrival(kNever)
{
  this->drops.reserve(kMaxDrops);
}

//////////////////////////////////////////////////
void Drops::SetParams(const Params &_params)
{
  const double before = SpawnRate(this->params, this->env);
  this->params = Sanitize(_params);
  if (SpawnRate(this->params, this->env) != before)
    this->nextArrival = kNever;
  ++this->generation;
}

//////////////////////////////////////////////////
const Params &Drops::Parameters() const
{
  return this->params;
}

//////////////////////////////////////////////////
void Drops::SetEnvironment(const Environment &_env)
{
  const double before = SpawnRate(this->params, this->env);
  this->env = Sanitize(_env);
  if (SpawnRate(this->params, this->env) != before)
    this->nextArrival = kNever;
}

//////////////////////////////////////////////////
const Environment &Drops::Env() const
{
  return this->env;
}

//////////////////////////////////////////////////
void Drops::Schedule(double _from, double _rate)
{
  if (std::isinf(this->nextArrival))
    this->nextArrival = _from - std::log(Uniform(this->rng)) / _rate;
}

//////////////////////////////////////////////////
void Drops::Step(double _dt)
{
  if (!std::isfinite(_dt) || _dt <= 0.0)
    return;
  // Whole periods only: the same total time gives the same steps. The
  // tolerance takes up the rounding of many short steps adding up.
  this->pending += std::min(_dt, kMaxStep);
  while (this->pending >= kStepPeriod * (1.0 - 1e-9))
  {
    this->pending = std::max(0.0, this->pending - kStepPeriod);
    this->StepOnce(kStepPeriod);
  }
}

//////////////////////////////////////////////////
void Drops::StepOnce(double _dt)
{
  const double dt = _dt;
  const double start = this->time;
  this->time += dt;
  bool changed = false;
  const double aspect = this->params.aspect;
  const double dryTime = this->params.dryTime;

  // Dry: each drop loses the water of its fresh contact circle over a dry
  // time, and goes when it has none left. Its contact circle stays.
  const std::size_t before = this->drops.size();
  for (Drop &d : this->drops)
    d.fill -= dt / dryTime;
  this->drops.erase(std::remove_if(this->drops.begin(), this->drops.end(),
      [](const Drop &_d) { return _d.fill <= 0.0; }),
      this->drops.end());
  changed = changed || this->drops.size() != before;
  const auto level = static_cast<std::int64_t>(
      std::floor(this->time * kDryLevels / dryTime));
  if (level != this->dryLevel && !this->drops.empty())
    changed = true;
  this->dryLevel = level;

  // Trails dry in a fraction of the time of a drop.
  for (Trail &t : this->trails)
    t.age += dt;
  const double trailLife = kTrailLife * dryTime;
  const std::size_t trailsBefore = this->trails.size();
  this->trails.erase(std::remove_if(this->trails.begin(), this->trails.end(),
      [trailLife](const Trail &_t) { return _t.age >= trailLife; }),
      this->trails.end());
  changed = changed || this->trails.size() != trailsBefore;

  // Slide: drops the push moves shed beads and leave trails.
  std::vector<Drop> beads;
  for (Drop &d : this->drops)
  {
    double ux{0.0};
    double uy{0.0};
    const double p = Push(d, this->params, this->env, ux, uy);
    if (p <= 1.0)
    {
      d.sliding = false;
      d.vx = 0.0;
      d.vy = 0.0;
      continue;
    }
    if (!d.sliding)
    {
      d.sliding = true;
      d.markX = d.x;
      d.markY = d.y;
    }
    changed = true;
    const double speed = kSlideSpeed * std::sqrt(p - 1.0);
    d.vx = ux * speed;
    d.vy = uy * speed;
    d.x += d.vx * dt / aspect;
    d.y += d.vy * dt;
    d.travelled += speed * dt;
    // Every shed distance slid leaves a bead, however far the step went:
    // the distance past it carries to the next
    while (d.travelled >= kShedDistance && d.radius > 3.0 * kMinRadius)
    {
      // Beads come at uneven spacing and in uneven sizes, as on glass: two
      // draws per bead, in this order.
      const double size = 0.5 + 0.5 * Uniform(this->rng);
      const double spacing = Uniform(this->rng) - 0.5;
      d.travelled -= kShedDistance;
      Drop bead;
      bead.radius = kBeadFraction * size * d.radius;
      // Water off the same drop, as full as it is: it dries with it
      bead.fill = d.fill;
      // Left where the drop was when it slid past this bead, at its tail
      const double back = kBeadGap * (d.radius + bead.radius) + d.travelled;
      bead.x = d.x - ux * back / aspect;
      bead.y = d.y - uy * back;
      bead.markX = bead.x;
      bead.markY = bead.y;
      beads.push_back(bead);
      // The drop keeps the rest of its water, as full as before: a sliding
      // drop's contact circle follows its water
      d.radius = std::cbrt(std::max(0.0, d.radius * d.radius * d.radius -
          bead.radius * bead.radius * bead.radius));
      // The next bead after 0.5 to 1.5 shed distances
      d.travelled += spacing * kShedDistance;

      if (this->trails.size() >= kMaxTrails)
        this->trails.erase(this->trails.begin());
      Trail t;
      t.x0 = d.markX;
      t.y0 = d.markY;
      t.x1 = bead.x;
      t.y1 = bead.y;
      t.halfWidth = kTrailWidth * d.radius;
      this->trails.push_back(t);
      d.markX = bead.x;
      d.markY = bead.y;
    }
  }
  // Out of the image.
  const std::size_t inView = this->drops.size();
  this->drops.erase(std::remove_if(this->drops.begin(), this->drops.end(),
      [aspect](const Drop &_d)
      {
        return _d.x * aspect < -_d.radius ||
               _d.x * aspect > aspect + _d.radius ||
               _d.y < -_d.radius || _d.y > 1.0 + _d.radius;
      }),
      this->drops.end());
  changed = changed || this->drops.size() != inView;
  // Only drops that moved, and those appended below, can newly touch
  // another: the rest were apart after the last step and have not moved.
  std::vector<char> check(this->drops.size(), 0);
  for (std::size_t i = 0; i < this->drops.size(); ++i)
    check[i] = this->drops[i].sliding || this->added ? 1 : 0;
  for (const Drop &b : beads)
  {
    if (this->drops.size() < kMaxDrops)
    {
      this->drops.push_back(b);
      check.push_back(1);
    }
  }

  // New drops, a Poisson process at the spawn rate. Before the merge, so a
  // drop that lands on another joins it at once.
  const double rate = SpawnRate(this->params, this->env);
  if (rate <= 0.0)
    this->nextArrival = kNever;
  else
    this->Schedule(start, rate);
  while (rate > 0.0 && this->nextArrival <= this->time)
  {
    // Always six draws, kept or not, so a full lens or a drop too small to
    // keep does not change what comes after.
    const double x = Uniform(this->rng);
    const double y = Uniform(this->rng);
    const double u = Uniform(this->rng);
    const double w = Uniform(this->rng);
    const double a = Uniform(this->rng);
    const double b = Uniform(this->rng);
    ++this->spawned;
    const double radius = kMinRadius + (kMaxRadius - kMinRadius) * u * u * u;
    if (this->drops.size() < kMaxDrops && radius >= this->params.minRadius)
    {
      Drop d;
      d.x = x;
      d.y = y;
      d.radius = radius;
      d.wobble = static_cast<float>(kMaxWobble * w);
      d.phaseA = static_cast<float>(kTwoPi * a);
      d.phaseB = static_cast<float>(kTwoPi * b);
      d.markX = x;
      d.markY = y;
      this->drops.push_back(d);
      check.push_back(1);
      changed = true;
    }
    this->nextArrival += -std::log(Uniform(this->rng)) / rate;
  }

  changed = this->Merge(check) || changed;
  this->added = false;

  // The map must be drawn again.
  if (changed)
    ++this->generation;
}

//////////////////////////////////////////////////
bool Drops::Merge(std::vector<char> &_check)
{
  // Drops that touch merge into the bigger, keeping the water. A drop that
  // grows by merging may touch more, so it is checked again; the merged
  // drop is left in place and the smaller marked gone, and the lens is
  // compacted once at the end, oldest first as before.
  const double aspect = this->params.aspect;
  std::vector<char> gone(this->drops.size(), 0);
  std::vector<std::size_t> queue;
  for (std::size_t i = 0; i < this->drops.size(); ++i)
  {
    if (_check[i])
      queue.push_back(i);
  }
  bool merged = false;
  while (!queue.empty())
  {
    const std::size_t i = queue.back();
    queue.pop_back();
    if (gone[i])
      continue;
    for (std::size_t j = 0; j < this->drops.size(); ++j)
    {
      if (j == i || gone[j])
        continue;
      const Drop &a = this->drops[i];
      const Drop &c = this->drops[j];
      const double dx = (a.x - c.x) * aspect;
      const double dy = a.y - c.y;
      const double reach = std::max(a.radius, c.radius);
      if (dx * dx + dy * dy >= reach * reach)
        continue;
      // The bigger keeps its place in the list; on a tie, the older
      const std::size_t lo = std::min(i, j);
      const std::size_t hi = std::max(i, j);
      const std::size_t bi =
          this->drops[hi].radius > this->drops[lo].radius ? hi : lo;
      const std::size_t si = bi == lo ? hi : lo;
      Drop &b = this->drops[bi];
      const Drop &s = this->drops[si];
      const double vb = Volume(b);
      const double vs = Volume(s);
      const double v = vb + vs;
      if (v > 0.0)
      {
        b.x = (b.x * vb + s.x * vs) / v;
        b.y = (b.y * vb + s.y * vs) / v;
      }
      // The water spreads to the bigger contact circle, or wider if it
      // holds more than that circle does fresh
      b.radius = std::min(std::max(b.radius, std::cbrt(v)), kMaxMergedRadius);
      b.fill = std::min(1.0, v / (b.radius * b.radius * b.radius));
      b.wobble = std::max(b.wobble, s.wobble);
      // Whether it slides is worked out on the next step, from its new
      // water; a trail starts where it is now
      b.markX = b.x;
      b.markY = b.y;
      gone[si] = 1;
      merged = true;
      queue.push_back(bi);
      if (si == i)
        break;
      // i grew: check it against every drop again
      j = static_cast<std::size_t>(-1);
    }
  }
  if (merged)
  {
    std::size_t k = 0;
    for (std::size_t i = 0; i < this->drops.size(); ++i)
    {
      if (!gone[i])
        this->drops[k++] = this->drops[i];
    }
    this->drops.resize(k);
  }
  return merged;
}

//////////////////////////////////////////////////
bool Drops::Add(double _x, double _y, double _radius)
{
  if (!std::isfinite(_x) || !std::isfinite(_y) || !std::isfinite(_radius) ||
      _x < 0.0 || _x > 1.0 || _y < 0.0 || _y > 1.0 || _radius <= 0.0 ||
      _radius > kMaxMergedRadius || this->drops.size() >= kMaxDrops)
  {
    return false;
  }
  Drop d;
  d.x = _x;
  d.y = _y;
  d.radius = _radius;
  d.markX = _x;
  d.markY = _y;
  this->drops.push_back(d);
  this->added = true;
  ++this->generation;
  return true;
}

//////////////////////////////////////////////////
void Drops::Clear()
{
  this->drops.clear();
  this->trails.clear();
  ++this->generation;
}

//////////////////////////////////////////////////
void Drops::Reset(std::uint32_t _seed)
{
  this->rng.seed(_seed);
  this->drops.clear();
  this->trails.clear();
  this->time = 0.0;
  this->pending = 0.0;
  this->dryLevel = 0;
  this->nextArrival = kNever;
  this->spawned = 0;
  ++this->generation;
}

//////////////////////////////////////////////////
const std::vector<Drop> &Drops::List() const
{
  return this->drops;
}

//////////////////////////////////////////////////
const std::vector<Trail> &Drops::Trails() const
{
  return this->trails;
}

//////////////////////////////////////////////////
double Drops::Time() const
{
  return this->time;
}

//////////////////////////////////////////////////
std::uint64_t Drops::Spawned() const
{
  return this->spawned;
}

//////////////////////////////////////////////////
std::size_t Drops::Sliding() const
{
  return static_cast<std::size_t>(std::count_if(this->drops.begin(),
      this->drops.end(), [](const Drop &_d) { return _d.sliding; }));
}

//////////////////////////////////////////////////
std::uint64_t Drops::Generation() const
{
  return this->generation;
}

}  // namespace gz::sim::lens_drops
