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
  out.contactAngle = Fix(_params.contactAngle, d.contactAngle, 0.17, 2.6);
  out.slideRadius = Fix(_params.slideRadius, d.slideRadius, 0.005, 0.2);
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
  const double s = _drop.radius / _params.slideRadius;
  const double a = std::hypot(_env.airX, _env.airY);
  const double air = (a / kAirPush) * (a / kAirPush) * s;
  const double px = _env.gravityX * s * s + (a > 0.0 ? _env.airX / a : 0.0) * air;
  const double py = _env.gravityY * s * s + (a > 0.0 ? _env.airY / a : 0.0) * air;
  const double p = std::hypot(px, py);
  _dirX = p > 0.0 ? px / p : 0.0;
  _dirY = p > 0.0 ? py / p : 0.0;
  return p;
}

//////////////////////////////////////////////////
double Volume(const Drop &_drop)
{
  return _drop.radius * _drop.radius * _drop.radius;
}

//////////////////////////////////////////////////
double ShownRadius(const Drop &_drop, double _dryTime)
{
  if (!(_dryTime > 0.0))
    return 0.0;
  return _drop.radius * std::sqrt(std::max(0.0, 1.0 - _drop.age / _dryTime));
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

  const double dt = std::min(_dt, kMaxStep);
  const double start = this->time;
  this->time += dt;
  bool changed = !this->drops.empty() || !this->trails.empty();
  const double aspect = this->params.aspect;

  // 3. Age, and dry out.
  for (Drop &d : this->drops)
    d.age += dt;
  const double dryTime = this->params.dryTime;
  this->drops.erase(std::remove_if(this->drops.begin(), this->drops.end(),
      [dryTime](const Drop &_d) { return _d.age >= dryTime; }),
      this->drops.end());

  // 4. Trails dry in half the time of a drop.
  for (Trail &t : this->trails)
    t.age += dt;
  const double filmLife = 0.5 * dryTime;
  this->trails.erase(std::remove_if(this->trails.begin(), this->trails.end(),
      [filmLife](const Trail &_t) { return _t.age >= filmLife; }),
      this->trails.end());

  // 5. Slide: drops the push moves meander, shed beads and leave trails.
  std::vector<Drop> beads;
  for (Drop &d : this->drops)
  {
    double dirX{0.0};
    double dirY{0.0};
    const double p = Push(d, this->params, this->env, dirX, dirY);
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
    const double speed = kSlideSpeed * std::sqrt(p - 1.0);
    const double turn = kMeander * dt * (Uniform(this->rng) - 0.5) * 2.0;
    const double c = std::cos(turn);
    const double s = std::sin(turn);
    const double ux = dirX * c - dirY * s;
    const double uy = dirX * s + dirY * c;
    d.vx = ux * speed;
    d.vy = uy * speed;
    d.x += d.vx * dt / aspect;
    d.y += d.vy * dt;
    d.travelled += speed * dt;
    if (d.travelled >= kShedDistance && d.radius > 3.0 * kMinRadius)
    {
      // The bead stays at the drop's tail, just clear of it, so it does
      // not merge straight back.
      Drop bead;
      bead.radius = kBeadFraction * d.radius;
      // Water off the same drop: it dries with it.
      bead.age = d.age;
      const double back = 1.05 * (d.radius + bead.radius);
      bead.x = d.x - ux * back / aspect;
      bead.y = d.y - uy * back;
      beads.push_back(bead);
      d.radius = std::cbrt(std::max(0.0, Volume(d) - Volume(bead)));
      d.travelled = 0.0;

      if (this->trails.size() >= kMaxTrails)
        this->trails.erase(this->trails.begin());
      Trail t;
      t.x0 = d.markX;
      t.y0 = d.markY;
      t.x1 = d.x;
      t.y1 = d.y;
      t.halfWidth = 0.35 * d.radius;
      this->trails.push_back(t);
      d.markX = d.x;
      d.markY = d.y;
    }
  }
  // Out of the image.
  this->drops.erase(std::remove_if(this->drops.begin(), this->drops.end(),
      [aspect](const Drop &_d)
      {
        return _d.x * aspect < -_d.radius ||
               _d.x * aspect > aspect + _d.radius ||
               _d.y < -_d.radius || _d.y > 1.0 + _d.radius;
      }),
      this->drops.end());
  // Only drops that moved, and those appended below, can newly touch
  // another: the rest were apart after the last step and have not moved.
  std::vector<char> dirty(this->drops.size(), 0);
  for (std::size_t i = 0; i < this->drops.size(); ++i)
    dirty[i] = this->drops[i].sliding || this->added ? 1 : 0;
  for (const Drop &b : beads)
  {
    if (this->drops.size() < kMaxDrops)
    {
      this->drops.push_back(b);
      dirty.push_back(1);
    }
  }

  // 6. New drops, a Poisson process at the spawn rate. Before the merge, so
  // a drop that lands on another joins it at once.
  const double rate = SpawnRate(this->params, this->env);
  if (rate <= 0.0)
    this->nextArrival = kNever;
  else
    this->Schedule(start, rate);
  while (rate > 0.0 && this->nextArrival <= this->time)
  {
    // Always six draws, kept or not, so a full lens does not change what
    // comes after.
    const double x = Uniform(this->rng);
    const double y = Uniform(this->rng);
    const double u = Uniform(this->rng);
    const double w = Uniform(this->rng);
    const double a = Uniform(this->rng);
    const double b = Uniform(this->rng);
    ++this->spawned;
    changed = true;
    if (this->drops.size() < kMaxDrops)
    {
      Drop d;
      d.x = x;
      d.y = y;
      d.radius = kMinRadius + (kMaxRadius - kMinRadius) * u * u * u;
      d.wobble = static_cast<float>(0.15 * w);
      d.phaseA = static_cast<float>(kTwoPi * a);
      d.phaseB = static_cast<float>(kTwoPi * b);
      this->drops.push_back(d);
      dirty.push_back(1);
    }
    this->nextArrival += -std::log(Uniform(this->rng)) / rate;
  }

  // 7. Drops that touch merge into the bigger, keeping the water. A drop
  // that grows by merging is checked again.
  bool again = true;
  while (again)
  {
    again = false;
    for (std::size_t i = 0; i < this->drops.size() && !again; ++i)
    {
      if (!dirty[i])
        continue;
      for (std::size_t j = 0; j < this->drops.size(); ++j)
      {
        if (j == i)
          continue;
        const Drop &a = this->drops[i];
        const Drop &c = this->drops[j];
        const double dx = (a.x - c.x) * aspect;
        const double dy = a.y - c.y;
        if (std::sqrt(dx * dx + dy * dy) >= std::max(a.radius, c.radius))
          continue;
        const std::size_t lo = std::min(i, j);
        const std::size_t hi = std::max(i, j);
        const std::size_t bi =
            this->drops[hi].radius > this->drops[lo].radius ? hi : lo;
        const std::size_t si = bi == lo ? hi : lo;
        Drop &b = this->drops[bi];
        const Drop &s = this->drops[si];
        const double vb = Volume(b);
        const double vs = Volume(s);
        b.x = (b.x * vb + s.x * vs) / (vb + vs);
        b.y = (b.y * vb + s.y * vs) / (vb + vs);
        b.radius = std::min(std::cbrt(vb + vs), kMaxMergedRadius);
        b.age = std::min(b.age, s.age);
        b.sliding = b.sliding || s.sliding;
        b.wobble = std::max(b.wobble, s.wobble);
        dirty[bi] = 1;
        this->drops.erase(this->drops.begin() +
            static_cast<std::ptrdiff_t>(si));
        dirty.erase(dirty.begin() + static_cast<std::ptrdiff_t>(si));
        changed = true;
        again = true;
        break;
      }
      if (!again)
        dirty[i] = 0;
    }
  }
  this->added = false;

  // 8. The map must be drawn again.
  if (changed)
    ++this->generation;
}

//////////////////////////////////////////////////
void Drops::Add(const Drop &_drop)
{
  if (this->drops.size() >= kMaxDrops)
    return;
  Drop d = _drop;
  d.age = 0.0;
  d.vx = 0.0;
  d.vy = 0.0;
  d.travelled = 0.0;
  d.sliding = false;
  d.markX = d.x;
  d.markY = d.y;
  this->drops.push_back(d);
  this->added = true;
  ++this->generation;
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

//////////////////////////////////////////////////
std::size_t Drops::Pack(float *_out, std::size_t _maxDrops) const
{
  if (_out == nullptr)
    return 0;
  const std::size_t n = std::min({this->drops.size(), _maxDrops, kMaxDrops});
  for (std::size_t i = 0; i < n; ++i)
  {
    const Drop &d = this->drops[i];
    _out[kDropFloats * i + 0] = static_cast<float>(d.x);
    _out[kDropFloats * i + 1] = static_cast<float>(d.y);
    _out[kDropFloats * i + 2] =
        static_cast<float>(ShownRadius(d, this->params.dryTime));
    _out[kDropFloats * i + 3] = 0.0f;
  }
  return n;
}
}  // namespace gz::sim::lens_drops
