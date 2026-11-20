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
  const Params defaults;
  const auto fix = [](double _value, double _fallback, double _lo, double _hi)
  {
    return std::clamp(std::isfinite(_value) ? _value : _fallback, _lo, _hi);
  };
  Params out;
  out.rainRate = fix(_params.rainRate, defaults.rainRate, 0.0, 1000.0);
  out.exposure = fix(_params.exposure, defaults.exposure, 0.0, 1.0);
  out.focus = fix(_params.focus, defaults.focus, 0.0, 1.0);
  out.dryTime = fix(_params.dryTime, defaults.dryTime, 0.1, 3600.0);
  out.aspect = fix(_params.aspect, defaults.aspect, 0.1, 10.0);
  return out;
}

//////////////////////////////////////////////////
double SpawnRate(const Params &_params)
{
  const Params p = Sanitize(_params);
  return p.rainRate * p.exposure * kDropsPerMmHour;
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
  const double before = SpawnRate(this->params);
  this->params = Sanitize(_params);
  if (SpawnRate(this->params) != before)
    this->nextArrival = kNever;
}

//////////////////////////////////////////////////
const Params &Drops::Parameters() const
{
  return this->params;
}

//////////////////////////////////////////////////
void Drops::Step(double _dt)
{
  if (!std::isfinite(_dt) || _dt <= 0.0)
    return;

  const double dt = std::min(_dt, kMaxStep);
  const double start = this->time;
  this->time += dt;

  // Age, and dry out.
  for (Drop &d : this->drops)
    d.age += dt;
  const double dryTime = this->params.dryTime;
  this->drops.erase(std::remove_if(this->drops.begin(), this->drops.end(),
      [dryTime](const Drop &_d) { return _d.age >= dryTime; }),
      this->drops.end());

  // Big drops slide down, faster the bigger, and leave at the bottom.
  for (Drop &d : this->drops)
  {
    if (d.radius >= kSlideRadius)
      d.y += kSlideSpeed * (d.radius / kSlideRadius) * dt;
  }
  this->drops.erase(std::remove_if(this->drops.begin(), this->drops.end(),
      [](const Drop &_d) { return _d.y - _d.radius > 1.0; }),
      this->drops.end());

  // New drops, a Poisson process at the spawn rate. Before the merge, so a
  // drop that lands on another joins it at once.
  const double rate = SpawnRate(this->params);
  if (rate <= 0.0)
    this->nextArrival = kNever;
  else if (std::isinf(this->nextArrival))
    this->nextArrival = start - std::log(Uniform(this->rng)) / rate;
  while (rate > 0.0 && this->nextArrival <= this->time)
  {
    // Always three draws, kept or not, so a full lens does not change what
    // comes after.
    const double x = Uniform(this->rng);
    const double y = Uniform(this->rng);
    const double u = Uniform(this->rng);
    ++this->spawned;
    if (this->drops.size() < kMaxDrops)
    {
      this->drops.push_back(
          {x, y, kMinRadius + (kMaxRadius - kMinRadius) * u * u, 0.0});
    }
    this->nextArrival += -std::log(Uniform(this->rng)) / rate;
  }

  // Drops that touch merge into the bigger, keeping the water.
  bool merged = true;
  while (merged)
  {
    merged = false;
    for (std::size_t i = 0; i < this->drops.size() && !merged; ++i)
    {
      for (std::size_t j = i + 1; j < this->drops.size() && !merged; ++j)
      {
        const Drop &a = this->drops[i];
        const Drop &c = this->drops[j];
        const double dx = (a.x - c.x) * this->params.aspect;
        const double dy = a.y - c.y;
        if (std::sqrt(dx * dx + dy * dy) >= std::max(a.radius, c.radius))
          continue;
        const std::size_t bi = c.radius > a.radius ? j : i;
        const std::size_t si = bi == i ? j : i;
        Drop &b = this->drops[bi];
        const Drop &s = this->drops[si];
        const double ab = b.radius * b.radius;
        const double as = s.radius * s.radius;
        b.x = (b.x * ab + s.x * as) / (ab + as);
        b.y = (b.y * ab + s.y * as) / (ab + as);
        b.radius = std::min(std::sqrt(ab + as), kMaxMergedRadius);
        b.age = std::min(b.age, s.age);
        this->drops.erase(this->drops.begin() +
            static_cast<std::ptrdiff_t>(si));
        merged = true;
      }
    }
  }
}

//////////////////////////////////////////////////
void Drops::Clear()
{
  this->drops.clear();
}

//////////////////////////////////////////////////
void Drops::Reset(std::uint32_t _seed)
{
  this->rng.seed(_seed);
  this->drops.clear();
  this->time = 0.0;
  this->nextArrival = kNever;
  this->spawned = 0;
}

//////////////////////////////////////////////////
const std::vector<Drop> &Drops::List() const
{
  return this->drops;
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
