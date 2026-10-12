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
#include "gz/sim/maritime_gui/Flow.hh"

#include <algorithm>
#include <cmath>
#include <utility>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief How bright a trail's tail is, against its head.
  constexpr double kTailBrightness{0.15};

  /// \brief The outer band of the region's disc where the trails fade out,
  /// as a fraction of its radius, so the area has no hard border.
  constexpr double kEdgeMargin{0.4};

  /// \brief The darkest bin's brightness.
  constexpr float kDarkest{0.15f};

  /// \brief A trail segment dimmer than this is not drawn, so the trails
  /// fade to nothing rather than to the darkest bin.
  constexpr double kInvisible{0.08};

  /// \brief A colour scaled towards black, alpha kept.
  /// \param[in] _c The colour.
  /// \param[in] _k The scale, 0 to 1.
  /// \return The scaled colour.
  math::Color Dim(const math::Color &_c, float _k)
  {
    return math::Color(_c.R() * _k, _c.G() * _k, _c.B() * _k, _c.A());
  }

  /// \brief Wrap degrees into (-180, 180].
  /// \param[in] _deg Degrees.
  /// \return The same angle.
  double Wrap180(double _deg)
  {
    double d = std::fmod(_deg + 180.0, 360.0);
    if (d < 0.0)
      d += 360.0;
    return d - 180.0;
  }
}

//////////////////////////////////////////////////
Colormap::Colormap(std::vector<math::Color> _stops, double _maxSpeed)
  : stops(std::move(_stops)), maxSpeed(_maxSpeed > 0.0 ? _maxSpeed : 1.0)
{
  if (this->stops.empty())
    this->stops.push_back(math::Color::White);
}

//////////////////////////////////////////////////
math::Color Colormap::At(double _speed) const
{
  if (this->stops.size() == 1u || !(_speed > 0.0))
    return this->stops.front();
  const double t = std::min(_speed / this->maxSpeed, 1.0) *
      static_cast<double>(this->stops.size() - 1);
  const std::size_t i =
      std::min(static_cast<std::size_t>(t), this->stops.size() - 2);
  const float f = static_cast<float>(t - static_cast<double>(i));
  const math::Color &a = this->stops[i];
  const math::Color &b = this->stops[i + 1];
  return math::Color(a.R() + (b.R() - a.R()) * f,
                     a.G() + (b.G() - a.G()) * f,
                     a.B() + (b.B() - a.B()) * f,
                     a.A() + (b.A() - a.A()) * f);
}

//////////////////////////////////////////////////
double Colormap::MaxSpeed() const
{
  return this->maxSpeed;
}

//////////////////////////////////////////////////
void Colormap::SetMaxSpeed(double _maxSpeed)
{
  if (_maxSpeed > 0.0 && std::isfinite(_maxSpeed))
    this->maxSpeed = _maxSpeed;
}

//////////////////////////////////////////////////
const std::vector<math::Color> &Colormap::Stops() const
{
  return this->stops;
}

//////////////////////////////////////////////////
Colormap Colormap::Water(double _maxSpeed)
{
  return Colormap({{0.02f, 0.12f, 0.45f, 1.0f},
                   {0.05f, 0.40f, 0.85f, 1.0f},
                   {0.10f, 0.80f, 0.95f, 1.0f},
                   {0.60f, 0.97f, 1.00f, 1.0f},
                   {1.00f, 1.00f, 1.00f, 1.0f}}, _maxSpeed);
}

//////////////////////////////////////////////////
Colormap Colormap::Air(double _maxSpeed)
{
  return Colormap({{0.20f, 0.05f, 0.45f, 1.0f},
                   {0.55f, 0.10f, 0.60f, 1.0f},
                   {0.90f, 0.25f, 0.45f, 1.0f},
                   {1.00f, 0.55f, 0.15f, 1.0f},
                   {1.00f, 0.95f, 0.30f, 1.0f}}, _maxSpeed);
}

//////////////////////////////////////////////////
ColorBins::ColorBins(const Colormap &_colormap, int _speedBins,
    int _fadeBins, msgs::Marker::Type _type)
  : colormap(_colormap), speedBins(std::max(1, _speedBins)),
    fadeBins(std::max(1, _fadeBins)), type(_type),
    points(static_cast<std::size_t>(this->speedBins * this->fadeBins))
{
}

//////////////////////////////////////////////////
std::size_t ColorBins::Bin(double _speed, double _brightness) const
{
  const double t = std::clamp(_speed / this->colormap.MaxSpeed(), 0.0, 1.0);
  const int s = std::min(this->speedBins - 1,
      static_cast<int>(std::lround(t * (this->speedBins - 1))));
  const double k = std::clamp(_brightness, 0.0, 1.0);
  const int f = std::min(this->fadeBins - 1,
      static_cast<int>(std::lround(k * (this->fadeBins - 1))));
  return static_cast<std::size_t>(s * this->fadeBins + f);
}

//////////////////////////////////////////////////
void ColorBins::Add(const math::Vector3d &_a, const math::Vector3d &_b,
    double _speed, double _brightness)
{
  auto &bin = this->points[this->Bin(_speed, _brightness)];
  bin.push_back(_a);
  bin.push_back(_b);
}

//////////////////////////////////////////////////
void ColorBins::AddTriangle(const math::Vector3d &_a,
    const math::Vector3d &_b, const math::Vector3d &_c, double _speed,
    double _brightness)
{
  // Both windings, so the renderer's back face culling cannot hide it.
  auto &bin = this->points[this->Bin(_speed, _brightness)];
  bin.insert(bin.end(), {_a, _b, _c, _a, _c, _b});
}

//////////////////////////////////////////////////
int ColorBins::Count() const
{
  return this->speedBins * this->fadeBins;
}

//////////////////////////////////////////////////
math::Color ColorBins::BinColor(int _bin) const
{
  const int s = _bin / this->fadeBins;
  const int f = _bin % this->fadeBins;
  const double speed = this->speedBins == 1 ? 0.0 :
      this->colormap.MaxSpeed() * s / (this->speedBins - 1);
  const float k = this->fadeBins == 1 ? 1.0f :
      kDarkest + (1.0f - kDarkest) * static_cast<float>(f) /
      static_cast<float>(this->fadeBins - 1);
  return Dim(this->colormap.At(speed), k);
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> ColorBins::Markers(const std::string &_ns,
    std::uint64_t _firstId) const
{
  std::vector<msgs::Marker> out;
  out.reserve(this->points.size());
  for (int b = 0; b < this->Count(); ++b)
  {
    msgs::Marker m;
    m.set_ns(_ns);
    m.set_id(_firstId + static_cast<std::uint64_t>(b));
    m.set_action(msgs::Marker::ADD_MODIFY);
    m.set_type(this->type);
    m.set_visibility(msgs::Marker::GUI);
    msgs::Set(m.mutable_pose(), math::Pose3d::Zero);
    const math::Color c = this->BinColor(b);
    auto *material = m.mutable_material();
    material->set_lighting(false);
    msgs::Set(material->mutable_ambient(), c);
    msgs::Set(material->mutable_diffuse(), c);
    msgs::Set(material->mutable_emissive(), c);
    for (const auto &p : this->points[static_cast<std::size_t>(b)])
      msgs::Set(m.add_point(), p);
    out.push_back(std::move(m));
  }
  return out;
}

//////////////////////////////////////////////////
msgs::Marker SolidMarker(const std::string &_ns, std::uint64_t _id,
    msgs::Marker::Type _type, const math::Color &_color,
    const std::vector<math::Vector3d> &_points)
{
  msgs::Marker m;
  m.set_ns(_ns);
  m.set_id(_id);
  m.set_action(msgs::Marker::ADD_MODIFY);
  m.set_type(_type);
  m.set_visibility(msgs::Marker::GUI);
  msgs::Set(m.mutable_pose(), math::Pose3d::Zero);
  auto *material = m.mutable_material();
  material->set_lighting(false);
  msgs::Set(material->mutable_ambient(), _color);
  msgs::Set(material->mutable_diffuse(), _color);
  msgs::Set(material->mutable_emissive(), _color);
  for (const auto &p : _points)
    msgs::Set(m.add_point(), p);
  return m;
}

//////////////////////////////////////////////////
ColorBins ColoredArrows(const std::vector<math::Vector3d> &_points,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity,
    double _scale, const Colormap &_colormap)
{
  ColorBins bins(_colormap, 8, 1);
  for (const auto &p : _points)
  {
    const math::Vector3d v = _velocity(p);
    const auto segments = ArrowSegments({p}, [&v](const math::Vector3d &)
        { return v; }, _scale);
    for (std::size_t i = 0; i + 1 < segments.size(); i += 2)
      bins.Add(segments[i], segments[i + 1], v.Length());
  }
  return bins;
}

//////////////////////////////////////////////////
ParticleField::ParticleField(std::uint32_t _seed)
  : rng(_seed)
{
}

//////////////////////////////////////////////////
void ParticleField::Configure(const Settings &_settings)
{
  this->settings = _settings;
  this->settings.count = std::max(0, this->settings.count);
  this->settings.trailLength = std::max(2, this->settings.trailLength);
  if (!(this->settings.lifetime > 0.0))
    this->settings.lifetime = 6.0;
  if (!(this->settings.trailStep > 0.0))
    this->settings.trailStep = 0.1;
  this->particles.assign(static_cast<std::size_t>(this->settings.count),
                         Particle());
  for (auto &p : this->particles)
    this->Respawn(p, true);
}

//////////////////////////////////////////////////
const ParticleField::Settings &ParticleField::Config() const
{
  return this->settings;
}

//////////////////////////////////////////////////
void ParticleField::Respawn(Particle &_p, bool _randomAge)
{
  const auto &r = this->settings.region;
  std::uniform_real_distribution<double> u(-0.5, 0.5);
  const math::Vector3d at(r.center.X() + u(this->rng) * r.extent,
                          r.center.Y() + u(this->rng) * r.extent,
                          this->settings.z);
  std::uniform_real_distribution<double> life(0.5 * this->settings.lifetime,
                                              this->settings.lifetime);
  _p.lifetime = life(this->rng);
  _p.age = _randomAge ?
      std::uniform_real_distribution<double>(0.0, _p.lifetime)(this->rng) :
      0.0;
  _p.trail.assign(1, at);
  _p.speed.assign(1, 0.0);
  _p.sinceTrail = 0.0;
}

//////////////////////////////////////////////////
bool ParticleField::Inside(const math::Vector3d &_p) const
{
  const auto &r = this->settings.region;
  const double h = 0.5 * r.extent;
  return std::abs(_p.X() - r.center.X()) <= h &&
         std::abs(_p.Y() - r.center.Y()) <= h;
}

//////////////////////////////////////////////////
void ParticleField::Step(double _dt,
    const std::function<math::Vector3d(const math::Vector3d &)> &_velocity)
{
  if (!(_dt > 0.0))
    return;
  for (auto &p : this->particles)
  {
    p.age += _dt;
    const math::Vector3d here = p.trail.back();
    const math::Vector3d v = _velocity(here);
    // Horizontal only: the trails lie on their plane.
    math::Vector3d next = here +
        math::Vector3d(v.X(), v.Y(), 0.0) * (_dt * this->settings.speedup);
    if (p.age >= p.lifetime || !this->Inside(next))
    {
      this->Respawn(p, false);
      continue;
    }

    // The head moves every step; a new trail point is kept every trailStep.
    p.sinceTrail += _dt;
    if (p.sinceTrail >= this->settings.trailStep || p.trail.size() < 2u)
    {
      p.sinceTrail = 0.0;
      p.trail.push_back(next);
      p.speed.push_back(v.Length());
      if (p.trail.size() >
          static_cast<std::size_t>(this->settings.trailLength))
      {
        p.trail.erase(p.trail.begin());
        p.speed.erase(p.speed.begin());
      }
    }
    else
    {
      p.trail.back() = next;
      p.speed.back() = v.Length();
    }
  }
}

//////////////////////////////////////////////////
ColorBins ParticleField::Trails(const Colormap &_colormap) const
{
  ColorBins bins(_colormap, 8, 4);
  for (const auto &p : this->particles)
  {
    const std::size_t n = p.trail.size();
    if (n < 2u)
      continue;
    // A trail fades in as the particle is born and out as it dies, and
    // from head to tail.
    const double fadeIn = std::min(1.0, p.age / 0.5);
    const double fadeOut = std::min(1.0, (p.lifetime - p.age) / 0.5);
    const double life = std::max(0.0, std::min(fadeIn, fadeOut));
    for (std::size_t i = 1; i < n; ++i)
    {
      const double headward = static_cast<double>(i) /
          static_cast<double>(n - 1);
      const double k = life * this->EdgeFade(p.trail[i]) *
          (kTailBrightness + (1.0 - kTailBrightness) * headward);
      if (k >= kInvisible)
        bins.Add(p.trail[i - 1], p.trail[i], p.speed[i], k);
    }
  }
  return bins;
}

//////////////////////////////////////////////////
double ParticleField::EdgeFade(const math::Vector3d &_p) const
{
  // Round, not square: full brightness in the middle, fading out over the
  // outer band to nothing at half the side from the centre.
  const auto &r = this->settings.region;
  const double radius = 0.5 * r.extent;
  const double margin = kEdgeMargin * radius;
  if (!(margin > 0.0))
    return 1.0;
  const double dist = std::hypot(_p.X() - r.center.X(), _p.Y() - r.center.Y());
  return std::clamp((radius - dist) / margin, 0.0, 1.0);
}

//////////////////////////////////////////////////
std::vector<math::Vector3d> ParticleField::Positions() const
{
  std::vector<math::Vector3d> out;
  out.reserve(this->particles.size());
  for (const auto &p : this->particles)
    out.push_back(p.trail.back());
  return out;
}

//////////////////////////////////////////////////
Steering::Steering(double _speedRate, double _turnRate,
    Clock::duration _period)
  : speedRate(_speedRate), turnRate(_turnRate), period(_period)
{
}

//////////////////////////////////////////////////
void Steering::SetTarget(double _speed, double _direction)
{
  double d = std::fmod(_direction, 360.0);
  if (d < 0.0)
    d += 360.0;
  this->target = std::make_pair(std::max(0.0, _speed), d);
  this->arrived = false;
}

//////////////////////////////////////////////////
bool Steering::Active() const
{
  return this->target.has_value();
}

//////////////////////////////////////////////////
bool Steering::Arrived() const
{
  return this->arrived;
}

//////////////////////////////////////////////////
std::optional<msgs::Param> Steering::Step(double _speed, double _direction,
    Clock::time_point _now)
{
  if (!this->target)
    return std::nullopt;
  if (this->lastStep && _now - *this->lastStep < this->period)
    return std::nullopt;

  // A fresh start ramps from the world's values, not from a stale command.
  if (!this->commanded)
    this->commanded = std::make_pair(_speed, _direction);

  const double dt = this->lastStep ?
      std::chrono::duration<double>(_now - *this->lastStep).count() :
      std::chrono::duration<double>(this->period).count();
  const double maxSpeedStep = this->speedRate * dt;
  const double maxTurnStep = this->turnRate * dt;

  auto &[speed, direction] = *this->commanded;
  const auto [toSpeed, toDirection] = *this->target;
  const double ds = toSpeed - speed;
  speed = std::abs(ds) <= maxSpeedStep ? toSpeed :
      speed + std::copysign(maxSpeedStep, ds);
  const double dd = Wrap180(toDirection - direction);
  direction = std::abs(dd) <= maxTurnStep ? toDirection :
      direction + std::copysign(maxTurnStep, dd);
  direction = std::fmod(direction, 360.0);
  if (direction < 0.0)
    direction += 360.0;

  this->lastStep = _now;
  this->arrived = speed == toSpeed && direction == toDirection;

  msgs::Param msg;
  auto &s = (*msg.mutable_params())["speed"];
  s.set_type(msgs::Any::DOUBLE);
  s.set_double_value(speed);
  auto &d = (*msg.mutable_params())["direction"];
  d.set_type(msgs::Any::DOUBLE);
  d.set_double_value(direction);

  if (this->arrived)
  {
    // Done; the next target starts again from the world's values.
    this->target.reset();
    this->commanded.reset();
    this->lastStep.reset();
  }
  return msg;
}
}  // namespace gz::sim::maritime_gui
