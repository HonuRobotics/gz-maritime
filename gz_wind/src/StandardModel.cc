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
#include "StandardModel.hh"

#include <algorithm>
#include <cmath>

#include <gz/math/Angle.hh>

namespace gz::sim::wind
{
namespace
{
/// \brief Components of a gust. Enough that the sum looks Gaussian, does
/// not repeat over any run and follows exp(-lag / T) to within 0.05, few
/// enough that a query stays cheap when every marked shape asks each step.
constexpr int kComponents{64};

/// \brief Band of the components, in multiples of 1 / T: the Lorentzian
/// spectrum holds about 99 % of its variance in it.
constexpr double kLowest{1e-3};

/// \brief Upper end of the band, in multiples of 1 / T.
constexpr double kHighest{10.0};

/// \brief Slowest speed the gusts travel at, m/s. Frozen turbulence does not
/// hold in near calm air, and below this the delay between two points of a
/// vehicle would grow without bound.
constexpr double kMinAdvection{1.0};
}  // namespace

//////////////////////////////////////////////////
void Gust::Build(double _sigma, double _time, std::mt19937 &_rng)
{
  this->amplitude.clear();
  this->omega.clear();
  this->phase.clear();
  if (_sigma <= 0.0 || _time <= 0.0)
    return;

  // Log spaced frequencies over the band, each standing for its share of
  // the one sided spectrum S(f) = 4 sigma^2 T / (1 + (2 pi f T)^2), whose
  // autocorrelation is sigma^2 exp(-lag / T).
  std::uniform_real_distribution<double> uniform(0.0, 2.0 * GZ_PI);
  const double f0 = kLowest / _time;
  const double ratio = std::pow(kHighest / kLowest, 1.0 / kComponents);
  double variance{0.0};
  for (int i = 0; i < kComponents; ++i)
  {
    const double lo = f0 * std::pow(ratio, i);
    const double hi = lo * ratio;
    const double f = std::sqrt(lo * hi);
    const double x = 2.0 * GZ_PI * f * _time;
    const double s = 4.0 * _sigma * _sigma * _time / (1.0 + x * x);
    const double a = std::sqrt(2.0 * s * (hi - lo));
    this->amplitude.push_back(a);
    this->omega.push_back(2.0 * GZ_PI * f);
    this->phase.push_back(uniform(_rng));
    variance += 0.5 * a * a;
  }

  // The band leaves out a little of the spectrum; scale back to sigma.
  const double scale = _sigma / std::sqrt(variance);
  for (double &a : this->amplitude)
    a *= scale;
}

//////////////////////////////////////////////////
double Gust::At(double _t) const
{
  double sum{0.0};
  for (std::size_t i = 0; i < this->amplitude.size(); ++i)
    sum += this->amplitude[i] * std::cos(this->omega[i] * _t + this->phase[i]);
  return sum;
}

//////////////////////////////////////////////////
void StandardModel::SetParameters(const WindParameters &_params)
{
  this->params = _params;
  // One generator for both, speed first, so the seed fixes both series.
  std::mt19937 rng(_params.seed);
  this->speedGust.Build(_params.speed_gust, _params.speed_gust_time, rng);
  this->directionGust.Build(_params.direction_gust,
                            _params.direction_gust_time, rng);
}

//////////////////////////////////////////////////
math::Vector3d StandardModel::Velocity(const math::Vector3d &_enu,
    double _time) const
{
  // A wind from a direction blows towards the opposite one.
  const double a = GZ_DTOR(this->params.direction);
  const math::Vector3d towards(-std::sin(a), -std::cos(a), 0.0);

  // The gusts travel with the mean wind: a point downwind sees what a point
  // upwind saw earlier (Taylor's frozen turbulence), so two points in line
  // with the wind see the same gust, one after the other. In light air they
  // travel at kMinAdvection, so the gust stays continuous down to calm.
  const math::Vector3d horizontal(_enu.X(), _enu.Y(), 0.0);
  const double t = _time - towards.Dot(horizontal) /
      std::max(this->params.speed, kMinAdvection);

  const double speed = std::max(0.0,
      this->params.speed + this->speedGust.At(t));
  const double b = GZ_DTOR(this->params.direction + this->directionGust.At(t));
  return {-speed * std::sin(b), -speed * std::cos(b), this->params.vertical};
}

//////////////////////////////////////////////////
bool StandardModel::TimeVarying() const
{
  return this->speedGust.Active() || this->directionGust.Active();
}
}  // namespace gz::sim::wind
