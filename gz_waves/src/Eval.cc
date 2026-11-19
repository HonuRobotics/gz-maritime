/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/Eval.hh"

#include <cmath>

#include <algorithm>
#include <cmath>

#include "gz/sim/waves/WaveSimulation.hh"

namespace gz::sim::waves
{

//////////////////////////////////////////////////
void Advance(WavefieldData &_wf, double _t)
{
  if (_wf.simulation)
    _wf.simulation->Update(_t);
}

//////////////////////////////////////////////////
double SurfaceElevation(const WavefieldData &_wf,
                        double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->Elevation(_x, _y, _t) : 0.0;
}

//////////////////////////////////////////////////
double SurfaceHeight(const WavefieldData &_wf,
                     double _x, double _y, double _t)
{
  if (!_wf.simulation)
    return 0.0;
  const WaveField2D *field = _wf.simulation->Field();
  if (nullptr == field || nullptr == field->dx || nullptr == field->dy ||
      0 == field->n || !(field->tile > 0.0))
  {
    return _wf.simulation->Elevation(_x, _y, _t);
  }

  // Bilinear, periodic over the tile: cell (i, j) is at (i, j) * tile / n.
  const int n = static_cast<int>(field->n);
  const double cell = field->tile / static_cast<double>(field->n);
  auto sample = [&](const double *_grid, double _px, double _py)
  {
    const double fi = _px / cell;
    const double fj = _py / cell;
    const double i0f = std::floor(fi);
    const double j0f = std::floor(fj);
    const double fx = fi - i0f;
    const double fy = fj - j0f;
    auto wrap = [n](double _i)
    {
      int i = static_cast<int>(std::fmod(_i, static_cast<double>(n)));
      return i < 0 ? i + n : i;
    };
    const int i0 = wrap(i0f);
    const int j0 = wrap(j0f);
    const int i1 = (i0 + 1) % n;
    const int j1 = (j0 + 1) % n;
    auto at = [&](int _i, int _j) { return _grid[_i + _j * n]; };
    return at(i0, j0) * (1 - fx) * (1 - fy) + at(i1, j0) * fx * (1 - fy) +
           at(i0, j1) * (1 - fx) * fy + at(i1, j1) * fx * fy;
  };

  // The water drawn over (x, y) came from (x0, y0) = (x, y) - D(x0, y0).
  // The map converges while the surface does not fold, where the drawn
  // surface is a height field at all, each step by the slope of the
  // displacement; stop once a step moves less than a tenth of a millimetre.
  static constexpr int kMaxSteps{8};
  static constexpr double kTolerance{1e-4};
  double x0 = _x;
  double y0 = _y;
  for (int k = 0; k < kMaxSteps; ++k)
  {
    const double nx = _x - sample(field->dx, x0, y0);
    const double ny = _y - sample(field->dy, x0, y0);
    const double moved = std::abs(nx - x0) + std::abs(ny - y0);
    x0 = nx;
    y0 = ny;
    if (moved < kTolerance)
      break;
  }
  return _wf.simulation->Elevation(x0, y0, _t);
}

//////////////////////////////////////////////////
gz::math::Vector3d ParticleVelocity(const WavefieldData &_wf,
                                    double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->ParticleVelocity(_x, _y, _t)
                        : gz::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
gz::math::Vector3d WaterVelocity(const WavefieldData &_wf, double _waterLevel,
                                 double _x, double _y, double _z, double _t)
{
  if (!_wf.simulation)
    return gz::math::Vector3d::Zero;
  const gz::math::Vector3d surface =
      _wf.simulation->ParticleVelocity(_x, _y, _t);
  const double depth =
      _waterLevel + _wf.simulation->Elevation(_x, _y, _t) - _z;
  if (depth <= 0.0)
    return surface;

  // Deep-water dispersion, omega^2 = g k, at the peak period.
  const WaveParameters p = WithSeaState(_wf.params);
  if (!(p.period > 0.0) || !(p.gravity > 0.0))
    return surface;
  const double omega = 2.0 * GZ_PI / p.period;
  const double k = omega * omega / p.gravity;
  return surface * std::exp(-k * depth);
}

//////////////////////////////////////////////////
gz::math::Vector3d Normal(const WavefieldData &_wf,
                          double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->Normal(_x, _y, _t)
                        : gz::math::Vector3d::UnitZ;
}

//////////////////////////////////////////////////
double Jacobian(const WavefieldData &_wf, double _x, double _y, double _t)
{
  return _wf.simulation ? _wf.simulation->Jacobian(_x, _y, _t) : 1.0;
}

//////////////////////////////////////////////////
double FoamMask(const WavefieldData &_wf, double _x, double _y, double _t,
                double _threshold)
{
  const double j = Jacobian(_wf, _x, _y, _t);
  if (j >= _threshold)
    return 0.0;
  return std::clamp(1.0 - j / _threshold, 0.0, 1.0);
}

}  // namespace gz::sim::waves
