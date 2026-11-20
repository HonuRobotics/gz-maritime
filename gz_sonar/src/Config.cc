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
#include "gz/maritime/sonar/Config.hh"

#include <cmath>

namespace gz::maritime::sonar
{
namespace
{
constexpr double kDeg = M_PI / 180.0;
}

double Backscatter(const AcousticProperties &_properties, double _incidence)
{
  const double c = std::cos(_incidence);
  double sigma = std::pow(10.0, _properties.lambertDb / 10.0) * c * c;
  if (_properties.specularDb)
  {
    const double w = _properties.specularWidth;
    sigma += std::pow(10.0, *_properties.specularDb / 10.0) *
      std::exp(-_incidence * _incidence / (2.0 * w * w));
  }
  return sigma;
}

double SonarConfig::AssumedSoundSpeed(double _trueSoundSpeed) const
{
  return this->soundSpeed ? *this->soundSpeed : _trueSoundSpeed;
}

double SonarConfig::SampleSpacing() const
{
  return (this->rangeMax - this->rangeMin) /
    static_cast<double>(this->samples);
}

double SonarConfig::SampleRange(std::size_t _k) const
{
  return this->rangeMin +
    (static_cast<double>(_k) + 0.5) * this->SampleSpacing();
}

double SonarConfig::BeamAzimuth(std::size_t _j) const
{
  const double spacing = this->horizontalFov /
    static_cast<double>(this->beams);
  return -0.5 * this->horizontalFov +
    (static_cast<double>(_j) + 0.5) * spacing;
}

SonarConfig EchoImaging()
{
  // Datasheet: 700 kHz to 30 m, 120 x 20 degrees, 256 beams, 1.5 degree
  // angular resolution, range resolution under 8 mm.
  SonarConfig c;
  c.frequency = 700000.0;
  c.horizontalFov = 120.0 * kDeg;
  c.verticalFov = 20.0 * kDeg;
  c.beams = 256;
  c.beamWidth = 1.5 * kDeg;
  c.rangeMin = 0.2;
  c.rangeMax = 30.0;
  c.samples = 1024;
  c.pulseLength = 1.0e-5;
  c.tvg = Tvg::kSpreadingAbsorption;
  return c;
}

SonarConfig OmniscanSidescan()
{
  // Datasheet: 450 kHz, 0.5 degree along track by 50 degrees, no tilt,
  // 600 samples typical, up to 150 m; we default to a 50 m window.
  SonarConfig c;
  c.frequency = 450000.0;
  c.horizontalFov = 0.5 * kDeg;
  c.verticalFov = 50.0 * kDeg;
  c.beams = 1;
  c.beamWidth = 0.5 * kDeg;
  c.rangeMin = 1.0;
  c.rangeMax = 50.0;
  c.samples = 600;
  c.pulseLength = 1.0e-4;
  c.tvg = Tvg::kSpreadingAbsorption;
  c.gainTimeConstant = 1.0;
  c.raysPerBeam = 8;
  c.elevationRays = 400;
  return c;
}

SonarConfig SurveyorProfiling()
{
  // Datasheet: 240 kHz, 4 degrees along track by 80 degrees across, angle
  // of arrival with under 1 degree resolution, 10 to 20 points per ping,
  // 200 to 800 range steps; we default to 50 m, the typical detection range.
  SonarConfig c;
  c.frequency = 240000.0;
  c.horizontalFov = 80.0 * kDeg;
  c.verticalFov = 4.0 * kDeg;
  c.beams = 160;
  c.beamWidth = 1.0 * kDeg;
  c.rangeMin = 2.5;
  c.rangeMax = 50.0;
  c.samples = 400;
  c.pulseLength = 1.0e-4;
  c.tvg = Tvg::kNone;
  c.readout = ProfilingReadout::kAngleOfArrival;
  c.maxSoundings = 20;
  c.elevationRays = 16;
  return c;
}
}  // namespace gz::maritime::sonar
