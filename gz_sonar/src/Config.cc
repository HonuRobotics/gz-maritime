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
    // The Kirchhoff approximation near normal incidence for Gaussian facet
    // slopes of rms w: R^2 / (8 pi w^2) at the peak. The 1 / (8 pi w^2)
    // makes the lobe carry the energy of a mirror: a sphere of radius a with
    // R^2 = 1 has the target strength of a rigid sphere, a^2 / 4, whatever
    // w, where a bare R^2 peak gave it 8 pi w^2 of that, 9 dB short at 0.07.
    const double w = _properties.specularWidth;
    sigma += std::pow(10.0, *_properties.specularDb / 10.0) /
      (8.0 * M_PI * w * w) *
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

// Three config presets: each sensor type's defaults, set to those of a standard
// device of its kind, so an empty <gz:sonar/> behaves like that device out of
// the box. They are starting values, not fixed ones: every parameter can be
// overridden in SDF (<gz:sonar>) or a vehicle's `sonar:` config without
// recompiling, which is also how a different device is simulated.

SonarConfig EchoImaging()
{
  // Sonoptix ECHO multibeam imaging sonar, datasheet: 700 kHz to 30 m, 120 x
  // 20 degrees, 256 beams, 1.5 degree angular resolution, range resolution
  // under 8 mm.
  SonarConfig c;
  c.frequency = 700000.0;
  c.horizontalFov = 120.0 * kDeg;
  c.verticalFov = 20.0 * kDeg;
  // Rays out to three half-widths (30 degrees, 27 dB down): the seabed
  // shows faintly from inside the -3 dB edge, as it does on a real sonar.
  c.elevationSpan = 3.0;
  c.beams = 256;
  // Sidelobes of a shaded array: Sonoptix does not publish the ECHO's
  // shading, and commercial imaging sonars shade their beams, so a strong
  // target's sidelobe arc stays near it and small shadows stay dark.
  c.sidelobeLevelDb = -25.0;
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
  // Cerulean Sonar Omniscan 450 SS sidescan sonar, datasheet: 450 kHz, 0.5
  // degree along track by 50 degrees, no tilt, 600 samples typical, up to 150
  // m; we default to a 50 m window.
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
  // Cerulean Sonar Surveyor 240-16 multibeam echosounder, datasheet: 240 kHz,
  // 4 degrees along track by 80 degrees across, angle of arrival with under 1
  // degree resolution, 10 to 20 points per ping, 200 to 800 range steps; we
  // default to 50 m, the typical detection range.
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
  c.detection = ProfilingDetection::kAngleOfArrival;
  c.maxSoundings = 20;
  c.elevationRays = 16;
  return c;
}
}  // namespace gz::maritime::sonar
