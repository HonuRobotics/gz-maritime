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
#ifndef GZ_MARITIME_SONAR_CONFIG_HH_
#define GZ_MARITIME_SONAR_CONFIG_HH_

#include <cstddef>
#include <cstdint>
#include <optional>

namespace gz::maritime::sonar
{
/// \brief How a surface scatters sound back: Lambert's law for diffuse
/// scattering plus an optional specular lobe for smooth surfaces.
struct AcousticProperties
{
  /// \brief Lambert strength, 10 log10 mu, dB.
  double lambertDb{-20.0};

  /// \brief Specular strength, 10 log10 S, dB. Empty for a purely diffuse
  /// surface.
  std::optional<double> specularDb;

  /// \brief Angular width of the specular lobe, rad.
  double specularWidth{0.07};
};

/// \brief Backscatter per unit area at an incidence angle:
/// mu cos^2(theta) + S exp(-theta^2 / 2 w^2).
/// \param[in] _properties The surface.
/// \param[in] _incidence Angle between the ray and the surface normal, rad.
/// \return The backscatter cross section per unit area, linear.
double Backscatter(const AcousticProperties &_properties, double _incidence);

/// \brief What the sonar does with the received level before readout.
enum class Tvg
{
  /// \brief Nothing: the received level.
  kNone,
  /// \brief Compensate spreading: tvgSpreading * log10(R).
  kSpreading,
  /// \brief Compensate spreading and absorption: plus 2 alpha R.
  kSpreadingAbsorption,
};

/// \brief How a profiling sonar finds the angle of each sounding.
enum class ProfilingReadout
{
  /// \brief One sounding per beam, at the beam's angle.
  kBeamformed,
  /// \brief The strongest echoes across the fan, each at its own angle.
  kAngleOfArrival,
};

/// \brief Everything that describes one sonar. The defaults are the
/// imaging sonar's; the presets below give each exemplar.
///
/// Frames: the sensor's x axis is the acoustic axis. Azimuth is the angle
/// about the sensor's z axis, positive toward +y; elevation is the angle out
/// of the x-y plane, positive toward +z. The fan spans the horizontal field
/// of view in azimuth and the vertical field of view in elevation.
struct SonarConfig
{
  /// \brief Center frequency, Hz.
  double frequency{700000.0};

  /// \brief Width of the fan in azimuth, rad.
  double horizontalFov{2.0944};

  /// \brief Thickness of the fan in elevation, rad: the -3 dB width of the
  /// vertical beam.
  double verticalFov{0.349};

  /// \brief Beams across the fan, evenly spaced; beam 0 is at the most
  /// negative azimuth.
  std::size_t beams{256};

  /// \brief -3 dB width of one beam across the fan, rad.
  double beamWidth{0.026};

  /// \brief First sidelobe of the beam's intensity pattern relative to its
  /// main lobe, dB. -13.3 is an unshaded line array.
  double sidelobeLevelDb{-13.26};

  /// \brief Range window, m.
  double rangeMin{0.2};
  double rangeMax{30.0};

  /// \brief Range samples per beam.
  std::size_t samples{1024};

  /// \brief Transmit pulse length, s: the range resolution is c tau / 2.
  double pulseLength{1.0e-5};

  /// \brief The speed of sound the sonar assumes when it turns travel time
  /// into range, m/s. Empty: the water's true value.
  std::optional<double> soundSpeed;

  /// \brief Fixed gain in dB; empty for automatic gain.
  std::optional<double> gainDb;

  /// \brief Time-varied gain.
  Tvg tvg{Tvg::kSpreadingAbsorption};

  /// \brief Spreading law of the time-varied gain, dB per decade of range:
  /// 30 for the seabed, 40 for point targets.
  double tvgSpreading{30.0};

  /// \brief Additive noise relative to the readout's full scale, dB.
  double noiseFloorDb{-50.0};

  /// \brief Span of levels below full scale that the 8-bit imaging
  /// readout maps onto 1 to 255, dB.
  double dynamicRangeDb{60.0};

  /// \brief Apply speckle.
  bool speckle{true};

  /// \brief Seed for every random draw; combined with the ping number.
  std::uint64_t seed{0};

  /// \brief Rays across one beam and rays across the vertical field of
  /// view: the geometry sampling density.
  std::size_t raysPerBeam{4};
  std::size_t elevationRays{64};

  /// \brief Profiling: how angles are found.
  ProfilingReadout readout{ProfilingReadout::kAngleOfArrival};

  /// \brief Profiling: most soundings per ping (angle of arrival).
  std::size_t maxSoundings{20};

  /// \brief Profiling: level above the noise floor that counts, dB.
  double detectionThresholdDb{10.0};

  /// \brief Profiling: range noise at high signal-to-noise ratio, m; empty
  /// for one range sample.
  std::optional<double> rangeNoise;

  /// \brief Profiling: angle noise, rad.
  double angleNoise{0.009};

  /// \brief The assumed sound speed, falling back to the water's.
  double AssumedSoundSpeed(double _trueSoundSpeed) const;

  /// \brief Spacing of the range samples, m.
  double SampleSpacing() const;

  /// \brief Range of the center of sample _k, m.
  double SampleRange(std::size_t _k) const;

  /// \brief Azimuth of beam _j, rad.
  double BeamAzimuth(std::size_t _j) const;
};

/// \brief The Sonoptix ECHO imaging sonar at 700 kHz.
SonarConfig EchoImaging();

/// \brief One Cerulean Omniscan 450 SS sidescan transducer.
SonarConfig OmniscanSidescan();

/// \brief The Cerulean Surveyor 240-16 profiling sonar.
SonarConfig SurveyorProfiling();
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_CONFIG_HH_
