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
  /// \brief Lambert strength, 10 log10 mu, dB: 
  /// The backscattering strength
  /// of a rough surface, mu being its backscattering cross section per unit
  /// area and per unit solid angle at normal incidence (APL-UW TR 9407,
  /// Sec. IV.C.2, Eq. 34, p. IV-22). Lambert-equivalent values for generic
  /// bottoms (TR 9407 Table 3, pp. IV-23 to IV-25, read at 30 kHz and 20
  /// degrees grazing): about +1 rough rock, -6 rock, -13 sandy gravel, -18
  /// sand, -23 to -28 silt; TR 9407 stops at 100 kHz, so above it these
  /// are an extrapolation. 
  double lambertDb{-20.0};

  /// \brief Specular strength, 10 log10 S, dB: the glint off a smooth
  /// surface seen near head on. Near normal incidence the return follows the
  /// square of the reflection coefficient, which the impedance contrast sets
  /// (APL-UW TR 9407, Eq. 37, p. IV-27). Typically -20 (a faint glint off a
  /// rough surface) to about 0 (glass, metal plate, smooth concrete). Larger
  /// values give a brighter glint; empty for a purely diffuse surface.
  std::optional<double> specularDb;

  /// \brief Angular width of the specular lobe, rad. Typically 0.02 to
  /// 0.03 for very smooth surfaces to 0.17 to 0.35 for rough ones; 0.07
  /// (4 degrees) suits most man-made objects. Larger values show the glint
  /// over a wider range of viewing angles, at the same peak; smaller values
  /// make it a brief flash seen only square on.
  double specularWidth{0.07};
};

/// \brief Backscatter per unit area at an incidence angle:
/// mu cos^2(theta) + S exp(-theta^2 / 2 w^2).
/// \param[in] _properties The surface.
/// \param[in] _incidence Angle between the ray and the surface normal, rad.
/// \return The backscatter cross section per unit area, linear.
double Backscatter(const AcousticProperties &_properties, double _incidence);

/// \brief How the received level is adjusted for range before the sonar
/// plugin publishes it.
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
enum class ProfilingDetection
{
  /// \brief One sounding per beam, at the beam's angle.
  kBeamformed,
  /// \brief The strongest echoes across the fan, each at its own angle.
  kAngleOfArrival,
};

/// \brief Most threads an engine takes when SonarConfig::threads is 0.
inline constexpr std::size_t kDefaultMaxEngineThreads = 8;

/// \brief Everything that describes one sonar. The defaults are a generic
/// imaging sonar close to the ECHO; the presets below give each exemplar
/// exactly (the ECHO's, for one, shades its sidelobes to -25 dB and reaches
/// three half-widths in elevation).
///
/// Frames (coordinate systems): the sensor's x axis is the acoustic axis. 
///Azimuth is the angle
/// about the sensor's z axis, positive toward +y; elevation is the angle out
/// of the x-y plane, positive toward +z. The fan spans the horizontal field
/// of view in azimuth and the vertical field of view in elevation.
struct SonarConfig
{
  /// \brief Center frequency, Hz. Typically 200 kHz (longer range) to
  /// 1 MHz or more (finer detail). Higher frequencies absorb faster, so
  /// echoes fade sooner with range; lower ones reach farther.
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

  /// \brief Range window, m. The window sets what is sampled: a longer
  /// rangeMax covers more but spreads the same samples thinner, and costs
  /// more rays and engine work.
  double rangeMin{0.2};
  double rangeMax{30.0};

  /// \brief Range samples per beam. Typically 400 to 1500. More samples
  /// give finer range steps (rangeMax over samples) and a larger message.
  std::size_t samples{1024};

  /// \brief Transmit pulse length, s: the range resolution is c tau / 2.
  /// Typically 1e-5 (7.5 mm, imaging) to 1e-4 (7.5 cm, sidescan and
  /// profiling). Longer pulses smear echoes along range.
  double pulseLength{1.0e-5};

  /// \brief The speed of sound the sonar assumes when it turns travel time
  /// into range, m/s. Empty: the water's true value. Typically 1450 to 1550;
  /// a value other than the water's (the world's <sound_speed>) scales
  /// every range by their ratio, as a mis-set real sonar does: 1 % off is
  /// 0.3 m at 30 m.
  ///
  /// Both are one constant, not a sound-speed profile. At the ranges these
  /// sonars work (tens of meters, in water a few to tens of meters deep)
  /// refraction is negligible.A sound speed profile matters for deep, long-range 
  /// bathymetric multibeams, whose ray tracing is out of scope here.
  std::optional<double> soundSpeed;

  /// \brief Fixed gain in dB; empty for automatic gain.
  std::optional<double> gainDb;

  /// \brief How quickly automatic gain follows the scene, s: it moves
  /// toward each ping's own gain with this time constant. 0 takes each
  /// ping's gain as it is.
  double gainTimeConstant{0.0};

  /// \brief Time-varied gain.
  Tvg tvg{Tvg::kSpreadingAbsorption};

  /// \brief Spreading law of the time-varied gain, dB per decade of range:
  /// 30 for the seabed, 40 for point targets. Larger values brighten far
  /// ranges more; smaller values leave them darker than near ones.
  double tvgSpreading{30.0};

  /// \brief Additive noise relative to full scale, the top of the published
  /// levels (255 in the imaging sonar's 8-bit image, 0 dB in the sidescan's
  /// levels), dB.
  /// Typically -60 (a clean image) to -20 (a hard one). Larger values
  /// (closer to 0) fill the shadows and lower the contrast; smaller values
  /// leave faint echoes and shadows visible.
  double noiseFloorDb{-50.0};

  /// \brief Span of levels below full scale that the 8-bit imaging
  /// image the sonar plugin publishes maps onto 1 to 255, dB. Typically 40 to 80. Larger values
  /// show fainter echoes at less contrast; smaller values give more contrast
  /// and drop the faint ones to black.
  double dynamicRangeDb{60.0};

  /// \brief Apply speckle.
  bool speckle{true};

  /// \brief Seed for every random draw; combined with the ping number.
  std::uint64_t seed{0};

  /// \brief Rays across one beam and rays across the vertical field of
  /// view: the geometry sampling density. raysPerBeam is typically 2 to 8.
  /// More rays catch small objects and smooth the image at more cost per
  /// ping; fewer are faster and can miss objects narrower than a ray.
  std::size_t raysPerBeam{4};
  std::size_t elevationRays{64};

  /// \brief How far the rays reach in elevation, in half-widths of the
  /// vertical beam (2: out to twice the beam's -3 dB edge, where the
  /// pattern is 12 dB down). elevationRays is the count at 2; it scales
  /// with the span, so the angular step stays the same. Typically 1 to 3.
  /// Larger values let surfaces just outside the beam show faintly, as on
  /// a real sonar, at more rays; 1 cuts the image off at the -3 dB edge.
  double elevationSpan{2.0};

  /// \brief Profiling: how angles are found.
  ProfilingDetection detection{ProfilingDetection::kAngleOfArrival};

  /// \brief Profiling: most soundings per ping (angle of arrival).
  std::size_t maxSoundings{20};

  /// \brief Profiling: level above the noise floor that counts, dB.
  /// Typically 6 to 20. Larger values reject weak echoes and drop more
  /// soundings at grazing angles; smaller values keep more, including
  /// spurious ones from noise.
  double detectionThresholdDb{10.0};

  /// \brief Profiling: range noise at high signal-to-noise ratio, m; empty
  /// for half a range sample.
  std::optional<double> rangeNoise;

  /// \brief Profiling: angle noise, rad. Typically 0.005 to 0.02 (0.3 to
  /// 1 degree). Larger values scatter soundings across the track, more so
  /// at long range.
  double angleNoise{0.009};

  /// \brief Threads the engine may use, the pinging thread included; 0 for
  /// one per core up to kDefaultMaxEngineThreads. Beyond about 8 a ping
  /// gets no faster, and the cores are better left to the simulation and
  /// to other sonars. Not settable from SDF. The result is the same
  /// whatever the number.
  std::size_t threads{0};

  /// \brief The assumed sound speed, falling back to the water's.
  double AssumedSoundSpeed(double _trueSoundSpeed) const;

  /// \brief Spacing of the range samples, m.
  double SampleSpacing() const;

  /// \brief Range of the center of sample _k, m.
  double SampleRange(std::size_t _k) const;

  /// \brief Azimuth of beam _j, rad.
  double BeamAzimuth(std::size_t _j) const;
};

// Presets: each sensor type's defaults, those of a standard device of its
// kind. Every value can be overridden in SDF; see Config.cc.

/// \brief The Sonoptix ECHO imaging sonar at 700 kHz.
SonarConfig EchoImaging();

/// \brief One Cerulean Omniscan 450 SS sidescan transducer.
SonarConfig OmniscanSidescan();

/// \brief The Cerulean Surveyor 240-16 profiling sonar.
SonarConfig SurveyorProfiling();
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_CONFIG_HH_
