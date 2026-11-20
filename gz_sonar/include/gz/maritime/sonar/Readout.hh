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
#ifndef GZ_MARITIME_SONAR_READOUT_HH_
#define GZ_MARITIME_SONAR_READOUT_HH_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Engine.hh"

namespace gz::maritime::sonar
{
/// \brief The levels a sonar reads out, in dB relative to full scale (0 dB
/// is the top of the readout): level + time-varied gain + gain, with the
/// noise floor added.
struct Levels
{
  std::size_t beams{0};
  std::size_t samples{0};

  /// \brief Beam-major, dB re full scale.
  std::vector<double> db;

  /// \brief The time-varied gain applied at each sample, dB.
  std::vector<double> tvgDb;

  /// \brief The gain applied, dB: fixed, or chosen so the median echo
  /// sits 24 dB below full scale, unless that would clip more than the
  /// brightest 0.1 % of echoes
  /// sit at full scale.
  double gainDb{0.0};

  std::vector<double> ranges;
  std::vector<double> azimuths;
};

/// \brief Time-varied gain at a range, dB.
double TimeVariedGain(const SonarConfig &_config, double _alpha,
  double _range);

/// \brief Automatic gain smoothed across pings: it moves toward each
/// ping's own gain with the configured time constant, so one bright target
/// does not darken the whole line it appears in.
class GainTracker
{
  /// \brief Take one ping's gain and return the gain to apply.
  /// \param[in] _pingGain The gain that would put this ping's median
  /// echoes at full scale, dB.
  /// \param[in] _dt Time since the previous ping, s.
  /// \param[in] _timeConstant Time constant, s; 0 follows each ping.
  public: double Update(double _pingGain, double _dt, double _timeConstant);

  private: bool started{false};
  private: double gain{0.0};
};

/// \brief Turn echo intensity into levels: time-varied gain, gain, and the
/// noise floor (an exponential draw per cell, so noise speckles too). Noise
/// is added after gain, so it sits at the noise floor below full scale at
/// every range. With automatic gain and a tracker, the gain is smoothed
/// across pings.
Levels ReadLevels(const BeamIntensity &_intensity, const SonarConfig &_config,
  double _alpha, std::uint64_t _ping, GainTracker *_tracker = nullptr,
  double _dt = 0.0);

/// \brief The imaging sonar's matrix: dynamicRangeDb below full scale maps
/// to 1, full scale to 255, and anything below the window to 0. No
/// thresholding or color map: those are display choices.
struct ImagingFrame
{
  std::size_t beams{0};
  std::size_t samples{0};

  /// \brief Beam-major, 8 bits.
  std::vector<std::uint8_t> data;

  std::vector<double> ranges;
  std::vector<double> azimuths;
  double gainDb{0.0};
};

ImagingFrame ReadImaging(const Levels &_levels, const SonarConfig &_config);

/// \brief One sidescan line: every sample in dB re full scale, with the
/// time-varied gain still in; subtract tvgDb to get the received level.
struct SidescanLine
{
  std::vector<float> db;
  std::vector<double> tvgDb;
  std::vector<double> ranges;
  double gainDb{0.0};
};

SidescanLine ReadSidescan(const Levels &_levels);

/// \brief Flags on a sounding, as marine_acoustic_msgs/DetectionFlag.
enum class SoundingFlag : std::uint8_t
{
  kOk = 0,
  kBadSonar = 1,
  kBadFilter = 2,
};

/// \brief One detection of a profiling sonar.
struct Sounding
{
  /// \brief Angle across the fan, rad.
  double angle{0.0};

  /// \brief Range the sonar reports, m, with its assumed sound speed.
  double range{0.0};

  /// \brief Two-way travel time, s.
  double travelTime{0.0};

  /// \brief Level above the noise floor, dB; NaN where the sonar reports
  /// no intensity (angle of arrival).
  double levelDb{0.0};

  SoundingFlag flag{SoundingFlag::kOk};
};

/// \brief Bottom detection: the soundings of one ping. Beamformed: one per
/// beam where the strongest sample clears the threshold, with soundings
/// inconsistent with their neighbors flagged. Angle of arrival: the fan is
/// split into maxSoundings equal sectors, and each sector that clears the
/// threshold reports its strongest echo, with angle noise and no intensity.
std::vector<Sounding> ReadProfiling(const Levels &_levels,
  const SonarConfig &_config, double _assumedSoundSpeed, std::uint64_t _ping);
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_READOUT_HH_
