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
#ifndef GZ_MARITIME_SONAR_ENGINE_HH_
#define GZ_MARITIME_SONAR_ENGINE_HH_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Rays.hh"
#include "gz/maritime/sonar/Water.hh"

namespace gz::maritime::sonar
{
/// \brief The echo intensity of one ping in every beam and range sample, as
/// the sonar receives it. Each value is a power ratio (not decibels): the
/// echo intensity relative to that of a target of 1 m^2 backscattering
/// cross section, 1 m away on the beam's axis, with no absorption. For each
/// hit that is sigma(theta) A / R^4 times absorption and the beam pattern,
/// A being the area the ray covers; 10 log10 of the value is the echo level
/// in dB. Readout.hh then turns it into what the sonar plugin publishes:
/// time-varied gain and gain, then an 8-bit image (imaging) or a line of dB
/// levels (sidescan) in a SonarImage message, or bottom-detected soundings
/// in a SonarDetections message (profiling).
struct BeamIntensity
{
  std::size_t beams{0};
  std::size_t samples{0};

  /// \brief Beam-major: data[beam * samples + sample].
  std::vector<double> data;

  /// \brief For each cell, the label of the surface that sent it the most
  /// energy; 0 where nothing did. Ground truth, never sensor output.
  std::vector<std::uint32_t> labels;

  /// \brief Center range of each sample, m, as the sonar reports it (with
  /// its assumed sound speed).
  std::vector<double> ranges;

  /// \brief Azimuth of each beam, rad.
  std::vector<double> azimuths;

  double &At(std::size_t _beam, std::size_t _sample)
  {
    return this->data[_beam * this->samples + _sample];
  }

  double At(std::size_t _beam, std::size_t _sample) const
  {
    return this->data[_beam * this->samples + _sample];
  }
};

/// \brief Intensity weight of a beam at an angle from its axis: a sinc^2
/// main lobe whose -3 dB width is the beam width, with its sidelobes set to
/// the configured level.
/// \param[in] _config The sonar.
/// \param[in] _offset Angle from the beam's axis, rad.
/// \return The weight, 1 on the axis.
double HorizontalPattern(const SonarConfig &_config, double _offset);

/// \brief Intensity weight of the vertical beam at an elevation: Gaussian,
/// -3 dB at half the vertical field of view.
/// \param[in] _config The sonar.
/// \param[in] _elevation Elevation, rad.
/// \return The weight, 1 on the axis.
double VerticalPattern(const SonarConfig &_config, double _elevation);

/// \brief The sonar-equation model: turns the ray hits of one ping into
/// echo intensity per beam and range sample.
///
/// Each hit at range R and incidence theta sends back
///   b_h b_v sigma(theta) dOmega / (R^2 cos theta) 10^(-2 alpha R / 10),
/// the sonar equation for a patch of area R^2 dOmega / cos theta. The energy
/// is added (not averaged) into every beam the horizontal pattern reaches,
/// spread over the range samples that the patch's footprint and the pulse
/// cover, at the range the sonar computes with its assumed sound speed.
/// The result is the mean intensity: speckle is drawn in the readout
/// (ReadLevels), on the echo and the noise together.
///
/// The engine keeps its own worker threads, started at construction and
/// joined when it is destroyed, and its per-ping buffers and beam weights,
/// so a ping starts no threads and allocates little. Pings on one engine
/// run one at a time; two engines run independently.
class Engine
{
  /// \brief An engine for one sonar in a body of water.
  /// \param[in] _config The sonar.
  /// \param[in] _water The water.
  public: Engine(const SonarConfig &_config, const Water &_water);

  /// \brief Joins the engine's threads.
  public: ~Engine();

  public: Engine(const Engine &) = delete;
  public: Engine &operator=(const Engine &) = delete;
  public: Engine(Engine &&) noexcept;
  public: Engine &operator=(Engine &&) noexcept;

  /// \brief One ping. Safe to call from any thread; concurrent calls on
  /// one engine wait for each other.
  /// \param[in] _hits What each ray of the fan found.
  /// \param[in] _ping The ping number. Unused since speckle moved to the
  /// readout; kept so callers need not change.
  /// \return Echo intensity per beam and sample.
  public: BeamIntensity Ping(const std::vector<RayHit> &_hits,
              std::uint64_t _ping) const;

  /// \brief Threads a ping runs on, the caller's included.
  /// \return The number of threads.
  public: std::size_t Threads() const;

  /// \brief The configuration.
  /// \return The configuration.
  public: const SonarConfig &Config() const;

  /// \brief The water.
  /// \return The water.
  public: const Water &WaterProperties() const;

  /// \brief Absorption at the sonar's frequency, dB per meter.
  /// \return The absorption, dB/m.
  public: double Absorption() const;

  private: SonarConfig config;
  private: Water water;
  private: double alpha{0.0};

  /// \brief The thread pool, the beam weights and the scratch buffers.
  private: struct Implementation;
  private: std::unique_ptr<Implementation> impl;
};
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_ENGINE_HH_
