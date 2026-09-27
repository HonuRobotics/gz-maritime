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
#ifndef GZ_SIM_WIND_STANDARDMODEL_HH_
#define GZ_SIM_WIND_STANDARDMODEL_HH_

#include <random>
#include <string_view>
#include <vector>

#include "gz/sim/wind/WindModel.hh"

namespace gz::sim::wind
{
/// \brief A random signal as a sum of sinusoids, with a Lorentzian spectrum:
/// zero mean, a given standard deviation, and an autocorrelation that falls
/// as exp(-lag / T), like a first order Gauss Markov process, but a function
/// of time alone, so any copy built from the same seed agrees at any time.
class Gust
{
  /// \brief Build the components.
  /// \param[in] _sigma Standard deviation; 0 is no gust.
  /// \param[in] _time Correlation time T, s.
  /// \param[in,out] _rng Draws the phases.
  public: void Build(double _sigma, double _time, std::mt19937 &_rng);

  /// \brief The signal at a time.
  /// \param[in] _t Time, s.
  /// \return Its value.
  public: double At(double _t) const;

  /// \brief Whether there is a gust at all.
  /// \return True if it has components.
  public: bool Active() const { return !this->amplitude.empty(); }

  /// \brief Amplitude of each component.
  private: std::vector<double> amplitude;

  /// \brief Angular frequency of each component, rad/s.
  private: std::vector<double> omega;

  /// \brief Phase of each component, rad.
  private: std::vector<double> phase;
};

/// \brief The standard wind: a mean speed and direction, with optional gusts
/// on both, carried along with the mean wind.
class StandardModel : public IWindModel
{
  // Documentation inherited.
  public: void SetParameters(const WindParameters &_params) override;

  // Documentation inherited.
  public: math::Vector3d Velocity(const math::Vector3d &_enu,
                                  double _time) const override;

  // Documentation inherited.
  public: bool TimeVarying() const override;

  // Documentation inherited.
  public: std::string_view Kind() const override { return "standard"; }

  /// \brief Parameters.
  private: WindParameters params;

  /// \brief Gusts on the speed, m/s.
  private: Gust speedGust;

  /// \brief Gusts on the direction, degrees.
  private: Gust directionGust;
};
}  // namespace gz::sim::wind

#endif  // GZ_SIM_WIND_STANDARDMODEL_HH_
