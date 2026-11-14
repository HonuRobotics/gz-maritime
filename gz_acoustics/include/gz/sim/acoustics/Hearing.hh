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
#ifndef GZ_SIM_ACOUSTICS_HEARING_HH_
#define GZ_SIM_ACOUSTICS_HEARING_HH_

#include <chrono>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/sim/acoustics/Pinger.hh"

/// \file
/// \brief The rules of the behavioural acoustics, apart from any world, so
/// they can be tested alone: when a pinger pings, where a receiver hears it
/// from, and whether it hears it at all. No acoustics: no propagation model,
/// only geometry, a delay and a few switchable rules.

namespace gz::sim::acoustics
{
/// \brief Simulation time.
using Duration = std::chrono::steady_clock::duration;

/// \brief Where a receiver hears a ping from, in the receiver's frame.
struct Direction
{
  /// \brief rad, counter clockwise from the receiver's x, in (-pi, pi].
  double bearing{0.0};

  /// \brief rad, above the receiver's xy plane, positive up, in
  /// [-pi/2, pi/2].
  double elevation{0.0};

  /// \brief m, straight line distance.
  double range{0.0};
};

/// \brief The times a pinger pings in (_after, _upTo]: its offset, then
/// every period, from time zero. An inactive pinger never pings.
/// \param[in] _pinger The pinger.
/// \param[in] _after Exclusive start.
/// \param[in] _upTo Inclusive end.
/// \return The times, in order.
std::vector<Duration> EmissionTimes(const PingerData &_pinger,
                                    const Duration &_after,
                                    const Duration &_upTo);

/// \brief Where a source is seen from a receiver.
/// \param[in] _receiver World pose of the receiver.
/// \param[in] _source World position of the source.
/// \return Bearing, elevation and range in the receiver's frame.
Direction Measure(const math::Pose3d &_receiver,
                  const math::Vector3d &_source);

/// \brief Whether a frequency is inside a receiver's band.
/// \param[in] _frequency Hz of the ping.
/// \param[in] _centre Hz, the band's centre; 0 or less hears everything.
/// \param[in] _bandwidth Hz, the band's full width.
/// \return True if heard.
bool InBand(double _frequency, double _centre, double _bandwidth);

/// \brief The chance a ping at a distance is lost: none up to the start of
/// the dropout zone, rising in a straight line to certain at the reach.
/// \param[in] _distance m.
/// \param[in] _reach m; 0 is no limit, and nothing is lost.
/// \param[in] _start Fraction of the reach where losses start, in [0, 1];
/// 1 (or more) turns the dropout off.
/// \return Probability in [0, 1].
double DropoutProbability(double _distance, double _reach, double _start);

/// \brief How loud a ping is at a distance, in dB relative to 1 m: the
/// spherical spreading loss, -20 log10(distance), and 0 within a metre.
/// \param[in] _distance m.
/// \return dB, zero or negative.
double Amplitude(double _distance);

/// \brief The time sound takes over a distance.
/// \param[in] _distance m.
/// \param[in] _speedOfSound m/s; 0 or less arrives at once.
/// \return The delay.
Duration TravelTime(double _distance, double _speedOfSound);

/// \brief Wrap an angle into (-pi, pi].
/// \param[in] _angle rad.
/// \return rad.
double WrapAngle(double _angle);
}  // namespace gz::sim::acoustics

#endif  // GZ_SIM_ACOUSTICS_HEARING_HH_
