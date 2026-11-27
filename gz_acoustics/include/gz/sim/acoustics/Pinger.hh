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
#ifndef GZ_SIM_ACOUSTICS_PINGER_HH_
#define GZ_SIM_ACOUSTICS_PINGER_HH_

#include <cstdint>
#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <string>

#include <gz/math/Pose3.hh>

namespace gz::sim::acoustics
{
/// \brief Every number a pinger is set with, as (member, name). The name is
/// the key in the plugin's SDF and on the pinger's `set` topic; one table
/// drives the serialization and the parsing, so there is no second list to
/// keep in sync. All are doubles.
///
/// * frequency: Hz of the ping, 25 000 to 40 000 on the RoboNation beacons.
/// * period: s between two pings, 0.5 to 2 on the real beacons.
/// * offset: s into the period at which this pinger pings, so pingers on one
///   period ping in turn.
/// * reach: m, the farthest a receiver hears it; 0 is no limit.
/// * pulse_length: s, how long a ping lasts at a receiver; two pings in a
///   receiver's band that arrive closer than this give invalid readings.
///   0 turns the overlap off.
#define GZ_ACOUSTICS_PINGER_TABLE(X)      \
  X(frequency,    "frequency")            \
  X(period,       "period")               \
  X(offset,       "offset")               \
  X(reach,        "reach")                \
  X(pulse_length, "pulse_length")

/// \brief A pinger, as the world sees it: what it is set to, which the
/// acoustics system reads to know when it pings and who can hear it.
struct PingerData
{
  /// \brief Name, unique in the world; scopes its `set` topic.
  std::string name;

  /// \brief Whether it pings at all.
  bool active{true};

  /// \brief Where it sits in the frame of the link that carries it.
  math::Pose3d pose;

  /// \brief Hz of the ping.
  double frequency{30000.0};

  /// \brief s between two pings.
  double period{1.0};

  /// \brief s into the period of its first ping.
  double offset{0.0};

  /// \brief m, farthest a receiver hears it; 0 is no limit.
  double reach{0.0};

  /// \brief s, how long one ping lasts at a receiver.
  double pulse_length{0.004};

  /// \brief Goes up on every change, so a reader knows it changed.
  std::uint64_t generation{0};
};

/// \brief Write a pinger at full precision.
/// \param[in] _os Stream.
/// \param[in] _d Pinger.
/// \return The stream.
inline std::ostream &operator<<(std::ostream &_os, const PingerData &_d)
{
  const auto old = _os.precision(std::numeric_limits<double>::max_digits10);
  // The pose element by element: Pose3's own stream operators round.
  const auto &pos = _d.pose.Pos();
  const auto rpy = _d.pose.Rot().Euler();
  _os << std::quoted(_d.name) << ' ' << _d.generation << ' ' << _d.active
      << ' ' << pos.X() << ' ' << pos.Y() << ' ' << pos.Z()
      << ' ' << rpy.X() << ' ' << rpy.Y() << ' ' << rpy.Z();
#define GZ_ACOUSTICS_WR(m, name) _os << ' ' << _d.m;
  GZ_ACOUSTICS_PINGER_TABLE(GZ_ACOUSTICS_WR)
#undef GZ_ACOUSTICS_WR
  _os.precision(old);
  return _os;
}

/// \brief Read a pinger written by operator<<.
/// \param[in] _is Stream.
/// \param[out] _d Pinger.
/// \return The stream.
inline std::istream &operator>>(std::istream &_is, PingerData &_d)
{
  double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
  _is >> std::quoted(_d.name) >> _d.generation >> _d.active
      >> x >> y >> z >> roll >> pitch >> yaw;
  _d.pose = math::Pose3d(x, y, z, roll, pitch, yaw);
#define GZ_ACOUSTICS_RD(m, name) _is >> _d.m;
  GZ_ACOUSTICS_PINGER_TABLE(GZ_ACOUSTICS_RD)
#undef GZ_ACOUSTICS_RD
  return _is;
}

/// \brief Set one number of a pinger by its name.
/// \param[in,out] _p Pinger.
/// \param[in] _name A name from the table, or "active" (non zero is on).
/// \param[in] _value Value.
/// \return False if the name is unknown or the value out of range: a
/// frequency or a period must be positive, the rest not negative.
bool SetParameter(PingerData &_p, const std::string &_name, double _value);
}  // namespace gz::sim::acoustics

#endif  // GZ_SIM_ACOUSTICS_PINGER_HH_
