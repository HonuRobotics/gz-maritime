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
#ifndef GZ_SIM_MARITIME_GUI_WINDSOCK_HH_
#define GZ_SIM_MARITIME_GUI_WINDSOCK_HH_

#include <cstdint>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>

/// \brief A wind sock, drawn: a pole, and a striped cone on top of it that
/// points where the wind goes and droops in light air, the way a real one
/// reads at a glance.
namespace gz::sim::maritime_gui
{
/// \brief Wind speed at which a sock flies straight out, m/s: 15 knots, the
/// common rating of an airfield sock.
constexpr double kSockFullSpeed{7.7};

/// \brief How far a sock droops below the horizontal in still air, degrees.
constexpr double kSockMaxDroop{80.0};

/// \brief The direction a sock's cone points along, from its mouth to its
/// tail: downwind, tipped down by the droop.
/// \param[in] _wind The wind at the sock, world frame.
/// \param[in] _fallback The horizontal direction to point along in still
/// air, where the wind gives none.
/// \return A unit vector.
math::Vector3d SockAxis(const math::Vector3d &_wind,
                        const math::Vector3d &_fallback);

/// \brief The sock as markers, TRIANGLE_LISTs seen from both sides: id
/// `_firstId` the grey pole, `_firstId + 1` the orange bands and
/// `_firstId + 2` the white ones.
/// \param[in] _ns Namespace.
/// \param[in] _firstId Id of the pole, not 0.
/// \param[in] _base The pole's foot.
/// \param[in] _height The pole's height, m; the cone hangs at its top.
/// \param[in] _axis The cone's axis, from SockAxis.
/// \param[in] _length The cone's length, m; its mouth is a fifth as wide.
/// \return Three markers.
std::vector<msgs::Marker> WindSockMarkers(const std::string &_ns,
    std::uint64_t _firstId, const math::Vector3d &_base, double _height,
    const math::Vector3d &_axis, double _length = 1.2);
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_WINDSOCK_HH_
