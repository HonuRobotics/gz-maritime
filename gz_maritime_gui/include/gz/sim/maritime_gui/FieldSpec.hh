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
#ifndef GZ_SIM_MARITIME_GUI_FIELDSPEC_HH_
#define GZ_SIM_MARITIME_GUI_FIELDSPEC_HH_

#include <string>

#include "gz/sim/maritime_gui/Flow.hh"

namespace gz::sim::maritime_gui
{
/// \brief What the panels know of one field, defined once: its name on its
/// topics, how fast the controls may steer it, and how it is drawn by
/// default.
struct FieldSpec
{
  /// \brief The field's name on its topics: `wind` or `ocean_current`.
  public: std::string name;

  /// \brief Fastest change of speed the controls send, m/s per second.
  public: double speedRate{1.0};

  /// \brief Fastest change of direction the controls send, degrees per
  /// second.
  public: double turnRate{90.0};

  /// \brief Colour by speed, and its default maximum.
  public: Colormap colormap{Colormap::Water(1.0)};

  /// \brief m of arrow per m/s.
  public: double scale{1.0};

  /// \brief The trails' defaults; the region and the plane come from the
  /// grid and the field.
  public: ParticleField::Settings trails;

  /// \brief A rate limiter for the controls, at the field's rates.
  /// \return The steering.
  public: Steering MakeSteering() const;
};

/// \brief The ocean current: steered at 0.5 m/s and 90 degrees a second;
/// water colours up to 1 m/s; arrows of 4 m per m/s, so a 0.5 m/s current is
/// 2 m; trails sped up eight times, so a slow current still visibly flows.
/// \return The spec.
FieldSpec OceanCurrentSpec();

/// \brief The wind: steered at 5 m/s and 90 degrees a second; air colours up
/// to 20 m/s; arrows of 0.3 m per m/s, so a 10 m/s wind is 3 m, within the
/// default 5 m spacing; trails at the wind's own speed.
/// \return The spec.
FieldSpec WindSpec();
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_FIELDSPEC_HH_
