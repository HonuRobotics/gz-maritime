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
#ifndef GZ_SIM_WATER_OPTICS_WATEROPTICSSAMPLER_HH_
#define GZ_SIM_WATER_OPTICS_WATEROPTICSSAMPLER_HH_

#include <chrono>
#include <memory>

#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "gz/sim/water_optics/WaterOptics.hh"

namespace gz::sim::water_optics
{
class WaterOpticsSamplerPrivate;

/// \brief Asks the world's water what it does to light, at any point.
///
/// A system keeps one sampler, calls Sync once per step (or per frame),
/// then asks as often as it likes. Sync reads the WaterOptics recipe and,
/// when the world has one, the wave field, so depth is measured below the
/// local wave surface, not below a flat level. A world without the water
/// optics system has clear water: nothing attenuates, there is no veil, and
/// the daylight does not fade.
class WaterOpticsSampler
{
  /// \brief Constructor.
  public: WaterOpticsSampler();

  /// \brief Destructor.
  public: ~WaterOpticsSampler();

  /// \brief Catch up with the world.
  /// \param[in] _ecm The entity component manager.
  /// \return True if the world has water optics.
  public: bool Sync(const EntityComponentManager &_ecm);

  /// \brief Whether the last Sync found water optics.
  /// \return True if there are.
  public: bool Valid() const;

  /// \brief The optics, clear water if there are none.
  /// \return The optics.
  public: const WaterOpticsData &Data() const;

  /// \brief Depth below the local water surface: the still water level plus
  /// the wave field's elevation above the point, minus the point's z.
  /// Negative above the surface.
  /// \param[in] _position Point, world frame, m.
  /// \param[in] _time Simulation time.
  /// \return Depth, m.
  public: double Depth(const math::Vector3d &_position,
                       const std::chrono::steady_clock::duration &_time) const;

  /// \brief The fraction of the surface daylight left at a depth, per
  /// channel: exp(-K_d depth). One at or above the surface.
  /// \param[in] _depth Depth, m.
  /// \return The fraction.
  public: Rgb Daylight(double _depth) const;

  /// \brief The veil, B-infinity, at a depth, per channel, as a fraction of
  /// the surface daylight: daylight veil exp(-K_d depth). Zero above the
  /// surface, and at night.
  /// \param[in] _depth Depth, m.
  /// \return The veil.
  public: Rgb Veil(double _depth) const;

  /// \brief The fraction of an object's light that crosses a range of
  /// water, per channel: exp(-beta^D range).
  /// \param[in] _range Range, m.
  /// \return The fraction.
  public: Rgb Transmittance(double _range) const;

  /// \brief How much of the veil has built up over a range, per channel:
  /// 1 - exp(-beta^B range).
  /// \param[in] _range Range, m.
  /// \return The fraction.
  public: Rgb VeilBuildUp(double _range) const;

  /// \brief Private data.
  private: std::unique_ptr<WaterOpticsSamplerPrivate> dataPtr;
};
}  // namespace gz::sim::water_optics

#endif  // GZ_SIM_WATER_OPTICS_WATEROPTICSSAMPLER_HH_
