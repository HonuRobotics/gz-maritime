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
#ifndef GZ_SIM_OCEAN_CURRENT_OCEANCURRENTSAMPLER_HH_
#define GZ_SIM_OCEAN_CURRENT_OCEANCURRENTSAMPLER_HH_

#include <chrono>
#include <memory>

#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace gz::sim::ocean_current
{
class OceanCurrentSamplerPrivate;

/// \brief Asks the world's ocean current at any point, whatever model is
/// behind it.
///
/// A system keeps one sampler, calls Sync once per step, then At as often as
/// it likes. Sync reads the OceanCurrentfield recipe from the world and
/// rebuilds a private copy of the model when the recipe changes, and reads
/// the world's spherical coordinates so north is the world's north. At takes
/// and returns world frame vectors. A sampler made at any step, by a system
/// loaded with the world or one spawned long after, reads the same recipe.
/// It is a point query and nothing more: a consumer that spans a gradient
/// integrates over its own extent with repeated queries.
class OceanCurrentSampler
{
  /// \brief Constructor.
  public: OceanCurrentSampler();

  /// \brief Destructor.
  public: ~OceanCurrentSampler();

  /// \brief Catch up with the world's recipe.
  /// \param[in] _ecm The entity component manager.
  /// \return True if the world has an ocean current.
  public: bool Sync(const EntityComponentManager &_ecm);

  /// \brief Whether the last Sync found a current.
  /// \return True if there is one.
  public: bool Valid() const;

  /// \brief Whether the current changes with time at a fixed point.
  /// \return True if the model varies in time.
  public: bool TimeVarying() const;

  /// \brief The current at a point.
  /// \param[in] _position Point, world frame, m.
  /// \param[in] _time Simulation time.
  /// \return The velocity of the water, world frame, m/s; zero without a
  /// current.
  public: math::Vector3d At(const math::Vector3d &_position,
                            const std::chrono::steady_clock::duration &_time)
                            const;

  /// \brief Private data.
  private: std::unique_ptr<OceanCurrentSamplerPrivate> dataPtr;
};

/// \brief The world's ocean current at a point, in one call. Handy for
/// occasional queries; a system that asks every step should keep its own
/// OceanCurrentSampler.
/// \param[in] _ecm The entity component manager.
/// \param[in] _position Point, world frame, m.
/// \param[in] _time Simulation time.
/// \return The velocity of the water, world frame, m/s; zero without a
/// current.
math::Vector3d OceanCurrentAt(const EntityComponentManager &_ecm,
                              const math::Vector3d &_position,
                              const std::chrono::steady_clock::duration &_time);
}  // namespace gz::sim::ocean_current

#endif  // GZ_SIM_OCEAN_CURRENT_OCEANCURRENTSAMPLER_HH_
