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
#ifndef GZ_SIM_COMPONENTS_WAVECOUPLING_HH_
#define GZ_SIM_COMPONENTS_WAVECOUPLING_HH_

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief A tag on the world entity of a world whose physics follows the
  /// waves: the wave buoyancy (gz_waves_buoyancy) floats its vehicles on the
  /// wave field, and sets it. A system that damps against the water, such
  /// as gz_hydrodynamics, adds the waves' water velocity only where it is.
  /// Without it, vehicles float on flat water and the waves are drawn only.
  ///
  /// A tag and nothing else: the still water level stays where the world
  /// already keeps it, in its ocean current recipe. It lives in the core so
  /// that a reader depends on the core alone; components.cc anchors its
  /// registration in the core library, so the GUI process knows it too.
  using WaveCoupling = Component<NoData, class WaveCouplingTag>;

  GZ_SIM_REGISTER_COMPONENT("gz_maritime_components.WaveCoupling",
                            WaveCoupling)
}
}
}  // namespace gz::sim

#endif  // GZ_SIM_COMPONENTS_WAVECOUPLING_HH_
