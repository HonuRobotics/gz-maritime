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
#ifndef GZ_SIM_COMPONENTS_WATEROPTICS_HH_
#define GZ_SIM_COMPONENTS_WATEROPTICS_HH_

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include "gz/sim/water_optics/WaterOptics.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief The optics of the world's water, on the world entity. Written by
  /// the water optics system; read through water_optics::WaterOpticsSampler
  /// by any system that sees through the water.
  using WaterOptics =
      Component<water_optics::WaterOpticsData, class WaterOpticsTag>;

  GZ_SIM_REGISTER_COMPONENT("gz_maritime_components.WaterOptics",
                            WaterOptics)
}
}
}  // namespace gz::sim

#endif  // GZ_SIM_COMPONENTS_WATEROPTICS_HH_
