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
#ifndef GZ_SIM_COMPONENTS_WINDFIELD_HH_
#define GZ_SIM_COMPONENTS_WINDFIELD_HH_

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include "gz/sim/wind/Windfield.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief The world's wind, as a recipe, on the world entity. Written by the
  /// wind system; read through wind::WindSampler by anything that needs the
  /// wind at a point.
  using Windfield = Component<wind::WindfieldData, class WindfieldTag>;

  GZ_SIM_REGISTER_COMPONENT("gz_maritime_components.Windfield", Windfield)
}
}
}  // namespace gz::sim

#endif  // GZ_SIM_COMPONENTS_WINDFIELD_HH_
