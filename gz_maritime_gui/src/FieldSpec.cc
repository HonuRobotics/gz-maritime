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
#include "gz/sim/maritime_gui/FieldSpec.hh"

namespace gz::sim::maritime_gui
{
//////////////////////////////////////////////////
Steering FieldSpec::MakeSteering() const
{
  return Steering(this->speedRate, this->turnRate);
}

//////////////////////////////////////////////////
FieldSpec OceanCurrentSpec()
{
  FieldSpec s;
  s.name = "ocean_current";
  s.speedRate = 0.5;
  s.turnRate = 90.0;
  s.colormap = Colormap::Water(1.0);
  s.scale = 4.0;
  s.trails.count = 400;
  s.trails.trailLength = 8;
  s.trails.lifetime = 6.0;
  s.trails.speedup = 8.0;
  return s;
}

//////////////////////////////////////////////////
FieldSpec WindSpec()
{
  FieldSpec s;
  s.name = "wind";
  s.speedRate = 5.0;
  s.turnRate = 90.0;
  s.colormap = Colormap::Air(20.0);
  s.scale = 0.3;
  s.trails.count = 400;
  s.trails.trailLength = 8;
  s.trails.lifetime = 4.0;
  s.trails.speedup = 1.0;
  return s;
}
}  // namespace gz::sim::maritime_gui
