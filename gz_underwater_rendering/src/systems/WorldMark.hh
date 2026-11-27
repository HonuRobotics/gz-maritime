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
#ifndef GZ_UNDERWATER_RENDERING_WORLDMARK_HH_
#define GZ_UNDERWATER_RENDERING_WORLDMARK_HH_

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "gz/sim/components/UnderwaterRendering.hh"

namespace gz::sim::maritime
{
/// \brief Keep the world's mark, which tells the reference cameras the
/// renderer draws the water, true for one plugin instance.
///
/// While it draws, the mark names it, unless it names another live
/// instance: it takes over a mark whose entity is gone, such as an instance
/// whose model was removed and that could not take the mark off, but two
/// live instances do not pass it back and forth (each change is sent on
/// the next state). While it does not draw, it takes off a mark of its
/// own, or one whose entity is gone; it leaves one of another live
/// instance.
/// \param[in,out] _ecm The entity component manager.
/// \param[in] _owner The plugin's entity.
/// \param[in] _drawing Whether this instance draws the water.
inline void UpdateWorldMark(EntityComponentManager &_ecm, Entity _owner,
                            bool _drawing)
{
  const Entity world = worldEntity(_ecm);
  if (world == kNullEntity)
    return;
  auto *mark = _ecm.Component<components::UnderwaterRendering>(world);
  if (_drawing && !mark)
  {
    _ecm.CreateComponent(world, components::UnderwaterRendering(_owner));
  }
  else if (_drawing && mark->Data() != _owner &&
           !_ecm.HasEntity(mark->Data()))
  {
    mark->Data() = _owner;
    _ecm.SetChanged(world, components::UnderwaterRendering::typeId,
                    ComponentState::OneTimeChange);
  }
  else if (!_drawing && mark &&
           (mark->Data() == _owner || !_ecm.HasEntity(mark->Data())))
  {
    _ecm.RemoveComponent<components::UnderwaterRendering>(world);
  }
}
}  // namespace gz::sim::maritime

#endif  // GZ_UNDERWATER_RENDERING_WORLDMARK_HH_
