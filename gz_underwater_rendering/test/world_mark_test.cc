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

// The world's mark, case by case: which of a plugin's instances it names.
// A removed instance cannot take the mark off; the next instance must take
// it over whether its first update comes before or after it draws.
#include <gtest/gtest.h>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>

#include "../src/systems/WorldMark.hh"

using namespace gz;
using namespace sim;
using maritime::UpdateWorldMark;

namespace
{
/// \brief A world and two plugin entities.
struct World
{
  World()
  {
    this->world = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->world, components::World());
    this->a = this->ecm.CreateEntity();
    this->b = this->ecm.CreateEntity();
  }

  /// \brief The entity the mark names, or none.
  Entity Mark()
  {
    const auto *m =
        this->ecm.Component<components::UnderwaterRendering>(this->world);
    return m ? m->Data() : kNullEntity;
  }

  EntityComponentManager ecm;
  Entity world, a, b;
};
}  // namespace

//////////////////////////////////////////////////
TEST(WorldMark, OnWhileDrawingOffWhenNot)
{
  World w;
  UpdateWorldMark(w.ecm, w.a, false);
  EXPECT_EQ(kNullEntity, w.Mark());
  UpdateWorldMark(w.ecm, w.a, true);
  EXPECT_EQ(w.a, w.Mark());
  UpdateWorldMark(w.ecm, w.a, false);
  EXPECT_EQ(kNullEntity, w.Mark());
}

//////////////////////////////////////////////////
// An instance's model removed while it drew, and a new one: the mark is the
// new one's as soon as it draws, even if its first update comes after.
TEST(WorldMark, TakenOverFromARemovedInstance)
{
  World w;
  UpdateWorldMark(w.ecm, w.a, true);
  w.ecm.RequestRemoveEntity(w.a);
  w.ecm.ProcessRemoveEntityRequests();
  ASSERT_FALSE(w.ecm.HasEntity(w.a));
  ASSERT_EQ(w.a, w.Mark()) << "nobody left to take it off";

  UpdateWorldMark(w.ecm, w.b, true);
  EXPECT_EQ(w.b, w.Mark());
}

//////////////////////////////////////////////////
// Two live instances both drawing: the mark stays with the first, and is
// not passed back and forth, which would send the state on every step.
TEST(WorldMark, TwoLiveInstancesDoNotTradeIt)
{
  World w;
  UpdateWorldMark(w.ecm, w.a, true);
  w.ecm.SetAllComponentsUnchanged();
  for (int i = 0; i < 10; ++i)
  {
    UpdateWorldMark(w.ecm, w.b, true);
    UpdateWorldMark(w.ecm, w.a, true);
    EXPECT_EQ(w.a, w.Mark()) << "step " << i;
  }
  EXPECT_EQ(ComponentState::NoChange,
            w.ecm.ComponentState(w.world,
                components::UnderwaterRendering::typeId))
      << "the mark changed hands";
}

//////////////////////////////////////////////////
// Not drawing yet: a mark whose entity is gone comes off, one of another
// live instance stays.
TEST(WorldMark, NotDrawing)
{
  World w;
  UpdateWorldMark(w.ecm, w.a, true);
  UpdateWorldMark(w.ecm, w.b, false);
  EXPECT_EQ(w.a, w.Mark()) << "took off another live instance's mark";

  w.ecm.RequestRemoveEntity(w.a);
  w.ecm.ProcessRemoveEntityRequests();
  UpdateWorldMark(w.ecm, w.b, false);
  EXPECT_EQ(kNullEntity, w.Mark());
}
