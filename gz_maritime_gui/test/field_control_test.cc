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
// A panel's control of a field, without a GUI or a server: a fake
// subscriber on the field's `set` topic stands for the server, and a fake
// state service for the world a panel added from the menu asks.

#include <gtest/gtest.h>

#include <gz/msgs/param.pb.h>
#include <gz/msgs/serialized_map.pb.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/OceanCurrentfield.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/maritime_gui/FieldControl.hh"
#include "gz/sim/maritime_gui/FieldSpec.hh"
#include "gz/sim/maritime_gui/Fields.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;
using std::chrono::milliseconds;

namespace
{
/// \brief An ECM with a world of that name, and an ocean current recipe if
/// asked for.
/// \param[in,out] _ecm The ECM.
/// \param[in] _world The world's name.
/// \param[in] _recipe Whether to add the recipe.
void World(EntityComponentManager &_ecm, const std::string &_world,
           bool _recipe)
{
  const Entity world = _ecm.CreateEntity();
  _ecm.CreateComponent(world, components::World());
  _ecm.CreateComponent(world, components::Name(_world));
  if (_recipe)
  {
    ocean_current::OceanCurrentfieldData d;
    d.generation = 4;
    d.params.speed = 0.25;
    d.params.direction = 135.0;
    _ecm.CreateComponent(world, components::OceanCurrentfield(d));
  }
}

/// \brief Stands for the server: keeps what comes on a world's ocean
/// current `set` topic.
class FakeServer
{
  /// \brief Constructor.
  /// \param[in] _world The world's name.
  public: explicit FakeServer(const std::string &_world)
  {
    this->node.Subscribe("/world/" + _world + "/ocean_current/set",
        &FakeServer::OnSet, this);
  }

  /// \brief A message came.
  /// \param[in] _msg The message.
  public: void OnSet(const msgs::Param &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->received.push_back(_msg);
  }

  /// \brief The messages so far.
  /// \return A copy.
  public: std::vector<msgs::Param> Received()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->received;
  }

  /// \brief Wait for a number of messages.
  /// \param[in] _count How many.
  /// \return True if they came within a few seconds.
  public: bool WaitFor(std::size_t _count)
  {
    for (int i = 0; i < 250 && this->Received().size() < _count; ++i)
      std::this_thread::sleep_for(milliseconds(20));
    return this->Received().size() >= _count;
  }

  /// \brief The messages.
  private: std::vector<msgs::Param> received;

  /// \brief Guards them.
  private: std::mutex mutex;

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief A message setting the speed.
/// \param[in] _speed m/s.
/// \return The message.
msgs::Param Speed(double _speed)
{
  msgs::Param msg;
  auto &any = (*msg.mutable_params())["speed"];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(_speed);
  return msg;
}

/// \brief Send until the publisher sees the subscriber: discovery takes a
/// moment after the topic is advertised.
/// \param[in,out] _control The control.
/// \param[in] _msg The message.
/// \param[in] _generation The recipe's generation.
/// \return True once it was sent.
bool SendWhenHeard(FieldControl &_control, const msgs::Param &_msg,
                   std::uint64_t _generation)
{
  for (int i = 0; i < 250; ++i)
  {
    _control.Send(_msg, _generation);
    if (_control.Track(_generation) != ChangeTracker::State::kUnheard)
      return true;
    std::this_thread::sleep_for(milliseconds(20));
  }
  return false;
}
}  // namespace

/////////////////////////////////////////////////
/// With nothing subscribed, a change is not sent, and says so; before the
/// world's name is known there is no topic at all.
TEST(FieldControl, NothingSubscribed)
{
  FieldControl control(OceanCurrentSpec(), "test");
  control.Send(Speed(1.0), 4);
  EXPECT_EQ(ChangeTracker::State::kUnheard, control.Track(4))
      << "no world yet";

  EntityComponentManager ecm;
  World(ecm, "nobody_listens", true);
  control.Update(ecm);
  control.Send(Speed(1.0), 4);
  EXPECT_EQ(ChangeTracker::State::kUnheard, control.Track(4));
}

/////////////////////////////////////////////////
/// A change sent reaches the field's topic, and is applied when the
/// recipe's generation moves, or not applied when it does not within the
/// timeout.
TEST(FieldControl, SentAppliedOrNot)
{
  FakeServer server("sent");
  EntityComponentManager ecm;
  World(ecm, "sent", true);
  FieldControl control(OceanCurrentSpec(), "test");
  control.Update(ecm);

  ASSERT_TRUE(SendWhenHeard(control, Speed(1.0), 4));
  ASSERT_TRUE(server.WaitFor(1u));
  EXPECT_DOUBLE_EQ(1.0,
      server.Received().back().params().at("speed").double_value());
  const auto now = ChangeTracker::Clock::now();
  EXPECT_EQ(ChangeTracker::State::kPending, control.Track(4, now));
  EXPECT_EQ(ChangeTracker::State::kApplied, control.Track(5, now));

  // The next one is not applied: the generation stays.
  control.Send(Speed(-1.0), 5, now);
  EXPECT_EQ(ChangeTracker::State::kPending,
            control.Track(5, now + milliseconds(100)));
  EXPECT_EQ(ChangeTracker::State::kRefused,
            control.Track(5, now + std::chrono::seconds(6)));
}

/////////////////////////////////////////////////
/// Steering sends a step per period at the field's rate, untracked but the
/// last, which is tracked like a change from Apply; it stops, and says so,
/// when someone else moves the field, until the next Steer or Send.
TEST(FieldControl, Steering)
{
  FakeServer server("steer");
  EntityComponentManager ecm;
  World(ecm, "steer", true);
  FieldControl control(OceanCurrentSpec(), "test");
  control.Update(ecm);
  ASSERT_TRUE(SendWhenHeard(control, Speed(0.0), 4));
  ASSERT_TRUE(server.WaitFor(1u));

  // 0 to 0.1 m/s at 0.5 m/s a second: 0.05, then 0.1, which arrives.
  auto t = Steering::Clock::now();
  control.Steer(0.1, 0.0);
  EXPECT_TRUE(control.IsSteering());
  control.StepSteering(0.0, 0.0, 5, t);
  t += milliseconds(100);
  control.StepSteering(0.05, 0.0, 6, t);
  EXPECT_FALSE(control.IsSteering());
  ASSERT_TRUE(server.WaitFor(3u));
  const auto got = server.Received();
  EXPECT_NEAR(0.05, got[1].params().at("speed").double_value(), 1e-9);
  EXPECT_NEAR(0.1, got[2].params().at("speed").double_value(), 1e-9);
  EXPECT_EQ(ChangeTracker::State::kPending, control.Track(6, t))
      << "the last step is tracked";
  EXPECT_EQ(ChangeTracker::State::kApplied, control.Track(7, t));

  // Steering again, our first step lands, then someone sets 0.4 m/s.
  control.Steer(1.0, 0.0);
  t += milliseconds(100);
  control.StepSteering(0.1, 0.0, 7, t);
  t += milliseconds(100);
  control.StepSteering(0.15, 0.0, 8, t);
  EXPECT_FALSE(control.Yielded());
  t += milliseconds(100);
  control.StepSteering(0.4, 0.0, 9, t);
  EXPECT_TRUE(control.Yielded());
  EXPECT_FALSE(control.IsSteering());

  // Apply clears it.
  control.Send(Speed(0.5), 9, t);
  EXPECT_FALSE(control.Yielded());
}

/////////////////////////////////////////////////
/// A panel whose GUI's ECM has no recipe asks the world's state service,
/// once, after about a second of updates, and reads the recipe from the copy.
TEST(FieldControl, StateServiceFallback)
{
  EntityComponentManager world;
  World(world, "late", true);
  std::atomic<int> asked{0};
  transport::Node service;
  std::function<bool(msgs::SerializedStepMap &)> cb =
      [&](msgs::SerializedStepMap &_rep)
      {
        ++asked;
        world.State(*_rep.mutable_state(), {}, {}, true);
        return true;
      };
  ASSERT_TRUE(service.Advertise("/world/late/state", cb));

  EntityComponentManager gui;
  World(gui, "late", false);
  FieldControl control(OceanCurrentSpec(), "test");
  control.Update(gui);
  ASSERT_FALSE(ReadCurrent(gui).has_value());

  std::optional<CurrentValues> current;
  for (int i = 0; i < 300 && !current; ++i)
  {
    control.Update(gui);
    current = ReadCurrent(control.Source(gui, false));
    if (!current)
      std::this_thread::sleep_for(milliseconds(20));
  }
  ASSERT_TRUE(current.has_value()) << "the copy has the recipe";
  EXPECT_DOUBLE_EQ(0.25, current->speed);
  EXPECT_DOUBLE_EQ(135.0, current->direction);
  EXPECT_EQ(4u, current->generation);

  // Once is enough, and a GUI's ECM with the recipe is read directly.
  for (int i = 0; i < 100; ++i)
    control.Source(gui, false);
  std::this_thread::sleep_for(milliseconds(100));
  EXPECT_EQ(1, asked.load());
  EntityComponentManager full;
  World(full, "late", true);
  EXPECT_EQ(&full, &control.Source(full, true));
}
