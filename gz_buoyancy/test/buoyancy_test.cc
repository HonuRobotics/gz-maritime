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
// Upstream's tests for the marked-collision change (gz-sim commit 229ec07e,
// test/integration/buoyancy.cc), run against the vendored plugin so the
// bridge behaves exactly as the gz-sim it stands in for.
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <string>
#include <vector>

#include <gz/common/Filesystem.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/components/CenterOfVolume.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Volume.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief What one iteration of the world tells us about a box.
struct MarkedBoxState
{
  /// \brief World Z of the model origin, at the waterline when zero.
  public: double z{0.0};

  /// \brief Whether its link carries both components the wrench pass needs.
  public: bool measured{false};

  /// \brief The link's volume component, if measured.
  public: double volume{0.0};

  /// \brief Names of the link's collisions marked as buoyancy geometry.
  public: std::vector<std::string> marked;

  /// \brief Whether the model was found at all.
  public: bool found{false};
};

/// \brief Read a model's state out of the ECM.
/// \param[in] _ecm Entity component manager.
/// \param[in] _model Name of the model to look up.
/// \param[in] _link Name of the link to check.
/// \return Its state this iteration.
MarkedBoxState ReadMarkedBox(const EntityComponentManager &_ecm,
    const std::string &_model, const std::string &_link = "link")
{
  MarkedBoxState state;

  const Entity model = _ecm.EntityByComponents(
      components::Model(), components::Name(_model));
  if (kNullEntity == model)
    return state;

  const Entity link = _ecm.EntityByComponents(
      components::Link(), components::Name(_link),
      components::ParentEntity(model));
  if (kNullEntity == link)
    return state;

  state.found = true;
  state.z = worldPose(model, _ecm).Pos().Z();
  const auto *volume = _ecm.Component<components::Volume>(link);
  const auto *cov = _ecm.Component<components::CenterOfVolume>(link);
  state.measured = nullptr != volume && nullptr != cov;
  if (state.measured)
    state.volume = volume->Data();
  for (const Entity collision :
      _ecm.ChildrenByComponents(link, components::Collision()))
  {
    const auto elem = _ecm.Component<components::CollisionElement>(
        collision)->Data().Element();
    if (elem->HasAttribute("gz:buoyancy") &&
        elem->GetAttribute("gz:buoyancy")->GetAsString() == "true")
    {
      state.marked.push_back(
          _ecm.Component<components::Name>(collision)->Data());
    }
  }

  return state;
}

/// \brief Path to one of the test worlds.
/// \param[in] _file World file name.
/// \return Its absolute path.
std::string World(const std::string &_file)
{
  return common::joinPaths(TEST_WORLD_DIR, _file);
}

/// \brief A 1 m, 500 kg box with a contact collision and a marked one, as a
/// vehicle would carry.
const char kMarkedBox[] = R"(<?xml version="1.0"?>
  <sdf version="1.9" xmlns:gz="http://gazebosim.org/schema">
    <model name="marked_box">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>500</mass>
          <inertia>
            <ixx>83.333</ixx><iyy>83.333</iyy><izz>83.333</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="contact">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <collision name="hull" gz:buoyancy="true">
          <geometry><box><size>1 1 1</size></box></geometry>
          <surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>
        </collision>
      </link>
    </model>
  </sdf>)";
}  // namespace

/////////////////////////////////////////////////
/// In restricted mode the world floats nothing through unmarked collisions,
/// and a link that marks a collision floats by exactly that one.
TEST(MarkedBuoyancy, MarkedCollisions)
{
  TestFixture fixture(World("buoyancy_marked_collisions.sdf"));

  MarkedBoxState plain;
  MarkedBoxState marked;
  MarkedBoxState ballast;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    plain = ReadMarkedBox(_ecm, "plain_box");
    marked = ReadMarkedBox(_ecm, "marked_box");
    ballast = ReadMarkedBox(_ecm, "marked_box", "ballast");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(plain.found);
  EXPECT_FALSE(plain.measured) << "restricted mode should measure nothing";
  EXPECT_TRUE(plain.marked.empty());
  EXPECT_LT(plain.z, -0.001) << "a box with only a contact collision sinks";

  ASSERT_TRUE(marked.found);
  EXPECT_EQ(std::vector<std::string>{"hull"}, marked.marked);
  EXPECT_TRUE(marked.measured);
  EXPECT_DOUBLE_EQ(1.0, marked.volume) << "the contact box must not count";
  EXPECT_GT(marked.z, -0.5) << "a box with a marked collision floats";

  ASSERT_TRUE(ballast.found);
  EXPECT_FALSE(ballast.measured)
      << "a link that marks nothing must not displace";
}

/////////////////////////////////////////////////
/// A vehicle spawned into a running world, under a name that is not the one
/// in its file. Nothing outside the model knows that name, and nothing needs
/// to: the mark travels with the model.
TEST(MarkedBuoyancy, MarkedSpawnedByAnyName)
{
  TestFixture fixture(World("buoyancy_marked_collisions.sdf"));

  MarkedBoxState spawned;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    spawned = ReadMarkedBox(_ecm, "rov_a");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_FALSE(spawned.found);

  msgs::EntityFactory req;
  req.set_sdf(kMarkedBox);
  req.set_name("rov_a");
  req.set_allow_renaming(false);
  req.mutable_pose()->mutable_position()->set_x(10.0);

  transport::Node node;
  msgs::Boolean rep;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/buoyancy_marked/create", req, 5000, rep,
      result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(spawned.found) << "the model should have been created";
  EXPECT_EQ(std::vector<std::string>{"hull"}, spawned.marked);
  EXPECT_TRUE(spawned.measured);
  EXPECT_DOUBLE_EQ(1.0, spawned.volume);
  EXPECT_GT(spawned.z, -0.5) << "and float rather than sink";
}

/////////////////////////////////////////////////
/// With neither <enable> nor <enable_by_default>, everything floats through
/// its collisions, as before. A link with a marked collision floats by that
/// alone: a neutrally buoyant submerged box with both a contact and a marked
/// collision is pushed exactly once.
TEST(MarkedBuoyancy, MarkedTakesPrecedence)
{
  TestFixture fixture(World("buoyancy_marked_default.sdf"));

  MarkedBoxState plain;
  MarkedBoxState both;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    plain = ReadMarkedBox(_ecm, "plain_box");
    both = ReadMarkedBox(_ecm, "both_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(plain.found);
  EXPECT_TRUE(plain.measured) << "no list at all should measure every link";
  EXPECT_GT(plain.z, -0.2) << "a box starting below the waterline rises";

  ASSERT_TRUE(both.found);
  EXPECT_TRUE(both.measured);
  EXPECT_DOUBLE_EQ(1.0, both.volume) << "measured from the mark, not both";
  // One second of free rise would move it about 4.9 m.
  EXPECT_NEAR(-3.0, both.z, 0.05)
      << "a neutrally buoyant box pushed twice would have shot up";
}

/////////////////////////////////////////////////
/// A marked collision in a nested model is found there.
TEST(MarkedBuoyancy, MarkedInNestedModel)
{
  TestFixture fixture(World("buoyancy_marked_nested.sdf"));

  MarkedBoxState pod;
  MarkedBoxState deck;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    pod = ReadMarkedBox(_ecm, "pod");
    deck = ReadMarkedBox(_ecm, "assembly", "deck");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(pod.found);
  EXPECT_EQ(std::vector<std::string>{"hull"}, pod.marked);
  EXPECT_TRUE(pod.measured);
  EXPECT_DOUBLE_EQ(1.0, pod.volume);
  EXPECT_GT(pod.z, -0.5) << "the nested mark should hold the assembly up";

  ASSERT_TRUE(deck.found);
  EXPECT_FALSE(deck.measured) << "the parent's link marks nothing";
}

/////////////////////////////////////////////////
/// A world reset restores the links that existed at load. Marked links are
/// measured again and the restriction still holds.
TEST(MarkedBuoyancy, MarkedSurvivesReset)
{
  TestFixture fixture(World("buoyancy_marked_collisions.sdf"));

  MarkedBoxState plain;
  MarkedBoxState marked;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    plain = ReadMarkedBox(_ecm, "plain_box");
    marked = ReadMarkedBox(_ecm, "marked_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 100, false));
  ASSERT_TRUE(marked.measured);

  server->ResetAll();
  ASSERT_TRUE(server->Run(true, 500, false));

  EXPECT_FALSE(plain.measured) << "the restriction should survive a reset";
  EXPECT_TRUE(marked.measured) << "measured again after the reset";
  EXPECT_DOUBLE_EQ(1.0, marked.volume);
  EXPECT_GT(marked.z, -0.5) << "and still floating";
}
