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
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <limits>
#include <string>

#include <gz/common/Filesystem.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/components/CenterOfVolume.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Volume.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief What one iteration of the world tells us about a box: where it is
/// and whether the buoyancy system has measured it.
struct BoxState
{
  /// \brief World Z of the model origin, at the waterline when zero.
  double z{0.0};

  /// \brief Whether its link carries both components the wrench pass needs.
  bool measured{false};

  /// \brief Whether the model was found at all.
  bool found{false};
};

/// \brief Read a model's state out of the ECM.
/// \param[in] _ecm Entity component manager.
/// \param[in] _model Name of the model to look up.
/// \return Its state this iteration.
BoxState Read(const EntityComponentManager &_ecm, const std::string &_model)
{
  BoxState state;

  const Entity model = _ecm.EntityByComponents(
      components::Model(), components::Name(_model));
  if (kNullEntity == model)
    return state;

  const Entity link = _ecm.EntityByComponents(
      components::Link(), components::Name("link"),
      components::ParentEntity(model));
  if (kNullEntity == link)
    return state;

  state.found = true;
  state.z = worldPose(model, _ecm).Pos().Z();
  state.measured =
      nullptr != _ecm.Component<components::Volume>(link) &&
      nullptr != _ecm.Component<components::CenterOfVolume>(link);

  return state;
}

/// \brief Call one of the buoyancy registration services.
/// \param[in] _service Full service name.
/// \param[in] _name Scoped entity name to send.
/// \return True if the server accepted the request.
bool Register(const std::string &_service, const std::string &_name)
{
  transport::Node node;
  msgs::StringMsg req;
  req.set_data(_name);

  msgs::Boolean rep;
  bool result{false};
  const bool executed = node.Request(_service, req, 5000, rep, result);

  return executed && result && rep.data();
}

/// \brief Path to one of the test worlds.
/// \param[in] _file World file name.
/// \return Its absolute path.
std::string World(const std::string &_file)
{
  return common::joinPaths(TEST_WORLD_DIR, _file);
}
}  // namespace

/////////////////////////////////////////////////
/// A world in restricted mode floats nothing on its own; the enable service
/// makes one model float, and the disable service stops it again. Both have to
/// work on a model that was already in the world when the call arrived, which
/// is the case the stock plugin cannot express: it only ever measures a link on
/// the iteration that link appears in.
TEST(BuoyancyRegistration, EnableAndDisableAnExistingModel)
{
  TestFixture fixture(World("restricted.sdf"));

  BoxState plain;

  // Highest the box has been since the last reset. Nothing damps this world,
  // so a box that is climbing back out of a dip is easiest to catch at its
  // peak rather than at whatever point of the swing a fixed step count lands
  // on.
  double peak{std::numeric_limits<double>::lowest()};

  fixture.OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &
      _ecm)
  {
    plain = Read(_ecm, "plain_box");
    peak = std::max(peak, plain.z);
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);

  // Unregistered: no volume components, and falling.
  ASSERT_TRUE(server->Run(true, 100, false));
  ASSERT_TRUE(plain.found);
  EXPECT_FALSE(plain.measured);
  EXPECT_LT(plain.z, -0.001) << "an unregistered box should sink";

  const double sankTo = plain.z;

  // Registered while it already exists, which is the whole point.
  ASSERT_TRUE(Register("/world/buoyancy_test/buoyancy/enable",
      "plain_box::link"));

  ASSERT_TRUE(server->Run(true, 1, false));
  EXPECT_TRUE(plain.measured) << "enabling should measure the existing link";

  // Two seconds is longer than one swing of this box, so it has to come back
  // up past where it was: it arrived carrying the speed of its fall, and has
  // to shed that before it climbs.
  peak = std::numeric_limits<double>::lowest();
  ASSERT_TRUE(server->Run(true, 1000, false));
  EXPECT_GT(peak, sankTo) << "a registered box should come back up";

  // And back off again. The components go with it, which is what stops the
  // wrench: the pass in PreUpdate only looks at links that carry them.
  const double roseTo = plain.z;
  ASSERT_TRUE(Register("/world/buoyancy_test/buoyancy/disable",
      "plain_box::link"));

  ASSERT_TRUE(server->Run(true, 1, false));
  EXPECT_FALSE(plain.measured) << "disabling should drop the components";

  // Long enough to outlast whatever upward speed it had when the wrench
  // stopped: with nothing holding it up it has to end below where it was,
  // and then keep going.
  ASSERT_TRUE(server->Run(true, 1000, false));
  EXPECT_LT(plain.z, roseTo) << "a disabled box should sink again";
}

/////////////////////////////////////////////////
/// The model-side plugin registers its own link, so the same restricted world
/// floats it with no service call from here and without naming it anywhere.
TEST(BuoyancyRegistration, ModelRegistersItself)
{
  TestFixture fixture(World("restricted.sdf"));

  BoxState self;
  BoxState plain;
  fixture.OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &
      _ecm)
  {
    self = Read(_ecm, "self_box");
    plain = Read(_ecm, "plain_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);

  ASSERT_TRUE(server->Run(true, 100, false));

  ASSERT_TRUE(self.found);
  EXPECT_TRUE(self.measured) << "the model plugin should have registered";
  EXPECT_GT(self.z, -0.05) << "a self-registered box should not be sinking "
                              "the way an unregistered one does";

  // Its neighbour, identical but for the plugin, is untouched: registration
  // reaches exactly what the model asked for.
  ASSERT_TRUE(plain.found);
  EXPECT_FALSE(plain.measured);
  EXPECT_LT(plain.z, -0.001);
}

/////////////////////////////////////////////////
/// The case the design exists for: a vehicle spawned into a running world,
/// under a name that is not the one in its file. Nothing outside the model
/// knows that name, so nothing outside the model could have put it in an
/// <enable> list. A model in the world file is configured with its parent
/// chain already in the ECM; one spawned at runtime is not, so this is a
/// genuinely different path through the plugin.
TEST(BuoyancyRegistration, SpawnedUnderADifferentName)
{
  TestFixture fixture(World("restricted.sdf"));

  BoxState spawned;
  fixture.OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &
      _ecm)
  {
    spawned = Read(_ecm, "renamed_boat");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_FALSE(spawned.found);

  // The model file calls it self_box; it arrives as renamed_boat.
  msgs::EntityFactory req;
  req.set_sdf(R"(<?xml version="1.0"?>
    <sdf version="1.9">
      <model name="self_box">
        <pose>0 0 0 0 0 0</pose>
        <link name="link">
          <inertial>
            <mass>500</mass>
            <inertia>
              <ixx>83.333</ixx><iyy>83.333</iyy><izz>83.333</izz>
              <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry><box><size>1 1 1</size></box></geometry>
          </collision>
        </link>
        <plugin filename="gz-maritime-buoyancy-enable-system"
                name="gz::sim::maritime::BuoyancyEnable">
          <link>link</link>
        </plugin>
      </model>
    </sdf>)");
  req.set_name("renamed_boat");
  req.set_allow_renaming(false);

  transport::Node node;
  msgs::Boolean rep;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/buoyancy_test/create", req, 5000, rep,
      result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  ASSERT_TRUE(server->Run(true, 50, false));

  ASSERT_TRUE(spawned.found) << "the model should have been created";
  EXPECT_TRUE(spawned.measured)
      << "a model spawned under a new name should still register itself";
  EXPECT_GT(spawned.z, -0.5)
      << "and float rather than sink";
}

/////////////////////////////////////////////////
/// Reset restores entities without marking them new, which upstream handles by
/// asking for one full rescan. Registration rides on that rescan, so the two
/// have to agree: a reset rewinds the world, not the decisions made about it.
TEST(BuoyancyRegistration, RegistrationSurvivesReset)
{
  TestFixture fixture(World("restricted.sdf"));

  BoxState plain;
  fixture.OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &
      _ecm)
  {
    plain = Read(_ecm, "plain_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);

  ASSERT_TRUE(Register("/world/buoyancy_test/buoyancy/enable",
      "plain_box::link"));
  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_TRUE(plain.found);
  ASSERT_TRUE(plain.measured);

  server->ResetAll();
  ASSERT_TRUE(server->Run(true, 10, false));
  EXPECT_TRUE(plain.measured)
      << "an enabled link should be measured again after a reset";

  // And the other direction: a disable has to survive too, including against
  // whatever the initial ECM snapshot the reset restores happened to hold.
  ASSERT_TRUE(Register("/world/buoyancy_test/buoyancy/disable",
      "plain_box::link"));
  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_FALSE(plain.measured);

  server->ResetAll();
  ASSERT_TRUE(server->Run(true, 10, false));
  EXPECT_FALSE(plain.measured)
      << "a disabled link should stay disabled across a reset";
}

/////////////////////////////////////////////////
/// A model that declares its own links is the exception, and deliberately so.
/// BuoyancyEnable does not implement Reset, so the reset destroys and reloads
/// it and its Configure runs again, re-asserting the declaration in its SDF
/// over a disable issued at runtime. The model's own declaration is part of
/// the state a reset returns to, so it should win.
TEST(BuoyancyRegistration, ModelPluginReassertsItselfOnReset)
{
  TestFixture fixture(World("restricted.sdf"));

  BoxState self;
  fixture.OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &
      _ecm)
  {
    self = Read(_ecm, "self_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);

  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_TRUE(self.found);
  ASSERT_TRUE(self.measured);

  ASSERT_TRUE(Register("/world/buoyancy_test/buoyancy/disable",
      "self_box::link"));
  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_FALSE(self.measured);

  server->ResetAll();
  ASSERT_TRUE(server->Run(true, 10, false));
  EXPECT_TRUE(self.measured)
      << "the reloaded model plugin should re-assert its own links";
}

/////////////////////////////////////////////////
/// With neither <enable> nor <enable_by_default>, the vendored plugin has to
/// behave exactly like the stock one: everything floats.
TEST(BuoyancyRegistration, UpstreamDefaultFloatsEverything)
{
  TestFixture fixture(World("upstream_default.sdf"));

  BoxState plain;
  fixture.OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &
      _ecm)
  {
    plain = Read(_ecm, "plain_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);

  ASSERT_TRUE(server->Run(true, 100, false));

  ASSERT_TRUE(plain.found);
  EXPECT_TRUE(plain.measured) << "no list at all should measure every link";
  EXPECT_GT(plain.z, -0.2) << "a box starting below the waterline should rise";
}
