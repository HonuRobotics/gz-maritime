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
// The vendored hydrodynamics in running worlds: it damps against the world's
// ocean current, on a vehicle loaded with the world and on one spawned
// later, and a world without one keeps upstream's behaviour.
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/data_load_options.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/spherical_coordinates.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Environment.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief Path to one of the test worlds.
/// \param[in] _file World file name.
/// \return Its absolute path.
std::string World(const std::string &_file)
{
  return common::joinPaths(TEST_WORLD_DIR, _file);
}

/// \brief Load a world and keep the velocity of every model's link.
class DriftWorld
{
  /// \brief Constructor.
  /// \param[in] _file World file name.
  public: explicit DriftWorld(const std::string &_file)
    : fixture(World(_file))
  {
    this->fixture.OnPreUpdate([this](const UpdateInfo &,
        EntityComponentManager &_ecm)
    {
      if (this->preUpdate)
        this->preUpdate(_ecm);
      // Track the velocity of every link, those spawned later included.
      _ecm.Each<components::Link>(
          [&](const Entity &_link, const components::Link *) -> bool
          {
            Link(_link).EnableVelocityChecks(_ecm);
            return true;
          });
    });
    this->fixture.OnPostUpdate([this](const UpdateInfo &,
        const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &_model, const components::Model *,
              const components::Name *_name) -> bool
          {
            const Entity link = _ecm.EntityByComponents(
                components::Link(), components::ParentEntity(_model));
            const auto *vel =
                _ecm.Component<components::WorldLinearVelocity>(link);
            if (nullptr != vel)
              this->velocity[_name->Data()] = vel->Data();
            return true;
          });
    });
    this->fixture.Finalize();
  }

  /// \brief Run some steps.
  /// \param[in] _steps Steps.
  /// \return True on success.
  public: bool Run(std::size_t _steps)
  {
    return this->fixture.Server()->Run(true, _steps, false);
  }

  /// \brief The fixture.
  public: TestFixture fixture;

  /// \brief Called at every PreUpdate before the velocities are tracked, to
  /// change the world from the test.
  public: std::function<void(EntityComponentManager &)> preUpdate;

  /// \brief World linear velocity of each model's link, by model name.
  public: std::map<std::string, math::Vector3d> velocity;
};

/// \brief A sphere like the worlds', to spawn at run time.
const char kLateSphere[] = R"(<?xml version="1.0"?>
  <sdf version="1.9">
    <model name="sphere_late">
      <pose>0 -4 0 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.25</radius></sphere></geometry>
        </collision>
      </link>
      <plugin filename="gz-maritime-hydrodynamics-system"
              name="gz::sim::maritime::Hydrodynamics">
        <link_name>link</link_name>
        <xU>-20</xU><yV>-20</yV><zW>-20</zW>
        <kP>-1</kP><mQ>-1</mQ><nR>-1</nR>
        <xUabsU>-20</xUabsU><yVabsV>-20</yVabsV><zWabsW>-20</zWabsW>
      </plugin>
    </model>
  </sdf>)";

/// \brief Tolerance on a settled drift, m/s.
constexpr double kTol{0.01};

/// \brief Spawn the late sphere looking the current up in the table.
/// \param[in] _world World name.
/// \return True if it was created.
bool SpawnTableSphere(const std::string &_world)
{
  std::string sdf(kLateSphere);
  const std::string tail = "      </plugin>";
  sdf.insert(sdf.find(tail),
      "        <lookup_current_x>current_x</lookup_current_x>\n"
      "        <lookup_current_y>current_y</lookup_current_y>\n"
      "        <lookup_current_z>current_z</lookup_current_z>\n");

  transport::Node node;
  msgs::EntityFactory req;
  req.set_sdf(sdf);
  msgs::Boolean rep;
  bool result{false};
  return node.Request("/world/" + _world + "/create", req, 5000, rep,
                      result) && result && rep.data();
}
}  // namespace

/////////////////////////////////////////////////
/// A sphere with nothing but damping settles at the water's velocity: the
/// world's 0.5 m/s current setting east, towards +x.
TEST(HydrodynamicsCurrent, DriftsWithTheWorldsCurrent)
{
  DriftWorld world("ocean_current.sdf");
  ASSERT_TRUE(world.Run(1));
  // It starts at rest; one 1 ms step of 20 N s/m against 0.5 m/s of water
  // gives a 10 kg body about 1.5 mm/s.
  EXPECT_LT(world.velocity["sphere"].Length(), 0.005);
  // The linear time constant is mass over damping, 0.5 s; six of them.
  ASSERT_TRUE(world.Run(3000));
  const math::Vector3d v = world.velocity["sphere"];
  EXPECT_NEAR(0.5, v.X(), kTol);
  EXPECT_NEAR(0.0, v.Y(), kTol);
  EXPECT_NEAR(0.0, v.Z(), kTol);
}

/////////////////////////////////////////////////
/// The world owns the current: a plugin's own <default_current> is
/// ignored when the world has one.
TEST(HydrodynamicsCurrent, WorldCurrentOverridesThePlugins)
{
  DriftWorld world("ocean_current.sdf");
  ASSERT_TRUE(world.Run(3000));
  const math::Vector3d v = world.velocity["sphere_own"];
  EXPECT_NEAR(0.5, v.X(), kTol);
  EXPECT_NEAR(0.0, v.Y(), kTol) << "its own current along +y is ignored";
}

/////////////////////////////////////////////////
/// A vehicle spawned after the world has run a while feels the same
/// current: the recipe is on the world entity, not in anyone's startup.
TEST(HydrodynamicsCurrent, VehicleSpawnedLaterDriftsTheSame)
{
  DriftWorld world("ocean_current.sdf");
  ASSERT_TRUE(world.Run(500));

  transport::Node node;
  msgs::EntityFactory req;
  req.set_sdf(kLateSphere);
  msgs::Boolean rep;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/ocean_current/create", req, 5000, rep,
                           result));
  ASSERT_TRUE(result && rep.data());

  ASSERT_TRUE(world.Run(3000));
  ASSERT_EQ(1u, world.velocity.count("sphere_late"));
  const math::Vector3d v = world.velocity["sphere_late"];
  EXPECT_NEAR(0.5, v.X(), kTol);
  EXPECT_NEAR(0.0, v.Y(), kTol);
}

/////////////////////////////////////////////////
/// A change on the world's ocean current topic reaches the hull: turned to
/// set north, towards +y, the sphere settles at the new current.
TEST(HydrodynamicsCurrent, FollowsAChangeOnTheTopic)
{
  DriftWorld world("ocean_current.sdf");
  ASSERT_TRUE(world.Run(3000));
  ASSERT_NEAR(0.5, world.velocity["sphere"].X(), kTol);

  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(
      "/world/ocean_current/ocean_current/set");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(pub.HasConnections());
  msgs::Param msg;
  auto &any = (*msg.mutable_params())["direction"];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(0.0);
  ASSERT_TRUE(pub.Publish(msg));
  // Delivery is asynchronous; the system queues it for the next step.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_TRUE(world.Run(3000));
  const math::Vector3d v = world.velocity["sphere"];
  EXPECT_NEAR(0.0, v.X(), kTol);
  EXPECT_NEAR(0.5, v.Y(), kTol);
}

/////////////////////////////////////////////////
/// Without a gz-maritime ocean current the plugin is upstream's: its own
/// <default_current> carries the sphere along +y.
TEST(HydrodynamicsCurrent, UpstreamBehaviourWithoutAWorldCurrent)
{
  DriftWorld world("no_ocean_current.sdf");
  ASSERT_TRUE(world.Run(3000));
  const math::Vector3d v = world.velocity["sphere"];
  EXPECT_NEAR(0.0, v.X(), kTol);
  EXPECT_NEAR(0.5, v.Y(), kTol);
}

/////////////////////////////////////////////////
/// A vehicle spawned at run time finds the current table that
/// EnvironmentPreload loaded at the start. Upstream looks for the table only
/// while the world entity is new, so the sphere would sit still.
TEST(HydrodynamicsCurrent, TableReachesAVehicleSpawnedLater)
{
  // EnvironmentPreload resolves a relative <data> against the world file's
  // directory only through the resource path.
  common::setenv("GZ_SIM_RESOURCE_PATH", TEST_WORLD_DIR);
  DriftWorld world("current_table.sdf");
  ASSERT_TRUE(world.Run(500));
  ASSERT_TRUE(SpawnTableSphere("current_table"));

  ASSERT_TRUE(world.Run(3000));
  ASSERT_EQ(1u, world.velocity.count("sphere_late"));
  const math::Vector3d v = world.velocity["sphere_late"];
  EXPECT_NEAR(0.5, v.X(), kTol);
  EXPECT_NEAR(0.0, v.Y(), kTol);
}

/////////////////////////////////////////////////
/// The world owns the current: a message on the plugin's own ocean current
/// topic is ignored when the world has one (with a warning).
TEST(HydrodynamicsCurrent, WorldCurrentOverridesTheTopic)
{
  DriftWorld world("ocean_current.sdf");
  ASSERT_TRUE(world.Run(1));

  transport::Node node;
  auto pub = node.Advertise<msgs::Vector3d>("/ocean_current");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(pub.HasConnections());
  msgs::Vector3d msg;
  msg.set_y(-0.5);
  ASSERT_TRUE(pub.Publish(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_TRUE(world.Run(3000));
  const math::Vector3d v = world.velocity["sphere"];
  EXPECT_NEAR(0.5, v.X(), kTol);
  EXPECT_NEAR(0.0, v.Y(), kTol) << "the topic's current along -y is ignored";
}

/////////////////////////////////////////////////
/// A table reloaded through EnvironmentPreload's topic is a new data set on
/// the world's Environment component, and the plugin rebuilds its lookup:
/// the sphere turns from the first table's +x to the second's +y.
TEST(HydrodynamicsCurrent, TableReloadRebuildsTheLookup)
{
  common::setenv("GZ_SIM_RESOURCE_PATH", TEST_WORLD_DIR);
  DriftWorld world("current_table.sdf");
  ASSERT_TRUE(world.Run(500));
  ASSERT_TRUE(SpawnTableSphere("current_table"));
  ASSERT_TRUE(world.Run(3000));
  ASSERT_NEAR(0.5, world.velocity["sphere_late"].X(), kTol);

  transport::Node node;
  auto pub = node.Advertise<msgs::DataLoadPathOptions>(
      "/world/current_table/environment");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(pub.HasConnections());
  msgs::DataLoadPathOptions msg;
  msg.set_path(World("current_table_north.csv"));
  msg.set_time("timestamp");
  msg.set_static_time(false);
  msg.set_x("x");
  msg.set_y("y");
  msg.set_z("z");
  msg.set_coordinate_type(msgs::SphericalCoordinatesType::GLOBAL);
  ASSERT_TRUE(pub.Publish(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_TRUE(world.Run(3000));
  const math::Vector3d v = world.velocity["sphere_late"];
  EXPECT_NEAR(0.0, v.X(), kTol);
  EXPECT_NEAR(0.5, v.Y(), kTol);
}

/////////////////////////////////////////////////
/// The world's Environment component removed, the plugin clears its lookup:
/// no table, no current, and the sphere comes to rest.
TEST(HydrodynamicsCurrent, TableRemovedLeavesSlackWater)
{
  common::setenv("GZ_SIM_RESOURCE_PATH", TEST_WORLD_DIR);
  DriftWorld world("current_table.sdf");
  ASSERT_TRUE(world.Run(500));
  ASSERT_TRUE(SpawnTableSphere("current_table"));
  ASSERT_TRUE(world.Run(3000));
  ASSERT_NEAR(0.5, world.velocity["sphere_late"].X(), kTol);

  bool removed{false};
  world.preUpdate = [&removed](EntityComponentManager &_ecm)
  {
    if (removed)
      return;
    const Entity w = worldEntity(_ecm);
    removed = _ecm.RemoveComponent<components::Environment>(w);
  };
  ASSERT_TRUE(world.Run(3000));
  ASSERT_TRUE(removed);
  EXPECT_NEAR(0.0, world.velocity["sphere_late"].Length(), kTol);
}
