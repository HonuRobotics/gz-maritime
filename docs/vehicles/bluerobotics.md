# BlueBoat and BlueROV2

```{figure} images/blueboat.jpg
:alt: The BlueBoat, a dark blue twin-hull surface vehicle with an antenna mast, floating on the ocean

The BlueBoat on the open water world.
```

The Blue Robotics vehicles from
[bluerobotics_models](https://github.com/HonuRobotics/bluerobotics_models)
run on this ocean too. Build that repository in the same workspace:

```bash
cd ~/maritime_ws/src
git clone https://github.com/HonuRobotics/bluerobotics_models.git
cd ~/maritime_ws
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
source install/setup.bash
```

Each vehicle keeps its floating volume on a link of its own
(`hull_displacement` for the boat, `buoyancy_displacement` for the ROV),
so that its many other collision shapes (brackets, thrusters, sensors) don't
add buoyancy, and those collisions are marked `gz:buoyancy="true"`. So the
vehicles float on this ocean as they are, and their own launch files bring
them up on it: each takes `world:=`, and `open_water.sdf` is a world like
any other.

## BlueBoat

```bash
ros2 launch blueboat_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share kai_gazebo)/worlds/open_water.sdf
```

Gazebo opens on the ocean with the boat floating at its waterline, its
bridge and `robot_state_publisher` running. Its topics
(`/blueboat/motor_port/thrust`, `/blueboat/ping/range`, ...) and the
`config_file:=` argument for a custom loadout are described in the
bluerobotics_models documentation; `gui:=false` runs it headless.

## BlueROV2

```bash
ros2 launch bluerov2_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share kai_gazebo)/worlds/open_water.sdf
```

The ROV drops in from half a metre up. It is trimmed almost exactly neutral
(2 g of net buoyancy), so it settles under the surface and stays roughly
where it stopped.

## Both, or next to the tutorial USV

Each of those launch files starts its own Gazebo, so they can't be combined.
To share one ocean, start the simulation once with one vehicle, then spawn
the second next to it by hand, each step in its own terminal (all sourced).

Terminal 1, the simulation with the BlueBoat:

```bash
ros2 launch blueboat_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share kai_gazebo)/worlds/open_water.sdf
```

Terminal 2, once the ocean is up, the BlueROV2 into it:

```bash
ros2 run ros_gz_sim create -name bluerov2 -x 5 -z 0.5 \
  -file $(ros2 pkg prefix --share bluerov2_gazebo)/models/bluerov2/model.sdf
```

Terminal 3, the BlueROV2's bridge, which keeps running:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix --share bluerov2_gazebo)/config/ros_gz_bridge.yaml
```

The second bridge also carries `/clock`, so the clock has two publishers in
that arrangement. A spawn launch per vehicle, with a bridge of its own
topics only, is what the
[multi-vehicle plan](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/MULTI_VEHICLE_PLAN.md)
provides for; once it lands, the two vehicles and the tutorial USV are one
include each next to `kai_bringup`'s `simulation.launch.xml`.
