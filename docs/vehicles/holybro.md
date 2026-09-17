# Holybro X500

[holybro_models](https://github.com/HonuRobotics/holybro_models) provides the
Holybro X500 V2, a quadcopter development kit. It is a drone, not a maritime
vehicle, so it runs in its own worlds rather than on this ocean.

It is listed here because it is built the same way as the vehicles in these
docs: a description package with the URDF (`x500_description`), a Gazebo
package with the simulation model, bridge configuration and launch file
(`x500_gazebo`), and a shared library of parts (`holybro_parts`). Where
the custom USV keeps its URDF and its Gazebo model in one package, the
Holybro and Blue Robotics vehicles split them in two. It is a good second
example to read when you structure your own vehicle.

## Run it

Build it in the same workspace and start its simulation:

```bash
cd ~/maritime_ws/src
git clone https://github.com/HonuRobotics/holybro_models.git
cd ~/maritime_ws
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
source install/setup.bash
ros2 launch x500_gazebo sim.launch.xml
```

The model comes with its rotors and flight sensors, but no flight controller.
Its [documentation](https://honurobotics.github.io/holybro_models/) covers
running and configuring the X500, its ROS interfaces and how its parts fit
together.

## On the ocean

Its launch file takes `world:=` like the others, so it can be brought up on
gz-maritime's world in one command:

```bash
ros2 launch x500_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share kai_gazebo)/worlds/open_water.sdf z:=1.0
```

What to expect: it appears 1 m above the water at the origin, and with no
flight controller its rotors don't spin, so it drops, and it sinks. It isn't
a floating vehicle: none of its collision shapes are marked as displacement
volume, and they shouldn't be, because they are the airframe, battery and
landing gear, sized for contact with the ground. Its place on this world is
the landing pad, a static deck 1 m above the water at (8, -8); spawned
there it drops onto its landing gear and stays:

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=x500 x:=8 y:=-8 z:=1.25 \
  generator:="$(ros2 pkg prefix x500_gazebo)/lib/x500_gazebo/configure_vehicle.py --config $(ros2 pkg prefix --share x500_description)/config/x500.yaml"
```

That is how the multi vehicle demo, `kai_bringup`'s
`multi_vehicle_demo.launch.py`, puts it next to the boats
([Several vehicles on one ocean](../how-to/several-vehicles.md)). Its bridge
comes up with it (`/x500/imu`, `/x500/mag`, `/x500/air_pressure`,
`/x500/gps/fix`, `/x500/joint_states`); the sensor topics are bridged
lazily, so they appear in `ros2 topic list` once something subscribes. The
world runs the air pressure system, so the barometer reports.
