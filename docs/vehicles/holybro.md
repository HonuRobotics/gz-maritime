# Holybro X500

[holybro_models](https://github.com/HonuRobotics/holybro_models) provides the
Holybro X500 V2, a quadcopter development kit. It is a drone, not a maritime
vehicle, so it runs in its own worlds rather than on this ocean.

It is listed here because it is built the same way as the vehicles in these
docs: a description package with the URDF (`x500_description`), a Gazebo
package with the simulation model, bridge configuration and launch file
(`x500_gazebo`), and a shared library of parts (`holybro_parts`). It is a
good second example to read when you structure your own vehicle.

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

The X500 can be spawned into gz-maritime's world like any model:

```bash
ros2 launch kai_bringup simulation.launch.xml &
ros2 run ros_gz_sim create -name x500 -z 1.0 \
  -file $(ros2 pkg prefix --share x500_gazebo)/models/x500/model.sdf
```

What to expect: it appears 1 m above the water, and with no flight
controller its rotors don't spin, so it drops, and it sinks. It isn't a
floating vehicle: none of its collision shapes are marked as displacement
volume, and they shouldn't be, because they are the airframe, battery and
landing gear, sized for contact with the ground. It has no water damping
either, so it falls fast.
