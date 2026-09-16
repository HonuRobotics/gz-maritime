# Gz Maritime

Maritime simulation for **Gazebo** and **ROS 2**: a moving ocean, an open
water world, and everything you need to put your own boat or underwater
vehicle on it.

```{figure} getting-started/images/custom-usv.jpg
:alt: A small twin-hull boat, the custom USV, floating on a calm simulated ocean

The custom USV on the open water world.
```

What you get:

- **An ocean.** Two wave models, a fast analytic one and a high-fidelity
  spectral one, driven by a single "sea state" setting you can change while
  the simulation runs.
- **An open water world** with seawater buoyancy and the systems that IMU,
  GPS and magnetometer sensors need.
- **Your own vehicles.** A vehicle marks which of its shapes float, so the
  world never has to know about it in advance, and several can share one
  simulation. A small example boat shows every step.
- **ROS 2 bring-up.** One launch file starts Gazebo, and one puts any
  vehicle on it, as many times as you like, with its topics bridged to
  ROS 2.

## Quick start

```bash
mkdir -p ~/maritime_ws/src && cd ~/maritime_ws/src
git clone https://github.com/HonuRobotics/gz-maritime.git
git clone https://github.com/HonuRobotics/ehukai.git
cd ~/maritime_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
source install/setup.bash
ros2 launch kai_custom_vehicle sim.launch.xml
```

A small catamaran appears on the ocean and floats at its waterline.
[First simulation](getting-started/first-simulation.md) shows how to drive
it. [Bring your own vehicle](how-to/own-vehicle.md) shows how it was built,
so you can build yours the same way.

These packages target **ROS 2 Lyrical** and **Gazebo Jetty** on Ubuntu 26.04.

```{toctree}
:hidden:
:maxdepth: 4

Getting started <getting-started/index>
Vehicles <vehicles/index>
How to guides <how-to/index>
Reference <reference/index>
Design <design/index>
Project <project/index>
```
