# Installation

gz-maritime builds from source in a colcon workspace, with Ehukai next to it.

## 1. Clone

```bash
mkdir -p ~/maritime_ws/src && cd ~/maritime_ws/src
git clone https://github.com/HonuRobotics/gz-maritime.git
git clone https://github.com/HonuRobotics/ehukai.git
```

Ehukai is a plain CMake package. In the same workspace, colcon builds it
before the wave package that needs it, so there is no separate install step.

## 2. Install the dependencies

```bash
cd ~/maritime_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
```

```{note}
If you installed Ehukai somewhere else instead of cloning it, add
`--skip-keys ehukai`. Ehukai has no rosdep key, and without that flag rosdep
stops before installing anything.
```

## 3. Build

```bash
colcon build --merge-install
source install/setup.bash
```

Source `install/setup.bash` in every new terminal. Besides ROS, it tells
Gazebo where to find the worlds, models and plugins.

## 4. Check

```bash
ros2 pkg list | grep -E "kai_|gz_buoyancy|gz_thruster"
```

You should see `gz_buoyancy`, `gz_thruster`, `kai_bringup`,
`kai_custom_vehicle` and `kai_gazebo`.

Next: [First simulation](first-simulation.md).
