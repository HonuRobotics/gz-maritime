# Getting started

## Requirements

- [ROS 2 Lyrical](https://docs.ros.org/en/lyrical/)
- [Gazebo Jetty](https://gazebosim.org/docs/jetty)
- [ros_gz](https://github.com/gazebosim/ros_gz) (matching the Lyrical / Jetty pairing)
- [Ehukai](https://github.com/HonuRobotics/ehukai), required by the
  FFT wave model. It has no rosdep key, so check it out as a peer in the same
  workspace and colcon builds it ahead of `gz_waves_provider_fft`. See its
  README for standalone build and install instructions.

A Docker Compose development environment with all of the above is maintained
separately in [HonuRobotics/drydock](https://github.com/HonuRobotics/drydock).

## Build

```bash
mkdir -p ~/maritime_ws/src && cd ~/maritime_ws/src
git clone https://github.com/HonuRobotics/gz-maritime.git
git clone https://github.com/HonuRobotics/ehukai.git
cd ~/maritime_ws
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
source install/setup.bash
```

Ehukai is a plain CMake package in the same workspace, so colcon builds
it ahead of `gz_waves_provider_fft` and no separate install step is needed.

If you installed Ehukai outside the workspace rather than cloning it as a
peer, add `--skip-keys ehukai` to the `rosdep install` command — the key
is unresolvable by design and rosdep will otherwise stop before installing
anything.

## Run

```bash
ros2 launch kai_bringup simulation.launch.xml
```

Launch arguments:

- `world` (default `open_water.sdf`): world file to load.
- `gazebo_gui` (default `true`): start the Gazebo GUI.
- `use_composition` (default `true`): run the Gazebo server and bridge as
  composable nodes in a shared container.

The default world starts with the FFT wave model on a moderate sea; sea
conditions can be changed while the simulation runs.
