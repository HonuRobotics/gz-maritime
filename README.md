# gz-maritime

Maritime simulation for Gazebo: realistic ocean waves, open water worlds, and
the ROS 2 integration to use them, targeting **ROS 2 Lyrical + Gazebo Jetty**.

This repository continues the `vrx4` rewrite of
[osrf/vrx](https://github.com/osrf/vrx).

## Packages

- [`gz_waves`](gz_waves/): the foundation for wave simulation, one simulated sea shared
  consistently by physics, rendering, and future consumers.
- [`gz_waves_provider_gerstner`](gz_waves_provider_gerstner/): a simple and fast wave model, well suited to
  lightweight scenarios.
- [`gz_waves_provider_fft`](gz_waves_provider_fft/): a high fidelity wave model that produces a
  realistic ocean from a single sea state setting.
- [`gz_waves_rendering`](gz_waves_rendering/): the visual ocean, drawing the moving water surface in
  the Gazebo GUI.
- [`vrx_gazebo`](vrx_gazebo/): the simulation worlds.
- [`vrx_bringup`](vrx_bringup/): launch files and configuration to start everything from
  ROS 2.

See [WAVES_DESIGN.md](WAVES_DESIGN.md) for the architecture behind these packages.

## Requirements

- [ROS 2 Lyrical](https://docs.ros.org/en/lyrical/)
- [Gazebo Jetty](https://gazebosim.org/docs/jetty)
- [ros_gz](https://github.com/gazebosim/ros_gz) (matching the Lyrical / Jetty pairing)
- [Ehukai](https://github.com/HonuRobotics/ehukai), required by the
  FFT wave model. Check it out into the same workspace and colcon builds it
  first; it has no rosdep key, so `rosdep install` cannot fetch it for you.
- `libimath-dev`, required transitively by Ehukai. There is no rosdep
  rule for Imath in any rosdistro list, so this one is a manual `apt install`.

A Docker Compose development environment with all of the above is maintained
separately in [HonuRobotics/drydock](https://github.com/HonuRobotics/drydock).

## Build

```bash
mkdir -p ~/maritime_ws/src && cd ~/maritime_ws/src
git clone git@github.com:HonuRobotics/gz-maritime.git
git clone git@github.com:HonuRobotics/encinowaves.git
cd ~/maritime_ws
sudo apt install libimath-dev          # no rosdep key; see Requirements
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
source install/setup.bash
```

EncinoWaves is a plain CMake package in the same workspace, so colcon builds
it ahead of `gz_waves_provider_fft` and no separate install step is needed.

## Run

```bash
ros2 launch vrx_bringup simulation.launch.xml
```

Launch arguments:

- `world` (default `open_water.sdf`): world file to load.
- `gazebo_gui` (default `true`): start the Gazebo GUI.
- `use_composition` (default `true`): run the Gazebo server and bridge as
  composable nodes in a shared container.

The default world starts with the FFT wave model on a moderate sea; sea
conditions can be changed while the simulation runs.

## License

Apache-2.0 (see [LICENSE](LICENSE)).
