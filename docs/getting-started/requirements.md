# Requirements

| You need | What for |
|---|---|
| Ubuntu 26.04 | The platform ROS 2 Lyrical targets. |
| [ROS 2 Lyrical](https://docs.ros.org/en/lyrical/) | Launch files and the ROS side of every vehicle. |
| [Gazebo Jetty](https://gazebosim.org/docs/jetty) | The simulator. With ROS it arrives as the `gz_*_vendor` packages, so you rarely install it by hand. |
| [ros_gz](https://github.com/gazebosim/ros_gz) | `ros_gz_sim` starts Gazebo and spawns vehicles; `ros_gz_bridge` connects Gazebo topics to ROS 2. Every launch file here uses both. |
| [Ehukai](https://github.com/HonuRobotics/ehukai) | The spectral wave library behind the high-fidelity ocean. It has no rosdep key, so you clone it next to gz-maritime (see [Installation](installation.md)). |
| A GPU | For the Gazebo GUI and camera-like sensors. Physics and the IMU, GPS and magnetometer run fine without one. |

Everything else (`xacro`, `robot_state_publisher`, the message packages) is
installed by `rosdep` during the build.

## Optional

- **[bluerobotics_models](https://github.com/HonuRobotics/bluerobotics_models)**,
  to put the BlueBoat and BlueROV2 on this ocean (see
  [BlueBoat and BlueROV2](../vehicles/bluerobotics.md)).

- **[holybro_models](https://github.com/HonuRobotics/holybro_models)**, with
  the Holybro X500 V2 quadcopter. It is a drone with its own world rather
  than a maritime vehicle, but it is built the same way as the vehicles here,
  from a URDF description and a separate Gazebo package, so it is a second
  example to follow when you structure your own (see
  [Holybro X500](../vehicles/holybro.md)).

## Docker

If you'd rather not install anything on your machine,
[HonuRobotics/drydock](https://github.com/HonuRobotics/drydock) maintains a
Docker Compose development environment with all of the above.
