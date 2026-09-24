# gz-maritime

Maritime simulation for ROS 2 and Gazebo Sim: a moving ocean, an open water
world with seawater buoyancy, and the launch files that put any vehicle on
it, under a name, as often as needed. Targets **ROS 2 Lyrical + Gazebo
Jetty** (the default pairing on Ubuntu 26.04), via `ros_gz`. This repository
continues the `vrx4` rewrite of [osrf/vrx](https://github.com/osrf/vrx).

**Documentation: <https://honurobotics.github.io/gz-maritime/>**

The site covers installation, a first simulation, bringing your own vehicle
onto the ocean, the Blue Robotics and Holybro vehicles, the launch files and
the world, and the design behind the waves and the buoyancy.

## Getting started

Three short pages take you from a fresh machine to driving a boat on the
ocean: [Requirements](docs/getting-started/requirements.md),
[Installation](docs/getting-started/installation.md) and
[First simulation](docs/getting-started/first-simulation.md), also on the
site under
[Getting started](https://honurobotics.github.io/gz-maritime/lyrical/getting-started/).

## Packages

- [`gz_waves`](gz_waves/), [`gz_waves_provider_gerstner`](gz_waves_provider_gerstner/),
  [`gz_waves_provider_fft`](gz_waves_provider_fft/) and
  [`gz_waves_rendering`](gz_waves_rendering/): one simulated sea shared by
  physics and rendering, from a fast analytic model or a high fidelity
  spectral one. See [WAVES_DESIGN.md](WAVES_DESIGN.md).
- [`gz_buoyancy`](gz_buoyancy/) and [`gz_thruster`](gz_thruster/): Gazebo's
  buoyancy and thruster systems with the changes vehicles here rely on,
  marked collisions and a normalized command, carried until a Gazebo release
  ships them.
- [`gz_wind`](gz_wind/): windage on the collisions a vehicle marks
  `gz:wind="true"`, from the world's wind, on the part above the waterline.
- [`kai_gazebo`](kai_gazebo/), [`kai_bringup`](kai_bringup/) and
  [`kai_custom_vehicle`](kai_custom_vehicle/): the worlds, the simulation
  and spawn launches, and the example vehicle.

## Contributing

Developer notes (build, tests, conventions) are in [AGENTS.md](AGENTS.md).

## License

Apache-2.0 (see [LICENSE](LICENSE)).
