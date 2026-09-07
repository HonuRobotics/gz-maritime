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
- [`kai_gazebo`](kai_gazebo/): the simulation worlds.
- [`kai_bringup`](kai_bringup/): launch files and configuration to start everything from
  ROS 2.

See [WAVES_DESIGN.md](WAVES_DESIGN.md) for the architecture behind these packages.

Full documentation, including build and run instructions, is at
[honurobotics.github.io/gz-maritime](https://honurobotics.github.io/gz-maritime/).

## License

Apache-2.0 (see [LICENSE](LICENSE)).
