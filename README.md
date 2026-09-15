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
- [`gz_buoyancy`](gz_buoyancy/): Gazebo's buoyancy with marked collisions: a
  vehicle marks the shapes that float it, so a world never has to name it.
  Carries an upstream gz-sim change until a release ships it.
- [`gz_thruster`](gz_thruster/): Gazebo's thruster system with a normalized
  command interface, so an autopilot or a custom controller commands a fraction
  of full thrust rather than a force in newtons. Vendored from gz-sim and
  intended to go back upstream; see
  [`gz_thruster/PROVENANCE.md`](gz_thruster/PROVENANCE.md).
- [`kai_gazebo`](kai_gazebo/): the simulation worlds.
- [`kai_bringup`](kai_bringup/): the simulation launch (Gazebo on the ocean, the
  GUI, `/clock`) that every vehicle's spawn launch sits next to.
- [`tutorial_usv_description`](tutorial_usv_description/) and
  [`tutorial_usv_gazebo`](tutorial_usv_gazebo/): a small example catamaran, the
  worked example for bringing your own vehicle.

See [WAVES_DESIGN.md](WAVES_DESIGN.md) for the architecture behind these packages.

Full documentation, including build and run instructions and how to bring your
own vehicle, is at
[honurobotics.github.io/gz-maritime](https://honurobotics.github.io/gz-maritime/).

## License

Apache-2.0 (see [LICENSE](LICENSE)).
