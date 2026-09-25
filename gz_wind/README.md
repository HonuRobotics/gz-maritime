# gz_wind

The wind of a world for Gazebo Sim, and windage on marked collisions, the
wind counterpart of `gz_buoyancy`'s marked displacement.

The world owns the wind: a speed and the direction it comes from, in the
world file and changed at run time on the topic `/world/<world>/wind/set`.
The system writes it into the wind entity every Gazebo world carries, where
the rotor, wing and air speed systems read it, and publishes it on
`/world/<world>/wind_info` as `gz.msgs.Wind`.

A vehicle marks the shapes the wind sees with `gz:wind="true"` on a
collision (a zero `collide_bitmask` keeps a dedicated shape out of contact,
and a buoyancy box can carry both marks). The `gz::sim::maritime::Wind`
world system finds those shapes on every model, spawned at any time under
any name, and applies quadratic drag, `0.5 * rho * Cd * A * |v| * v` per
shape axis, on the projected area of the part above the waterline, at the
centre of that part, so a tall shape heels and turns its link. The velocity
is the wind relative to the shape, so a boat running with the wind feels
less of it.

Without `<speed>` or `<direction>` the wind starts as the world's
`<wind><linear_velocity>`. Gazebo's `enable_wind` flag is not used: it
belongs to the mass based force of its `WindEffects` system, which should
not run in the same world, since both would write the wind.

## World plugin

```xml
<plugin filename="gz-maritime-wind-system" name="gz::sim::maritime::Wind">
  <speed>6</speed>
  <direction>270</direction>
  <air_density>1.225</air_density>
  <water_level>0</water_level>
  <default_drag_coefficient>1</default_drag_coefficient>
</plugin>
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `<speed>` | the world's `<wind>` | m/s. |
| `<direction>` | the world's `<wind>` | Degrees clockwise from north the wind comes from: 270 is from the west, blowing towards +x. |
| `<speed_gust>` | 0 | Standing deviation of the speed gusts, m/s; 0 is a steady speed. |
| `<speed_gust_time>` | 2 | Their correlation time, s. |
| `<direction_gust>` | 0 | Standing deviation of the direction gusts, degrees. |
| `<direction_gust_time>` | 10 | Their correlation time, s. |
| `<reference_height>` | 10 | m, the height `<speed>` is given at. |
| `<roughness_length>` | 0 | m, the surface roughness of a logarithmic profile, about 0.0002 over open sea; 0 is a uniform wind. |
| `<seed>` | 0 | Seed of the gusts; 0 draws a new one each run. A reset replays them. |
| `<publish_rate>` | 10 | Hz of simulation time for `wind_info`. |
| `<air_density>` | 1.225 | kg/m^3. |
| `<water_level>` | 0 | World z of the waterline; the part of a shape below it is not in the wind. |
| `<default_drag_coefficient>` | 1 | Cd for shapes without `gz:wind_cd`. |

## Gusts

Each gust is a first order Gauss Markov process, the one VRX uses on the
speed, here on the direction too: it wanders around zero with the standing
deviation asked for, and forgets itself over the correlation time. The
update is exact, `x' = a x + sigma sqrt(1 - a^2) n` with `a = exp(-dt / T)`,
so the spread does not depend on the step size, unlike VRX's Euler step.

## Wind with height

With a roughness length `z0`, the wind at a height `h` above the water is
the reference wind times `ln(h / z0) / ln(h_ref / z0)`: over the sea, about
half of the 10 m wind at a small boat's freeboard, and none below `z0`. The
windage evaluates it at the centre of each marked shape's exposed part. The
wind entity holds the reference height wind, so Gazebo's rotor, wing and
air speed systems, which read that single value, do not see the profile.

## Run time

A `gz.msgs.Param` on `/world/<world>/wind/set` with any of the parameter
names above as keys changes them, the rest keep their values. From ROS, bridge it as
`ros_gz_interfaces/msg/ParamVec`, each key a double parameter.

```bash
gz topic -t /world/default/wind/set -m gz.msgs.Param \
  -p 'params {key: "speed" value {type: DOUBLE double_value: 6}}'
```

## Collision attributes

```xml
<collision name="superstructure" gz:wind="true" gz:wind_cd="1.2">
  <geometry><box><size>1 0.5 0.8</size></box></geometry>
  <surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>
</collision>
```

Box, cylinder, sphere, capsule and ellipsoid use their own projected areas.
A mesh is treated as its bounding box.

## Test

```bash
colcon test --packages-select gz_wind && colcon test-result --verbose
```
