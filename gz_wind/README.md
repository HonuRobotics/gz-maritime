# gz_wind

Windage on marked collisions for Gazebo Sim, the wind counterpart of
`gz_buoyancy`'s marked displacement.

A vehicle marks the shapes the wind sees with `gz:wind="true"` on a
collision (a zero `collide_bitmask` keeps a dedicated shape out of contact,
and a buoyancy box can carry both marks). The `gz::sim::maritime::Wind`
world system finds those shapes on every model, spawned at any time under
any name, and applies quadratic drag, `0.5 * rho * Cd * A * |v| * v` per
shape axis, on the projected area of the part above the waterline, at the
centre of that part, so a tall shape heels and turns its link. The velocity
is the wind relative to the shape, so a boat running with the wind feels
less of it.

The wind is the one every Gazebo world carries: the wind entity's linear
velocity, set by `<wind><linear_velocity>` in the world or by any system that
writes it (Gazebo's `WindEffects` included). Gazebo's `enable_wind` flag is
not used: it belongs to the mass based force of `WindEffects`.

## World plugin

```xml
<plugin filename="gz-maritime-wind-system" name="gz::sim::maritime::Wind">
  <air_density>1.225</air_density>
  <water_level>0</water_level>
  <default_drag_coefficient>1</default_drag_coefficient>
</plugin>
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `<air_density>` | 1.225 | kg/m^3. |
| `<water_level>` | 0 | World z of the waterline; the part of a shape below it is not in the wind. |
| `<default_drag_coefficient>` | 1 | Cd for shapes without `gz:wind_cd`. |

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
