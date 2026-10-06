# gz_wind

The wind of a world for Gazebo Sim, built the way the wave field is.

The world owns the wind: a speed and the direction it comes from, in the
world file and changed at run time on the topic `/world/<world>/wind/set`.
The `gz::sim::maritime::Wind` world system keeps it as a recipe on the world
entity, the `Windfield` component: which model, with which parameters. Any
system rebuilds its own copy of the model from that recipe and asks the wind
at any point and time, so a new wind model is a new registered model and no
change to the systems that use the wind.

The same system pushes on the shapes the wind sees. A vehicle marks them
with `gz:wind="true"` on a collision (a zero `collide_bitmask` keeps a
dedicated shape out of contact, and a buoyancy box can carry both marks).
The system finds those shapes on every model, spawned at any time under any
name, and applies quadratic drag, `0.5 * rho * Cd * A * |v| * v` per shape
axis, on the projected area of the part above the water, at the centre of
that part, with the wind asked at that centre, so a tall shape heels and
turns its link. The velocity is the wind relative to the shape, so a boat
running with the wind feels less of it. `gz:wind_cd` sets a shape's drag
coefficient. A mark is read as a bool, so `"1"` works too. Boxes,
cylinders, spheres, capsules and ellipsoids keep their shape; a mesh counts
as its bounding box.

The system also writes the wind at the reference height above the world's
origin into the wind entity every Gazebo world carries, in both the values
Gazebo's rotor, wing and air speed systems read, and publishes it on
`/world/<world>/wind_info` as `gz.msgs.Wind` and on
`/world/<world>/wind/velocity` as `gz.msgs.Twist`, which ROS can bridge.
Both values hold the same wind, gusts included. That differs on purpose
from Gazebo's `WindEffects`, which keeps the seed as the mean and puts its
noise only in the entity's velocity, so there the air speed sensor misses
the gusts the rotors and wings feel; here all three see the same wind.

## Anemometer

A custom sensor on any link of any vehicle reads the apparent wind: the wind
at the sensor, asked through the recipe, less the sensor's own velocity, in
the sensor frame, as the linear part of a `gz.msgs.Twist`:

```xml
<sensor name="anemometer" type="custom" gz:type="anemometer">
  <frame_id>my_boat/mast_link</frame_id>
  <topic>my_boat/anemometer</topic>
  <update_rate>10</update_rate>
  <gz:anemometer>
    <noise type="gaussian">
      <mean>0</mean>
      <stddev>0.2</stddev>
    </noise>
  </gz:anemometer>
</sensor>
```

The wind system finds it, like a marked collision, on any model spawned at
any time. Bridge it to `geometry_msgs/msg/TwistStamped`.

It is a gz-sensors custom sensor, so it takes `<topic>`, `<frame_id>`,
`<update_rate>` and `<pose>` like any other, and the standard `<noise>`
block inside `<gz:anemometer>`, applied to each axis on its own and seeded
from Gazebo's seed (`gz sim --seed`). Without one its readings are exact.

Without `<speed>` or `<direction>` the wind starts as the world's
`<wind><linear_velocity>`. Gazebo's `WindEffects` system should not run in
the same world, since both would write the wind, and Gazebo's
`enable_wind` flag, which belongs to its mass based force, is not the mark.

## World plugin

```xml
<plugin filename="gz-maritime-wind-system" name="gz::sim::maritime::Wind">
  <speed>6</speed>
  <direction>270</direction>
</plugin>
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `<speed>` | the world's `<wind>` | m/s. |
| `<direction>` | the world's `<wind>` | Degrees clockwise from north the wind comes from: 270 is from the west, blowing towards +x. |
| `<vertical>` | the world's `<wind>` | m/s, positive up. |
| `<speed_gust>` | 0 | Standard deviation of the speed gusts, m/s; 0 is a steady speed. |
| `<speed_gust_time>` | 2 | Their correlation time, s. |
| `<direction_gust>` | 0 | Standard deviation of the direction gusts, degrees. |
| `<direction_gust_time>` | 10 | Their correlation time, s. |
| `<reference_height>` | 10 | m above the water, the height `<speed>` is given at. |
| `<roughness_length>` | 0 | m, the surface roughness of a logarithmic profile, about 0.0002 over open sea; 0 is a uniform wind. |
| `<water_level>` | 0 | World z of the water. |
| `<model>` | standard | The registered wind model. |
| `<air_density>` | 1.225 | kg/m^3, for the windage. |
| `<default_drag_coefficient>` | 1 | Cd for shapes without `gz:wind_cd`. |
| `<seed>` | 0 | Seed of anything random in the model; 0 draws one each run. |
| `<publish_rate>` | 10 | Hz of simulation time for `wind_info`. |

Every parameter but `<model>` and `<publish_rate>` is also a key of the wind topic, a
`gz.msgs.Param` with double values, any of them in one message.

## Direction and units

- **Speed** in metres per second, at the reference height, 10 m unless the
  world says otherwise, as in weather reports.
- **Direction** the wind comes from, in degrees clockwise from true north,
  as in weather reports, ArduPilot and the marine textbooks: 0 is a north
  wind, blowing south; 270 is a west wind, blowing east.
- **North** is the world's, from its `<spherical_coordinates>`, the same
  north its GPS and magnetometer use. Without them, or with the `ENU`
  orientation and a `heading_deg` of 0, north is the world's +y axis and
  east its +x axis.
- **The ground truth** on `/world/<world>/wind_info` is the air's velocity
  in the world frame, not a direction.

## Gusts

Each gust, on the speed and on the direction, is a sum of 64 sinusoids with
random phases drawn from the seed, weighted to a Lorentzian spectrum: it
wanders around zero with the standard deviation asked for and forgets
itself over the correlation time, like the first order Gauss Markov process
VRX uses, but it is a function of time alone. Every system that rebuilds the
model from the recipe gets the same gust at the same time, and a reset
replays it.

The gusts travel with the mean wind (Taylor's frozen turbulence): a point
downwind sees what a point upwind saw earlier, so the bow and the stern of a
boat in line with the wind feel a gust one after the other.
They never travel slower than 1 m/s, since frozen turbulence does not hold
in near calm air: below that the delay across a boat would grow without
bound and the gusts at bow and stern would stop resembling each other.
At every height they travel at the reference height speed, not at the
slower wind of the profile, a small approximation near the water.

The speed never goes below zero. When `<speed_gust>` comes close to
`<speed>` the clamp cuts off the lulls, so the speed averages above
`<speed>` and spreads less than `<speed_gust>`; keep the gust well under
the mean for the statistics to hold.

## Wind with height

With a roughness length `z0`, the wind at a height `h` above the water is
the mean reference wind times `ln(h / z0) / ln(h_ref / z0)`: over the sea,
about 0.7 of the 10 m wind half a metre above the water, and none below
`z0`. The gusts are added after the profile, at full strength, since near
the sea they are about as strong at any height; `<vertical>` is not
profiled either. A `z0` at or above the reference height gives a uniform
wind, with a warning.
The wind entity holds the reference height wind, so Gazebo's rotor, wing
and air speed systems, which read that single value, do not see the
profile; the windage and any system that asks `WindAt` do.

## Asking the wind from a system

Link `gz_wind::gz_wind` and ask the wind at a point, in the world frame:

```cpp
#include <gz/sim/wind/WindSampler.hh>

// Once per step, occasional queries:
const gz::math::Vector3d v =
    gz::sim::wind::WindAt(_ecm, position, _info.simTime);

// Many queries per step: keep a sampler as a member.
this->sampler.Sync(_ecm);
const gz::math::Vector3d w = this->sampler.At(position, _info.simTime);
```

The sampler rebuilds its model only when the recipe's generation changes.

## Adding a wind model

Implement `gz::sim::wind::IWindModel`, a pure function of position (east,
north, up) and time, and register it before the first sampler asks for it:

```cpp
gz::sim::wind::RegisterWindModelFactory("my_model",
    [] { return std::make_unique<MyModel>(); });
```

A world then picks it with `<model>my_model</model>`.
