# gz_wind

The wind of a world for Gazebo Sim, built the way the wave field is.

The world owns the wind: a speed and the direction it comes from, in the
world file and changed at run time on the topic `/world/<world>/wind/set`.
The `gz::sim::maritime::Wind` world system keeps it as a recipe on the world
entity, the `Windfield` component: which model, with which parameters. Any
system rebuilds its own copy of the model from that recipe and asks the wind
at any point and time, so a new wind model is a new registered model and no
change to the systems that use the wind.

The system also writes the wind at the world's origin into the wind entity
every Gazebo world carries, where the rotor, wing and air speed systems read
it, and publishes it on `/world/<world>/wind_info` as `gz.msgs.Wind`.

Without `<speed>` or `<direction>` the wind starts as the world's
`<wind><linear_velocity>`. Gazebo's `WindEffects` system should not run in
the same world, since both would write the wind.

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
| `<model>` | standard | The registered wind model. |
| `<seed>` | 0 | Seed of anything random in the model; 0 draws one each run. |
| `<publish_rate>` | 10 | Hz of simulation time for `wind_info`. |

`speed`, `direction`, `vertical` and `seed` are also the keys of the wind topic, a
`gz.msgs.Param` with double values, any of them in one message.

## Direction and units

- **Speed** in metres per second, at 10 m, as in weather reports.
- **Direction** the wind comes from, in degrees clockwise from true north,
  as in weather reports, ArduPilot and the marine textbooks: 0 is a north
  wind, blowing south; 270 is a west wind, blowing east.
- **North** is the world's, from its `<spherical_coordinates>`, the same
  north its GPS and magnetometer use. Without them, or with the `ENU`
  orientation and a `heading_deg` of 0, north is the world's +y axis and
  east its +x axis.
- **The ground truth** on `/world/<world>/wind_info` is the air's velocity
  in the world frame, not a direction.

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
