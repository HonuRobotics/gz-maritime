# gz_ocean_current

The ocean current of a world for Gazebo Sim, built the way the wind and the
wave field are.

The world owns the current: a recipe on the world entity, the
`OceanCurrentfield` component, naming a current model and its parameters,
read from the world file by the `gz::sim::maritime::OceanCurrent` world
system and changed at run time on a topic. Any system rebuilds its own copy of the
model from that recipe and asks the current at a point, whether it was
loaded with the world or spawned long after: the recipe is a component, so
every consumer, in every process, sees it on the same step, and nothing
travels over transport on the way. A new current model, simpler or richer,
is one class registered under a name; the system that owns the recipe and
the systems that ask for the current do not change.

The model built in, `standard`, is a speed and the direction the current
sets towards, uniform over the world, horizontal and constant. On the
kilometre and hour scales this simulation works at, that is the water.

The system publishes the current at the world's origin as ground truth on
`/world/<world>/ocean_current_info`, a `gz.msgs.Twist` in the world frame,
which ROS can bridge.

## World plugin

```xml
<plugin filename="gz-maritime-ocean-current-system"
        name="gz::sim::maritime::OceanCurrent">
  <speed>0.5</speed>
  <direction>90</direction>
</plugin>
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `<model>` | standard | The registered current model. |
| `<speed>` | 0 | m/s. |
| `<direction>` | 0 | Degrees clockwise from north the current sets towards: 90 sets east, towards +x. |
| `<source>` | empty | An external source for a model that reads one, such as the file of a gridded current; unread by `standard`. |
| `<seed>` | 0 | Seed of anything random in a model; 0 draws one each run. `standard` has nothing random. |
| `<publish_rate>` | 10 | Hz of simulation time for the ground truth. |

## Changing it while the world runs

The current changes on the topic `/world/<world>/ocean_current/set`, a
`gz.msgs.Param` whose keys are parameter names: `speed` and `direction` as
doubles, `source` as a string, any of them in one message. A message is
queued and applied at the next step, as a new recipe, so every consumer sees
the change on the same step; a reset puts the world file's current back.
From ROS, through the simulation launch's bridge:

```bash
ros2 topic pub --once /world/default/ocean_current/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: speed, value: {type: 3, double_value: 0.5}}, {name: direction, value: {type: 3, double_value: 90.0}}]}"
```

A change is a step: the current does not ramp from one value to the next.

## Direction and units

- **Speed** in metres per second.
- **Direction** the current sets towards, in degrees clockwise from true
  north, as charts draw it and tide tables predict it: 0 sets north, 90 sets
  east. This is the opposite convention from the wind, which is given by the
  direction it comes from: a wind from 270 and a current setting 090 move a
  boat the same way.
- **North** is the world's, from its `<spherical_coordinates>`, the same
  north its GPS and magnetometer use. Without them, or with the `ENU`
  orientation and a `heading_deg` of 0, north is the world's +y axis and
  east its +x axis.
- **The ground truth** on `/world/<world>/ocean_current_info` is the water's
  velocity in the world frame, not a direction.

## Asking the current

A system keeps a `gz::sim::ocean_current::OceanCurrentSampler`, calls
`Sync` once per step and `At` as often as it likes, or asks
`gz::sim::ocean_current::OceanCurrentAt(ecm, position, time)` in one call.
Both take and return world frame vectors. It is a point query and nothing
more: a consumer that spans a gradient integrates over its own extent with
repeated queries.

## Adding a model

Implement `gz::sim::ocean_current::IOceanCurrentModel`, a pure function of
position (east north up, metres) and time (seconds) built from the recipe's
parameters, and register it:

```cpp
ocean_current::RegisterOceanCurrentModelFactory("my_model",
    [] { return std::make_unique<MyModel>(); });
```

A world then names it with `<model>my_model</model>`. The system that owns
the recipe and every consumer run it unchanged; a model that needs a file
reads `<source>`, one that needs randomness draws from `<seed>`.
