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
sets towards, uniform over the world and horizontal. On the
kilometre and hour scales this simulation works at, that is the water.

The same system pushes on the shapes the water sees, a hydrodynamic model
for a vehicle without identified coefficients. A vehicle marks them with
`gz:ocean_current="true"` on a collision (a zero `collide_bitmask` keeps a
dedicated shape out of contact, and a buoyancy box can carry the buoyancy
and wind marks too: the wind takes the part above the water, the current the
part below). The system finds those shapes on every model, spawned at any
time under any name, and applies quadratic drag,
`0.5 * rho * Cd * A * |v| * v` per shape axis, on the projected area of the
part below `<water_level>`, at the centre of that part, with the current
asked at that centre. The velocity is the water relative to the shape, so a
boat with nothing else holding it drifts at the current's speed.
`gz:ocean_current_cd` sets a shape's drag coefficient. The shape code is
shared with the wind's windage, in `gz_marked_shapes`.

Each face of a shape is cut by how vertical its axis is, so a shape on
its side, such as a cylinder lying along the hull, is cut on the face the
current sees and keeps its plan area whole. The cut is at the flat
`<water_level>`, not the wave surface, like the windage and the buoyancy.

A vehicle uses one model, its Hydrodynamics plugin or the marks, never
both. The marks damp more than surge and sway: the plan area damps heave,
drag on the part below the centre of mass rolls and pitches the hull, and a
shape off the centre line yaws it, so any plugin damping beside them counts
those axes twice. A Hydrodynamics plugin with yaw terms only (`nR`,
`nRabsR`) is the most a marked vehicle may add, and only when no marked
shape is off the centre line.
The drag acts at the centre of each wet part, so a single centred box has
no yaw damping at all; a hull of two pontoons does.

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
| `<water_level>` | 0 | World z of the water's surface: where the load on marked shapes cuts them, and where a model that varies with depth puts the surface; `standard` does not read it. |
| `<parameters>` | none | The parameters the model owns, one element each, such as `<source>`, the file of a gridded current. Opaque to the system, which only stores and replicates them as text; the model accepts or refuses them. A leaf is its text; an element with children or attributes of its own, such as `<layer><depth>10</depth></layer>`, is its SDF text, for the model to parse. A name that repeats is a list, numbered in order: three `<constituent>` elements are `constituent.0`, `constituent.1` and `constituent.2`. `standard` takes none. |
| `<seed>` | 1 | Seed of anything random in a model, fixed by default so a run repeats; 0 draws a new one each run. `standard` has nothing random. |
| `<publish_rate>` | 10 | Hz of simulation time for the ground truth. |
| `<water_density>` | 1025 | kg/m^3, for the load on marked shapes. |
| `<default_drag_coefficient>` | 1 | Cd for marked shapes without `gz:ocean_current_cd`. |

Any other element is warned about and ignored, so a typo does not leave
slack water in silence. The system checks the model once, at the first step
(a plugin may register it after the world is loaded): a model nobody
registered, or one that refuses its parameters, is one error, and the world
has no current and no ground truth.

## Changing it while the world runs

The current changes on the topic `/world/<world>/ocean_current/set`, a
`gz.msgs.Param` whose keys are parameter names: `speed`, `direction`,
`water_level` and `seed` take a double or an integer; any other key is a
parameter the model owns and takes a string or a number. A message is
queued and applied at the next step, as a new recipe, so every consumer sees
the change on the same step. It is applied whole or not at all: one key out
of range, of the wrong type, or refused by the model, and it changes
nothing. It cannot change `<model>`: a `model` key is refused. A reset puts the world file's current
back.
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
`gz::sim::ocean_current::OceanCurrentAt(ecm, position, time)` in one call,
which builds the model each time and suits an occasional query; a system
that asks every step keeps a sampler. Both take and return world frame
vectors. It is a point query and nothing
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
the recipe and every consumer run it unchanged. A model's own parameters
come from the world's `<parameters>` block, or a key on the topic, as text
in `params.extra`: a gridded current reads `source`, a tide its
constituents, `constituent.0`, `constituent.1` and so on, and a depth
profile the SDF text of each `<layer>`, which it parses itself. The model refuses what it cannot read in `Validate`, and the
system then refuses the change. A model that places itself on the Earth gets
the world's spherical coordinates in `SetSphericalCoordinates`; one that
needs randomness draws from `<seed>`.

```xml
<plugin filename="gz-maritime-ocean-current-system"
        name="gz::sim::maritime::OceanCurrent">
  <model>my_model</model>
  <parameters>
    <source>noaa_blended_currents.nc</source>
    <constituent>M2 0.25 0</constituent>
    <constituent>S2 0.1 30</constituent>
    <layer><depth>10</depth><speed>0.2</speed></layer>
  </parameters>
</plugin>
```
