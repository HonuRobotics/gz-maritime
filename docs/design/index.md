# Design

How the pieces fit together, and why they are built this way.

## The waves

One wave field is shared by everything that needs it: physics, the renderer,
and future sensors. Wave models are plugins, and nothing that reads the waves
depends on a particular model, so adding a new wave model changes nothing
else. The full architecture, requirements and the guide to adding a wave
model are in
[WAVES_DESIGN.md](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/WAVES_DESIGN.md).

## Floating vehicles the world doesn't know

**The problem.** Gazebo's buoyancy system floats the links named in the
world file's `<enable>` list, or, with no list, every collision of every
model. A world shared by many vehicles, some spawned while it runs, can't
know their names in advance; and floating everything is wrong, because a
vehicle's contact geometry (brackets, hatches, thruster housings) is rarely
what displaces its water.

**The options considered:**

| Option | Why it was not chosen |
|---|---|
| List every vehicle's links in the world | The world has to know every vehicle before it starts, and a vehicle spawned later, or a second copy of one, never floats. |
| No list at all | Everything floats, contact geometry included. |
| Generate the world from a template at launch, filling in `<enable>` | The world still needs every vehicle's link names, and every custom world needs the same template machinery. |
| Enable and disable services on the world plugin, with a model plugin that calls them | Works, but needs transport, retries until the service exists, and a re-scan of links that already exist. Two plugins where one will do. |
| A second, model-scoped instance of the plugin reading the world's fluid | Works, but two instances can push the same link twice, and every plugin loop has to be scoped to its model. |
| **Mark the collisions that displace** | Chosen. |

**The chosen design.** A collision carries an attribute, `gz:buoyancy="true"`,
and a link with any marked collision floats by those alone, whatever the
model is called. The world adds `<enable_by_default>false</enable_by_default>`
so that unmarked collisions never float, and names nobody.

```{mermaid}
flowchart LR
  M["model.sdf<br/><i>collision gz:buoyancy=&quot;true&quot;</i>"]
  S["ros_gz_sim create<br/><i>any name</i>"]
  B["Buoyancy system<br/><i>enable_by_default false</i>"]
  M --> S --> B
  B -->|floats| L["the marked collisions only"]
```

Why it is the simplest of the six:

- **Plain SDF.** The mark is data in the model file. No service, no second
  plugin, no timing, no name lookup.
- **Contact and displacement are separate shapes.** A bracket keeps its
  contact box and adds no buoyancy; a hull gets both.
- **Renaming and copies are free**, because nothing refers to a name.
- **Nothing changes for existing worlds.** With neither tag the plugin
  behaves as it always has.

**Where it lives.** The change is upstream in Gazebo,
[gz-sim commit 229ec07e](https://github.com/gazebosim/gz-sim/commit/229ec07e673b6317fec57af50fd652bcd9bde0ed).
Until a Gazebo release with it reaches the ROS vendor packages,
gz-maritime's `gz_buoyancy` package carries the plugin with that change
applied, plus a workaround for a gz-math bug that crashed the simulation
when a box hull floated almost level
([gazebosim/gz-math#847](https://github.com/gazebosim/gz-math/pull/847)).
When the release arrives, worlds switch the plugin name back to
`gz-sim-buoyancy-system` and no vehicle changes; `gz_buoyancy/PROVENANCE.md`
records the details.

## One name per instance

Several vehicles in one simulation need separate topics, ROS names and TF
trees. The [multi-vehicle plan](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/MULTI_VEHICLE_PLAN.md)
lays the rules out; the tutorial USV follows them:

- the **simulation part** of a launch (server, GUI, `/clock`) is separate
  from the **spawn part** (one instance: model, bridge, state publisher),
  so the first runs once and the second once per vehicle;
- one **name** per instance drives the model name, every topic, the ROS
  namespace and `robot_state_publisher`'s `frame_prefix`; the URDF carries
  no name, so one description serves every instance;
- a vehicle's bridge carries **no `/clock`**; the simulation part bridges it
  once.

## Buoyancy that follows the waves

Today vehicles float on the flat water level, and the drawn waves pass
through their hulls ([Waves and physics today](../how-to/own-vehicle.md#waves-and-physics-today)).
Coupling buoyancy to the waves is planned, as a separate consumer of the wave
field. The main points it has to handle:

- **A local water surface per collision shape.** Sample the wave height
  around each shape, fit a plane, and measure the volume under that plane
  instead of under z = 0.
- **Long hulls.** One plane per shape is only good when the shape is short
  compared to the wavelength, so long hulls need splitting into segments.
- **Update rate.** The wave field updates at `<update_rate>` (30 Hz in
  `open_water.sdf`), slower than physics (250 Hz), so the forces need
  interpolating.
- **Choppy seas.** The high-fidelity sea also moves the surface sideways;
  finding the true height at a point needs that displacement.

## Related

The [bluerobotics_models](https://github.com/HonuRobotics/bluerobotics_models)
documentation explains how the BlueBoat and BlueROV2 are assembled from parts,
and how they keep their floating volume on a link of its own.
