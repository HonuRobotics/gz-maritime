# Ocean current plan

## Scope and success

What matters most is the architecture, the way the waves and the wind have
it. We want an ocean current that the world owns and any system can ask at a
point, a plugin structure that takes new current models, simpler or richer,
without a change to the system that owns the current or to the systems that
read it, and vehicles that feel it whether they were loaded with the world or
spawned at run time. On that, the model this phase ships is the simplest that
fits the use case: for the kilometre and the hour this simulation works at,
the current is **uniform and horizontal**. The world file gives its speed and
the direction it sets towards, and a topic bridged to ROS changes them while
the world runs.

A vehicle's hydrodynamic model is one set of equations, added mass, Coriolis
and damping, on its velocity relative to the water, and it belongs in one
plugin. So the vehicles here keep the hydrodynamics plugin and the
coefficients they were identified with, and that plugin reads the current
from the world. The end state is upstream Hydrodynamics reading an upstream
current component; a vendored copy reading `OceanCurrentAt` is the step
towards it.

The demo uses a USV: the custom USV, engines off, drifts with the current at
the current's speed and in its direction, a second boat spawned later drifts
the same way, and a boat holding station under thrust has to push against it
with the force its damping predicts. The BlueBoat and the BlueROV2 drift the
same way. In a slack world, the default, nothing about any vehicle changes.

The work lands in three pull requests, and a fourth later:

1. **A current the world owns** (§1, §4): the recipe, the model registry, the
   sampler, the world system, the `set` topic and the ground truth, in every
   world.
2. **A hull that feels the current** (§2): Gazebo's Hydrodynamics vendored as
   `gz_hydrodynamics`, taking the current from `OceanCurrentAt`, with the late
   spawn fix as its own commit, meant for upstream.
3. **The vehicles drift with the current** (§3): the custom USV, the BlueBoat
   and the BlueROV2 switched to it, coefficients unchanged; the Blue Robotics
   change in `bluerobotics_models`.
4. **Later, a geometric model** (§5): the load on collisions marked
   `gz:ocean_current="true"`, for vehicles without identified coefficients,
   and the upstream work that retires the vendored copy.

Out of this phase, each a separate issue, upstream where the code is, for
whoever needs it: a ramp between two currents (a change on the topic is a
step, which the plugin's added mass terms turn into a force spike, so a
vehicle that carries added mass sees one until a ramp lands); a current that
varies in time on its own (gusts, a tide), with depth, with place (a gridded
current from NOAA data, which the recipe leaves room for), or has a vertical
component; the thrusters feeling the current, which only an advance ratio
nobody here turns on would notice; rudders, keels and fins and the
slipstream over a rudder, which no vehicle here has, and which stock
LiftDrag cannot do since it cannot read a gz-maritime component (the path is
a vendored LiftDrag reading `OceanCurrentAt` until the upstream component
lands); a speed log sensor, and Gazebo's DVL; the waves' own water motion,
which stays out of the relative velocity on purpose, since manoeuvring
coefficients are identified in calm water; and anything on an anchor.

The Gazebo side of this, what upstream has and what we should send back, is
in `OCEAN_CURRENT_UPSTREAM_REVIEW.md`, written to become a gz-sim issue.

## 1. A current the world owns

**Problem.** Gazebo's current is a topic: a bare vector that the
hydrodynamics plugin of each vehicle listens to, and that nothing else
reads. It arrives a variable number of steps after it was sent, so no two
runs agree; a vehicle with a namespace never hears it; and anyone else
publishing on it fights the publisher. VRX has no current at all.

**Example.** To try the same boat in slack water and then in a 0.5 m/s
current, you publish a vector by hand and hope every vehicle subscribed; a
sensor or a scoring system has no way to ask what the current is, and the
drift test cannot say on which step the current began.

**Solution.** Treat the current the way we treat the wind, the waves and
buoyancy: in the ECM, with no transport on the data path. An
`OceanCurrentfield` component on the world holds a recipe: the name of a
current model, its parameters and a generation, written as a component
change and nothing else, so every consumer, in every process, sees it on the
same step. The parameters are a speed and the direction the current sets
towards; the world z of the water (`water_level`), as the wind's recipe has
it, so a depth model can find the surface; and a `source` string and a seed
for models that need them. The `source` is opaque to the system: it stores
and replicates it, and only the model that reads it gives it meaning, such
as the file of a gridded current. Because the recipe lives on the world
entity, a system on a vehicle spawned an hour into the run reads it exactly
as one loaded with the world.

Models register under a name in an `IOceanCurrentModel` registry; any system
asks for the current at a point and a time with `OceanCurrentAt`, or keeps
an `OceanCurrentSampler`, without knowing which model is behind it, so a new
model, a tide, a depth profile or a grid, is one class and one registration
and no change to anything else. The model built in, `standard`, is uniform
and horizontal. The query is a point query and nothing more: a consumer that
spans a gradient integrates over its own extent with repeated queries.

A world system, `gz-maritime-ocean-current-system`, reads the recipe from the
world file and writes it. Topics touch it only at the boundary, as the
wind's do: `/world/<world>/ocean_current/set`, a `gz.msgs.Param` with `speed`
and `direction` as doubles and `source` as a string, is queued under a mutex
and applied at the next `PreUpdate`, so a change lands on one known step; the
simulation launch bridges it from ROS as `ros_gz_interfaces/msg/ParamVec`.
The current is published as ground truth on `ocean_current_info`, a twist in
the world frame, bridged to ROS. A reset puts the world file's current back.
The system does not publish on Gazebo's `/ocean_current`.

## 2. A hull that feels the current

**Problem.** Gazebo's hydrodynamics damps a vehicle against the ground, not
against the water, unless that vehicle subscribes to the current topic; and a
vehicle spawned after the first step never sees a current loaded from a file
at all, since the plugin looks for the data only while the world entity is
new, so every vehicle here, spawned at run time under its own name, would sit
in a current it cannot feel.

**Example.** A boat in a 0.5 m/s current with its engines off should drift at
0.5 m/s. Today it sits still, because its hull is damped against a world that
is not moving, and a boat spawned as `boat_b` stays still even in a world
whose current came from a file.

**Solution.** Vendor Gazebo's Hydrodynamics as `gz_hydrodynamics`, the way
`gz_thruster` and `gz_buoyancy` are vendored, from the gz-sim release ROS
ships, verbatim in its own commit so the delta reads on its own. One change:
when the world has an ocean current recipe, the plugin takes ν_c from
`OceanCurrentAt` at its link's centre of mass every step, so damping, added
mass and Coriolis all use the velocity relative to that water, Fossen's
model as written. With no recipe in the world, its old inputs,
`<default_current>`, `<lookup_current_*>` and the `/ocean_current` topic,
behave exactly as today. A vehicle keeps the coefficients it was identified
with, needs no retune, and feels the current the day it is spawned.

The late spawn fix goes in a separate commit, with its test, meant for
upstream: read the world's environmental data whenever it is present instead
of only while the world entity is new, and find the world by its component
at the first step rather than from the model at configure time, since a
model spawned at run time is not parented yet when it is configured.
`PROVENANCE.md` lists both changes and states that the package is a fork
until upstream has a current component, and is retired then.

Added mass goes through the plugin's own coefficients (`<xDotU>` and the
rest), never the SDF `<fluid_added_mass>`, which only DART implements and
would tie a vehicle to one physics engine. Plugin added mass is numerically
unstable as it approaches the body's mass, so it stays small or out, as it is
on all three vehicles today. The plugin samples the current once per link,
which is all a uniform current needs; a current that varies along one hull
is out of this phase.

## 3. The vehicles drift with the current

**Problem.** The custom USV, the BlueBoat and the BlueROV2 run Gazebo's own
Hydrodynamics, which ignores the world's current.

**Example.** In a world with a 0.5 m/s current, all three sit still.

**Solution.** Switch each to `gz-maritime-hydrodynamics-system`, with its
coefficients unchanged, and add `gz_hydrodynamics` to its dependencies. In a
slack world nothing changes. The custom USV's switch is in this repository;
the BlueBoat's and the BlueROV2's are a pull request in `bluerobotics_models`,
which also corrects the BlueROV2's comment that recommends `<fluid_added_mass>`
as the stable path for added mass (`bluerov2_gazebo/model.sdf.xacro:426-429`),
to point to the plugin coefficients instead.

Each vehicle gets a headless drift test: in a world with a 0.5 m/s current
setting east, with no thrust, it settles at the current's velocity over the
ground. With the vehicles' linear damping terms the approach is close to
exponential and quick; a vehicle with quadratic damping only approaches the
current as `u / (1 + k u t)` (`k` the quadratic coefficient over the mass),
about 6 % short after a minute for the custom USV's size, so a test of such a
vehicle compares against that curve rather than against the current itself.
The custom USV's end to end test also spawns a second boat after the first
step and checks it drifts the same, and holds one boat against the current
with the thrust its surge damping predicts at the current's speed.

## 4. Direction, units, worlds and documentation

**Problem.** A current is given by the direction it flows towards, its set,
in degrees clockwise from north, as charts draw it and NOAA predicts it, and
the wind by the direction it comes from. VRX gave the wind the other way
again, the direction it blows towards in degrees counter clockwise from east.
DAVE gives a current's horizontal angle in radians and builds the vector as
`(cos a, sin a)` (`dave_gazebo_world_plugins/src/ocean_current_world_plugin.cc:126-129`,
Field-Robotics-Lab/dave `e54ea16`), counter clockwise from the world's +x,
which is east in an ENU world. Tide tables give speeds in knots.

**Example.** A westerly wind of 270 and a current setting 090 move a boat the
same way, and a DAVE current with a horizontal angle of 1.57 sets north.

**Solution.** The direction the current sets towards, in degrees clockwise
from north, and the speed in metres per second. North is the world's, from
its spherical coordinates, the one its GPS uses. Say in the docs, next to the
wind, that the two directions follow their own trades, and convert once at
the edge. The name is `ocean_current` everywhere: the package, the system and
the topics, matching Gazebo's and DAVE's topic.

Every world runs the ocean current system, slack: `open_water.sdf` with a
speed of 0 and a note that open water takes any current, 0.5 m/s being a
brisk coastal set; the site worlds the same, with a note on what fits each
site, a tidal set in the harbour, weak in the gulf, none on the rowing lakes.
The reference gets the world contract line, the parameters and the topic
with its ROS and Gazebo commands; the composition how to says which
hydrodynamics plugin a vehicle should name, and that gz-sim's own does not
see the world's current; `AGENTS.md` gets that as an invariant.

## 5. Later: a geometric model, and upstream

**Problem.** A vehicle without identified coefficients has nothing to damp
it, and gz-maritime carries a fork of Hydrodynamics.

**Example.** Someone brings a new hull with no tank or sea trial data and
wants it to drift and turn plausibly in a current.

**Solution.** The load on marked collisions, as a different hydrodynamic
model rather than a supplement: a vehicle marks the shapes the water sees
with `gz:ocean_current="true"` (and `gz:ocean_current_cd`), and the ocean
current system applies quadratic drag on the submerged part of each, against
the water at that part, with the shape code shared with the wind's windage.
A vehicle uses one model on each axis, Fossen or marks, never both, and both
read the current from the ECM. Separately, send the late spawn fix, the
current component and the Hydrodynamics change upstream, and retire
`gz_hydrodynamics` once a Gazebo release that ROS ships carries them.
