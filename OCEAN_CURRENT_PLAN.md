# Ocean current plan

## Scope and success

What matters most is the architecture, the way the waves and the wind have
it. We want an ocean current that every vehicle feels, whether it was loaded
with the world or spawned at run time, and a plugin structure that takes new
current models, simpler or richer, without a change to the system that owns
the current or to the systems that read it. On that, the model this phase
ships is the simplest that fits the use case: for the kilometre and the hour
this simulation works at, the current is **constant and horizontal**. The
world file gives its speed and the direction it sets towards, and that is
the water for the whole run.

The demo uses a USV: the custom USV, engines off, drifts with the current at
the current's speed and in its direction, a second boat spawned later drifts
the same way, and a boat holding station under thrust has to push against
it with the force its wetted area predicts. A headless end to end test runs
that demo.

Out of this phase, each a separate issue, upstream where the code is, for
whoever needs it: a current that changes while the world runs (and the ramp
such a change needs, since a step in the current is a force spike through
any added mass term), that varies in time (gusts, a tide), with depth, with
place (a gridded current from NOAA data, which the recipe leaves room for),
or has a vertical component; a vendored hydrodynamics that damps Fossen
coefficients against the water, for a vehicle that keeps its identified
surge and sway damping instead of marking its hull; the thrusters feeling the
current, which only an advance ratio nobody here turns on would notice;
rudders, keels and fins and the slipstream over a rudder, which no vehicle
here has; a speed log sensor, and Gazebo's DVL; the wave surface's height in
the wetted area, which stays at the flat water level as the wind's does; the
waves' own water motion, which stays out of the relative velocity on
purpose; and anything on an anchor.

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
current model, its parameters (a speed and the direction it sets towards,
and a `source` string and a seed for models that need them) and a
generation, written as a component change and nothing else, so every
consumer, in every process, sees it on the same step. Because it lives on
the world entity, a system on a vehicle spawned an hour into the run reads
it exactly as one loaded with the world. Models register under a name in an
`IOceanCurrentModel` registry; any system asks for the current at a point
and a time with `OceanCurrentAt`, or keeps an `OceanCurrentSampler`,
without knowing which model is behind it, so a new model, a tide, a depth
profile or a grid from NOAA data, is one class and one registration and no
change to anything else. The model built in, `standard`, is uniform,
horizontal and constant. The query is a point query and nothing more: a
consumer that spans a gradient integrates over its own extent with repeated
queries. A world system, `gz-maritime-ocean-current-system`, reads the
recipe from the world file, writes it, and publishes the current as ground
truth on `ocean_current_info`, a twist in the world frame that the
simulation launch bridges to ROS. It does not publish on Gazebo's
`/ocean_current`.

## 2. A load on any vehicle

**Problem.** Gazebo damps a vehicle against the ground, not against the
water, unless that vehicle subscribes to the current topic, and a vehicle
spawned after the first step never sees a current loaded from a file at all.

**Example.** A boat in a 0.5 m/s current with its engines off should drift
at 0.5 m/s. Today it sits still, because its hull is damped against a world
that is not moving, and a boat spawned as `boat_b` stays still even in a
world whose current came from a file.

**Solution.** Do what buoyancy and the wind do. A vehicle marks the shapes
the water sees with `gz:ocean_current="true"` on their collisions, with
`gz:ocean_current_cd` for a drag coefficient other than one. The pontoon box
a boat already marks for buoyancy and wind carries this mark too, and the two
drag systems split it at the waterline: the wind takes the part above, the
current the part below. The ocean current system looks for marked shapes on
every model, whenever it shows up, takes their projected areas from the
geometry, cuts them at the water level, asks `OceanCurrentAt` at the centre
of each submerged part, and applies quadratic drag, `0.5 * rho * Cd * A *
|v| * v` per shape axis, on the water's velocity relative to that point, at
that point. A current across an offset shape turns the boat, as a mast heels
it in the wind, and a submerged vehicle has every marked shape wetted. The
shape bookkeeping the wind system already has, the areas, the cut and the
per shape drag, moves to a small library both systems share rather than
being written twice. The system takes `<water_density>` (1025),
`<water_level>` (0) and `<default_drag_coefficient>` (1).

The current is the water, not a force on top of it, so a body with no other
horizontal drag drifts at exactly the current's speed, and a body damped
against the ground as well does not: kept beside ground relative Fossen
damping, the marked load makes a boat drift at about a quarter of the
current. The rule, in the invariants: *a vehicle that marks its hull drops
the surge and sway terms from its hydrodynamics plugin* (`xU`, `xUabsU`,
`yV`, `yVabsV` and their cross terms), *which the marks now provide relative
to the water, and keeps heave, roll, pitch and yaw*, which a horizontal
current leaves alone. Those stay on Gazebo's own hydrodynamics system,
unchanged: nothing it still computes depends on the current.

This changes how the custom USV moves, and the change is stated so nobody is
surprised. Its Fossen coefficients are placeholders borrowed from the
BlueBoat; the marked load on its pontoons is quadratic only and much lighter
(about 6 N against 50 N in surge at 1 m/s, 10 N against 42 N in sway at
0.5 m/s), so its top speed rises, it turns more freely and the wind drift
numbers change. That is the drag its shape predicts; the drag coefficient
of the marks is the knob to tune it against the real boat.

## 3. Direction and units

**Problem.** A current is given by the direction it flows towards, its set,
in degrees clockwise from north, as charts draw it and NOAA predicts it,
and the wind by the direction it comes from. DAVE and UUV Simulator give a
current as an angle counter clockwise from east, in radians, as VRX gave
the wind, and the speed in metres per second where tide tables use knots.

**Example.** A westerly wind of 270 and a current setting 090 move a boat
the same way, and a DAVE current of 1.57 is a current setting north.

**Solution.** The direction the current sets towards, in degrees clockwise
from north, and the speed in metres per second. Say in the docs, next to
the wind, that the two directions follow their own trades, and convert once
at the edge. The name is `ocean_current` everywhere: the package, the mark,
the system and the topic, matching Gazebo's and DAVE's topic.

## 4. Worlds, tests and documentation

**Problem.** No world has a current, no vehicle is marked, no test checks
it, and the docs never mention it.

**Example.** A change that broke the marks would go unnoticed until a boat
stopped drifting in someone's demo.

**Solution.** Put the current system in every world, slack by default, with
a sensible current for each site noted beside it: a tidal set in the
harbour, weak in the gulf, none on the rowing lakes. Mark the custom USV's
pontoons and drop the surge and sway terms from its hydrodynamics. Add
system tests for the load: a marked box drifts at the current's speed, a
half submerged one takes half the area, `gz:ocean_current_cd` is read, a box
spawned later is found, an unmarked one does not move. Add a headless end to
end test that loads the custom USV into a world with a set current and
checks that it drifts at the current's speed and in its direction, spawns a
second boat after the first step and checks that it drifts the same, and
holds one with the thrust its wetted area predicts. Add an "Ocean current
markup" table to the reference next to the wind's, the mark and the rule to
the composition how to, and the rule to `AGENTS.md`. Mark the Blue Robotics
vehicles with the same rule applied to their coefficients, in their own
repository.
