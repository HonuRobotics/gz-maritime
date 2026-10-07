# Ocean current plan

## Scope and success

What matters most is the architecture, the way the waves and the wind have
it: a current that every vehicle feels, whether it was loaded with the world
or spawned at run time, and a plugin structure that takes new current
models, simpler or richer, without a change to the system that owns the
current or to the systems that read it. On that, the model this phase ships
is the simplest that fits the use case. For the kilometre and the hour this
simulation works at, the current is **constant and horizontal**: the world file gives its speed
and the direction it sets towards, and that is the water for the whole run.
The demo uses a USV: the custom USV, engines off, drifts with the current at
the current's speed and in its direction, a second boat spawned later drifts
the same way, and a boat holding station under thrust has to push against
it with the force its damping predicts. A headless end to end test runs
that demo.

Out of this phase, each a separate issue, upstream where the code is, for
whoever needs it: a current that changes while the world runs (and the ramp
such a change needs, since a step in the current is a force spike through
any added mass term), that varies in time (gusts, a tide), with depth, with
place (a gridded current from NOAA data), or has a vertical component; the
thrusters feeling the current, which only an advance ratio nobody here
turns on would notice; rudders, keels and fins and the slipstream over a
rudder, which no vehicle here has; a speed log sensor, and Gazebo's DVL; a
marked collision load for vehicles without identified coefficients, since
every vehicle here has them (if it comes back: Fossen damping or the marked
load on the same axes, never both); the waves' own water motion, which stays
out of the relative velocity on purpose; and anything on an anchor.

The Gazebo side of this, what upstream has and what we should send back, is
in `OCEAN_CURRENT_UPSTREAM_REVIEW.md`, written to become a gz-sim issue.

## 1. A current the world owns

**Problem.** Gazebo's current is a topic: a bare vector that the
hydrodynamics plugin of each vehicle listens to, and that nothing else
reads. It arrives a variable number of steps after it was sent, so no two
runs agree; a vehicle with a namespace never hears it; and anyone else
publishing on it fights the publisher. VRX has no current at all.

**Example.** To try the same boat in slack water and then in a 0.5 m/s
current, you publish a vector by hand and hope every vehicle subscribed, a
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
horizontal and constant, so the point and the time change nothing today.
The query is a point query and nothing more. A world system,
`gz-maritime-ocean-current-system`, reads the recipe from the world file,
writes it, and publishes the current as ground truth on
`ocean_current_info`, a twist in the world frame that the simulation launch
bridges to ROS. It does not publish on Gazebo's `/ocean_current`; the
vehicles here get the current from the recipe.

## 2. A hull that feels it

**Problem.** Gazebo's hydrodynamics damps a vehicle against the ground, not
against the water, unless that vehicle subscribes to the current topic; and
a vehicle spawned after the first step never sees a current loaded from a
file at all, since the plugin looks for the data only while the world is
new, so every vehicle here, spawned at run time under its own name, would
sit in a current it cannot feel.

**Example.** A boat in a 0.5 m/s current with its engines off should drift
at 0.5 m/s. Today it sits still, because its hull is damped against a world
that is not moving, and a boat spawned as `boat_b` stays still even in a
world whose current came from a file.

**Solution.** Vendor Gazebo's hydrodynamics as `gz_hydrodynamics`, the way
`gz_thruster` and `gz_buoyancy` are vendored, with one change: it asks
`OceanCurrentAt` at its link's centre of mass and damps against that. This
is Fossen's model as written, on the velocity relative to the water, so a
vehicle keeps the coefficients it was identified with, needs no retune, and
any vehicle with Fossen coefficients feels the current the day it is
spawned. The vendored copy also fixes the late spawn, with a test, and that
fix is the first thing to send upstream. It is a fork until upstream reads
a current from the ECM; we retire it then.

Added mass goes through the plugin's own coefficients, never the SDF
`<fluid_added_mass>`, which only DART implements and would tie a vehicle to
one physics engine. On that path the plugin already uses the relative
velocity for both added mass terms, and with a constant current there is
no step for them to turn into a spike. One limit, stated so nobody expects
otherwise: the hydrodynamics asks the field once, at the centre of mass,
which is all a uniform current needs.

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
at the edge. The name is `ocean_current` everywhere: the package, the
system and the topic, matching Gazebo's and DAVE's topic.

## 4. Worlds, tests and documentation

**Problem.** No world has a current, no test checks it, and the docs never
mention it.

**Example.** A change that broke the late spawn would go unnoticed until a
second boat stopped drifting in someone's demo.

**Solution.** Put the current system in every world, slack by default, with
a sensible current for each site noted beside it: a tidal set in the
harbour, weak in the gulf, none on the rowing lakes. Switch the custom USV
to `gz-maritime-hydrodynamics-system` with its coefficients unchanged. Add a
headless test that loads the custom USV into a world with a set current and
checks that it drifts at the current's speed and in its direction, spawns a
second boat after the first step and checks that it drifts the same, and
holds one with thrust and checks the thrust it takes against its damping.
Write a short how to and design note on the recipe next to the wind's, add
the line to the world contract and the parameters and topic to the
reference, and the invariant to `AGENTS.md`.
