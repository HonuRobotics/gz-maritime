# Ocean current plan

## Scope and success

We want an ocean current that a world sets and can change while it runs,
that every vehicle in the water feels through the hydrodynamics it already
carries no matter what the vehicle is called or when it appeared, that a
submerged vehicle feels at its own depth, and that the thrusters feel as
well as the hull. The demo uses a USV: the custom USV, engines off, drifts
with the current at the current's speed and in its direction, a second boat
spawned later drifts the same way, and a boat holding station under thrust
has to push against it with the force its damping predicts. A headless end
to end test runs that demo. No submerged vehicle is in the demo, but the
current has to be right at depth so that nothing changes the day a
BlueROV2 is added.

Out of this phase, each with its own issue once the uniform current works:
a current that changes from place to place, such as a map built from NOAA's
blended currents, which fits as one more registered model behind the same
recipe and is open to the contribution offered on #38; the waves' own
water motion, which stays out of the relative velocity on purpose, since
manoeuvring coefficients are identified in calm water and the wave response
belongs to buoyancy and the wave forces; the wind driving the surface
current; rudders and keels, and the slipstream of a propeller over a rudder;
a speed log sensor, and Gazebo's DVL, whose water mass mode reads only the
environment component and needs rendering, so it will not see this current;
and anything on an anchor.

The Gazebo side of this, what upstream has and what we should send back, is
in `OCEAN_CURRENT_UPSTREAM_REVIEW.md`, written to become a gz-sim issue.

## 1. A current the world owns

**Problem.** Gazebo's current is a topic: a bare vector that the
hydrodynamics plugin of each vehicle listens to, and that nothing else
reads. DAVE and UUV Simulator add a world plugin that publishes on that
topic, and inherit the same limits: the current arrives a variable number
of steps after it was sent, so no two runs agree; it is one vector for the
whole world; a vehicle with a namespace never hears it; and anyone else
publishing on it fights the publisher. VRX has no current at all.

**Example.** To try the same boat in slack water and then in a 1 m/s
current, you publish a vector by hand and hope every vehicle subscribed;
a sensor or a scoring system has no way to ask what the current is at the
boat, and the drift test cannot say on which step the current began.

**Solution.** Treat the current the way we treat the wind, the waves and
buoyancy: in the ECM, with no transport on the data path. An
`OceanCurrentfield` component on the world holds a recipe: the name of a
current model, its parameters, its seed and a generation that goes up on
every change, written as a component change and nothing else. Models
register under a name in an `IOceanCurrentModel` registry, and any system
asks for the current at a point and a time with `OceanCurrentAt`, or keeps
an `OceanCurrentSampler`, without knowing which model is behind it, so a
new model later means a new engine and no change to the systems that use
it. A world system, `gz-maritime-ocean-current-system`, owns the recipe: it
reads it from the world file, and topics touch it only at the boundary, as
the wind's do: `ocean_current/set` is queued under a mutex and applied in
`PreUpdate`, so a change lands on one known step, and `ocean_current_info`
is the ground truth. The system does not publish on Gazebo's
`/ocean_current`; the vehicles here get the current from the recipe. A
change through `set` ramps over a time the world sets rather than stepping,
because a step in the current is a step in the relative velocity and the
added mass turns it into a force spike; the recipe keeps the current before
the change and the time of the change, so the ramp is a function of time
like everything else and every copy agrees on it.

## 2. Variability

**Problem.** Gazebo's current is constant until someone publishes a new
vector. DAVE and UUV Simulator vary it with a Gauss Markov process that
draws from the clock, so no two runs agree.

**Example.** You cannot write a station keeping test against a current
that wavers, and running a wavering test twice gives two different drifts.

**Solution.** The same seeded spectral fluctuation the wind uses, on the
speed and on the direction: a sum of sinusoids with phases drawn from a
seed, with the correlation time of the Gauss Markov process the textbooks
prescribe, but a function of time alone, so every system that asks
`OceanCurrentAt` computes the same current without sharing any state. The
fluctuation is slower and smaller than the wind's, as water is, and the
world sets how strong and how long. The `Gust` the wind uses is private to
`gz_wind`; it moves to a shared header in a small library both packages
link, rather than a copy here or a dependency on the wind package.

## 3. Tide

**Problem.** Every Gazebo current, and every third party one but DAVE's,
has no tide: it never turns.

**Example.** A mission that runs over an afternoon in Sydney Harbour or
Honolulu sees the current reverse once, and the simulation never shows it.

**Solution.** A tidal current as a sum of harmonic constituents, the way
tide tables are made: each constituent an amplitude, a phase and a speed in
degrees per hour, with the standard set (M2, S2, N2, K1, O1) and room for
more, along a flood direction and an ebb direction the world sets, from a
start time the world sets. It is one more sum of sinusoids, so it is a
function of time alone like the variability, it adds to the mean current,
and being continuous it needs no ramp. A world that wants to see a tidal
cycle in minutes scales the constituent speeds; a world that wants a site's
real tide copies the constituents from the published harmonic analysis.

## 4. Current with depth

**Problem.** Every Gazebo current is the same at the surface as on the
bottom.

**Example.** A ROV inspecting a hull sees the same current as the boat
above it, where the real current is a fraction of the surface's and often
turned.

**Solution.** The speed and direction are given at the surface, and an
optional profile gives the current at depths below it: a short table of
layers, each a depth, a speed and a direction, interpolated between layers
and held below the last, uniform without one. It lives inside the current
model, so `OceanCurrentAt` returns the current at the depth it is asked
for, and the variability and the tide apply to the surface current and
scale with it. Every consumer asks at its own point, the hull at its centre
of mass, a propeller at the propeller, a marked shape at its submerged
centre, so a ROV gets the current at its depth and a boat the surface's,
with no logic on the vehicle.

## 5. A hull that feels it

**Problem.** Gazebo's hydrodynamics damps a vehicle against the ground, not
against the water, unless that vehicle subscribes to the current topic; and
a vehicle spawned after the first step never sees a current loaded from a
file at all, since the plugin looks for the data only while the world is
new, so every vehicle here, spawned at run time under its own name, would
sit in a current it cannot feel.

**Example.** A boat in a 1 m/s current with its engines off should drift at
1 m/s. Today it sits still, because its hull is damped against a world that
is not moving, and a boat spawned as `boat_b` stays still even in a world
whose current came from a file.

**Solution.** Vendor Gazebo's hydrodynamics as `gz_hydrodynamics`, the way
`gz_thruster` and `gz_buoyancy` are vendored, with one change: it asks
`OceanCurrentAt` at its link's centre of mass every step and damps against
that. This is Fossen's model as written, on the velocity relative to the
water, so a vehicle keeps the coefficients it was identified with, needs no
retune, and any vehicle with Fossen coefficients feels the current the day
it is spawned. The vendored copy also fixes the late spawn, with a test,
and that fix is the first thing to send upstream. It is a fork until
upstream has a current component; we retire it then.

The marked load stays, for vehicles without identified coefficients. A
vehicle marks the shapes the water sees with `gz:ocean_current="true"` on
their collisions, with `gz:ocean_current_cd` for a drag coefficient other
than one, and the system applies quadratic drag on the submerged part of
each shape, on the water's velocity relative to that part, at its centre,
with the shape bookkeeping the wind system uses moved to the shared library
rather than written twice. It is not a substitute for Fossen damping where
that exists: on the custom USV's pontoons it gives about 10 N against 42 N
at 0.5 m/s in sway and 6 N against 50 N at 1 m/s in surge, with no linear
term at low speed, so top speed, turning and the wind drift numbers would
all change; and kept beside ground relative Fossen damping it makes a boat
drift at about a quarter of the current, since both resist. The rule, in
the invariants: a vehicle uses Fossen damping or the marked load on the
same axes, never both.

Added mass goes through the plugin's own coefficients, never the SDF
`<fluid_added_mass>`, which only DART implements and would tie a vehicle to
one physics engine. On that path the plugin already uses the relative
velocity for both added mass terms, so upstream's correction for the SDF
path does not concern us; but plugin added mass is numerically unstable as
it approaches the body's mass, so it stays small or out, as on the custom
USV, and it is why a change of the current ramps.

## 6. Thrusters that feel it

**Problem.** Gazebo's thruster, when its advance ratio is on, reads the
vehicle's speed over the ground, as a magnitude, where it should read the
water flowing into the propeller along the shaft; so the hull may feel the
current while the actuators behave as if the boat were in still water.

**Example.** A boat stemming a 1 m/s current at rest over the ground has
water flowing through its propellers at 1 m/s, and its thrusters compute
their thrust as if the water stood still.

**Solution.** `gz_thruster` is already vendored, so the fix lands here now,
not after the hull: the advance ratio uses the water's velocity relative to
the propeller along the shaft, the vehicle's velocity at the propeller less
`OceanCurrentAt` there. With `alpha_2` at its default of zero it changes
nothing for any vehicle that has not asked for it. Gazebo's lift and drag
systems take the wind and never the current, so a rudder or a keel would
feel none; that waits for a vehicle that has one.

## 7. Direction and units

**Problem.** A current is given by the direction it flows towards, its set,
in degrees clockwise from north, as charts draw it and NOAA predicts it,
and the wind by the direction it comes from. DAVE and UUV Simulator give a
current as an angle counter clockwise from east, in radians, as VRX gave
the wind, and the speed in metres per second where tide tables use knots.

**Example.** A westerly wind of 270 and a current setting 090 move a boat
the same way, and a DAVE current of 1.57 is a current setting north.

**Solution.** The direction the current sets towards, in degrees clockwise
from north, and the speed in metres per second at the surface; depths in
the profile in metres below the water, positive down. Say in the docs, next
to the wind, that the two directions follow their own trades, and convert
once at the edge. The name is `ocean_current` everywhere: the package, the
mark, the system and the topics, matching Gazebo's and DAVE's topic.

## 8. Reading the current

**Problem.** ROS never hears about the current, and the drift test has no
signal to check.

**Example.** A controller that should hold a heading against the set can
only react to the drift once it has happened.

**Solution.** The current system publishes the surface current above the
world's origin on `ocean_current_info` as ground truth, bridged to ROS, and
takes its changes on `ocean_current/set`, bridged the other way, as the
wind does. No sensor in this phase: a speed log and Gazebo's DVL are listed
above, out of it.

## 9. Worlds, tests and documentation

**Problem.** No world has a current, no test checks it, and the docs never
mention it.

**Example.** A change that broke the late spawn would go unnoticed until a
second boat stopped drifting in someone's demo.

**Solution.** Put the current system in every world, slack by default, with
a sensible current for each site noted in its header: tidal in the harbour
and the reef, none on the rowing lake. Switch the custom USV to
`gz-maritime-hydrodynamics-system` with its coefficients unchanged. Add a
headless test that spawns the custom USV in a set current and checks that
it drifts at the current's speed and in its direction, spawns a second boat
after the first step and checks that it drifts the same, holds one with
thrust and checks the thrust it takes against its damping, repeats a
variable run from the same seed and gets the same drift, runs a tide sped
up to minutes and sees the drift reverse, changes the current through `set`
and sees it ramp without a spike, drifts a marked box with no hydrodynamics
at the current's speed, and sees thrust fall with inflow on a thruster with
its advance ratio on. Write a how to page on the current and a short design
note on the recipe and the models next to the wind's, add the line to the
world contract and the markup, parameters and topics to the reference, and
the Fossen or mark rule to the invariants.
