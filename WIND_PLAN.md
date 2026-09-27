# Wind plan

## Scope and success

We want a wind that a world sets and can change while it runs, that pushes
on any vehicle in proportion to what it shows to the air no matter what the
vehicle is called or when it appeared, and that aerial vehicles feel through
the rotor and wing systems Gazebo already ships. The demo uses a USV: the
custom USV drifts downwind at the speed its exposed area predicts, a second
boat drifts the same way, and a boat under thrust has to work against the
wind. A headless end to end test runs that demo. The X500 stays out of the
demo because it cannot fly yet, but the same wind has to reach its rotors so
that nothing changes on the day it does.

Out of this phase: a wind that changes from place to place, sails, and
letting the wind drive the sea state. Each gets its own issue once the
uniform wind works.

## 1. A wind the world owns

**Problem.** VRX's wind cannot be changed while the simulation runs, and
nothing reads it except the plugin that made it.

**Example.** To try the same boat in calm air and then in a breeze, you edit
the world file and restart Gazebo, and a sensor or a scoring system has no
way to know what the wind is at the boat.

**Solution.** Treat the wind the way we treat the waves. A `Windfield`
component on the world holds a recipe: the name of a wind model, its
parameters and a counter that goes up on every change. Wind models register
under a name, and any system asks for the wind at a point and a time with
`WindAt`, without knowing which model is behind it, so a new model later
means a new engine and no change to the systems that use it. A world system,
`gz-maritime-wind-system`, owns the recipe: it reads it from the world file
and changes it from a topic that the simulation launch bridges to ROS.

## 2. Gusts

**Problem.** VRX only lets the speed gust, never the direction, and Gazebo's
own wind system varies both with noise that nobody can seed.

**Example.** You cannot write a station keeping test against a wind that
veers, and running a gusty test twice gives two different results.

**Solution.** Spectral gusts on the speed and the direction: a sum of
sinusoids with phases drawn from a seed, as offshore engineering does. The
gust is a function of time alone, so every system that asks `WindAt`
computes the same gust without sharing any state, the way every consumer of
the waves rebuilds the same sea. The gusts move across the water with the
mean wind, so one reaches a boat downwind a little after the boat upwind.
The world sets how strong the gusts are and how long they last.

## 3. Wind with height

**Problem.** Every Gazebo wind is the same at the waterline as twenty metres
up.

**Example.** A drone hovering over a boat sees the same breeze as the deck,
where the real wind is nearly half.

**Solution.** A logarithmic profile over the sea, referenced to 10 m, with a
roughness length the world can set. It lives inside the wind model, so
`WindAt` returns the wind at the height it is asked for.

## 4. A load on any vehicle

**Problem.** VRX only pushes on the models a world lists by name, and
Gazebo's wind system pushes on links in proportion to their mass, about
thirty times too hard for a small hull.

**Example.** A boat spawned as `boat_b` feels no wind in a VRX world, and with
Gazebo's system the custom USV drifts at 1 m/s in a 5 m/s breeze when 0.07
m/s would be right.

**Solution.** Do what buoyancy does. A vehicle marks the shapes the wind sees
with `gz:wind="true"` on their collisions, with a drag coefficient of one
unless a shape says otherwise. A world system looks for marked shapes on
every model, whenever it shows up, takes their projected areas from the
geometry, cuts them off at the waterline, asks `WindAt` at each shape, and
applies quadratic drag at the centre of each shape in the body frame, so a
mast heels the boat and turns it. The generators write the marks the same
way they write the buoyancy ones. Gazebo's `enable_wind` flag stays
untouched, since it belongs to the mass based force.

## 5. Direction and units

**Problem.** VRX gives the direction the wind blows towards, measured counter
clockwise from east; weather reports, ArduPilot and the marine textbooks
give the direction it comes from, measured clockwise from north.

**Example.** A wind of 240 in VRX is a wind of 30 to everyone else.

**Solution.** The direction the wind comes from, in degrees clockwise from
north, and the speed at 10 m in metres per second. Convert once at the edge
and say so in the docs.

## 6. Reading the wind

**Problem.** A vehicle cannot measure the wind it is in, ROS never hears
about it, and Gazebo's own air speed sensor reads a different value from the
one its rotor and wing systems read.

**Example.** A controller that should feed the wind forward can only react to
the drift once it has happened, and the drift test has no signal to check.

**Solution.** The wind system writes the wind at the reference height into
Gazebo's wind entity, in both the values its systems read, so rotors, wings
and the air speed sensor all feel it. A custom `anemometer` sensor, declared
on any link of any vehicle, asks `WindAt` at the sensor and reports the
apparent wind in its own frame. The ground truth and each anemometer are
bridged to ROS, the anemometer under the instance name.

## 7. Worlds, tests and documentation

**Problem.** No world has wind, no test checks it, and the docs never mention
it.

**Example.** A change that broke the marks would go unnoticed until a boat
stopped drifting in someone's demo.

**Solution.** Put the wind system in every world, calm by default, with a
sensible breeze for each site noted in its header. Add a headless test that
spawns the custom USV in a set wind and checks the direction and speed of
the drift against its areas, adds a second boat, holds one with thrust, and
repeats a gusty run from the same seed. Write a how to page on the wind and
a short design note on the recipe and the models, add the line to the world
contract, and put the marks on the custom USV and the Blue Robotics
vehicles.
