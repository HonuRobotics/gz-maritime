# Multi-vehicle support plan

Temporary document for issue #23 (SOW task 1.4.2). It states what we build,
what success looks like, and the kinds of problems that stop us from running
several vehicles in one simulation, with an example of each and the direction
we take to fix it. Remove this file once the work has landed.

The vehicles come from `bluerobotics_models` (BlueBoat, BlueROV2) and
`holybro_models` (X500). Most fixes live there; the world, the launch files and
the docs live here; the buoyancy change goes to Gazebo itself.

## Scope and success

The deliverable is a walkthrough under `docs/how-to` and the software behind
it. One launch command brings up a minimal open water world with one USV
(BlueBoat), one UUV (BlueROV2) and one UAV (X500) resting on a landing pad
above the water. The walkthrough shows how to list the ROS topics of each
vehicle under its own namespace, how to read a sensor and command an actuator
of one vehicle with `ros2 topic` commands, and how to run the teleop stack
bound to one vehicle. An end to end test runs the same demo without a GUI
and checks those topics.

Several copies of the same vehicle should work for free with the same
mechanisms. We try it after the demo works, document it if it does, and open
an issue if it does not. It never blocks the demo.

Out of this phase: the autopilots (SITL for the three vehicles gets its own
issues; in the long run every vehicle must be drivable by SITL, not just one),
the competition server (SOW 1.4.1, in its own repository), and performance
work beyond one measurement of the real time factor with the three vehicles
(SOW 1.4.3).

The work starts after the SITL pull requests that change the thruster
interface are merged, since they touch the same launch files, generators and
topic names.

## 1. One simulation, many vehicles

**Problem.** Each vehicle launch starts its own Gazebo server, so two launches
give two worlds instead of two vehicles in one world.

**Example.** Launching the BlueBoat and then the BlueROV2 opens two Gazebo
windows.

**Solution.** Split every vehicle launch into a spawn part (model, bridge, state
publisher) and a simulation part (server, GUI). gz-maritime starts the world
once and includes the spawn part once per vehicle.

## 2. Names and poses in Gazebo

**Problem.** The model name is fixed in the generated files and the launch only
lets you choose the height.

**Example.** Every vehicle appears at the origin, so the boat spawns inside the
ROV.

**Solution.** One name per instance drives everything: Gazebo model name, topic
namespace, ROS namespace and TF prefix. The launch exposes the full pose,
position and orientation.

## 3. Topics on both sides

**Problem.** Gazebo and ROS topics already follow a namespace from the vehicle
config, and the bridge is generated from the same config, but changing the
namespace needs a hand written copy of the config.

**Example.** Two boats generated from the same config both listen on the same
thrust topic, so one command moves both.

**Solution.** Add a `--namespace` option to the vehicle generator that writes a
derived config and rebuilds every artifact from it.

## 4. ROS graph

**Problem.** A few ROS names are absolute and a few node names are fixed.

**Example.** Every bridge publishes `/joint_states` and `/clock`, and every
vehicle runs a node called `robot_state_publisher`.

**Solution.** Bridge joint states under the vehicle namespace, bridge the clock
once from the world launch, and run each vehicle's nodes inside its namespace.
The teleop launch gets a namespace argument so it binds to one vehicle.

## 5. TF frames

**Problem.** Every vehicle roots at `base_link` and sensor messages use bare
frame names.

**Example.** The boat and the ROV both publish `base_link` into one tree, and
RViz shows one robot jumping between two poses.

**Solution.** A `frame_prefix` option applied both to the state publisher and
to every sensor frame id in the generated model. In ROS 2 the state publisher
applies its prefix to every frame it publishes, so the old ROS 1 problems with
partial prefixes are gone; what remains is that nothing prefixes the frame
ids inside sensor messages, which is why the generated model does it.

## 6. Buoyancy

**Problem.** Gazebo buoyancy is a world plugin that floats only links listed
by model name in the world, and floating everything is wrong because chassis
parts have box collisions. This is the one problem namespacing cannot solve,
because the world is loaded before it knows the names.

**Example.** A boat spawned as `boat_a` sinks because the world only enables
`blueboat::hull_displacement`.

**Solution.** Mark the collisions that displace water. gz-sim commit
[229ec07e](https://github.com/gazebosim/gz-sim/commit/229ec07e673b6317fec57af50fd652bcd9bde0ed)
lets a link mark them with `gz:buoyancy="true"`: such a link floats by those
collisions alone, whatever the model is called, and its other collisions stay
contact geometry. A world that only describes the fluid adds
`<enable_by_default>false</enable_by_default>`, so it floats only marked
links and names nobody. `gz_buoyancy` carries the change until a Gazebo
release ships it; vehicles never change when it does. The vehicle generators
emit the mark on their displacement collisions behind a flag. Wave coupled
buoyancy stays a later step.

## 7. Controllers

**Problem.** Nothing drives a vehicle unless something publishes on its
topics, and the demo must show that each vehicle can be commanded on its own.

**Example.** A command on the boat's thrust topic must move the boat and
nothing else.

**Solution.** Custom controllers and the teleop stack use the namespaced
topics. Autopilots are out of this phase and tracked in their own issues;
their per instance port setting is a small change.

## 8. World contents

**Problem.** The open water world misses systems some vehicles need and has no
ground for an aerial vehicle.

**Example.** The X500 GPS and barometer stay silent, and the quad falls through
the water.

**Solution.** Add the NavSat and AirPressure systems and a small static landing
pad to `open_water.sdf`. The vehicles page in the docs lists, in plain words,
which systems a world must load and what a vehicle must provide to be spawned
into it.

## 9. Tests and documentation

**Problem.** No test spawns several vehicles, and the docs say multi vehicle is
not supported.

**Example.** A regression that puts the model name back into a topic would go
unnoticed.

**Solution.** The main check is the end to end test that runs the documented
demo without a GUI and verifies that every vehicle's sensors and actuators are
reachable over ROS under its namespace. Each vehicle repository also gets a
cheap test that generates two copies of its vehicle and checks their topics
do not overlap. Plus the walkthrough and the vehicles page described above.
