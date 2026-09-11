# Multi-vehicle support plan

Temporary document for issue #23. It lists the kinds of problems that stop us
from running several vehicles in one simulation, with an example of each and
the direction we take to fix it. Remove this file once the work has landed.

The vehicles come from `bluerobotics_models` (BlueBoat, BlueROV2) and
`holybro_models` (X500). Most fixes live there; the world, the launch files and
the docs live here; the buoyancy change goes to Gazebo itself.

## 1. One simulation, many vehicles

**Problem.** Each vehicle launch starts its own Gazebo server, so two launches
give two worlds instead of two vehicles in one world.

**Example.** Running the BlueBoat launch twice opens two Gazebo windows.

**Solution.** Split every vehicle launch into a spawn part (model, bridge, state
publisher) and a simulation part (server, GUI). gz-maritime starts the world
once and includes the spawn part once per vehicle.

## 2. Names in Gazebo

**Problem.** The model name is fixed in the generated files and the launch only
lets you choose the height.

**Example.** A second BlueBoat is also called `blueboat` and Gazebo refuses it,
or both boats appear at the origin.

**Solution.** One name per instance drives everything: Gazebo model name, topic
namespace, ROS namespace and TF prefix. The launch exposes the full pose.

## 3. Gazebo topics

**Problem.** Topics already follow a namespace from the vehicle config, but
changing it needs a hand written copy of the config.

**Example.** Two boats generated from the same config both listen on
`/blueboat/motor_port/thrust`, so one command moves both.

**Solution.** Add a `--namespace` option to the vehicle generator that writes a
derived config and rebuilds every artifact from it.

## 4. ROS graph

**Problem.** A few ROS names are absolute and a few node names are fixed.

**Example.** Every bridge publishes `/joint_states` and `/clock`, and every
vehicle runs a node called `robot_state_publisher`.

**Solution.** Bridge joint states under the vehicle namespace, bridge the clock
once from the world launch, and run each vehicle's nodes inside its namespace.

## 5. TF frames

**Problem.** Every vehicle roots at `base_link` and sensor messages use bare
frame names.

**Example.** Two boats publish two different `base_link` transforms into one
tree and RViz shows a jumping robot.

**Solution.** A `frame_prefix` option applied both to the state publisher and
to every sensor frame id in the generated model.

## 6. Buoyancy

**Problem.** Gazebo buoyancy is a world plugin that floats only links listed
by model name in the world, and floating everything is wrong because chassis
parts have box collisions. This is the one problem namespacing cannot solve,
because the world is loaded before it knows the names.

**Example.** A boat spawned as `boat_b` sinks because the world only enables
`blueboat::hull_displacement`.

**Solution.** Extend the upstream Gazebo plugin so it also works as a model
plugin. The world instance owns the water: it sets a generic density component
on the water entity. Each vehicle carries a model instance on its displacement
link, reads that density, and floats whatever it is called. No list of names in
the world. The vehicle generators emit the model instance behind a flag. Wave
coupled buoyancy stays a later step.

## 7. Autopilots and controllers

**Problem.** The ArduPilot plugin uses a fixed UDP port and the X500 has no
autopilot at all.

**Example.** Two SITL boats both bind port 9002 and the second one never
connects.

**Solution.** A `sitl_instance` option that sets the port to 9002 plus ten per
instance, matching `sim_vehicle.py -I N`. An ArduCopter variant of the X500.
Custom controllers simply use the namespaced topics. PX4 stays community
supported.

## 8. World contents

**Problem.** The open water world misses systems some vehicles need and has no
ground for an aerial vehicle.

**Example.** The X500 GPS and barometer stay silent, and the quad falls through
the water.

**Solution.** Add the NavSat and AirPressure systems and a small static landing
platform to `open_water.sdf`.

## 9. Tests and documentation

**Problem.** No test spawns two vehicles and the docs say multi vehicle is not
supported.

**Example.** A regression that puts the model name back into a topic would go
unnoticed.

**Solution.** Two vehicle tests in each vehicle repo, a headless demo test in
`kai_bringup`, a multi vehicle how to guide and a vehicles page describing the
world contract.
