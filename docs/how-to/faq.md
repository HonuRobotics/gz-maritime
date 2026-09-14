# Troubleshooting

## My vehicle sinks

Nothing floats it, or the floating volume has nothing to measure. Check in
this order:

1. **Is a collision marked?** The world floats only collisions marked
   `gz:buoyancy="true"`
   ([Mark what displaces](gazebo-composition.md#mark-the-collisions-that-displace)).
   Look at the model file Gazebo actually loaded, not the xacro: the mark
   must survive expansion, and the root element must declare
   `xmlns:gz="http://gazebosim.org/schema"`. In a URDF the mark only
   survives inside a model-level `<gazebo>` block; on a URDF `<collision>`
   or in `<gazebo reference="...">` the conversion drops it
   ([Marking from a URDF](gazebo-composition.md#marking-from-a-urdf)).
2. **Can buoyancy measure the shape?** Box, sphere, cylinder, capsule,
   ellipsoid or cone. A mesh has no volume to it, and you get a warning.
3. **Is it too heavy?** If the draft is deeper than the hulls are tall, it
   sinks ([Work out the draft](vehicle-description.md#work-out-the-draft)).
4. **Does the world run gz-maritime's buoyancy?** Until a Gazebo release
   ships the marked-collision change, stock `gz-sim-buoyancy-system` ignores
   the marks, and with `<enable_by_default>false</enable_by_default>` it
   floats nothing.

## My vehicle bumps into things it shouldn't, or sits on the water oddly

A marked collision is still a collision to the physics engine. Give every
mark `<surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>`
so the displacement volume touches nothing, and let the URDF's shapes do the
bumping.

## "Unsupported collision geometry for graded buoyancy"

A marked collision, or an enabled link's collision, has a shape buoyancy
can't slice, usually a mesh. Use boxes, spheres, cylinders, capsules,
ellipsoids or cones for the floating volume, and keep the mesh as contact
geometry.

## The vehicle capsizes

- **The centre of mass is too high** for the hull's width. Lower it, or widen
  the stance.
- **The marked shapes don't match the hull.** Buoyancy only knows the marked
  shapes; if they're off-centre, so is the lift.
- **Roll and pitch aren't damped.** Without `kP` and `mQ` in the Hydrodynamics
  plugin, the vehicle rocks and can tip over.

## The boat moves sideways, or spins, instead of going forward

- **The commands arrived one after the other.** They latch, so the first
  propeller pushes alone for a moment. Start both commands together.
- **Sideways motion isn't damped enough.** `yV` and `yVabsV` should be much
  larger than `xU` and `xUabsU`.
- **Both propellers turn the same way.** Counter-rotating propellers need
  opposite signs of `thrust_coefficient`.

## Two boats answer the same command, or RViz shows a jumping robot

Both instances share a name somewhere. Every instance needs its own `name`,
which must reach the model's topics and frame ids, the ROS namespace and
`robot_state_publisher`'s `frame_prefix`
([One name per instance](spawn-and-drive.md#one-name-per-instance)). If you
spawned the installed default model twice, generate an instance per boat
with `configure_vehicle.py --name` instead.

## A plugin can't find a link

Gazebo merges links attached by fixed joints into their parent when it reads
a URDF, so those links no longer exist, only frames with their names.
Reference a link that survives, or place things `relative_to` the frame. See
what survives with `gz sdf -p`
([What survives the conversion](gazebo-composition.md#what-survives-the-conversion)).

## The server stops with "ODE INTERNAL ERROR ... aabbBound"

A box-shaped hull floating almost exactly level made the buoyancy
calculation return a wildly wrong result, which blew the simulation up. This
is a gz-math bug, fixed upstream in
[gazebosim/gz-math#847](https://github.com/gazebosim/gz-math/pull/847), and
gz-maritime's buoyancy system already works around it. If you see it, check
that your world loads `gz-maritime-buoyancy-system` and not
`gz-sim-buoyancy-system`.

## The boat falls very slowly when spawned high

The Hydrodynamics plugin doesn't know where the water is, and damps motion in
the air too. Spawn at the waterline (`z:=0` when `base_link` is on the
waterline).

## RViz shows nothing, or "Fixed Frame does not exist"

In a simulation every frame carries the instance name: the fixed frame is
`tutorial_usv/base_link`, not `base_link`, the description is on
`/tutorial_usv/robot_description`, and the RobotModel display only finds
the links if its **TF Prefix** property is the instance name; without it
the display reports "No transform from [base_link] to [tutorial_usv/base_link]"
and draws nothing. `ros2 launch tutorial_usv_gazebo rviz.launch.xml name:=<name>`
starts RViz on a config with all three set
([Look at it in RViz](../getting-started/first-simulation.md#4-look-at-it-in-rviz)).
For your own vehicle, set the same three things in your RViz config.
`display.launch.xml`, which runs without Gazebo, uses no prefix.

## robot_state_publisher warns about the root link inertia

```text
The root link base_link has an inertia specified in the URDF, but KDL does not
support a root link with an inertia.
```

Harmless: KDL only computes TF and ignores inertia, while Gazebo reads it.
Don't add a dummy root link to silence it. Gazebo would merge `base_link` into
the dummy, and every plugin that refers to `base_link` would break.

## The ocean looks flat or white in the GUI

- **Check the GUI output for wave errors.** The GUI needs the wave engine's
  GUI plugin, which the `water_surface` model loads at its `<visual>`; a
  world that includes the model gets it.
- **On a machine with several network interfaces** (Wi-Fi, Docker, VPN), the
  server and the GUI can pick different ones and miss part of each other's
  data. Try `export GZ_IP=127.0.0.1` in both terminals.

## Camera images look green or blue

`open_water.sdf`'s ambient light is cyan. See the note in
[Add sensors](sensors.md#a-camera).

## Two simulations interfere with each other

Give each simulation its own `GZ_PARTITION`, and each ROS graph its own
`ROS_DOMAIN_ID`.
