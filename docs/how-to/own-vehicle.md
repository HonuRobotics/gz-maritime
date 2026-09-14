# Bring your own vehicle

You have a boat or an underwater vehicle that isn't a BlueBoat or a BlueROV2,
and you want it on the gz-maritime ocean, driven from ROS 2. This series
shows how. The [tutorial USV](../vehicles/tutorial-usv.md) was built exactly
this way, so each step points at a real file you can copy.

## Who does what

The world and the vehicle split the work:

| The world (`open_water.sdf`) provides | Your vehicle brings |
|---|---|
| Seawater (1025 kg/m³) below z = 0, air above | Collision shapes **marked** as the volume that displaces water |
| Gravity and physics | Links, joints, mass and inertia |
| A moving sea to look at, changeable at runtime | Thrusters and hydrodynamic damping |
| The systems that simulate IMU, magnetometer and GPS, and a GPS origin | The sensors themselves, placed on its frames |
| A service to spawn models, and `/clock` bridged once | A spawn launch file with its bridge configuration |

The world never names a vehicle, so the same world serves every vehicle,
including ones spawned while it runs, and several copies of the same one.

## The path

```{mermaid}
flowchart LR
  A["<b>1. Describe</b><br/>URDF / xacro<br/><i>what the vehicle is</i>"]
  B["<b>2. Compose</b><br/>model.sdf.xacro<br/><i>how it is simulated</i>"]
  C["<b>3. Spawn</b><br/>spawn.launch.xml<br/><i>how an instance is started</i>"]
  G["Gazebo<br/>open_water world"]
  R["ROS 2<br/>topics and TF"]
  A --> B --> C
  C --> G
  C --> R
  G <-->|ros_gz_bridge| R
```

1. **Describe** the vehicle in URDF: links, joints, mass, contact shapes and
   frames, with nothing Gazebo-specific and no instance name.
   [Describe the vehicle](vehicle-description.md)
2. **Compose** the simulation model: a short SDF file that merges your URDF
   and adds what Gazebo needs, starting with the marked displacement volume.
   [Compose the Gazebo model](gazebo-composition.md)
3. **Spawn** it from a launch file that sits next to gz-maritime's simulation
   launch, once per instance, with its bridge and its TF.
   [Spawn and drive](spawn-and-drive.md)

### Why the URDF comes first

ROS tools (RViz, `robot_state_publisher`, navigation and manipulation stacks)
read URDF, and Gazebo can read it too. With one URDF describing the vehicle,
the robot you see in RViz and the one Gazebo simulates can't drift apart.
Everything that only matters to the simulator (the displacement volume,
plugins, sensors) lives in the separate simulation model. The Blue Robotics
and Holybro vehicles are built the same way.

## How a vehicle floats

The world's buoyancy floats **nothing** it isn't told about. A vehicle tells
it by marking, in its own model, the collision shapes that displace water:

```xml
<collision name="pontoon_port" gz:buoyancy="true">
  <pose>0 0.25 0.036 0 0 0</pose>
  <geometry><box><size>1.0 0.15 0.15</size></box></geometry>
  <surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>
</collision>
```

A link with marked collisions floats by those alone, whatever the model is
called. Its other collisions stay contact geometry. The `gz:` prefix needs
`xmlns:gz="http://gazebosim.org/schema"` declared on the file's root
element. Three things make this work well:

- **Nothing outside the model is involved.** The mark travels with the model
  file, so the world needs no list of vehicles and no edit per vehicle.
- **Renaming is free.** Spawn the same model twice under two names and both
  float.
- **Contact and displacement stay separate.** The hulls you bump with and the
  volume that floats you are different shapes, even when they coincide, so a
  bracket or a sensor housing never adds buoyancy by accident.

This is Gazebo's own buoyancy system with a small upstream change
([gz-sim 229ec07e](https://github.com/gazebosim/gz-sim/commit/229ec07e673b6317fec57af50fd652bcd9bde0ed)),
which gz-maritime carries as `gz_buoyancy` until a Gazebo release ships it.
Why not simply list the vehicles in the world file? Then the world would
have to know every vehicle before it starts, and a vehicle spawned later, or
a second copy, could never float. The [design notes](../design/index.md)
compare the options.

## What the Gazebo maritime tutorials cover

The Gazebo Jetty documentation has a good maritime series, worth reading
alongside this one. It explains the physics plugins used here:

- Theory: [buoyancy](https://gazebosim.org/api/sim/10/theory_buoyancy.html)
  and [hydrodynamics](https://gazebosim.org/api/sim/10/theory_hydrodynamics.html).
- Building a vehicle:
  [create a maritime vehicle](https://gazebosim.org/api/sim/10/create_vehicle.html),
  [adding visuals](https://gazebosim.org/api/sim/10/adding_visuals.html),
  [frame of reference](https://gazebosim.org/api/sim/10/frame_reference.html),
  [adding system plugins](https://gazebosim.org/api/sim/10/adding_system_plugins.html).
- Worked vehicles: [a lander](https://gazebosim.org/api/sim/10/lander.html),
  [an underwater vehicle](https://gazebosim.org/api/sim/10/underwater_vehicles.html),
  [a surface vehicle](https://gazebosim.org/api/sim/10/surface_vehicles.html).

What those tutorials don't cover, and this series does:

- **This world.** Vehicles mark their own floating volume instead of being
  listed in the world, and the water is seawater (1025 kg/m³), where the
  buoyancy tutorial's example uses fresh water (1000 kg/m³).
- **The ROS 2 side.** The URDF as the source of truth, spawning through
  `ros_gz_sim`, the bridge, `robot_state_publisher`, and several vehicles in
  one simulation, each with its own name, namespace and TF prefix.
- **The waves.** A moving sea you can change while the simulation runs.

```{warning}
The *surface vehicle* tutorial uses the `maritime::Surface` and
`maritime::PublisherPlugin` plugins from the old VRX wave field. **They don't
exist in gz-maritime.** Take the thruster and hydrodynamics parts from that
tutorial, and buoyancy and waves from this series.
```

## Waves and physics today

```{figure} images/waves-through-hulls.jpg
:alt: Two views of the tutorial USV in a moderate sea. Left, over a wave trough, the hulls are completely out of the drawn water. Right, under a crest, the drawn water covers the hulls.

The tutorial USV at sea state 3. The drawn sea rises and falls around the
boat, but the boat stays on the flat water level.
```

Vehicles float on the **flat** water level at z = 0. The moving sea is drawn,
but its height doesn't push vehicles around yet, so in a rough sea the waves
pass through the hulls, as in the picture. That is why `open_water.sdf`
starts calm, at sea state 1: the picture then matches the physics. Buoyancy
that follows the waves is planned.

## Already have a model?

You don't need the two-package layout to try your vehicle out. Mark the
collision that should float it, as above, then start the ocean in one
terminal:

```bash
ros2 launch kai_bringup simulation.launch.xml
```

and spawn the file into it from another:

```bash
ros2 run ros_gz_sim create -name my_boat -file /path/to/model.sdf -z 0
```

That works for an SDF model as it is. For a URDF, the mark has to sit in a
model-level `<gazebo>` block, because the conversion drops attributes on
URDF collisions ([Marking from a URDF](gazebo-composition.md#marking-from-a-urdf)).
The next page explains why the split into a description and a Gazebo model
is still worth it once you go further.

Next: [Describe the vehicle](vehicle-description.md).
