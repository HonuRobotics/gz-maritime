# Compose the Gazebo model

The URDF says what the vehicle **is**. The Gazebo model adds how it is
**simulated**: what floats it, what drives it, how the water slows it down,
and what it senses. It is one short file, `model.sdf.xacro`, that includes
your URDF and adds the rest. The custom USV's is
`kai_custom_vehicle/models/custom_usv/model.sdf.xacro`, and every snippet
on this page is taken from it.

```{mermaid}
flowchart LR
  U["<b>custom_usv.urdf</b><br/>links, joints, mass,<br/>contact shapes, frames"]
  M["<b>model.sdf</b><br/>the URDF, plus:<br/>marked displacement volume<br/>Thruster × 2: propulsion<br/>Hydrodynamics: damping<br/>IMU, magnetometer, GPS<br/>JointStatePublisher"]
  U -->|merged into| M
```

## The wrapper

```xml
<sdf version="1.12" xmlns:xacro="http://ros.org/wiki/xacro"
     xmlns:gz="http://gazebosim.org/schema">
  <xacro:arg name="name" default="custom_usv"/>
  <xacro:arg name="urdf_uri" default="model://custom_usv/custom_usv.urdf"/>
  <xacro:property name="name" value="$(arg name)"/>
  <xacro:include filename="../../urdf/dimensions.xacro"/>

  <model name="${name}">
    <include merge="true">
      <uri>$(arg urdf_uri)</uri>
    </include>
    <!-- displacement, plugins and sensors go here -->
  </model>
</sdf>
```

Three things to notice:

- **`merge="true"`** puts the URDF's links and joints directly into this
  model, so what follows can refer to them by name: `base_link`,
  `motor_port_joint` and so on.
- **`xmlns:gz`** on the root declares the namespace the displacement mark
  below uses. Without it the file doesn't parse.
- **`name`** is the instance name. It becomes the model name, the prefix of
  every topic and the prefix of every sensor frame, so one file serves any
  number of boats. The spawn launch passes it, together with a `urdf_uri`
  pointing at the URDF it rendered for the same name
  ([One name per instance](spawn-and-drive.md#one-name-per-instance)).

The include of `dimensions.xacro`, by a path relative to this file, gives it
the same hull sizes the URDF was built from, so the displacement boxes below
cannot drift from the hulls.

### What survives the conversion

Gazebo converts the URDF when it loads it, and in doing so **merges every
link attached by a fixed joint into its parent**. On the custom USV,
`imu_link` and `gps_link` stop being links, but survive as **frames** with the
same names. That has two consequences:

- A plugin that needs a *link* must name one that survives: here `base_link`
  or a propeller.
- A sensor can still be placed at `imu_link`, because poses can be given
  relative to a frame.

To see exactly what survives, print the converted model:

```bash
SDF_PATH=$GZ_SIM_RESOURCE_PATH \
  gz sdf -p $(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf
```

`gz sdf` looks for `model://` URIs in `SDF_PATH`, not in
`GZ_SIM_RESOURCE_PATH`, hence the variable.

## Mark the collisions that displace

Every gz-maritime vehicle needs this step. Add a link fixed to `base_link`
whose collision shapes are the volume that floats the vehicle, and mark each
of them with `gz:buoyancy="true"`. The custom USV's macro makes one box per
hull:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:macro name="pontoon"
:end-at: </xacro:macro>
```

and puts the two on a link of their own:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <link name="hull_displacement">
:end-at: </joint>
```

The rules:

- **A marked collision floats; an unmarked one doesn't.** The world floats
  nothing it isn't told about, and this is how it is told. A link that marks
  any of its collisions floats by those alone; its other collisions stay
  contact geometry.
- **Use shapes buoyancy can slice:** box, sphere, cylinder, capsule,
  ellipsoid and cone. Meshes and planes are skipped, with a warning. A box
  per hull is a good start; the custom USV's are the same size and place as
  the URDF's hulls, from the shared `dimensions.xacro`.
- **Give a mark a zero `collide_bitmask`.** A marked collision is still a
  collision to the physics engine. With the bitmask it touches nothing, and
  the URDF's hulls do the bumping.
- **Put the marks on their own link.** They could sit on `base_link` too, but
  `base_link` comes from the URDF, and a merged include can't add elements
  to it. A separate link, fixed-jointed, is one rigid body to physics and
  keeps contact and displacement visibly apart.
- **Get the centre of buoyancy above the centre of mass**, or at least the
  waterplane wide, or the vehicle capsizes. The custom USV floats its 12 kg
  on two boxes centred 3.6 cm above the waterline, well above its centre of
  mass.

```{important}
Without a marked collision your vehicle sinks. The mark is a plain SDF
attribute, so it travels with the model file: nothing in the world and
nothing in a launch file refers to it.
```

## Thrusters

Add one Thruster plugin per propeller joint. It is gz-maritime's thruster
system, `gz_thruster`: Gazebo's own with a **normalized command** mode, the
interface every vehicle here is driven with:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:macro name="thruster"
:end-at: </xacro:macro>
```

- **Commands are a fraction of full thrust**, in [-1, 1], on the topic: 1
  gives `max_thrust_cmd` ahead, -1 gives `min_thrust_cmd` astern, 0 stops.
  Those two limits, in newtons, are the only place a force appears. That is
  how a real thruster is driven: an autopilot or a controller produces a
  normalized output and the ESC maps it onto the motor, so a controller
  written against this boat drives the BlueBoat unchanged. The propeller
  speed comes back on the same topic followed by `/ang_vel`.
- **Leave `<namespace>` empty and put the name in the topic yourself.**
  Otherwise the plugin prefixes the topic with the model name on its own,
  and the bridge configuration can't know what to expect.
- **Counter-rotating propellers get opposite signs** of `thrust_coefficient`:
  the macro is called with `0.02` for port and `-0.02` for starboard.
- **`fluid_density` is seawater**, and `propeller_diameter` comes from the
  shared dimensions, so it matches the propeller in the URDF.

`gz_thruster` is on its way upstream. When a Gazebo release carries the
mode, only the plugin's filename and name change.

## Hydrodynamic damping

Water resists motion. The Hydrodynamics plugin applies that resistance to
one link:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <plugin filename="gz-sim-hydrodynamics-system"
:end-at: </plugin>
```

Each pair of coefficients resists one kind of motion:

| Coefficients | Motion | |
|---|---|---|
| `xU`, `xUabsU` | Surge | forwards and backwards |
| `yV`, `yVabsV` | Sway | sideways |
| `zW`, `zWabsW` | Heave | up and down |
| `kP`, `kPabsP` | Roll | tipping to the side |
| `mQ`, `mQabsQ` | Pitch | nose up and down |
| `nR`, `nRabsR` | Yaw | turning |

The first of each pair grows with speed, the `abs` one with speed squared. A
boat should resist sideways motion much more than forward motion, or it
slides out of its turns. The custom USV's values are placeholders borrowed
from the BlueBoat; tune them for your vehicle.

```{note}
The damping acts in the air too, so a boat spawned well above the water falls
slowly. Spawn it at the waterline.
```

### Added mass

When a hull speeds up it also drags some water along, which makes it feel
heavier than it is. Gazebo models this with `<fluid_added_mass>` inside a
link's `<inertial>`. A URDF can't express that element, and a merged include
can't add it to a link that comes from the URDF. For a small, slow boat it is
safe to leave out.

## Sensors

Each sensor sits on a tiny link fixed to `base_link`, placed at a frame the
URDF declared. The custom USV's macro builds the link and the joint and
takes the `<sensor>` as a block:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:macro name="sensor_link"
:end-at: </xacro:macro>
```

The IMU is then:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:sensor_link sensor_name="imu"
:end-at: </xacro:sensor_link>
```

- **Why a separate link:** `base_link` comes from the URDF, and a merged
  include can't add a sensor inside it.
- **Why `frame_id` carries the name:** messages then say they are in
  `boat_a/imu_link`, which is the frame `robot_state_publisher` publishes for
  the instance `boat_a` ([Spawn and drive](spawn-and-drive.md#one-name-per-instance)).

[Add sensors](sensors.md) covers the other sensor types.

## Joint states

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <plugin filename="gz-sim-joint-state-publisher-system"
:end-at: </plugin>
```

The bridge turns this into `/<name>/joint_states`, which the instance's
`robot_state_publisher` reads to spin the propellers in RViz.

## Build it

The package looks like this:

```text
kai_custom_vehicle/
├── CMakeLists.txt
├── package.xml
├── urdf/custom_usv.urdf.xacro, dimensions.xacro
├── models/custom_usv/model.sdf.xacro, model.config
├── config/ros_gz_bridge.yaml.in
├── rviz/custom_usv.rviz
├── launch/sim.launch.xml, rviz.launch.xml, display.launch.xml
├── scripts/instance_rviz.py
└── hooks/resource_paths.dsv.in, resource_paths.sh.in
```

`CMakeLists.txt` expands the URDF and the model for the default instance,
installs them with a rendered bridge config and `model.config`, and adds
the models directory to Gazebo's resource path, so that `model://custom_usv`
resolves:

```{literalinclude} ../../kai_custom_vehicle/CMakeLists.txt
:language: cmake
:start-after: find_package(xacro REQUIRED)
:end-before: if(BUILD_TESTING)
```

Any other instance is rendered on demand by the spawn launch, which runs the
same xacros with another name and fills the same bridge template
([Spawn and drive](spawn-and-drive.md)). You can run that step yourself:

```bash
ros2 run kai_bringup instantiate_vehicle.py --name boat_b --out-dir ~/boat_b \
  --xacro $(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
  --bridge $(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
  --urdf $(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro
```

That writes `model.sdf`, `robot.urdf` and `ros_gz_bridge.yaml` for `boat_b`.

```{tip}
Don't name the model directory after the package. Gazebo first looks for
`model://` names relative to the current directory, and a source checkout
contains a directory called `kai_custom_vehicle`. Hence `models/custom_usv`.
```

## Marking from a URDF

If you would rather not keep a separate SDF file, the mark can travel in the
URDF, but only one way. Gazebo converts a URDF when it loads it, and the
conversion rebuilds every `<collision>` from the parsed geometry, so an
attribute on a URDF collision is lost, and so is one inside a
`<gazebo reference="...">` block. What does come through unchanged is a
model-level `<gazebo>` block: everything inside it is copied into the
converted model as-is. Put the displacement link and its joint there, in SDF:

```xml
<robot name="my_boat" xmlns:gz="http://gazebosim.org/schema">
  <link name="base_link"> ... contact shapes ... </link>

  <gazebo>
    <link name="hull_displacement">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
      <inertial>
        <mass>1e-5</mass>
        <inertia><ixx>1e-8</ixx><iyy>1e-8</iyy><izz>1e-8</izz></inertia>
      </inertial>
      <collision name="pontoon_port" gz:buoyancy="true">
        <pose>0 0.25 0.036 0 0 0</pose>
        <geometry><box><size>1.0 0.15 0.15</size></box></geometry>
        <surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>
      </collision>
    </link>
    <joint name="hull_displacement_joint" type="fixed">
      <parent>base_link</parent>
      <child>hull_displacement</child>
    </joint>
  </gazebo>
</robot>
```

The `xmlns:gz` declaration goes on `<robot>`. Thrusters, damping and sensors
can sit in the same block as `<plugin>` and `<sensor>` elements, and the file
spawns directly, with `ros_gz_sim create -file my_boat.urdf` or from the
`robot_description` topic. `check_urdf` and RViz ignore the block.

It is quicker to start with, but it ties the URDF to Gazebo, and the block
is SDF inside a URDF, with no xacro sharing of the hull dimensions. The
separate model file above scales better, which is why the custom USV uses
it.

Next: [Spawn and drive](spawn-and-drive.md).
