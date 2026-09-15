# Compose the Gazebo model

The URDF says what the vehicle **is**. The Gazebo model adds how it is
**simulated**: what floats it, what drives it, how the water slows it down,
and what it senses. It is one short file, `model.sdf.xacro`, that includes
your URDF and adds the rest. The tutorial USV's is
`tutorial_usv_gazebo/model.sdf.xacro`.

```{mermaid}
flowchart LR
  U["<b>tutorial_usv.urdf</b><br/>links, joints, mass,<br/>contact shapes, frames"]
  M["<b>model.sdf</b><br/>the URDF, plus:<br/>marked displacement volume<br/>Thruster × 2: propulsion<br/>Hydrodynamics: damping<br/>IMU, magnetometer, GPS<br/>JointStatePublisher"]
  U -->|merged into| M
```

## The wrapper

```xml
<sdf version="1.12" xmlns:xacro="http://ros.org/wiki/xacro"
     xmlns:gz="http://gazebosim.org/schema">
  <xacro:arg name="name" default="tutorial_usv"/>
  <xacro:arg name="urdf_uri" default="model://tutorial_usv/tutorial_usv.urdf"/>
  <xacro:property name="name" value="$(arg name)"/>
  <xacro:include filename="$(find tutorial_usv_description)/urdf/dimensions.xacro"/>

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
  number of boats ([One name per instance](spawn-and-drive.md#one-name-per-instance)).

The include of `dimensions.xacro` gives this file the same hull sizes the
URDF was built from, so the displacement boxes below cannot drift from the
hulls.

### What survives the conversion

Gazebo converts the URDF when it loads it, and in doing so **merges every
link attached by a fixed joint into its parent**. On the tutorial USV,
`imu_link` and `gps_link` stop being links, but survive as **frames** with the
same names. That has two consequences:

- A plugin that needs a *link* must name one that survives: here `base_link`
  or a propeller.
- A sensor can still be placed at `imu_link`, because poses can be given
  relative to a frame.

To see exactly what survives, print the converted model:

```bash
SDF_PATH=$GZ_SIM_RESOURCE_PATH \
  gz sdf -p $(ros2 pkg prefix --share tutorial_usv_gazebo)/models/tutorial_usv/model.sdf
```

`gz sdf` looks for `model://` URIs in `SDF_PATH`, not in
`GZ_SIM_RESOURCE_PATH`, hence the variable.

## Mark the collisions that displace

Every gz-maritime vehicle needs this step. Add a link fixed to `base_link`
whose collision shapes are the volume that floats the vehicle, and mark each
of them with `gz:buoyancy="true"`:

```xml
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
  <collision name="pontoon_stbd" gz:buoyancy="true">
    ...
  </collision>
</link>
<joint name="hull_displacement_joint" type="fixed">
  <parent>base_link</parent>
  <child>hull_displacement</child>
</joint>
```

The rules:

- **A marked collision floats; an unmarked one doesn't.** The world floats
  nothing it isn't told about, and this is how it is told. A link that marks
  any of its collisions floats by those alone; its other collisions stay
  contact geometry.
- **Use shapes buoyancy can slice:** box, sphere, cylinder, capsule,
  ellipsoid and cone. Meshes and planes are skipped, with a warning. A box
  per hull is a good start; the tutorial's are the same size and place as
  the URDF's hulls, from the shared `dimensions.xacro`.
- **Give a mark a zero `collide_bitmask`.** A marked collision is still a
  collision to the physics engine. With the bitmask it touches nothing, and
  the URDF's hulls do the bumping.
- **Put the marks on their own link.** They could sit on `base_link` too, but
  `base_link` comes from the URDF, and a merged include can't add elements
  to it. A separate link, fixed-jointed, is one rigid body to physics and
  keeps contact and displacement visibly apart.
- **Get the centre of buoyancy above the centre of mass**, or at least the
  waterplane wide, or the vehicle capsizes. The tutorial USV floats its 12 kg
  on two boxes centred 3.6 cm above the waterline, well above its centre of
  mass.

```{important}
Without a marked collision your vehicle sinks. The mark is a plain SDF
attribute, so it travels with the model file: nothing in the world and
nothing in a launch file refers to it.
```

## Thrusters

Add one Thruster plugin per propeller joint:

```xml
<plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
  <joint_name>motor_port_joint</joint_name>
  <namespace></namespace>
  <topic>${name}/motor_port/thrust</topic>
  <thrust_coefficient>0.02</thrust_coefficient>
  <fluid_density>1025.0</fluid_density>
  <propeller_diameter>0.08</propeller_diameter>
  <velocity_control>true</velocity_control>
  <max_thrust_cmd>20.0</max_thrust_cmd>
  <min_thrust_cmd>-15.0</min_thrust_cmd>
</plugin>
```

- **Commands are thrust in newtons** on the topic. The propeller speed comes
  back on the same topic followed by `/ang_vel`.
- **Leave `<namespace>` empty and put the name in the topic yourself.**
  Otherwise the plugin prefixes the topic with the model name on its own,
  and the bridge configuration can't know what to expect.
- **Counter-rotating propellers get opposite signs** of `thrust_coefficient`:
  the starboard one uses `-0.02`.
- **`fluid_density` is seawater**, and `propeller_diameter` comes from the
  shared dimensions, so it matches the propeller in the URDF.

## Hydrodynamic damping

Water resists motion. The Hydrodynamics plugin applies that resistance to
one link:

```xml
<plugin filename="gz-sim-hydrodynamics-system"
        name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <water_density>1025.0</water_density>
  <xU>-20</xU>  <yV>-40</yV>  <zW>-30</zW>
  <kP>-20</kP>  <mQ>-20</mQ>  <nR>-15</nR>
  <xUabsU>-30</xUabsU>  <yVabsV>-90</yVabsV>  <zWabsW>-100</zWabsW>
  <kPabsP>-40</kPabsP>  <mQabsQ>-40</mQabsQ>  <nRabsR>-30</nRabsR>
</plugin>
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
slides out of its turns. The tutorial's values are placeholders borrowed from
the BlueBoat; tune them for your vehicle.

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
URDF declared:

```xml
<link name="imu_sensor">
  <pose relative_to="imu_link">0 0 0 0 0 0</pose>
  <inertial>
    <mass>1e-5</mass>
    <inertia><ixx>1e-8</ixx><iyy>1e-8</iyy><izz>1e-8</izz></inertia>
  </inertial>
  <sensor name="imu" type="imu">
    <frame_id>${name}/imu_link</frame_id>
    <topic>${name}/imu</topic>
    <update_rate>50</update_rate>
    <always_on>true</always_on>
  </sensor>
</link>
<joint name="imu_sensor_joint" type="fixed">
  <parent>base_link</parent>
  <child>imu_sensor</child>
</joint>
```

- **Why a separate link:** `base_link` comes from the URDF, and a merged
  include can't add a sensor inside it.
- **Why `frame_id` carries the name:** messages then say they are in
  `boat_a/imu_link`, which is the frame `robot_state_publisher` publishes for
  the instance `boat_a` ([Spawn and drive](spawn-and-drive.md#one-name-per-instance)).

[Add sensors](sensors.md) covers the other sensor types.

## Joint states

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <topic>${name}/joint_states</topic>
  <joint_name>motor_port_joint</joint_name>
  <joint_name>motor_stbd_joint</joint_name>
</plugin>
```

The bridge turns this into `/<name>/joint_states`, which the instance's
`robot_state_publisher` reads to spin the propellers in RViz.

## Build it

The package looks like this:

```text
tutorial_usv_gazebo/
├── CMakeLists.txt
├── package.xml
├── model.sdf.xacro
├── models/tutorial_usv/model.config
├── config/ros_gz_bridge.yaml.in
├── scripts/configure_vehicle.py
├── launch/spawn.launch.xml, sim.launch.xml, two_usvs.launch.xml
└── hooks/resource_paths.dsv.in, resource_paths.sh.in
```

`CMakeLists.txt` expands the model for the default instance, installs it
next to a copy of the URDF and a rendered bridge config, and adds the models
directory to Gazebo's resource path, so that `model://tutorial_usv`
resolves:

```cmake
find_package(xacro REQUIRED)
find_package(tutorial_usv_description REQUIRED)

ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh.in")

xacro_add_files(model.sdf.xacro INSTALL DESTINATION models/tutorial_usv)
install(FILES
  models/tutorial_usv/model.config
  ${tutorial_usv_description_DIR}/../urdf/tutorial_usv.urdf
  DESTINATION share/${PROJECT_NAME}/models/tutorial_usv)

set(name tutorial_usv)
configure_file(config/ros_gz_bridge.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_gz_bridge.yaml @ONLY)
```

Any other instance is generated on demand by `configure_vehicle.py`, which
runs the same xacro with another name and fills the same bridge template:

```bash
ros2 run tutorial_usv_gazebo configure_vehicle.py --name boat_a --out-dir ~/boat_a
```

That writes `model.sdf`, `model.config`, `tutorial_usv.urdf`,
`ros_gz_bridge.yaml` and an RViz config for `boat_a`; the launch files call
it for you.

```{tip}
Don't name the model directory after the package. Gazebo first looks for
`model://` names relative to the current directory, and a source checkout
contains a directory called `tutorial_usv_gazebo`. Hence `models/tutorial_usv`.
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
two-package layout above scales better, which is why the tutorial USV uses
it.

Next: [Spawn and drive](spawn-and-drive.md).
