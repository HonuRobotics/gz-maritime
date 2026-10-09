# Launch files, world and services

## Launch files

### `kai_bringup` `simulation.launch.xml`

The simulation part: Gazebo on the ocean, the GUI, and a bridge for
`/clock`. Names no vehicle.

| Argument | Default | Description |
|---|---|---|
| `world` | `open_water.sdf` | World file, by name on `GZ_SIM_RESOURCE_PATH` or as a full path. |
| `gazebo_gui` | `true` | Start the Gazebo GUI. |
| `use_composition` | `true` | Run the Gazebo server and the bridge as composable nodes in `ros_gz_container`. |

### `kai_bringup` `spawn_vehicle.launch.xml`

The spawn part for one instance of any vehicle: renders its files for the
name, spawns it, bridges its topics in its namespace and runs its
`robot_state_publisher` with its frame prefix. Run once per instance while
the simulation runs.

| Argument | Default | Description |
|---|---|---|
| `name` | | Instance name: model name, topic prefix, ROS namespace and TF prefix. Letters, digits and underscores, starting with a letter. |
| `xacro` | | Model xacro, rendered with `name:=<name>` and, when a URDF is given, `urdf_uri:=file://<rendered URDF>`. A plain SDF file is copied as it is. |
| `bridge` | | Bridge config template with `@name@` where the name goes; a `/clock` entry is dropped. Optional. |
| `urdf` | | URDF, xacro or plain, for `robot_state_publisher`. Optional. |
| `generator` | | Instead of `xacro`: a command run as `<command> --name <name> --out-dir <dir>` that writes `model.sdf`, `ros_gz_bridge.yaml` and a URDF there. |
| `x`, `y`, `z` | `0` | Spawn position [m]; z = 0 is the waterline. |
| `roll`, `pitch`, `yaw` | `0` | Spawn orientation [rad]. |
| `world` | empty | Name of the world to spawn into; empty picks the one running. |
| `instance_dir` | `$ROS_HOME/kai_bringup/<name>` | Where the instance's files are written; `~/.ros/kai_bringup/<name>` by default. |
| `use_composition` | `true` | Load the bridge and state publisher into `container_name`. |
| `container_name` | `ros_gz_container` | The simulation launch's container. |

### `kai_bringup` `multi_vehicle_demo.launch.py`

The simulation part plus one vehicle of each type through their own
generators: a BlueBoat, a BlueROV2 six metres ahead and one metre down, an
X500 on the landing pad. Needs bluerobotics_models and holybro_models in
the workspace. [Several vehicles on one ocean](../how-to/several-vehicles.md)
walks through it.

| Argument | Default | Description |
|---|---|---|
| `world` | `sydney_regatta.sdf` | World file, as for the simulation launch, among those the demo has poses for: `sydney_regatta.sdf` (off the start point) or `open_water.sdf` (around the origin). |
| `gazebo_gui` | `true` | Start the Gazebo GUI. |
| `use_composition` | `true` | Run Gazebo, the bridges and the state publishers in one container. |

### `kai_custom_vehicle` `sim.launch.xml`

The simulation part plus one spawn of the custom USV, from the package's
own model xacro, bridge template and URDF. Takes the simulation part's
arguments, `name` (default `custom_usv`) and `x`, `y`, `z`, `roll`, `pitch`,
`yaw`.

### `kai_custom_vehicle` `rviz.launch.xml`

RViz on one instance in a running simulation: the fixed frame, the
description topic and the RobotModel display's TF Prefix all set to the
instance, with simulation time.

| Argument | Default | Description |
|---|---|---|
| `name` | `custom_usv` | Instance to look at. |
| `instance_dir` | `$ROS_HOME/kai_bringup/<name>` | Where the config is written, next to the instance's other files. |

### `kai_custom_vehicle` `display.launch.xml`

Shows the custom USV's URDF in RViz, with no Gazebo and no frame prefix.

| Argument | Default | Description |
|---|---|---|
| `gui` | `true` | Start `joint_state_publisher_gui` to move the propellers. |

### `instantiate_vehicle.py`

What the spawn launch runs to produce an instance's files. You can also run
it on its own:

```bash
ros2 run kai_bringup instantiate_vehicle.py --name NAME --out-dir DIR \
  --xacro FILE [--bridge FILE] [--urdf FILE]
ros2 run kai_bringup instantiate_vehicle.py --name NAME --out-dir DIR --generator CMD
```

It writes `model.sdf` and, when given, `ros_gz_bridge.yaml` (without
`/clock`) and `robot.urdf` for the instance `NAME` into `DIR`.

### `instance_rviz.py`

What `rviz.launch.xml` runs:

```bash
ros2 run kai_custom_vehicle instance_rviz.py --name NAME --config BASE.rviz --out FILE
```

It writes the base RViz config pointed at the instance (fixed frame,
description topic, TF Prefix) to `FILE` and prints the path.

## Worlds

Five worlds share one contract: the systems every vehicle needs, the
gz-maritime buoyancy reading marked collisions, the waterline at z = 0 and
the wave field for the drawn sea. Pass any of them to the simulation launch
as `world:=<name>.sdf`; the spawn launch then puts vehicles into it by name.
Four of them are venues of the Virtual RobotX competition (VRX) and the
Virtual Ocean Robotics Challenge (VORC).
All of them are named `default` inside, so the services below work on
every one.

| World | Site | Terrain | Where to start | Water and sea states |
|---|---|---|---|---|
| `open_water.sdf` | Open sea, Monterey Bay coordinates | None; a landing pad at (8, -8) | Spawn anywhere | Open sea, any |
| `sydney_regatta.sdf` | Sydney International Regatta Centre, VRX 2022 to 2024 | Fuel, fetched on first use (about 140 MB) | around x -530, y 170; start at x -532, y 162 | Rowing lake, 0 to 2 |
| `benderson_park.sdf` | Nathan Benderson Park, Sarasota, RobotX 2022 | Fuel, fetched on first use (about 220 MB) | the lake runs along y; start at the origin, heading 1.57 | Rowing lake, 0 to 2 |
| `sand_island.sdf` | Sand Island, Honolulu, RobotX 2018 and VRX 2019 | VRX mesh, fetched by the build; the shore camp from Fuel | start at x 158, y 108 | Sheltered lagoon, 0 to 3 |
| `la_spezia.sdf` | La Spezia marina, VORC 2020 | VORC mesh, fetched by the build | start at x 10, y -372 | Marina and gulf, 0 to 3 |

Every world starts at sea state 1 and takes any of the ten codes, but the
last column says what fits the site: a rowing lake never sees more than
wind chop (code 2 is 0.3 m waves), a sheltered lagoon or gulf at most a
slight sea (code 3 is about 0.9 m), and the open water world takes them
all. Change it with the wave service below.

The site worlds place the water surface model once, at the centre of their
water, and size the drawn sea with `<tiles_radius>` in their wave source:
the model draws that many 200 m tiles around itself in every direction,
sharing one wave field, and the terrain hides them wherever there is land.
Open water keeps the model's own radius, a 1 km square. Buoyancy does not
depend on any of that.
The terrain models are Apache 2.0 assets from VRX and VORC,
credited in `kai_gazebo/NOTICE`.

## The `open_water.sdf` world

World name: `default`.

| Element | Setting |
|---|---|
| Physics | DART, 4 ms step |
| `gz-sim-user-commands-system` | Spawning, moving and removing models |
| `gz-sim-scene-broadcaster-system` | Scene for the GUI |
| `gz-sim-sensors-system` | Rendered sensors (ogre2) |
| `gz-sim-imu-system`, `gz-sim-magnetometer-system`, `gz-sim-navsat-system`, `gz-sim-air-pressure-system` | IMU, magnetometer, GPS and barometer sensors |
| `gz-maritime-buoyancy-system` | Seawater 1025 kg/m³ below z = 0, air 1 kg/m³ above; `<enable_by_default>false</enable_by_default>`, no `<enable>` list |
| `gz-maritime-wind-system` | Still air by default (`<speed>0</speed>`) at a 10 m reference height, falling off towards the water with a 0.0002 m roughness length, changed at run time on the wind topic; pushes only marked collisions, with air at 1.225 kg/m³ and a drag coefficient of 1 unless a shape sets its own |
| `gz-maritime-ocean-current-system` | Slack water by default (`<speed>0</speed>`); a uniform, horizontal current set in the world file and changed at run time on the ocean current topic, kept on the world as a recipe any system can ask at a point and published as ground truth. A vehicle feels it through `gz-maritime-hydrodynamics-system`; gz-sim's own Hydrodynamics does not see it |
| `gz-sim-waves-fft-system` | Sea state 1, updated at 30 Hz (Gerstner alternative in the file, commented out) |
| `model://water_surface` | Draws the sea |
| `model://landing_pad` at (8, -8) | A static 4 m deck 1 m above the water, the only solid ground; spawn a quad on it at z = 1.25 |
| `<spherical_coordinates>` | 36.693509° N, 121.936568° W, elevation 0, ENU |

## Buoyancy markup

What a model says to the buoyancy system, and what a world says to it.

| Where | Markup | Meaning |
|---|---|---|
| A `<collision>` in a model | `gz:buoyancy="true"` (with `xmlns:gz="http://gazebosim.org/schema"` on the root) | This shape displaces water. A link with any marked collision floats by its marked collisions alone, in nested models too. |
| A URDF | The same `<collision>`, inside a `<link>` in a model-level `<gazebo>` block | The only place the conversion keeps the attribute; on a URDF collision or under `<gazebo reference>` it is dropped. |
| The same `<collision>` | `<surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>` | The shape touches nothing; recommended for every mark. |
| The world plugin | `<enable_by_default>false</enable_by_default>` | Unmarked collisions never float. Defaults to `true` with no `<enable>` list and `false` with one. |
| The world plugin | `<enable>model</enable>`, `<enable>model::link</enable>` | Unmarked collisions of the named model or link float. Names as spawned. |

## Wind markup

What a model says to the wind system.

| Where | Markup | Meaning |
|---|---|---|
| A `<collision>` in a model | `gz:wind="true"` (same `xmlns:gz` root attribute) | The wind pushes on this shape, by the part of it above the water, with quadratic drag on its projected area per axis, taking the wind at the centre of that part. `"1"` works too. A mesh counts as its bounding box. A buoyancy box can carry both marks. |
| The same `<collision>` | `gz:wind_cd="1.2"` | Its drag coefficient; the plugin's `<default_drag_coefficient>` otherwise. |
| A `<sensor>` on a link | `type="custom" gz:type="anemometer"` | Reads the apparent wind at the sensor, in the sensor frame, as `gz.msgs.Twist` on its `<topic>`, at its `<update_rate>` (every step without one), with its `<frame_id>`. |

## Wind parameters

What a world says to the wind system. Each parameter but `<model>`,
`<publish_rate>`, `<air_density>` and `<default_drag_coefficient>` is also a
key on the wind topic, below.

| Markup | Meaning |
|---|---|
| `<speed>`, `<direction>` | The wind: m/s, and the direction it comes from in degrees clockwise from north (270, from the west, blows towards +x). |
| `<vertical>` | The vertical wind, m/s, positive up; the z of the world's `<wind>` by default. |
| `<wind><linear_velocity>x y z</linear_velocity></wind>` on the world | Used as the starting wind when the plugin sets neither `<speed>` nor `<direction>`. |
| `<speed_gust>`, `<speed_gust_time>`, `<direction_gust>`, `<direction_gust_time>` | Gusts: standard deviation (m/s, degrees) and correlation time (s, 2 and 10 by default) of the speed and the direction; 0 is steady. The gusts travel with the mean wind. |
| `<reference_height>`, `<roughness_length>`, `<water_level>` | The height above the water `<speed>` is given at, 10 m by default, the surface roughness of a logarithmic wind profile, 0.0002 m over open sea (0, the default, is a uniform wind; it slows the mean wind, not the gusts), and the world z of the water. Gazebo's rotor and wing systems see the reference height wind. |
| `<model>` | The wind model the recipe names, `standard` by default. |
| `<seed>` | Seed of the gusts; the same seed repeats them, and 0, the default, draws one each run. |
| `<publish_rate>` | Rate of the ground truth, 10 Hz by default. |
| `<air_density>`, `<default_drag_coefficient>` | For the windage: 1.225 kg/m³ and 1 by default. |

The system keeps the wind on the world entity as a recipe, the way the wave
field works, and writes it into gz-sim's wind entity at the world's origin.
A system that needs the wind at a point asks `gz::sim::wind::WindAt`, or
keeps a `gz::sim::wind::WindSampler`, from the `gz_wind` library.

## Ocean current parameters

What a world says to the ocean current system.

| Markup | Meaning |
|---|---|
| `<speed>`, `<direction>` | The current: m/s, and the direction it sets towards in degrees clockwise from north (90, setting east, flows towards +x). The opposite convention from the wind, which is given by where it comes from. Also keys on the ocean current topic, below. |
| `<model>` | The current model the recipe names, `standard` by default: uniform and horizontal. A new model is one class registered under a name; nothing else changes. |
| `<water_level>` | World z of the water's surface, 0 by default, so a model that varies with depth knows where the surface is; `standard` does not read it. |
| `<parameters>` | The parameters a model owns, one element each, such as `<source>`, the file of a gridded current. Opaque to the system, which only stores and replicates them; the model accepts or refuses them. A leaf is its text; an element with children or attributes of its own, such as `<layer><depth>10</depth></layer>`, is its SDF text, for the model to parse. A name that repeats is a list, numbered in order: three `<constituent>` elements are `constituent.0`, `constituent.1` and `constituent.2`. `standard` takes none. |
| `<seed>` | For a model that draws random numbers; `standard` draws none. 1 by default, so a run repeats; 0 draws a new one each run. |
| `<publish_rate>` | Rate of the ground truth, 10 Hz by default. |

Any other element is warned about and ignored, so a typo such as `<speeed>`
does not leave slack water in silence. A `<model>` nobody registered, or one
that refuses its parameters, is one error from the system, and the world has
no current and no ground truth.

The current is horizontal and uniform with the `standard` model, and
constant between changes: on the kilometre and hour scales these worlds work at, that is the
water. The system keeps it on the world entity as a recipe, the way the wind
and the wave field work, written as a component change and nothing else, so
every consumer, loaded with the world or spawned later, sees it on the same
step. A system that asks the current every step keeps a
`gz::sim::ocean_current::OceanCurrentSampler`, from the `gz_ocean_current`
library; `gz::sim::ocean_current::OceanCurrentAt` builds the model on every
call and suits an occasional query. The query is a point query: a consumer that spans a gradient
integrates over its own extent with repeated queries.

## Thruster command

What a vehicle's Thruster plugins expect, with `gz-maritime-thruster-system`
in normalized mode (`<use_normalized_cmd>true</use_normalized_cmd>`).

| Topic | Type | Meaning |
|---|---|---|
| `/<name>/motor_<side>/cmd` | `gz.msgs.Double` (`std_msgs/msg/Float64` over the bridge) | A fraction of full thrust in [-1, 1]: 1 is `<max_thrust_cmd>` ahead, -1 is `<min_thrust_cmd>` astern, 0 stops. Latches. |
| `/<name>/motor_<side>/cmd/ang_vel` | `gz.msgs.Double` | Propeller speed [rad/s]. |

## Services

All take and return Gazebo messages; call them with `gz service`.

| Service | Request | Reply | What it does |
|---|---|---|---|
| `/world/default/wave/set_parameters` | `gz.msgs.Param` | `gz.msgs.Boolean` | Change wave parameters while running. |
| `/world/default/create` | `gz.msgs.EntityFactory` | `gz.msgs.Boolean` | Spawn a model. |
| `/world/default/set_pose` | `gz.msgs.Pose` | `gz.msgs.Boolean` | Move a model. |

### Examples

Change the sea state (0 to 9):

```bash
gz service -s /world/default/wave/set_parameters --reqtype gz.msgs.Param \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'params {key: "sea_state" value {type: INT32 int_value: 3}}'
```

The other wave parameters are listed in the
[wave design notes](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/WAVES_DESIGN.md).

Spawn a model, and move it:

```bash
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'sdf_filename: "/path/to/model.sdf", name: "my_boat", pose: {position: {z: 0}}'
gz service -s /world/default/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "my_boat", position: {x: 5, y: 0, z: 0}, orientation: {w: 1}'
```

## Wind

The wind changes while the simulation runs on the topic
`/world/default/wind/set`: a `gz.msgs.Param` with `speed` (m/s),
`direction` (degrees the wind comes from, clockwise from north, kept in
[0, 360)), `vertical` (m/s, positive up), or any of the gust parameters, in
any combination.
The simulation launch bridges it from ROS as
`ros_gz_interfaces/msg/ParamVec`, with each key a double parameter. The
current wind is published on `/world/default/wind_info` as `gz.msgs.Wind`,
and on `/world/default/wind/velocity` as `gz.msgs.Twist`, which the
simulation launch bridges to ROS as `geometry_msgs/msg/TwistStamped` in the
`world` frame. Gazebo's rotor, wing and air speed systems all read the same
wind, at the reference height.

### Direction and units

- **Speed** in metres per second, at the reference height, 10 m unless the
  world says otherwise, as in weather reports.
- **Direction** the wind comes from, in degrees clockwise from true north,
  as in weather reports, ArduPilot and the marine textbooks: 0 is a north
  wind, blowing south; 270 is a west wind, blowing east.
- **North** is the world's, from its `<spherical_coordinates>`, the same
  north its GPS and magnetometer use. With the `ENU` orientation and a
  `heading_deg` of 0, which every world here has, north is the world's +y
  axis and east its +x axis.
- **The ground truth** on `/world/default/wind_info` is the air's velocity
  in the world frame, not a direction: a west wind of 6 m/s reads
  `x: 6, y: 0` in these worlds.

A 6 m/s wind from the west, from ROS:

```bash
ros2 topic pub --once /world/default/wind/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: speed, value: {type: 3, double_value: 6.0}}, {name: direction, value: {type: 3, double_value: 270.0}}]}"
```

Gusts of 1.5 m/s that last about 3 s, from ROS:

```bash
ros2 topic pub --once /world/default/wind/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: speed_gust, value: {type: 3, double_value: 1.5}}, {name: speed_gust_time, value: {type: 3, double_value: 3.0}}]}"
```

From Gazebo, turning it to come from the south, and reading it back:

```bash
gz topic -t /world/default/wind/set -m gz.msgs.Param \
  -p 'params {key: "direction" value {type: DOUBLE double_value: 180}}'
gz topic -e -t /world/default/wind_info -n 1
```

## Ocean current

The ocean current is set in the world file and changes while the simulation
runs on the topic `/world/default/ocean_current/set`: a `gz.msgs.Param` with
`speed` (m/s), `direction` (degrees the current sets towards, clockwise
from north, kept in [0, 360)) or `water_level` (m), in any combination, each
a double or an integer; any other key is a parameter the model owns and takes
a string or a number. The simulation launch bridges it from ROS as
`ros_gz_interfaces/msg/ParamVec`. A message is applied whole or not at all:
one key out of range, of the wrong type, or refused by the model, and it
changes nothing. The model itself cannot change at run time: a `model` key
is refused. A change takes effect on the next step, for every vehicle
at once. The current is published on `/world/default/ocean_current_info` as
`gz.msgs.Twist`, which the simulation launch bridges to ROS as
`geometry_msgs/msg/TwistStamped` in the `world` frame.

### Direction and units

- **Speed** in metres per second.
- **Direction** the current sets towards, in degrees clockwise from true
  north, as charts draw it and tide tables predict it: 0 sets north, 90 sets
  east. The wind is given the other way round, by where it comes from: a
  wind from 270 and a current setting 090 move a boat the same way.
- **North** is the world's, the same one the wind and the GPS use: with the
  `ENU` orientation and a `heading_deg` of 0, which every world here has,
  north is the world's +y axis and east its +x axis.
- **The ground truth** on `/world/default/ocean_current_info` is the water's
  velocity in the world frame, not a direction: a 0.5 m/s current setting
  east reads `x: 0.5, y: 0` in these worlds.

A 0.5 m/s current setting east, from ROS:

```bash
ros2 topic pub --once /world/default/ocean_current/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: speed, value: {type: 3, double_value: 0.5}}, {name: direction, value: {type: 3, double_value: 90.0}}]}"
```

From Gazebo, turning it to set south, and reading it back:

```bash
gz topic -t /world/default/ocean_current/set -m gz.msgs.Param \
  -p 'params {key: "direction" value {type: DOUBLE double_value: 180}}'
gz topic -e -t /world/default/ocean_current_info -n 1
```

## Topics

The custom USV's topics are listed on its
[vehicle page](../vehicles/custom-usv.md#topics).
