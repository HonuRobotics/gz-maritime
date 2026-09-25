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
| `gz-maritime-wind-system` | Still air by default (`<speed>0</speed>`), changed at run time with the wind service; air 1.225 kg/m³ above z = 0, drag coefficient 1 unless a shape sets its own; pushes only marked collisions |
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

What a model says to the wind system, and what a world says to it.

| Where | Markup | Meaning |
|---|---|---|
| A `<collision>` in a model | `gz:wind="true"` (same `xmlns:gz` root attribute) | The wind pushes on this shape, by the part of it above the waterline, with quadratic drag on its projected area per axis. A buoyancy box can carry both marks. |
| The same `<collision>` | `gz:wind_cd="1.2"` | Its drag coefficient; the plugin's `<default_drag_coefficient>` otherwise. |
| The world plugin | `<speed>`, `<direction>` | The wind: m/s, and the direction it comes from in degrees clockwise from north (270, from the west, blows towards +x). |
| The world | `<wind><linear_velocity>x y z</linear_velocity></wind>` | Used as the starting wind when the plugin sets neither `<speed>` nor `<direction>`. |
| The world plugin | `<air_density>`, `<water_level>`, `<default_drag_coefficient>`, `<publish_rate>` | 1.225 kg/m³, z = 0, 1 and 10 Hz by default. |

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
`/world/default/wind/set`: a `gz.msgs.Param` with `speed` (m/s) and
`direction` (degrees the wind comes from, clockwise from north), either or
both. The simulation launch bridges it from ROS as
`ros_gz_interfaces/msg/ParamVec`, with each key a double parameter. The
current wind is published on `/world/default/wind_info` as `gz.msgs.Wind`.

A 6 m/s wind from the west, from ROS:

```bash
ros2 topic pub --once /world/default/wind/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: speed, value: {type: 3, double_value: 6.0}}, {name: direction, value: {type: 3, double_value: 270.0}}]}"
```

From Gazebo, turning it to come from the south, and reading it back:

```bash
gz topic -t /world/default/wind/set -m gz.msgs.Param \
  -p 'params {key: "direction" value {type: DOUBLE double_value: 180}}'
gz topic -e -t /world/default/wind_info -n 1
```

## Topics

The custom USV's topics are listed on its
[vehicle page](../vehicles/custom-usv.md#topics).
