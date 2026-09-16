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

### `kai_bringup` `multi_vehicle_demo.launch.xml`

The simulation part plus one vehicle of each type through their own
generators: a BlueBoat at the origin, a BlueROV2 six metres ahead and one
metre down, an X500 on the landing pad. Needs bluerobotics_models and
holybro_models in the workspace. Takes the simulation part's arguments.

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

| World | Site | Terrain | Where to start |
|---|---|---|---|
| `open_water.sdf` | Open sea, Monterey Bay coordinates | None; a landing pad at (8, -8) | Spawn anywhere |
| `sydney_regatta.sdf` | Sydney International Regatta Centre, VRX 2022 to 2024 | Fuel, fetched on first use (about 140 MB) | around x -530, y 170; start at x -532, y 162 |
| `benderson_park.sdf` | Nathan Benderson Park, Sarasota, RobotX 2022 | Fuel, fetched on first use (about 220 MB) | the lake runs along y; start at the origin, heading 1.57 |
| `sand_island.sdf` | Sand Island, Honolulu, RobotX 2018 and VRX 2019 | VRX mesh, fetched by the build; the shore camp from Fuel | start at x 158, y 108 |
| `la_spezia.sdf` | La Spezia marina, VORC 2020 | VORC mesh, fetched by the build | start at x 10, y -372 |

The site worlds place the water surface model once, at the centre of their
water: it draws a 3 km square of wave tiles around itself, and the terrain
hides it wherever there is land. Buoyancy does not depend on any of that.
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

## Topics

The custom USV's topics are listed on its
[vehicle page](../vehicles/custom-usv.md#topics).
