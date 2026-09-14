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

### `tutorial_usv_gazebo` `spawn.launch.xml`

The spawn part for one instance of the tutorial USV: generates its model
and bridge config, spawns it, bridges its topics in its namespace and runs
its `robot_state_publisher` with its frame prefix. Include once per boat.

| Argument | Default | Description |
|---|---|---|
| `name` | `tutorial_usv` | Instance name: model name, topic prefix, ROS namespace and TF prefix. Letters, digits and underscores. |
| `x`, `y`, `z` | `0` | Spawn position [m]; z = 0 is the waterline. |
| `yaw` | `0` | Spawn heading [rad]. |
| `use_composition` | `true` | Load the bridge and state publisher into `ros_gz_container`. `false` when spawning into a simulation started by another `ros2 launch`. |

### `tutorial_usv_gazebo` `sim.launch.xml`

The simulation part plus one spawn. Takes the union of the arguments above.

### `tutorial_usv_gazebo` `two_usvs.launch.xml`

The simulation part plus two spawns, `boat_a` at y = 2 and `boat_b` at
y = -2. Takes the simulation part's arguments.

### `tutorial_usv_gazebo` `rviz.launch.xml`

RViz on one instance in a running simulation: the fixed frame, the
description topic and the RobotModel display's TF Prefix all set to the
instance, with simulation time.

| Argument | Default | Description |
|---|---|---|
| `name` | `tutorial_usv` | Instance to look at. |

### `tutorial_usv_description` `display.launch.xml`

Shows the tutorial USV's URDF in RViz, with no Gazebo and no frame prefix.

| Argument | Default | Description |
|---|---|---|
| `gui` | `true` | Start `joint_state_publisher_gui` to move the propellers. |

### `configure_vehicle.py`

What the spawn launch runs. You can also run it on its own:

```bash
ros2 run tutorial_usv_gazebo configure_vehicle.py --name NAME (--out-dir DIR | --cache)
```

It writes `model.sdf`, `model.config`, `tutorial_usv.urdf`,
`ros_gz_bridge.yaml` and `tutorial_usv.rviz` for the instance `NAME`. With `--cache` the directory
is `$ROS_HOME/tutorial_usv_gazebo/NAME` and its path is printed.

## The `open_water.sdf` world

World name: `default`.

| Element | Setting |
|---|---|
| Physics | DART, 4 ms step |
| `gz-sim-user-commands-system` | Spawning, moving and removing models |
| `gz-sim-scene-broadcaster-system` | Scene for the GUI |
| `gz-sim-sensors-system` | Rendered sensors (ogre2) |
| `gz-sim-imu-system`, `gz-sim-magnetometer-system`, `gz-sim-navsat-system` | IMU, magnetometer and GPS sensors |
| `gz-maritime-buoyancy-system` | Seawater 1025 kg/m³ below z = 0, air 1 kg/m³ above; `<enable_by_default>false</enable_by_default>`, no `<enable>` list |
| `gz-sim-waves-fft-system` | Sea state 1, updated at 30 Hz (Gerstner alternative in the file, commented out) |
| `model://water_surface` | Draws the sea |
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

The tutorial USV's topics are listed on its
[vehicle page](../vehicles/tutorial-usv.md#topics).
