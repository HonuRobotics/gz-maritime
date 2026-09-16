# Spawn and drive

A gz-maritime bring-up is two parts. The **simulation part**,
`kai_bringup/launch/simulation.launch.xml`, starts Gazebo on the ocean, the
GUI, and a bridge for `/clock`. It names no vehicle. The **spawn part**,
`kai_bringup/launch/spawn_vehicle.launch.xml`, puts one instance of any
vehicle in, with its bridge and its TF, given the vehicle's files and a
name. Run the first once and the second once per vehicle.

```{mermaid}
flowchart TB
  subgraph sim["kai_bringup simulation.launch.xml (once)"]
    S["Gazebo server<br/><i>open_water.sdf</i>"]
    GUI["Gazebo GUI"]
    CB["ros_gz_bridge<br/><i>/clock</i>"]
  end
  subgraph a["spawn_vehicle.launch.xml name:=boat_a"]
    SA["ros_gz_sim create<br/><i>boat_a</i>"]
    BA["ros_gz_bridge<br/><i>/boat_a/...</i>"]
    RA["robot_state_publisher<br/><i>frame_prefix boat_a/</i>"]
  end
  subgraph b["spawn_vehicle.launch.xml name:=boat_b"]
    SB["ros_gz_sim create<br/><i>boat_b</i>"]
    BB["ros_gz_bridge<br/><i>/boat_b/...</i>"]
    RB["robot_state_publisher<br/><i>frame_prefix boat_b/</i>"]
  end
  SA --> S
  SB --> S
  GUI --- S
  S <--> CB
  S <--> BA
  S <--> BB
  BA -->|/boat_a/joint_states| RA
  BB -->|/boat_b/joint_states| RB
```

## The simulation part

| Argument | Default | What it does |
|---|---|---|
| `world` | `open_water.sdf` | World to load. |
| `gazebo_gui` | `true` | Start the Gazebo GUI. |
| `use_composition` | `true` | Run the Gazebo server and the bridges in one process, `ros_gz_container`. |

Run on its own it gives you the ocean and nothing else.

## Spawn an instance

One command, with the vehicle's three files:

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=boat_a y:=2 \
  xacro:=$(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
  bridge:=$(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
  urdf:=$(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro
```

For one instance, all from the `name`, the launch:

1. **Renders the files for the name.** The model xacro with `name:=boat_a`,
   the URDF xacro, and the bridge template with `@name@` filled in, minus
   any `/clock` entry, into `instance_dir` (by default
   `~/.ros/kai_bringup/boat_a`). The model xacro gets the rendered URDF as
   `urdf_uri:=file://...` to merge.
2. **Spawns the model** under the name, at `x`, `y`, `z`, `roll`, `pitch`
   and `yaw`.
3. **Bridges its topics** in the `/boat_a` namespace.
4. **Publishes its TF**, every frame prefixed `boat_a/`, from a
   `robot_state_publisher` in the same namespace, on simulation time.

With composition on, the default, the bridge and the state publisher load
into the simulation's container, `ros_gz_container`, from wherever the
command is run, and the command returns once the model is in. The nodes
stay up in the container and go down with the simulation.

| Argument | Default | What it does |
|---|---|---|
| `name` | | Instance name: letters, digits and underscores, starting with a letter. |
| `xacro`, `bridge`, `urdf` | | The vehicle's model xacro, bridge template and URDF. `bridge` and `urdf` are optional. |
| `generator` | | Instead of the three files: a command that writes them for a name ([Other vehicles](#other-vehicles)). |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` | `0` | Spawn pose; z = 0 is the waterline. |
| `world` | empty | Name of the world to spawn into; empty is the one running. |
| `instance_dir` | `$ROS_HOME/kai_bringup/<name>` | Where the rendered files go. |
| `use_composition` | `true` | Load the bridge and state publisher into the simulation's container. |

### Your `sim.launch.xml`

A vehicle package usually keeps a convenience launch that includes the
simulation part and one spawn with its own three files.
`kai_custom_vehicle/launch/sim.launch.xml` is the template; copy it and
change the three paths:

```{literalinclude} ../../kai_custom_vehicle/launch/sim.launch.xml
:language: xml
:start-at: <let name="share"
:end-at: </launch>
```

Its arguments are the simulation part's plus `name` and the pose. The spawn
include is handed an empty `world` on purpose: to the spawn launch `world`
is the world's name, while to the simulation launch it is the SDF file, and
without that line the file name would leak into the include.

### One name per instance

Everything about an instance follows its `name`:

| | `name:=boat_a` gives |
|---|---|
| Gazebo model | `boat_a` |
| Gazebo and ROS topics | `/boat_a/motor_port/cmd`, `/boat_a/imu`, ... |
| ROS namespace | `/boat_a/ros_gz_bridge`, `/boat_a/robot_state_publisher` |
| TF frames | `boat_a/base_link`, `boat_a/imu_link`, ... |
| Sensor `frame_id` | `boat_a/imu_link` |
| RViz | fixed frame `boat_a/base_link`, RobotModel TF Prefix `boat_a`, description on `/boat_a/robot_description` |

The model file carries the name in its topics and frame ids (the xacro
`name` argument), the launch puts the nodes in the namespace,
`robot_state_publisher`'s `frame_prefix` puts it on the TF frames, and
`rviz.launch.xml` writes an RViz config that looks the robot up under it.
The URDF itself never contains it. A name must work as all four at once:
letters, digits and underscores, starting with a letter.

## What the launch needs from a vehicle

- **A model xacro** with a `name` argument used for the model name and every
  topic and frame id, and a `urdf_uri` argument for the URDF to merge.
- **A bridge template** listing the vehicle's own topics, with `@name@` where
  the name goes. No `/clock`: the launch drops one anyway, since the
  simulation part bridges it once.
- **A URDF**, xacro or plain, for `robot_state_publisher`; its frames get
  the `<name>/` prefix in TF, so sensor messages should carry the name in
  their `frame_id` too.
- **Absolute or `model://` references** for meshes and files inside the
  model, since the rendered model is written to the instance directory.

A vehicle laid out that way spawns any number of times with nothing else.

## The bridge configuration

Each entry connects one Gazebo topic to one ROS topic. The instance's
config is rendered from a template with `@name@` where the name goes:

```{literalinclude} ../../kai_custom_vehicle/config/ros_gz_bridge.yaml.in
:language: yaml
:start-at: - ros_topic_name: /@name@/motor_port/cmd
:lines: 1-5
```

The pairs a maritime vehicle usually needs:

| What | ROS type | Gazebo type | Direction |
|---|---|---|---|
| Propeller command | `std_msgs/msg/Float64` | `gz.msgs.Double` | `ROS_TO_GZ` |
| Propeller speed | `std_msgs/msg/Float64` | `gz.msgs.Double` | `GZ_TO_ROS` |
| Joint states | `sensor_msgs/msg/JointState` | `gz.msgs.Model` | `GZ_TO_ROS` |
| IMU | `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | `GZ_TO_ROS` |
| GPS | `sensor_msgs/msg/NavSatFix` | `gz.msgs.NavSat` | `GZ_TO_ROS` |
| Magnetometer | `sensor_msgs/msg/MagneticField` | `gz.msgs.Magnetometer` | `GZ_TO_ROS` |
| Camera | `sensor_msgs/msg/Image` | `gz.msgs.Image` | `GZ_TO_ROS` |

```{important}
No `/clock` in a vehicle's bridge. The simulation part bridges it once;
a second bridge of the same topic would publish it twice. The spawn launch
drops a clock entry if it finds one.
```

## Drive

One terminal per propeller. A command is a fraction of full thrust, from -1
(full astern) to 1 (full ahead). Terminal 1:

```bash
ros2 topic pub -r 5 /custom_usv/motor_port/cmd std_msgs/msg/Float64 "{data: 0.25}"
```

Terminal 2:

```bash
ros2 topic pub -r 5 /custom_usv/motor_stbd/cmd std_msgs/msg/Float64 "{data: 0.25}"
```

Commands latch until the next one arrives, so start the second right after
the first, and stop by sending `0.0` to both
([First simulation](../getting-started/first-simulation.md#3-drive-it)).

## Change the sea while it runs

```bash
gz service -s /world/default/wave/set_parameters --reqtype gz.msgs.Param \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'params {key: "sea_state" value {type: INT32 int_value: 3}}'
```

Sea states go from 0 (glassy) to 9. Remember that vehicles float on the flat
water level, not on the drawn waves
([Waves and physics today](own-vehicle.md#waves-and-physics-today)).

## Several vehicles

Run the spawn launch once per instance, each with its own name, after the
simulation part. Terminal 1, the ocean:

```bash
ros2 launch kai_bringup simulation.launch.xml
```

Terminal 2, the first boat:

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=boat_a y:=2 \
  xacro:=$(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
  bridge:=$(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
  urdf:=$(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro
```

Terminal 3, the second:

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=boat_b y:=-2 yaw:=1.57 \
  xacro:=$(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
  bridge:=$(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
  urdf:=$(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro
```

Both boats float, because each marks its own displacement; each has its own
topics, namespace and TF prefix; and `/clock` has one publisher. A boat
added an hour later joins the same way.

### Other vehicles

Different vehicles mix the same way: one spawn command each, next to the
one simulation part. A vehicle that generates its files with a script of
its own, as the Blue Robotics vehicles do, is given the script instead of
the three files:

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=blueboat x:=6 \
  generator:="$(ros2 pkg prefix blueboat_gazebo)/lib/blueboat_gazebo/configure_vehicle.py --config $(ros2 pkg prefix --share blueboat_description)/config/blueboat.yaml"
```

The launch runs the command with `--name <name> --out-dir <dir>` and expects
`model.sdf`, `ros_gz_bridge.yaml` and a URDF in that directory
([BlueBoat and BlueROV2](../vehicles/bluerobotics.md)).

Next: [Add sensors](sensors.md).
