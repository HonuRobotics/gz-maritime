# Spawn any vehicle into the ocean

The simulation and the vehicles are separate launches. The simulation launch
starts Gazebo on the open water world, the GUI and one bridge for the clock,
and names no vehicle:

```bash
ros2 launch kai_bringup simulation.launch.xml
```

The spawn launch adds one vehicle instance to it, under a name and at a
pose, and can be run as many times as there are instances:

```bash
boat="$(ros2 pkg prefix blueboat_gazebo)/lib/blueboat_gazebo/configure_vehicle.py --config $(ros2 pkg prefix --share blueboat_description)/config/blueboat.yaml"
ros2 launch kai_bringup spawn_vehicle.launch.xml generator:="$boat" name:=boat_a y:=2
ros2 launch kai_bringup spawn_vehicle.launch.xml generator:="$boat" name:=boat_b y:=-2 yaw:=1.57
```

Each instance gets its own model, its own Gazebo and ROS topics under
`/<name>/`, its own bridge and state publisher in the `/<name>` namespace,
and its own TF frames prefixed with `<name>/`. The world knows nothing about
any of them.

## What the launch needs from a vehicle

For one name, a model to spawn and, when the vehicle has them, a bridge
config and a URDF. The launch gets them from one of two sources:

- `generator:=<command>`: a command that, called with `--name <name>
  --out-dir <dir>`, writes `model.sdf`, `ros_gz_bridge.yaml` and a URDF for
  the name into that directory. The Blue Robotics vehicles ship one, and
  the command carries whatever else it needs, such as the vehicle config
  above. The BlueROV2 works the same way with its own generator and config.
- `xacro:=<file>` with optional `bridge:=<file>` and `urdf:=<file>`: a
  model xacro that takes a `name` argument and uses it for the model name
  and every topic, a bridge config template with `@name@` wherever the name
  goes, and a URDF (xacro or plain) for the state publisher. The launch
  renders the xacros for the name, and hands the model xacro the rendered
  URDF as `urdf_uri:=file://...` so it can merge it. A vehicle that has no
  generator of its own needs nothing else to be spawned any number of
  times. With a vehicle laid out that way, say `my_usv`:

  ```bash
  ros2 launch kai_bringup spawn_vehicle.launch.xml name:=usv_1 \
      xacro:=$(ros2 pkg prefix --share my_usv)/model.sdf.xacro \
      bridge:=$(ros2 pkg prefix --share my_usv)/config/ros_gz_bridge.yaml.in \
      urdf:=$(ros2 pkg prefix --share my_usv)/urdf/my_usv.urdf.xacro
  ```

The pose arguments are `x`, `y`, `z`, `roll`, `pitch` and `yaw`. `world`
picks the world when several run; empty spawns into the one running.
`instance_dir` is where the files of the instance are written, by default
`$ROS_HOME/kai_bringup/<name>` (`~/.ros/kai_bringup/<name>`).

## Rules for a vehicle to be spawnable more than once

- Every topic its plugins and sensors use carries the instance name, from
  the xacro `name` argument or because the vehicle's generator writes it. A
  topic without the name is shared by every instance.
- The bridge config lists only the vehicle's own topics. A `/clock` entry is
  dropped by the launch, since the simulation bridges the clock once.
- The URDF is the one for `robot_state_publisher`; its frames get the
  `<name>/` prefix in TF. Sensor messages carry whatever frame id the model
  gives them, so those should carry the name too.
- File and mesh references inside the model are absolute or `model://`
  URIs, since the model is written to the instance directory.
