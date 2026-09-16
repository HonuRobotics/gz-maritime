# Custom USV

```{figure} ../getting-started/images/custom-usv.jpg
:alt: The custom USV, a twin-hull boat with a grey deck, floating on the ocean

The custom USV.
```

A small twin-hull surface vehicle made for these docs. It is deliberately
plain (box hulls, two propellers, three sensors) so that every file behind
it is a short read, and you can copy its package, `kai_custom_vehicle`, as
the starting point for your own vehicle.

| | |
|---|---|
| Size | 1.0 m long, 0.65 m wide |
| Mass | 12.1 kg: 12 kg hull and deck, plus 0.05 kg per propeller |
| Draft | 3.9 cm |
| Propulsion | Two propellers, steered by thrust difference; up to 20 N ahead and 15 N astern each, commanded as a fraction of that |
| Sensors | IMU, magnetometer, GPS |

## Run it

```bash
ros2 launch kai_custom_vehicle sim.launch.xml                     # one boat, with the GUI
ros2 launch kai_custom_vehicle sim.launch.xml gazebo_gui:=false   # headless
ros2 launch kai_custom_vehicle sim.launch.xml name:=boat_a y:=2   # under another name, at a pose
ros2 launch kai_custom_vehicle rviz.launch.xml name:=boat_a       # RViz on it
```

| Argument | Default | What |
|---|---|---|
| `world` | `open_water.sdf` | World to load |
| `gazebo_gui` | `true` | Start the Gazebo GUI |
| `use_composition` | `true` | Run Gazebo, the bridge and the state publisher in one process |
| `name` | `custom_usv` | Instance name: model, topic prefix, namespace and TF prefix; `rviz.launch.xml` takes it too |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` | `0` | Spawn pose; z = 0 is the waterline |

`sim.launch.xml` is the simulation part plus one spawn. More boats join a
running simulation through gz-maritime's spawn launch, with this package's
three files and a new name each:

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=boat_b y:=-2 \
  xacro:=$(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
  bridge:=$(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
  urdf:=$(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro
```

In a simulation every frame carries the boat's name, which is why RViz goes
through `rviz.launch.xml` rather than a bare `rviz2`.

To look at the description alone, without Gazebo:

```bash
ros2 launch kai_custom_vehicle display.launch.xml
```

RViz opens with sliders that spin the propellers.

## Drive it

Each propeller takes a command in [-1, 1] on `/<name>/motor_<side>/cmd`,
the fraction of its full thrust to apply (1 full ahead, -1 full astern),
where `<name>` is the instance the boat was spawned as, and keeps the last
command it got. Drive with one terminal per propeller, and stop by sending
`0.0` to both (see
[First simulation](../getting-started/first-simulation.md#3-drive-it)).

The one boat of `sim.launch.xml` is `custom_usv`. Straight ahead at a
quarter of full thrust, terminal 1:

```bash
ros2 topic pub -r 5 /custom_usv/motor_port/cmd std_msgs/msg/Float64 "{data: 0.25}"
```

Terminal 2:

```bash
ros2 topic pub -r 5 /custom_usv/motor_stbd/cmd std_msgs/msg/Float64 "{data: 0.25}"
```

A second boat spawned as `boat_b` listens only to `/boat_b/...`. To turn it
to port while the first goes straight, more starboard thrust than port, in
two more terminals:

```bash
ros2 topic pub -r 5 /boat_b/motor_port/cmd std_msgs/msg/Float64 "{data: 0.1}"
```

```bash
ros2 topic pub -r 5 /boat_b/motor_stbd/cmd std_msgs/msg/Float64 "{data: 0.3}"
```

To see which boats are there:

```bash
ros2 topic list | grep '/cmd$'
```

## Topics

For the instance `custom_usv`; another name replaces the prefix.

| ROS topic | Type | Direction | What |
|---|---|---|---|
| `/custom_usv/motor_port/cmd` | `std_msgs/msg/Float64` | to Gazebo | Port propeller command, a fraction of full thrust in [-1, 1] |
| `/custom_usv/motor_stbd/cmd` | `std_msgs/msg/Float64` | to Gazebo | Starboard propeller command, likewise |
| `/custom_usv/motor_port/cmd/ang_vel` | `std_msgs/msg/Float64` | from Gazebo | Port propeller speed [rad/s] |
| `/custom_usv/motor_stbd/cmd/ang_vel` | `std_msgs/msg/Float64` | from Gazebo | Starboard propeller speed [rad/s] |
| `/custom_usv/imu` | `sensor_msgs/msg/Imu` | from Gazebo | 50 Hz, in `custom_usv/imu_link` |
| `/custom_usv/magnetometer` | `sensor_msgs/msg/MagneticField` | from Gazebo | 50 Hz, in `custom_usv/imu_link` |
| `/custom_usv/navsat` | `sensor_msgs/msg/NavSatFix` | from Gazebo | 5 Hz, in `custom_usv/gps_link` |
| `/custom_usv/joint_states` | `sensor_msgs/msg/JointState` | from Gazebo | Propeller joint angles |
| `/clock` | `rosgraph_msgs/msg/Clock` | from Gazebo | Simulation time, bridged once by the simulation launch |

`/custom_usv/robot_state_publisher` turns the joint states into `/tf`,
publishes the fixed frames on `/tf_static`, all prefixed `custom_usv/`,
and latches the URDF on `/custom_usv/robot_description`, which is where
RViz reads it.

## Frames

```{figure} ../how-to/images/usv-frames.svg
:alt: Top view of the custom USV with base_link at the centre, imu_link above it, gps_link towards the stern and the two propeller joints at the stern of each hull
:width: 100%

The custom USV's frames, to scale. `base_link`'s origin sits on the
waterline. In a simulation each carries the instance name:
`custom_usv/base_link`.
```

## What it is made of

One package, `kai_custom_vehicle`. The Blue Robotics vehicles split the same
files into a description package and a Gazebo package; here they sit side by
side, because there are only a handful:

```{mermaid}
flowchart TB
  subgraph pkg["kai_custom_vehicle"]
    D["urdf/dimensions.xacro"]
    X["urdf/custom_usv.urdf.xacro"]
    W["models/custom_usv/model.sdf.xacro"]
    T["config/ros_gz_bridge.yaml.in"]
    R["rviz/custom_usv.rviz"]
    S["launch/sim.launch.xml"]
    Z["launch/rviz.launch.xml<br/>scripts/instance_rviz.py"]
  end
  subgraph kb["kai_bringup"]
    SIM["simulation.launch.xml"]
    L["spawn_vehicle.launch.xml<br/><i>one instance name</i>"]
  end
  D --> X
  D --> W
  X -->|urdf:=| L
  W -->|xacro:=| L
  T -->|bridge:=| L
  L --> M["model.sdf, robot.urdf, ros_gz_bridge.yaml<br/><i>rendered per name</i>"]
  S -->|includes| SIM
  S -->|includes| L
  R --> Z
```

- **`urdf/`** is the URDF: shape, mass, joints and frames, with nothing
  Gazebo-specific and no instance name in it.
  [Describe the vehicle](../how-to/vehicle-description.md) walks through it.
- **`models/custom_usv/`** is the simulation model: the URDF plus the marked
  displacement volume, thrusters, damping and sensors, for one instance
  name. With `config/ros_gz_bridge.yaml.in`, the bridge template, it is what
  the spawn launch renders for each instance; the build renders the default
  one, so `model://custom_usv` works in plain `gz sim` too.
  [Compose the Gazebo model](../how-to/gazebo-composition.md) and
  [Spawn and drive](../how-to/spawn-and-drive.md) walk through them.
- **`launch/`** holds the conveniences: `sim` (the simulation part plus one
  spawn), `rviz` (RViz on one instance, on a config `instance_rviz.py`
  writes for its name) and `display` (the URDF alone).

## Limitations

- The hydrodynamic damping values are placeholders, not measured.
- The sensors are ideal: no noise, no bias, no drift.
- It floats on the flat water level: waves are drawn but don't move it yet.
