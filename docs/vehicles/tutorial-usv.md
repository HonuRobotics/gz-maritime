# Tutorial USV

```{figure} ../getting-started/images/tutorial-usv.jpg
:alt: The tutorial USV, a twin-hull boat with a grey deck, floating on the ocean

The tutorial USV.
```

A small twin-hull surface vehicle made for these docs. It is deliberately
plain (box hulls, two propellers, three sensors) so that every file behind
it is a short read, and you can copy it as the starting point for your own
vehicle.

| | |
|---|---|
| Size | 1.0 m long, 0.65 m wide |
| Mass | 12.1 kg: 12 kg hull and deck, plus 0.05 kg per propeller |
| Draft | 3.9 cm |
| Propulsion | Two propellers, steered by thrust difference; up to 20 N ahead and 15 N astern each |
| Sensors | IMU, magnetometer, GPS |

## Run it

```bash
ros2 launch tutorial_usv_gazebo sim.launch.xml                     # one boat, with the GUI
ros2 launch tutorial_usv_gazebo sim.launch.xml gazebo_gui:=false   # headless
ros2 launch tutorial_usv_gazebo two_usvs.launch.xml                # boat_a and boat_b
ros2 launch tutorial_usv_gazebo spawn.launch.xml name:=boat_c y:=-6 use_composition:=false   # one more, into a running simulation
ros2 launch tutorial_usv_gazebo rviz.launch.xml name:=boat_a                              # RViz on one of them
```

| Argument | Default | |
|---|---|---|
| `world` | `open_water.sdf` | World to load |
| `gazebo_gui` | `true` | Start the Gazebo GUI |
| `use_composition` | `true` | Run Gazebo, the bridges and the state publishers in one process |
| `name` | `tutorial_usv` | Instance name: model, topic prefix, namespace and TF prefix |
| `x`, `y`, `z`, `yaw` | `0` | Spawn pose; z = 0 is the waterline |

To look at the description alone, without Gazebo:

```bash
ros2 launch tutorial_usv_description display.launch.xml
```

RViz opens with sliders that spin the propellers.

## Drive it

Each propeller takes a thrust command in newtons on
`/<name>/motor_<side>/thrust`, where `<name>` is the instance the boat was
spawned as. Commands latch, so send both of a boat's commands together and
send `0.0` to stop (see
[First simulation](../getting-started/first-simulation.md#3-drive-it)).

The one boat of `sim.launch.xml` is `tutorial_usv`. Straight ahead:

```bash
ros2 topic pub -t 5 -r 5 /tutorial_usv/motor_port/thrust std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -t 5 -r 5 /tutorial_usv/motor_stbd/thrust std_msgs/msg/Float64 "{data: 5.0}" &
wait
```

The two boats of `two_usvs.launch.xml` are `boat_a` and `boat_b`, and each
listens only to its own topics. Send `boat_a` ahead and turn `boat_b` to
port with more starboard thrust, all at once:

```bash
ros2 topic pub -t 5 -r 5 /boat_a/motor_port/thrust std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -t 5 -r 5 /boat_a/motor_stbd/thrust std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -t 5 -r 5 /boat_b/motor_port/thrust std_msgs/msg/Float64 "{data: 2.0}" &
ros2 topic pub -t 5 -r 5 /boat_b/motor_stbd/thrust std_msgs/msg/Float64 "{data: 6.0}" &
wait
```

A boat added later with `spawn.launch.xml name:=boat_c` answers on
`/boat_c/...` the same way. To see which boats are there:

```bash
ros2 topic list | grep '/thrust$'
```

## Topics

For the instance `tutorial_usv`; another name replaces the prefix.

| ROS topic | Type | Direction | What |
|---|---|---|---|
| `/tutorial_usv/motor_port/thrust` | `std_msgs/msg/Float64` | to Gazebo | Port thrust command [N] |
| `/tutorial_usv/motor_stbd/thrust` | `std_msgs/msg/Float64` | to Gazebo | Starboard thrust command [N] |
| `/tutorial_usv/motor_port/thrust/ang_vel` | `std_msgs/msg/Float64` | from Gazebo | Port propeller speed [rad/s] |
| `/tutorial_usv/motor_stbd/thrust/ang_vel` | `std_msgs/msg/Float64` | from Gazebo | Starboard propeller speed [rad/s] |
| `/tutorial_usv/imu` | `sensor_msgs/msg/Imu` | from Gazebo | 50 Hz, in `tutorial_usv/imu_link` |
| `/tutorial_usv/magnetometer` | `sensor_msgs/msg/MagneticField` | from Gazebo | 50 Hz, in `tutorial_usv/imu_link` |
| `/tutorial_usv/navsat` | `sensor_msgs/msg/NavSatFix` | from Gazebo | 5 Hz, in `tutorial_usv/gps_link` |
| `/tutorial_usv/joint_states` | `sensor_msgs/msg/JointState` | from Gazebo | Propeller joint angles |
| `/clock` | `rosgraph_msgs/msg/Clock` | from Gazebo | Simulation time, bridged once by the simulation launch |

`/tutorial_usv/robot_state_publisher` turns the joint states into `/tf`, and
publishes the fixed frames on `/tf_static`, all prefixed `tutorial_usv/`.

## Frames

```{figure} ../how-to/images/usv-frames.svg
:alt: Top view of the tutorial USV with base_link at the centre, imu_link above it, gps_link towards the stern and the two propeller joints at the stern of each hull
:width: 100%

The tutorial USV's frames, to scale. `base_link`'s origin sits on the
waterline. In a simulation each carries the instance name:
`tutorial_usv/base_link`.
```

## What it is made of

It is two packages, split the same way as the Blue Robotics vehicles:

```{mermaid}
flowchart TB
  subgraph desc["tutorial_usv_description"]
    D["dimensions.xacro"]
    X["tutorial_usv.urdf.xacro"] -->|xacro, at build time| U["tutorial_usv.urdf"]
    D --> X
  end
  subgraph gz["tutorial_usv_gazebo"]
    W["model.sdf.xacro"] -->|xacro, per instance name| M["model.sdf"]
    T["ros_gz_bridge.yaml.in"] -->|per instance name| Y["ros_gz_bridge.yaml"]
    C["configure_vehicle.py"]
    L["spawn.launch.xml"]
  end
  D --> W
  U -->|merged into| M
  U -->|robot_state_publisher| L
  L -->|runs| C
  C --> M
  C --> Y
  L -->|spawns| M
  L -->|loads| Y
```

- **`tutorial_usv_description`** is the URDF: shape, mass, joints and frames,
  with nothing Gazebo-specific and no instance name in it.
  [Describe the vehicle](../how-to/vehicle-description.md) walks through it.
- **`tutorial_usv_gazebo`** is the simulation model (the URDF plus the marked
  displacement volume, thrusters, damping and sensors), the bridge
  configuration template, the generator that expands both for a name, and
  the launch files. [Compose the Gazebo model](../how-to/gazebo-composition.md)
  and [Spawn and drive](../how-to/spawn-and-drive.md) walk through them.

## Limitations

- The hydrodynamic damping values are placeholders, not measured.
- It floats on the flat water level: waves are drawn but don't move it yet.
