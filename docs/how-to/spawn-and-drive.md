# Spawn and drive

A gz-maritime bring-up is two parts. The **simulation part**,
`kai_bringup/launch/simulation.launch.xml`, starts Gazebo on the ocean, the
GUI, and a bridge for `/clock`. It names no vehicle. The **spawn part** is
yours: one launch file that puts one instance of your vehicle in, with its
bridge and its TF. Include the first once and the second once per vehicle.

```{mermaid}
flowchart TB
  subgraph sim["kai_bringup simulation.launch.xml (once)"]
    S["Gazebo server<br/><i>open_water.sdf</i>"]
    GUI["Gazebo GUI"]
    CB["ros_gz_bridge<br/><i>/clock</i>"]
  end
  subgraph a["spawn.launch.xml name:=boat_a"]
    SA["ros_gz_sim create<br/><i>boat_a</i>"]
    BA["ros_gz_bridge<br/><i>/boat_a/...</i>"]
    RA["robot_state_publisher<br/><i>frame_prefix boat_a/</i>"]
  end
  subgraph b["spawn.launch.xml name:=boat_b"]
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

## Your spawn launch file

Start from `tutorial_usv_gazebo/launch/spawn.launch.xml`. It does four
things for one instance, all from one `name` argument:

```xml
<arg name="name" default="tutorial_usv"/>
<arg name="x" default="0"/> <arg name="y" default="0"/>
<arg name="z" default="0"/> <arg name="yaw" default="0"/>
<arg name="use_composition" default="true"/>

<!-- 1. Generate this instance's model, URDF and bridge config. -->
<let name="vehicle_dir"
     value="$(command '$(find-pkg-prefix tutorial_usv_gazebo)/lib/tutorial_usv_gazebo/configure_vehicle.py --name $(var name) --cache')"/>
<let name="robot_description" value="$(command 'cat $(var vehicle_dir)/tutorial_usv.urdf')"/>

<!-- 2. Spawn the model under the name. -->
<node pkg="ros_gz_sim" exec="create" name="spawn" namespace="$(var name)"
      args="-name $(var name) -file $(var vehicle_dir)/model.sdf
            -x $(var x) -y $(var y) -z $(var z) -Y $(var yaw)"/>

<!-- 3. Bridge this instance's topics, in its namespace. -->
<ros_gz_bridge bridge_name="ros_gz_bridge" namespace="$(var name)"
               config_file="$(var vehicle_dir)/ros_gz_bridge.yaml"
               container_name="ros_gz_container" create_own_container="false"
               use_composition="$(var use_composition)"/>

<!-- 4. Publish its TF, prefixed with the name. -->
<node pkg="robot_state_publisher" exec="robot_state_publisher"
      name="robot_state_publisher" namespace="$(var name)">
  <param name="robot_description" value="$(var robot_description)" type="str"/>
  <param name="frame_prefix" value="$(var name)/" type="str"/>
  <param name="use_sim_time" value="true" type="bool"/>
</node>
```

Replace the package and the file names with yours. With `use_composition`
on, the real file loads the state publisher as a composable node into the
same container as Gazebo and the bridges; the plain node above is what it
does otherwise.

Your `sim.launch.xml` is then two includes, the simulation part and your
spawn part, with the arguments passed through. `tutorial_usv_gazebo`'s is
the template.

### One name per instance

Everything about an instance follows its `name`:

| | `name:=boat_a` gives |
|---|---|
| Gazebo model | `boat_a` |
| Gazebo and ROS topics | `/boat_a/motor_port/thrust`, `/boat_a/imu`, ... |
| ROS namespace | `/boat_a/ros_gz_bridge`, `/boat_a/robot_state_publisher` |
| TF frames | `boat_a/base_link`, `boat_a/imu_link`, ... |
| Sensor `frame_id` | `boat_a/imu_link` |
| RViz | fixed frame `boat_a/base_link`, RobotModel TF Prefix `boat_a`, description on `/boat_a/robot_description` |

The model file carries the name in its topics and frame ids (that is what
`configure_vehicle.py` expands), the launch puts the nodes in the namespace,
`robot_state_publisher`'s `frame_prefix` puts it on the TF frames, and the
generated RViz config looks the robot up under it (`rviz.launch.xml`
starts RViz on that config). The URDF itself never contains it. A name must work as all four at once:
letters, digits and underscores, starting with a letter.

## The bridge configuration

Each entry connects one Gazebo topic to one ROS topic. The instance's
config is rendered from a template with `@name@` where the name goes:

```yaml
- ros_topic_name: /@name@/motor_port/thrust
  gz_topic_name: /@name@/motor_port/thrust
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: gz.msgs.Double
  direction: ROS_TO_GZ
```

The pairs a maritime vehicle usually needs:

| What | ROS type | Gazebo type | Direction |
|---|---|---|---|
| Thrust command | `std_msgs/msg/Float64` | `gz.msgs.Double` | `ROS_TO_GZ` |
| Propeller speed | `std_msgs/msg/Float64` | `gz.msgs.Double` | `GZ_TO_ROS` |
| Joint states | `sensor_msgs/msg/JointState` | `gz.msgs.Model` | `GZ_TO_ROS` |
| IMU | `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | `GZ_TO_ROS` |
| GPS | `sensor_msgs/msg/NavSatFix` | `gz.msgs.NavSat` | `GZ_TO_ROS` |
| Magnetometer | `sensor_msgs/msg/MagneticField` | `gz.msgs.Magnetometer` | `GZ_TO_ROS` |
| Camera | `sensor_msgs/msg/Image` | `gz.msgs.Image` | `GZ_TO_ROS` |

```{important}
No `/clock` in a vehicle's bridge. The simulation part bridges it once;
a second bridge of the same topic would publish it twice.
```

## Drive

```bash
ros2 topic pub -t 5 -r 5 /tutorial_usv/motor_port/thrust std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -t 5 -r 5 /tutorial_usv/motor_stbd/thrust std_msgs/msg/Float64 "{data: 5.0}" &
wait
```

Commands latch until the next one arrives, so start the two commands together
and send `0.0` to stop.

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

Include your spawn launch once per instance, each in its own `<group>` so
the variables one derives from its name don't leak into the next.
`tutorial_usv_gazebo/launch/two_usvs.launch.xml` does it for two boats:

```xml
<include file="$(find-pkg-share kai_bringup)/launch/simulation.launch.xml"/>

<group>
  <include file="$(find-pkg-share tutorial_usv_gazebo)/launch/spawn.launch.xml">
    <arg name="name" value="boat_a"/>
    <arg name="y" value="2"/>
  </include>
</group>
<group>
  <include file="$(find-pkg-share tutorial_usv_gazebo)/launch/spawn.launch.xml">
    <arg name="name" value="boat_b"/>
    <arg name="y" value="-2"/>
  </include>
</group>
```

```bash
ros2 launch tutorial_usv_gazebo two_usvs.launch.xml
```

Both boats float, because each marks its own displacement; each has its own
topics, namespace and TF prefix; and `/clock` has one publisher. A boat can
also be added to a running simulation:

```bash
ros2 launch tutorial_usv_gazebo spawn.launch.xml name:=boat_c y:=-6 use_composition:=false
```

`use_composition:=false` because a separate `ros2 launch` can't load nodes
into the first one's container.

Different vehicles mix the same way: one spawn launch from each vehicle's
package, next to the one simulation part.

Next: [Add sensors](sensors.md).
