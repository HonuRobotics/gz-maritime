# Add sensors

A sensor in Gazebo has two halves. A **world system** does the simulating, and
your **model** says where the sensor is and what to publish. The world
already has its half; this page is about yours.

## What the world provides

| World system in `open_water.sdf` | Simulates |
|---|---|
| `gz-sim-imu-system` | `imu` sensors |
| `gz-sim-magnetometer-system` | `magnetometer` sensors |
| `gz-sim-navsat-system` | `navsat` (GPS) sensors |
| `gz-sim-sensors-system` | Rendered sensors: `camera`, `depth_camera`, `gpu_lidar` and so on. These need a GPU. |

The world also sets `<spherical_coordinates>`: where on Earth the origin is
(Portuguese Ledge, Monterey Bay). The GPS reports positions relative to that
spot, and the magnetometer computes Earth's magnetic field there.

## The pattern

Every sensor follows the same four steps:

1. **A frame in the URDF** where the sensor sits, such as `imu_link`
   ([Sensor frames](vehicle-description.md#sensor-frames)).
2. **A tiny link in the Gazebo model**, placed at that frame and fixed to
   `base_link`, holding the `<sensor>`
   ([Sensors](gazebo-composition.md#sensors)).
3. **`<frame_id>` set to `${name}/` plus the URDF frame**, so the messages
   name the frame `robot_state_publisher` publishes for this instance.
4. **A bridge entry** for its topic
   ([The bridge configuration](spawn-and-drive.md#the-bridge-configuration)).

The IMU is shown in [Compose the Gazebo model](gazebo-composition.md#sensors).
The other two sensors on the custom USV differ only in their `<sensor>`
element:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:sensor_link sensor_name="magnetometer"
:end-at: </xacro:sensor_link>
```

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:sensor_link sensor_name="navsat"
:end-at: </xacro:sensor_link>
```

## A camera

The custom USV has no camera, but adding one follows the same pattern. With
a `camera_link` frame in your URDF:

```xml
<link name="camera_sensor">
  <pose relative_to="camera_link">0 0 0 0 0 0</pose>
  <inertial>
    <mass>1e-5</mass>
    <inertia><ixx>1e-8</ixx><iyy>1e-8</iyy><izz>1e-8</izz></inertia>
  </inertial>
  <sensor name="camera" type="camera">
    <frame_id>${name}/camera_link</frame_id>
    <topic>${name}/camera/image</topic>
    <update_rate>15</update_rate>
    <always_on>true</always_on>
    <camera>
      <horizontal_fov>1.2</horizontal_fov>
      <image><width>640</width><height>480</height></image>
      <clip><near>0.05</near><far>200</far></clip>
    </camera>
  </sensor>
</link>
<joint name="camera_sensor_joint" type="fixed">
  <parent>base_link</parent>
  <child>camera_sensor</child>
</joint>
```

Bridge it as `sensor_msgs/msg/Image` to `gz.msgs.Image`, from Gazebo to ROS.

```{warning}
`open_water.sdf` lights its scene with a cyan ambient light
(`<scene><ambient>0.0 1.0 1.0</ambient>`) for its deep-ocean look. Camera
sensors render with that light, so objects look green and blue in their
images. The Gazebo GUI uses its own light and isn't affected. If colours
matter to you, use a copy of the world with a neutral ambient light, such as
`0.4 0.4 0.4` ([Use your own world](own-world.md)).
```

## Check a sensor

```bash
ros2 topic echo /custom_usv/imu --once
```

Look for `frame_id: custom_usv/imu_link` in the header. If the command
waits forever, check the bridge entry and that the world runs the matching
system.

Next: [Use your own world](own-world.md).
