# See through the water

Under water, a camera sees the water as much as what is in it: far objects
fade into a veil of the water's colour, red goes first, and everything gets
darker the deeper you go. Give the world its water, then give a vehicle an
underwater camera.

## Give a world its water

Add the water optics system to the world and name the kind of water, by its
Jerlov type:

```xml
<plugin filename="gz-maritime-water-optics-system"
        name="gz::sim::maritime::WaterOptics">
  <jerlov>3C</jerlov>
</plugin>
```

The open ocean types run from `I`, the clearest, through `IA`, `IB`, `II` and
`III`; the coastal ones from `1C` to `9C`, the murkiest. Coastal water is
greener and murkier than ocean water. Without the system, the world has
clear water.

| If your water is | Try | Visibility |
|---|---|---|
| Tropical open ocean, very clear | `IB` | about 46 m |
| Open ocean | `II` | about 11 m |
| A clear coast | `1C` | about 10 m |
| A typical harbour or coast | `3C` | about 4 m |
| A murky harbour, a river mouth | `7C` | about 1.7 m |

```{figure} ../design/images/uw-jerlov-presets.svg
Every type: how fast each colour fades, the colour of the veil, and how far
you can see.
```

## Set how far you can see

Divers and pilots describe water by its visibility. Give it in metres and
the system scales the water type to it. Clearer water turns bluer, as
real water does, and none is clearer than pure water, about 100 m:

```xml
<plugin filename="gz-maritime-water-optics-system"
        name="gz::sim::maritime::WaterOptics">
  <jerlov>3C</jerlov>
  <visibility>8</visibility>
</plugin>
```

This is coastal, green water, but clearer than usual: a black target
disappears at about 8 m.

## Change it while the world runs

```bash
gz topic -t /world/default/water_optics/set -m gz.msgs.Param \
  -p 'params {key: "visibility" value {type: DOUBLE double_value: 3}}'
```

Any element of the plugin is a key: `jerlov` takes a string, `visibility`
and `water_level` a number, the per-channel values a string `"r g b"`.

## Check what the world has

```bash
gz topic -e -t /world/default/water_optics_info -n 1
```

prints the water type, the visibility, and per channel the numbers a camera
uses. The [reference](../reference/launch-and-world.md#water-optics-parameters)
lists every element and the presets.

## Add an underwater camera

Use an `rgbd_camera` sensor, which renders colour and depth together, give
it a `<topic>`, and put the underwater camera plugin inside it:

```xml
<sensor name="camera" type="rgbd_camera">
  <topic>rov/camera</topic>
  <update_rate>15</update_rate>
  <camera>
    <horizontal_fov>1.2</horizontal_fov>
    <image><width>640</width><height>480</height></image>
    <clip><near>0.1</near><far>60</far></clip>
    <depth_camera><clip><near>0.1</near><far>60</far></clip></depth_camera>
  </camera>
  <plugin filename="gz-maritime-underwater-camera-system"
          name="gz::sim::maritime::UnderwaterCamera">
    <exposure>auto</exposure>
  </plugin>
</sensor>
```

The far clip is where the camera stops seeing anything but water: past it,
every pixel is the veil. Keep it beyond the water's visibility.

## See its image

The camera publishes `rov/camera/underwater/image` beside the plain
`rov/camera/image`. In the GUI, add an Image Display and pick the topic; in
ROS, bridge it like any camera:

```yaml
- gz_topic_name: rov/camera/underwater/image
  ros_topic_name: rov/camera/underwater/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
- gz_topic_name: rov/camera/underwater/camera_info
  ros_topic_name: rov/camera/underwater/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
```

The demo world `kai_gazebo/worlds/underwater_camera_demo.sdf` has colour
charts at 0.5 to 16 m in front of a camera, to see each water type:

```bash
gz sim underwater_camera_demo.sdf
```

```{figure} ../design/images/uw-expected-chart.png
What the chart must look like from 3 m deep, by range, in three waters.
```

## The GUI view, and every camera at once

To see through the water in the GUI too, and in every camera without a
plugin on each, include the `underwater_optics` model:

```xml
<include>
  <uri>model://underwater_optics</uri>
</include>
```

The renderer then draws the water into every surface, on the GPU. An
underwater camera plugin in the same world passes its image through, so the
water is not applied twice. The sky and the water surface are not drawn
through the water yet: point the GUI at a seabed or a structure.

## What comes next

The vehicle's own lights. The plan is in
[UNDERWATER_CAMERA_PLAN.md](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/UNDERWATER_CAMERA_PLAN.md).
