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
the system scales the water type to it, keeping its colour:

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
water is not applied twice. The water surface seen from below fades into
the water too, and open water with nothing in view is the water's colour.

## Lights at depth

At night, or deep, there is no daylight: set the world's water to
`<daylight>0</daylight>` and give the vehicle spot lights. With the
`underwater_optics` model in the world, every spot and point light is a lamp
whose light fades through the water both ways, and whose beam the water
scatters back to the camera:

```xml
<light type="spot" name="lamp_port">
  <pose>0.3 0.2 0 0 0 0</pose>
  <direction>1 0.1 -0.1</direction>
  <intensity>40</intensity>
  <attenuation><range>40</range><quadratic>0.02</quadratic></attenuation>
  <spot><inner_angle>0.5</inner_angle><outer_angle>1.0</outer_angle><falloff>1</falloff></spot>
  <cast_shadows>false</cast_shadows>
</light>
```

Turn a lamp off or move it while running with the world's `light_config`
service, like any light. Lamps cast no shadows here. A lamp set to an
intensity of 0 costs nothing.

## Check it against real footage

To know how close the simulation is to your water, photograph a colour chart
under water, and simulate the same shot:

1. **Dive with a chart.** A ColorChecker (24 patches) facing the camera at a
   measured range (1 to 4 m), at a measured depth, in daylight; note the
   water's visibility (or measure its Jerlov type). Shoot a few frames at
   each range, with the camera's white balance and exposure fixed.
2. **Simulate the shot.** `make_chart_world.py --out chart.sdf --range 2
   --depth 5 --jerlov 3C --width 1280 --height 720 --hfov 1.2`, with your
   camera's resolution and field of view, then run it and save one frame of
   `chart/camera/underwater/image`.
3. **Mark the patches** in your photo: a `chart.json` of the 24 boxes, in
   the same order (the simulated one shows the format).
4. **Compare.** `chart_validation.py --real dive.png --sim sim.png
   --patches chart.json --report report.png`. Under 10 degrees on average is
   the target; OceanSim reports 8 to 14 for its scenes.

Adjust the water (`<jerlov>`, `<visibility>`, or explicit coefficients)
until the error is smallest: those are your water's optics.

## More of the water

- **Forward scatter** blurs what is far: `<forward_scatter>0.01</forward_scatter>`
  in the camera plugin.
- **The camera's port**: `<port>flat</port>` refracts like a flat port
  (a narrower, magnified view); `<port>dome</port>` changes nothing.
- **Caustics** dance on a shallow seabed under waves: on by default with the
  `underwater_optics` model (`<caustics>` in the water optics, 0 to turn
  them off). None on flat water.
- **Marine snow**: `<marine_snow>0.2</marine_snow>` in the water optics;
  specks show in the lamps' light.
- **The surface from below** shows Snell's window: the sky through a
  48.6 degree cone overhead, a mirror of the water outside it. The surface
  casts no shadow: the sun, and the caustics, reach the seabed under it
  whether or not the sun casts shadows. The plan is in
[UNDERWATER_CAMERA_PLAN.md](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/UNDERWATER_CAMERA_PLAN.md).
