# Put rain on a camera's lens

A camera on a small boat rides close to the water, and in rain or spray
its lens is rarely clear. The `gz_lens_drops` package puts drops of water
on a camera sensor's images. The drops land at a rate set by the rain, dry
out, slide down when they are big, and merge when they touch. Each drop
shows the scene behind it upside down and blurred, as a real drop on a lens
does. The drops are drawn on the GPU, after the image is rendered, so a
camera in the rain costs about what a dry one does.

It is behaviour, not optics: no light goes through water, and the wind and
the direction the lens points change nothing. It works with a plain
`camera` sensor and the ogre2 render engine only, and it needs a GPU, as
every camera does. Only the sensor's images show the drops; the Gazebo GUI's
view of the world stays clear.

## Add it to a camera

Put the plugin inside the camera's `<sensor>`, next to `<camera>`. With the
camera of [Add sensors](sensors.md):

```xml
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
  <plugin filename="gz-maritime-lens-drops-system"
          name="gz::sim::maritime::LensDrops">
    <rain_rate>10</rain_rate>
    <exposure>1</exposure>
    <focus>0.5</focus>
    <dry_time>20</dry_time>
    <seed>0</seed>
  </plugin>
</sensor>
```

| Element | Default | What |
|---|---|---|
| `<rain_rate>` | 0 | Rain, in mm/h. 20 mm/h lands one drop a second on an open lens. |
| `<exposure>` | 1 | How open the lens is to the rain, from 0, fully sheltered, to 1. |
| `<focus>` | 0.5 | 0 for drops on the lens itself, wide and blurred; 1 for drops on a window some way ahead of it, sharp. |
| `<dry_time>` | 20 | Seconds a drop lasts. |
| `<seed>` | 0 | The same seed gives the same drops on every run; 0 draws a new one. |

The plugin is installed with the workspace, and the workspace's setup
script tells Gazebo where to find it and its shaders.

## Change the rain while it runs

Each camera with the plugin takes changes on its own topic, named after the
sensor:

```bash
gz topic -t /world/default/model/my_boat/link/camera_sensor/sensor/camera/lens_drops/set \
  -m gz.msgs.Param \
  -p 'params { key: "rain_rate" value { type: DOUBLE double_value: 40 } }'
```

The keys are `rain_rate`, `exposure`, `focus`, `dry_time` and `clear`. A
non-zero `clear` wipes the lens, as a wiper would. Bridge the topic to
`ros_gz_interfaces/msg/ParamVec` to change the rain from ROS.

Five times a second of simulation time, the plugin reports the lens on
`.../lens_drops/state`: the values, `count`, the number of drops on the lens,
and `attached`, which turns true once the drops are being drawn. If
`attached` stays false while the camera publishes images, the server's log
says why, under `[LensDrops]`.

## Things to know

- **Simulation time.** The drops are stepped on simulation time: a paused
  world keeps its drops as they are, and a reset wipes the lens.
- **The sensor's noise is under the drops.** The drops are drawn after the
  camera's own `<noise>` and `<distortion>`, so the noise does not cover
  them.
- **At most 64 drops** sit on a lens at once.
- **The behaviour is a guess.** How big the drops are, how fast they slide
  and how many land per mm/h are estimates, not measured on a real camera.
  The [package README](https://github.com/HonuRobotics/gz-maritime/tree/lyrical/gz_lens_drops)
  lists every number.
