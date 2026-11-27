# Put rain on a camera's lens

A camera on a small boat rides close to the water, and in rain or spray
its lens is rarely clear. The `gz_lens_drops` package puts drops of water
on a camera sensor's images. Each drop is a small lens: it shows a wide,
upside-down view of the scene, darkens at its rim and blurs, as a real drop
does.

The drops also behave like water on glass:

- They land where the rain moves into the lens, so a forward camera
  catches more at speed or into the wind.
- They stay put until they are big enough to slide, then slide down the
  world, following the boat's roll and pitch.
- They merge, and leave beads and wet trails behind them.
- They dry from the top, keeping their outline, as water on glass does.

It works with a plain `camera` sensor and the ogre2 render engine, and it
needs a GPU, as every camera does. Only the sensor's images show the drops;
the Gazebo GUI's view stays clear.

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
    <focus>0.5</focus>
  </plugin>
</sensor>
```

| Element | Default | What |
|---|---|---|
| `<rain_rate>` | 0 | Rain, in mm/h: 2 is light rain, 10 moderate, 50 a downpour. |
| `<exposure>` | 1 | How open the lens is to the rain, from 0 behind a deep hood to 1. |
| `<focus>` | 0.5 | 0 for drops on the lens itself, out of focus; 1 for drops on a window some way ahead, sharp. |
| `<dry_time>` | 20 | Seconds a drop lasts. |
| `<contact_angle>` | 0.96 | How water sits on the lens, in radians, up to pi/2; raise it for a water-repellent coating, which beads water. |
| `<slide_radius>` | 0.03 | How big a drop must be, in image heights, before gravity alone moves it; lower it for a water-repellent coating, which sheds water sooner. |
| `<wind>` | true | Use the world's wind for the rain's drift and the push on the drops. |
| `<mask>` | false | Publish a mask of where the drops are. |
| `<enabled>` | true | Draw the drops; false keeps them but leaves the image clear. |
| `<seed>` | 1 | The same seed gives the same drops on every run; 0 draws a new one and logs it. |

The package README lists one more element, `<map_scale>`, and the ranges
of every value.

The plugin is installed with the workspace, and the workspace's setup
script tells Gazebo where to find it.

## How much rain lands

Rain falls at about 6.5 m/s. What reaches the lens is the part of that
motion, with the wind's drift and the boat's own motion, that goes into
the lens:

- **A camera facing up** gets all of it.
- **A level camera on a still boat in calm air** gets almost none; a
  little comes from splashes off its housing.
- **The same camera on a boat at 5 m/s** gets about three quarters as
  much as one facing up, and more into a headwind.

So the same `<rain_rate>` gives a dry lens in the harbour and a wet one at
speed. The state topic below reports this as `flux`.

## Change the rain while it runs

Each camera with the plugin takes changes on its own topic, named after the
sensor:

```bash
gz topic -t /world/default/model/my_boat/link/camera_sensor/sensor/camera/lens_drops/set \
  -m gz.msgs.Param \
  -p 'params { key: "rain_rate" value { type: DOUBLE double_value: 40 } }'
```

- **Keys:** `enabled`, `rain_rate`, `exposure`, `focus`, `dry_time`,
  `contact_angle`, `slide_radius` and `clear`. A non-zero `clear` wipes the lens, as a wiper
  would.
- **One drop:** `add_x`, `add_y` and `add_radius` together put one drop on
  the lens, which is handy for testing a perception pipeline.
- **From ROS:** bridge the topic to `ros_gz_interfaces/msg/ParamVec`.

Five times a second of simulation time, the plugin reports the lens on
`.../lens_drops/state`:

- the values;
- `count` and `sliding`, the drops on the lens and how many slide;
- `covered`, the share of the image with water;
- `flux` and the gravity along the lens;
- `draws`, how many times the drops were drawn;
- `attached`, which turns true once the render pass is on the camera (a
  shader that fails is in the log), and `drawing`, whether it drew the
  drops in the last frame.

If `attached` stays false while the camera publishes images, the server's
log says why, under `[LensDrops]`.

## Ground truth for perception

With `<mask>true</mask>`, the plugin also publishes `.../lens_drops/mask`.
It is a grey image at the drop map's size (half the camera's resolution
with the default `<map_scale>`), 255 where a drop is,
published with each camera frame while someone subscribes and stamped as
that frame. Use it to train or
score a raindrop detector against the camera's images.

## Things to know

- **Simulation time.** The drops are stepped on simulation time: a paused
  world keeps its drops, and a reset wipes the lens.
- **Under the drops.** The camera's own `<noise>` and `<distortion>` are
  drawn first, so the drops sit on top of them.
- **Cameras.** Only `type="camera"`: an RGBD, depth, wide-angle or
  thermal camera is refused with an error.
- **Cost.** The drops are drawn only for frames the camera renders, and
  only when they changed; a dry lens costs nothing.
- **Image edges.** A drop near the edge would show parts of the scene
  outside the image, which the camera never rendered; they come out a
  little dark.
- **The behaviour is an estimate.** How many drops land per mm/h, their
  sizes, how fast they slide and how blurred they look are not measured on
  a real camera. The
  [package README](https://github.com/HonuRobotics/gz-maritime/tree/lyrical/gz_lens_drops)
  lists every number.
