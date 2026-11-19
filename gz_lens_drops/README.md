# gz_lens_drops

Water drops on a camera's lens, as behaviour, not optics. Drops land at a
rate set by the rain, dry out, slide down when they are big and merge when
they touch. A render pass on the camera draws each one as a small lens that
shows the scene behind it upside down and blurred. There is no light
transport through water, no film of water, no trails behind a sliding
drop, and nothing set by the wind or by where the lens points.

What it works with:

- **Cameras:** only a plain `<sensor type="camera">`; not wide-angle,
  depth, RGBD, thermal or segmentation cameras.
- **Rendering:** the ogre2 engine with OpenGL only (no Vulkan or Metal).
- **Who sees the drops:** only the sensor's images. The GUI's view of the
  world is not affected.
- **Pass order:** the drops are drawn after the sensor's own noise and
  distortion passes, so the noise is not on top of the drops.

| Library | What |
|---|---|
| `gz_lens_drops` | The drop model, with no Gazebo or rendering dependency. |
| `gz-maritime-lens-drops-system` | The system, inside the camera sensor. |
| `lens-drops-ogre2-bridge` | The render pass, loaded by the system with `dlopen`. It is the only library that links `gz-rendering-ogre2`. |

## On a camera

```xml
<sensor name="camera" type="camera">
  <topic>my_boat/camera/image</topic>
  <update_rate>15</update_rate>
  <camera>...</camera>
  <plugin filename="gz-maritime-lens-drops-system"
          name="gz::sim::maritime::LensDrops">
    <rain_rate>10</rain_rate>   <!-- mm/h; default 0 -->
    <exposure>1</exposure>      <!-- 0 fully sheltered to 1 open; default 1 -->
    <focus>0.5</focus>          <!-- 0 blurred, on the lens, to 1 sharp, on a port ahead of it; default 0.5 -->
    <dry_time>20</dry_time>     <!-- s a drop lasts; default 20 -->
    <seed>0</seed>              <!-- 0 draws one -->
  </plugin>
</sensor>
```

Values out of range are clamped:

| Value | Range |
|---|---|
| rain rate | 0 to 1000 mm/h |
| exposure, focus | 0 to 1 |
| drying time | 0.1 to 3600 s |

A seed gives the same drops on every run. Two cameras with one seed get
different drops: the seed is mixed with the camera's name.

The environment hook sets `GZ_LENS_DROPS_SHADER_PATH` to the installed
shaders; set it yourself to use other ones.

## Topics

With `<sensor>` the sensor's scoped name, for example
`/world/default/model/boat/link/camera_link/sensor/camera`:

- **`<sensor>/lens_drops/set`**, a `gz.msgs.Param`:
  - `rain_rate`, `exposure`, `focus` and `dry_time`, each a number (a
    double, an integer or a boolean);
  - `clear`: non-zero wipes the lens.

  An unknown key, or a value that is not a finite number, is skipped with a
  warning; the rest of the message still applies.
- **`<sensor>/lens_drops/state`**, a `gz.msgs.Param`, every 0.2 s of
  simulation time:
  - the four values above;
  - `count`: the drops on the lens;
  - `attached`: true once the render pass is on the camera.

```bash
gz topic -t /world/default/model/boat/link/camera_link/sensor/camera/lens_drops/set \
  -m gz.msgs.Param \
  -p 'params { key: "rain_rate" value { type: DOUBLE double_value: 25 } }'
```

## How the drops behave

The numbers are in `include/gz/sim/lens_drops/Drops.hh`.

- **Landing:**
  - Drops land as a Poisson process at 0.05 drops a second per mm/h, scaled
    by the exposure: 20 mm/h of rain is one drop a second on an open lens.
  - Each drop lands at a uniformly random place, with a radius from 1.5 % to
    6 % of the image height, small ones more often.
- **Drying:** a drop shrinks as it dries and is gone after `dry_time`.
- **Sliding:** drops of 5 % of the image height or more slide down, at
  4 % of the image height a second and faster when bigger, and leave at the
  bottom.
- **Merging:**
  - Two drops whose centres are closer than the bigger radius merge into
    the bigger one.
  - The merged drop keeps the water of both: the areas add.
  - It is no bigger than 12 % of the image height, and it takes the age of
    the younger drop.
- **Capacity:** the lens holds at most 64 drops; more arrivals are counted
  but not kept.
- **Time:**
  - The drops are stepped on simulation time, so a paused world keeps its
    drops as they are.
  - A world reset wipes the lens and starts the seed again.

## Tests

```bash
colcon test --packages-select gz_lens_drops && colcon test-result --verbose
```

- **`lens_drops_core_test`** checks the model without a world: the rate
  against the rain, the limit of 64, drying, sliding, merging, a paused
  step, a seed repeating, clearing and the packing for the shader.
- **`lens_drops_test`** runs the system in a world that does not render:
  - a dry lens stays dry and rain leaves drops;
  - the set topic starts the rain, wipes the lens and skips bad values;
  - a seeded run repeats;
  - the system does nothing on a sensor that is not a camera.
- **`lens_drops_camera_test`** renders a camera. A dry lens must leave the
  image unchanged, rain must change it, and a wipe must give the dry image
  back. It needs a GPU and skips without one.
