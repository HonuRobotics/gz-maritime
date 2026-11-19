# gz_lens_drops

Water drops on a camera's lens. The drops behave like water on glass, and
the camera sees through them as through small lenses. There is no fluid
simulation and no ray tracing.

**How the drops behave.** A simple physical model, stepped on simulation
time:

- **Landing.** Rain lands where the rain moves into the lens. A camera
  looking ahead in calm rain catches few drops; speed, a headwind or a
  lens tilted up bring many.
- **Sliding.** A drop stays pinned until gravity along the lens and the
  apparent wind overcome a threshold that grows with its size.
- **While sliding.** It slides down the world, not down the image, so it
  follows the camera's roll and pitch. It meanders, leaves beads and a wet
  trail, and merges with the drops in its way, keeping its water.
- **Drying.** Drops and trails dry.

**How they look.** Each frame the drops are drawn into a map of the height
of water on the lens. A render pass on the camera then refracts the image
through that water by Snell's law and the camera's field of view:

- each drop shows a wide, upside-down, smaller view of the scene;
- light reflected inside the water goes back into the camera's dark housing,
  which darkens the rims; total internal reflection is black;
- drops on the lens are out of focus as a whole;
- wet trails blur and haze the image.

What it works with:

- **Cameras:** only a plain `<sensor type="camera">`; not wide-angle,
  depth, RGBD, thermal or segmentation cameras.
- **Rendering:** the ogre2 engine with OpenGL only (no Vulkan or Metal).
- **Who sees the drops:** only the sensor's images; the GUI's view of the
  world is not affected.
- **Pass order:** the drops are drawn after the sensor's own noise and
  distortion passes.

| Library | What |
|---|---|
| `gz_lens_drops` | The drop model (`Drops`) and the drop map (`DropMap`), with no Gazebo or rendering dependency. |
| `gz-maritime-lens-drops-system` | The system, inside the camera sensor. |
| `lens-drops-ogre2-bridge` | The render pass and its texture, loaded by the system with `dlopen`. It is the only library that links `gz-rendering-ogre2`. |

## On a camera

```xml
<sensor name="camera" type="camera">
  <topic>my_boat/camera/image</topic>
  <update_rate>15</update_rate>
  <camera>...</camera>
  <plugin filename="gz-maritime-lens-drops-system"
          name="gz::sim::maritime::LensDrops">
    <rain_rate>10</rain_rate>
    <exposure>1</exposure>
    <focus>0.5</focus>
    <dry_time>20</dry_time>
    <contact_angle>0.96</contact_angle>
    <slide_radius>0.03</slide_radius>
    <map_scale>0.5</map_scale>
    <wind>true</wind>
    <mask>false</mask>
    <seed>0</seed>
  </plugin>
</sensor>
```

| Element | Default | Range | What |
|---|---|---|---|
| `<rain_rate>` | 0 | 0 to 1000 mm/h | Rain. |
| `<exposure>` | 1 | 0 to 1 | How open the lens is to the rain: less behind a hood. |
| `<focus>` | 0.5 | 0 to 1 | 0 for drops on the lens itself, out of focus; 1 for drops on a window some way ahead, sharp. |
| `<dry_time>` | 20 | 0.1 to 3600 s | How long a drop lasts; a trail lasts half that. |
| `<contact_angle>` | 0.96 | 0.17 to 2.6 rad | How water sits on the lens: 0.96 (55 degrees) is plain glass; a hydrophobic coating is higher. |
| `<slide_radius>` | 0.03 | 0.005 to 0.2 | Radius, in image heights, of a drop that gravity alone along the lens just moves. Smaller on a hydrophobic coating. |
| `<map_scale>` | 0.5 | 0.125 to 1 | Size of the drop map over the image's size. |
| `<wind>` | true | | Take the world's wind (`gz_wind`). |
| `<mask>` | false | | Publish the drop mask. |
| `<seed>` | 0 | | The same seed gives the same drops; 0 draws one. Two cameras with one seed differ: the seed is mixed with the camera's name. |

The environment hook sets `GZ_LENS_DROPS_SHADER_PATH` to the installed
shaders; set it yourself to use other ones.

## Topics

With `<sensor>` the sensor's scoped name, for example
`/world/default/model/boat/link/camera_link/sensor/camera`:

- **`<sensor>/lens_drops/set`**, a `gz.msgs.Param`:
  - `rain_rate`, `exposure`, `focus`, `dry_time`, `contact_angle` and
    `slide_radius`, each a number (a double, an integer or a boolean);
  - `clear`: non-zero wipes the lens, trails included;
  - `add_x`, `add_y` and `add_radius`, together, put one still drop on the
    lens: x and y from the image's top left, as fractions of its width and
    height, the radius in image heights.

  An unknown key, or a value that is not a finite number, is skipped with a
  warning; the rest of the message still applies.
- **`<sensor>/lens_drops/state`**, a `gz.msgs.Param`, every 0.2 s of
  simulation time:
  - the parameters above;
  - `count`, drops on the lens, and `sliding`, how many of them slide;
  - `covered`, the share of the drop map with water;
  - `flux`, the rain flux factor: 1 is a lens facing up in still air;
  - `gravity_x` and `gravity_y`, gravity along the lens, in g, to the
    image's right and down it;
  - `attached`, true once the render pass is on the camera.
- **`<sensor>/lens_drops/mask`**, with `<mask>true</mask>` only: a
  `gz.msgs.Image`, `L_INT8`, at the drop map's size. It is 255 where a drop
  is and 0 elsewhere, stamped with the simulation time of the drops it
  shows. It is ground truth for raindrop detection.

```bash
gz topic -t /world/default/model/boat/link/camera_link/sensor/camera/lens_drops/set \
  -m gz.msgs.Param \
  -p 'params { key: "rain_rate" value { type: DOUBLE double_value: 25 } }'
```

## How the drops behave

The numbers are in `include/gz/sim/lens_drops/Drops.hh` and `DropMap.hh`.

### Landing

- Drops land as a Poisson process at `0.4 * rain_rate * exposure * (flux + 0.1)`
  drops a second; the 0.1 stands for splashes off the camera's housing.
- **The flux** is the speed at which the rain moves into the lens, over
  its fall speed of 6.5 m/s. The rain falls and drifts with the wind,
  relative to the camera's own motion.
- A lens facing up in still air gets a flux of 1. A level camera standing
  in calm rain gets 0, so only the splashes reach it.
- **Size.** A new drop's radius runs from 0.2 % to 5 % of the image height,
  most of them small: mist. Its rim wobbles a little.

### Sliding

- **The push.** It is gravity along the lens times the radius squared over
  the slide radius squared, plus the apparent air along the lens. The air
  term is `(speed / 15 m/s)^2` times the radius over the slide radius.
- **The threshold.** Over 1 the drop slides, at `0.06 * sqrt(push - 1)`
  image heights a second, and turns a little at random (up to 0.6 rad per
  second).
- **Beads and trails.** About every 3 % of the image height it slid, a
  sliding drop leaves a bead behind its tail, of 15 % to 30 % of its
  radius, and lays a wet trail.

### Merging and drying

- **Merging.** Two drops whose centres are closer than the bigger radius
  merge. They keep their water: volumes, radius cubed, add. The merged
  drop is no bigger than 12 % of the image height.
- **Drying.** A drop shrinks as it dries and is gone after `dry_time`; a
  bead dries with the drop it left; a trail fades in half the time.
- **Capacity.** The lens holds at most 2048 drops.
- **Time.** A paused world keeps its drops. A world reset wipes the lens
  and starts the seed again.

### The camera's view

- **The drop map.** A drop's height is a spherical cap meeting the lens at
  the contact angle. The map is blurred by up to 4 texels of a 360 texel
  high map at focus 0.
- **Refraction.** The shader takes the water's normal from the map. It
  refracts the camera ray out of the water (refractive index 1.333) and
  projects it with the camera's horizontal field of view. Fresnel's share
  is lost into the dark housing.
- **Cost.** A full lens of 2048 unmerged drops takes about 9 ms on the CPU
  to draw at 640x360, on the render thread. A lens in rain merges into a
  few hundred drops, which takes far less.

## Limitations

- **Image edges.** Near the image's edge a drop refracts directions outside
  the frame, which were never rendered. They are darkened a little rather
  than invented.
- **Optics left out.** No dispersion, no reflection of the outside on the
  drop's outer surface, and no light scattered inside the drop.
- **Focus** is a perceptual value, not an aperture and a focus distance.
- **Rain** is uniform: no gusts of rain and no spray from the waves.
- **The numbers are estimates.** Rates, sizes, speeds and blur are not
  calibrated against real cameras.

## Tests

```bash
colcon test --packages-select gz_lens_drops && colcon test-result --verbose
```

- **`lens_drops_core_test`** checks the drop model without a world:
  - the rate against the rain and the flux;
  - the limit of drops;
  - pinning, and sliding by gravity and by air;
  - beads and trails, merging keeping the water, and drying;
  - a paused step, a seed repeating, clearing.
- **`drop_map_test`** checks the drop map: the cap's profile, a drop's area
  and peak, wobbly rims, the blur keeping the water, sliding drops
  stretched, trails, the mask, and the time for a full lens.
- **`lens_drops_test`** runs the system in worlds that do not render:
  - the topics, a seeded run, a sensor that is not a camera;
  - gravity and rain flux for a level camera, a rolled one and one looking
    up;
  - rain brought by the wind.
- **`lens_drops_camera_test`** renders a camera; it needs a GPU and skips
  without one:
  - drops change the image and a wipe gives the dry image back;
  - a drop centred on the horizon shows the sea above and the sky below,
    the reverse of the scene beside it, and the mask has its area.
