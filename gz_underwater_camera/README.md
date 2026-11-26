# gz_underwater_camera

A camera that sees through the world's water.

Gazebo renders a camera under water as if it were in air. This package turns
an RGBD camera's frames into what the camera sees through the water the
world describes ([`gz_water_optics`](../gz_water_optics/README.md)): far
things fade into a veil of the water's colour, red first; everything is lit
by daylight that has faded to its depth; and, if asked, the camera's
exposure, white balance and noise come on top.

It is the reference: the per-pixel model, on the CPU, exactly tested. A GPU
path for every camera and the GUI follows, and must match it.

## Use it

Put the plugin inside an `rgbd_camera` sensor, which renders colour and
depth in one pass, and give the sensor a `<topic>`:

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
          name="gz::sim::maritime::UnderwaterCamera"/>
</sensor>
```

The world needs the water optics system for there to be any water; without
it the image is the plain camera's.

| Topic | Type | What |
|---|---|---|
| `<topic>/underwater/image` | `gz.msgs.Image`, RGB 8 bit | What the camera sees through the water |
| `<topic>/underwater/camera_info` | `gz.msgs.CameraInfo` | Its intrinsics, stamped like the image |
| `<topic>/image`, `<topic>/depth_image`, ... | as the `rgbd_camera` | Unchanged: the plain camera, and the depth a depth sensor sees |

## Parameters

| Element | Default | Meaning |
|---|---|---|
| `<output_topic>` | `<topic>/underwater` | Base of the output topics. |
| `<exposure>` | `1` | A gain on linear light, or `auto`: the mean luminance aimed at 0.18. |
| `<max_gain>` | `16` | With `auto`, the gain stays within 1/max and max. |
| `<white_balance>` | `none` | `none`, or `grey_world`: each channel's mean made the mean luminance, so the water's tint is taken out, as a camera's auto white balance does. |
| `<shot_noise>`, `<read_noise>` | `0` | Gaussian noise on linear light, of variance shot_noise · signal + read_noise². |
| `<seed>` | `1` | The noise's seed. |
| `<forward_scatter>` | `0` | Forward scatter: a Gaussian blur whose width grows with the range through the water, sigma = forward_scatter · range radians. `0.005` to `0.02` looks like coastal water. |
| `<port>` | `none` | The port in front of the lens: `flat` refracts (Snell's law: the image is magnified about 1.33 times at the centre, more at the edges), `dome` (centred on the lens) does not. |
| `<water_index>` | `1.333` | The water's refractive index, for a flat port. |

## What it computes

![How an underwater image forms](../docs/design/images/uw-image-formation.svg)

For each pixel, from the in-air frame's colour J and z-depth:

1. **Range.** The z-depth along the camera's axis becomes the range r along
   the pixel's ray, from the camera's intrinsics (its `camera_info`).
2. **Linear light.** J is decoded from sRGB.
3. **The daylight that lit it.** The object is at depth z_o = z_cam − r ·
   (the ray's climb); its colour is faded by exp(−K_d z_o), the daylight
   left there.
4. **Through the water.** Only the water up to the surface counts, if the
   ray climbs out first: w = min(r, z_cam / climb). Then

   ```
   I = J exp(−K_d z_o) exp(−β^D w) + B∞(z_cam) (1 − exp(−β^B w))
   ```

   with B∞(z_cam) = B∞ exp(−K_d z_cam), the veil at the camera's depth, in
   units of a white surface lit by the surface daylight. A pixel that hits
   nothing sees the open water: the veil, or the sky through the surface.
5. **Forward scatter**, a blur whose width grows with range (recursive
   Gaussian levels, each pixel between the two around its sigma), and the
   **port**: a flat one maps each output radius r to the rendered radius
   f tan(asin(sin(atan(r / f)) / n)).
6. **The camera.** Exposure, white balance, noise; back to sRGB.

With the camera above the water, the image is copied unchanged. Depth comes
from the world's water optics: below the wave field's local surface.

## Speed

The image pipeline runs on the CPU, its rows on up to 16 cores. A
1920x1080 frame on a 13th generation Core laptop: 24 ms for the water
alone, 28 ms with auto exposure and white balance, about 90 ms with noise
(its random numbers are drawn one by one, in order, so a seed gives the same
frame), 55 ms with a flat port, 320 ms with forward scatter. For every
camera at full frame rate, use the GPU path
([`gz_underwater_rendering`](../gz_underwater_rendering/README.md)).

## Limits

- The water's surface between the camera and an object above it is treated
  as clear: Snell's window and the surface seen from below come later.
- The camera's pose is the one of the last simulation step, not of the
  frame's exact render.
- The sensor's own `<noise>` and `<distortion>` apply to its in-air image,
  before the water.
- No vehicle lights, caustics or marine snow: those are drawn by the GPU
  path ([`gz_underwater_rendering`](../gz_underwater_rendering/README.md)).
  The `CameraInfo` stays the in-air pinhole's, also with a flat port.

## Checking it against real footage

Three tools, installed in `lib/gz_underwater_camera` (`ros2 run
gz_underwater_camera <tool>`):

- `make_chart_world.py` writes a world that reproduces a colour chart
  photographed under water (a 24-patch ColorChecker at a given range and
  depth, in a given Jerlov type or visibility, facing this camera), and the
  patches' pixel boxes in its image.
- `chart_validation.py` compares the real and the simulated images patch by
  patch: the RGB angular error of each patch's mean colour in linear light,
  the metric of OceanSim (arXiv:2503.01074), which ignores brightness, and
  the luminance ratio per patch, which does not. Given one render per water,
  each with its own boxes, it ranks the waters closest first; it fails only
  with `--threshold`, or when more than `--max-dark` patches are too dark.
- `save_frame.py` saves one frame of a camera topic as a PNG.

```bash
ros2 run gz_underwater_camera make_chart_world.py --out chart.sdf --range 2 --depth 5 --jerlov 3C
gz sim -s -r chart.sdf &
ros2 run gz_underwater_camera save_frame.py --topic /chart/camera/underwater/image --out sim.png
ros2 run gz_underwater_camera chart_validation.py --real dive.png --real-patches dive.json \
  --sim 3C=sim.png --sim-patches chart.json --report report.png
```

![An example report](../docs/design/images/uw-validation-example.png)

The example above is **not a validation**: its "real" image is another
simulation, in 5C water against 3C (13.4 degrees).
Real footage is still needed.

## Tests

```bash
colcon test --packages-select gz_underwater_camera
```

- `underwater_image_test`: the ray table against its formula; sRGB round
  trip for all 256 values, and its lookup against the formula; the model, every pixel against the closed form,
  at two ranges and with the object's depth; β^D ≠ β^B against the fog
  model; a climbing ray clipped at the surface; nothing hit is the veil; in
  air and in clear water the image unchanged; fixed and auto exposure; grey
  world neutralising a tint; the noise's mean and variance; 640×480 within a
  15 Hz frame, and a parallel frame the same every time; forward scatter's edge width at 2, 4 and 8 m within 10 % of
  the Gaussian's; a flat port's points within half a pixel of Snell's law,
  a dome port unchanged; every parameter documented.
- `test_chart_validation`: the metric (zero for the same colour, 90 for
  orthogonal ones, brightness not counted), patch means away from the
  edges, a closed loop on synthetic charts (the same water near zero, an
  exposure difference near zero, another water far), the nearest water
  ranked first, each image with its own boxes, dark patches counted and the
  JSON free of NaN, the scale from the data type, boxes too small or outside,
  the luminance ratio telling exposure from water, the command line, a
  camera image saved, and the chart world (valid SDF, patches inside the
  image, in order; the water type checked; `--visibility`).
- `underwater_camera_test`, rendered (needs a GPU): the published image
  against the model computed in the test from the in-air frame, pixel by
  pixel; the far wall fading into the veil more than the near plate; above
  the water, byte for byte the plain image; the depth image untouched.
