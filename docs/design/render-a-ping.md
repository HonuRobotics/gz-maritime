# Render a ping without Gazebo

For people working on the sonar engine itself. To use a sonar on a vehicle, start with [First look: imaging sonar from the ROV](../how-to/sonar-first-look.md).

The sonar engine in `gz_sonar` runs on its own, with a small scene of analytic shapes standing in for Gazebo's renderer, so each stage of the model can be seen and tested without a simulation. This page renders an imaging ping, a sidescan pass and a profiling pass and draws them. The model itself is described in the [sonar simulation design](sonar.md#theory-of-operation).

## Why it is here

In Gazebo, a sonar image is the product of two things: the scene geometry that gz-rendering's cameras sample, and the acoustic model that turns that geometry into echoes. When an image looks wrong, the first question is which of the two is at fault. Here the geometry is exact: the synthetic scene intersects each ray with its spheres, planes and boxes analytically, so the range, incidence angle and surface of every hit are known, and whatever the image shows is the acoustic model's doing. That makes this the place to reproduce a suspicious image, to try a change to the model before it meets a renderer, and to compare against other sonar simulators on the same scenes.

## What it tests

Each stage of the model, against what physics and geometry predict:

- Geometry: a target appears at the range and bearing it is placed at, and the seabed's first return at the altitude the sonar is at.
- The sonar equation: echoes fall with range as spreading and absorption predict, scale with a surface's backscatter (Lambert's law and the specular glint) and with the area the beam covers, and vanish behind an object (shadows).
- The beams: a small target spreads across neighboring beams as the beam pattern says, sidelobes included, and targets at the same range but different elevations land in the same place (the imaging sonar's elevation ambiguity).
- Speckle and noise: their statistics, and a fixed seed giving the same ping every time, on any number of threads.
- The readouts: the imaging sonar's 8-bit frame, the sidescan's line in dB with time-varied gain a consumer can take out again, and the profiling sonar's soundings.

The gtests check these numerically; the pictures below let a person see them.

Run everything inside drydock, from `~/maritime_ws`.

## Build and test the engine

```bash
colcon build --merge-install --packages-select gz_sonar
colcon test --merge-install --packages-select gz_sonar
colcon test-result --verbose
```

The gtests are the analytic cases of the design's testing plan: a sphere at its range and bearing, 40 log R spreading plus absorption, Lambert's law and the dependence on area, a shadow behind a box, the beam pattern across beams, elevation ambiguity, speckle statistics, the same seed giving the same ping, and the three readouts.

## Render and draw

```bash
source install/setup.bash
mkdir -p /tmp/sonar
for s in A B C; do ros2 run gz_sonar gz_sonar_render_ping imaging $s /tmp/sonar; done
ros2 run gz_sonar gz_sonar_render_ping sidescan /tmp/sonar
ros2 run gz_sonar gz_sonar_render_ping profiling /tmp/sonar
ros2 run gz_sonar draw_ping.py /tmp/sonar
```

`gz_sonar_render_ping` writes NumPy arrays and the axes that go with them; `draw_ping.py` turns them into PNGs in the same directory. The scenes:

| Scene | What | Sonar |
|---|---|---|
| A | A sphere, 0.25 m radius, 5 m ahead in open water | Imaging, the ECHO at 700 kHz with a 10 m window, level |
| B | A 1 × 1 × 0.5 m box on a flat seabed, 5 m ahead | Imaging, 2 m up, pitched down 15° |
| C | A seabed rising 10° to starboard | Imaging, 2 m up, pitched down 15° |
| Pass | A box and a glass float on a flat seabed | Sidescan and profiling, 3 m up, 250 pings 0.1 m apart along a straight track |

## What to look for

```{figure} images/gz_sonar_B.png
:alt: Imaging sonar fan of scene B: bright seabed from about 4.7 m, the box as a bright patch near 5 m with a dark shadow behind it

Scene B. The seabed starts where the lowest edge of the 20° beam meets it, the box returns brightly from its near face and top, and the seabed behind it is in shadow. The dimmer band closer in is seabed seen by the vertical beam's taper, below its −3 dB edge.
```

```{figure} images/gz_sonar_A.png
:alt: Imaging sonar fan of scene A: a bright spot at 4.75 m on the axis and a faint arc at the same range across the fan

Scene A. The sphere's near face, at 4.75 m on the axis. The faint arc at the same range across the fan is the beam pattern's sidelobes: every beam hears a little of a strong target.
```

```{figure} images/gz_sonar_sidescan.png
:alt: Sidescan waterfall: no returns before about 3 m, then seabed; the box as a bright band with a black shadow, the float as a small shadow

Sidescan. Nothing returns before the seabed (the nadir gap, 3 m at this altitude); the box returns brightly and casts a shadow; the float casts a small one. Automatic gain is smoothed across pings, so the box dims the lines around it only a little.
```

```{figure} images/gz_sonar_profiling.png
:alt: Profiling soundings seen from above, colored by height: a flat seabed with the box standing out half a meter higher

Profiling. Soundings from the pass, georeferenced with the true poses: the seabed, the box standing half a meter proud, and the float.
```

## Where things live

| File | What |
|---|---|
| `include/gz/maritime/sonar/Water.hh` | Francois–Garrison absorption |
| `include/gz/maritime/sonar/Config.hh` | Parameters, backscatter, and the ECHO, Omniscan and Surveyor presets |
| `include/gz/maritime/sonar/Rays.hh` | The fan of rays and what each ray hit |
| `include/gz/maritime/sonar/Engine.hh` | The incoherent beam model: hits to intensity per beam and sample |
| `include/gz/maritime/sonar/Readout.hh` | Levels, the 8-bit imaging frame, sidescan lines and profiling soundings |
| `include/gz/maritime/sonar/SyntheticScene.hh` | The analytic scene and its ray caster |
| `examples/render_ping.cc`, `examples/draw_ping.py` | This page |
