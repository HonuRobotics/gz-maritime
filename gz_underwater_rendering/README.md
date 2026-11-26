# gz_underwater_rendering

The world's water, drawn by the renderer: every camera, and the GUI.

[`gz_underwater_camera`](../gz_underwater_camera/README.md) is the
reference: one sensor, on the CPU, exactly tested. This package does the
same per fragment on the GPU, inside Ogre's PBS shaders, so **every** ogre2
camera sees through the world's water ([`gz_water_optics`](../gz_water_optics/README.md)):
plain cameras, RGBD cameras' colour, wide-angle cameras, and the GUI's own
3D view, at full frame rate.

## Use it

Include the `underwater_optics` model in a world that has water optics:

```xml
<plugin filename="gz-maritime-water-optics-system"
        name="gz::sim::maritime::WaterOptics">
  <jerlov>3C</jerlov>
</plugin>
...
<include>
  <uri>model://underwater_optics</uri>
</include>
```

The model is an invisible visual carrying the plugin, so it loads wherever
the world is rendered: in the server's sensors and in every GUI. Cameras
above the water are untouched.

### Quality and speed

The plugin takes a `<quality>`: `low`, `medium` (the default) or `high`.

| | low | medium | high |
|---|---|---|---|
| Steps in each lamp's beam | 8 | 12 | 16 |
| Marine snow drawn out to | 1.5 m | 3 m | 3 m |
| Lamps used (the first in the scene) | 2 | 4 | 4 |

The `underwater_optics` model has no `<quality>`, so it is medium. For
another, put the plugin on a visual of your own instead of including the
model:

```xml
<model name="underwater_optics">
  <static>true</static>
  <link name="link">
    <visual name="underwater_optics">
      <geometry><box><size>0.001 0.001 0.001</size></box></geometry>
      <transparency>1</transparency>
      <cast_shadows>false</cast_shadows>
      <plugin filename="gz-maritime-underwater-rendering-system"
              name="gz::sim::maritime::UnderwaterRendering">
        <quality>low</quality>
      </plugin>
    </visual>
  </link>
</model>
```

What it costs, a 1920x1080 camera at a wreck at medium quality: on an
Intel Iris Xe, nothing measurable for the water and the caustics, about
3 ms a frame for two lamps' beams, 4 to 12 ms for marine snow (more with
the lamps on and the denser it is); on an NVIDIA RTX 4090 Laptop, about
1 ms for everything. Lamps that are off, and snow at 0, cost nothing. At
low quality the snow costs the Intel 1 to 4 ms.

With it, a reference underwater camera (`gz_underwater_camera`) in the same
world applies only its own response: the renderer has already drawn the
water.

To turn it off and on while running, publish a `gz.msgs.Boolean` on
`/world/<world>/underwater_rendering/enable`; to start with it off, set
`<enabled>false</enabled>` in the plugin (default true). Off, or with its
model removed, the renderer draws nothing and the reference cameras draw the
water again.

## What it computes

For each fragment Ogre's PBS shader has shaded, on its final linear
colour, the same model as the reference camera:

```
lit = J exp(−K_d z_object)
I   = lit exp(−β^D w) + B∞(z_camera) (1 − exp(−β^B w))
```

with w the range through the water, up to the surface for a climbing ray.
The fragment's range and world position come from its view-space position;
the camera's pose and the water come in the pass buffer.

## The vehicle's lamps

At depth, at night or in murky water, the vehicle's own lights are the only
light. A spot or point light whose own name (after the model and link
that scope it) contains `lamp`, in any case, is a lamp, up to four, the
first by name; other lights are lit by Ogre as in air, with no beam, so a
harbour light on a pier stays as it is. A lamp:

- **It lights as Ogre would**: the diffuse term of Ogre's default BRDF,
  with Ogre's attenuation, spot cone and range fade, so in air, or seen by a
  camera above the water, a lamp looks as any other light.
- **Its light fades both ways**: from the lamp to what it lights, and from
  there to the camera, `exp(−β^D (d_lamp + r))`.
- **Its beam**: the light the water scatters back to the camera inside the
  lamp's cone, ray-marched along each view ray (single scatter,
  Henyey–Greenstein phase function, g = 0.9). Each lamp is marched only
  over the part of the view ray it lights (inside its range and its cone),
  in 12 steps spaced densely at the near end, over as far as light
  survives; a ray that misses the cone costs nothing.

Lamps are ordinary SDF lights, on the vehicle or in the world:

```xml
<light type="spot" name="lamp">
  <pose>0.3 0.2 0 0 0 0</pose>
  <direction>1 0 -0.1</direction>
  <diffuse>1 0.95 0.85 1</diffuse>
  <intensity>40</intensity>
  <attenuation><range>40</range><quadratic>0.02</quadratic></attenuation>
  <spot><inner_angle>0.5</inner_angle><outer_angle>1.0</outer_angle><falloff>1</falloff></spot>
  <cast_shadows>false</cast_shadows>
</light>
```

Set the world's `<daylight>` to 0 for night: then only the lamps light the
water.

The beams are the costliest part of the shader, and they are marched for
every lamp. A lamp that gives no light (its `<intensity>` times pi times its
brightest diffuse channel under 0.01) is left out, and with every lamp out
the lamp code is not compiled into the shaders at all. So to switch a lamp
off, set its intensity to 0: it then costs nothing.

## Caustics, marine snow and the surface from below

- **Caustics**: on what faces up under waves, a moving pattern of light
  focused by the surface, its contrast fading with depth as `exp(−K_d z)`.
  Its strength is the water's `<caustics>`, off by default, scaled by how
  high the waves are: the significant height at 64 points around the
  cameras, smoothed over 0.5 s, from the water optics sampler's surface, so
  the GUI, whose wave field has no engine of its own, shows them too. None
  on flat water. The pattern's phases are wrapped on the CPU in double, so
  it moves on smoothly however long the simulation runs.
- **Marine snow**: the water's `<marine_snow>` is the fraction of 25 cm
  cells near the camera holding a speck. Specks are fixed in the world
  (hashed per cell, found by walking the cells each view ray crosses, up to
  3 m, as far as the first speck the ray touches), lit by the lamps and
  faintly by the daylight.
- **The surface from below**: `gz_waves_rendering`'s water surface is
  two-sided and, seen from below (the view ray along the surface's up
  normal, so a crest above a camera on a boat is still seen from above),
  shows Snell's window: the sky through a
  cone of 48.6 degrees about the vertical, refracted, and the water's mirror
  outside it. With this package in the world the surface is seen through
  the water like everything else: the mirror shows the water's own colour,
  the sky dims with the `<daylight>`, and far away the surface fades into
  the veil. The surface casts no shadow, so the sun reaches the seabed, and
  the caustics, under it.
- **Open water**: a camera under water that looks at nothing sees the
  water's colour, not the sky or the scene's background. A black box around
  the world (100 km wide, from 100 m above the water to 12 km below) is
  drawn behind everything, at the far plane and after the sky; through the
  water it is the veil. Bounding box, segmentation and thermal cameras do not see it:
  it is hidden while they render. It is drawn after the opaque items, so the pixels
  they cover are not shaded twice. Cameras above the water, depth images, lidars and
  segmentation do not see it.

## How it works

- **Shader pieces** (`share/hlms/Underwater`): the water in the pass
  buffer (`custom_passBuffer_underwater`) and the model at the end of the
  pixel shader (`custom_ps_posExecution_underwater`), switched on per pass
  by the `gz_underwater` property; never in shadow-caster or depth-only
  passes, nor on gz-rendering's solid colours (segmentation keeps its exact
  labels).
- **Unlit shaders** (`share/hlms/UnderwaterUnlit`): a smaller block in the
  unlit pass buffer and the fade and the veil at the end of the unlit pixel
  shader. Those shaders have no world position, so the vertex shader works
  out the ray from the camera from the clip-space position, and they do
  not say whether the image keeps linear light, so the listener does.
  gz-rendering 10 gives the unlit shaders no composite pieces of its own,
  so these are the only ones; a gz-rendering that does is not handled, and
  unlit is then drawn as in air, with a warning. Both listeners derive from
  one that wraps gz-rendering's and forwards what they do not change.
- **Chained, not replaced.** gz-rendering's own composite pieces
  (`Hlms/Gz/Pbs`) are read at run time and copied with one more
  `@insertpiece` each; gz-rendering's Hlms listener is called first and the
  water appended after it.
- **Lamps.** The shader lights with them itself, so they, and only they,
  are hidden from Ogre's own lighting while the water is drawn (and given
  back when it is not). The set is rebuilt each frame from gz-rendering's
  lights, so a lamp removed with its vehicle is simply gone. Turning Ogre's
  Forward+ off instead would change the pass buffer's layout under the
  listener's data; hiding the lamps does not. Hidden for every camera, they
  are lit in the shader for every camera too, the same as Ogre lights them
  in air: the hiding shows only under water.
- **An Ogre bridge** (`libunderwater-ogre2-bridge.so`), loaded with dlopen,
  as `gz_waves_rendering` does: linking gz-rendering's ogre2 library into a
  plugin stops gz-rendering loading its engine. Its C interface and the
  water it takes are in `src/systems/UnderwaterOgre2Bridge.hh`, with a
  version the plugin checks first. It finds the Ogre Next headers with
  `gz_find_ogre_next()`, from `gz_waves_rendering`.
- **The world's mark** names the plugin's entity, so it no longer holds
  once that entity is gone: removing the model leaves no plugin to take it
  off. The plugin, going, tells the bridge to draw nothing; it cannot
  uninstall from outside the render thread, so the bridge stays, idle, until
  a plugin sets the water again.

## Limits

- **What PBS and the unlit shaders shade**, and the water surface (in its
  own shader). Terrain (heightmaps) is drawn as in air.
- **Unlit materials** (a particle emitter's particles; gz-rendering draws
  markers, lines and the grid with PBS) keep their own colour, faded over
  the range, with the veil: no daylight fades them and no lamp lights them.
  The lamps' beams and marine snow in front of them are not drawn.
- **An orthographic camera** (the GUI's orthographic view) is drawn as in
  air, with no backdrop: the model's ranges do not fit it. The lamps, hidden
  from Ogre while the water is drawn, are dark in it.
- **Depth below the wave over the camera.** As in the reference camera,
  the water's surface is a flat level for each camera: the still level
  plus the wave over the camera, which the renderer asks the plugin's water
  optics sampler for in each pass, for any number of cameras and in the GUI
  too (the sampler builds its own wave engine from the recipe). The wave
  over each thing in view is not followed.
- **Lamps**: lights named `lamp` only, up to four (the others are lit by
  Ogre, with a warning); lit by Ogre's diffuse term (no specular), with no
  shadows; their beams show only where something is drawn behind them (the
  open water's background is not shaded). The reference camera has no
  lamps.
- **ogre2 and GLSL only** (the pieces use GLSL's `fract`).
- **Caustics** are a procedural pattern scaled by the wave height, not
  traced from the actual surface; the snow does not drift.
- **ogre2 only, and gz-rendering 10.** The bridge chains its pieces into
  the pieces that define Ogre's hooks, in a copy kept in memory. With
  another render engine, another gz-rendering major version, its own pieces
  missing, or gz-rendering's pieces not as expected, it does not install,
  the world is not marked, and the reference camera draws the water.

## Tests

```bash
colcon test --packages-select gz_underwater_rendering
```

`lamps_test`, rendered: at night, 20 m down, a plate lit by a lamp against
its clear-water colour faded both ways (worst 1 step); the beam along the
centre row against a single-scatter integration in the test (within 2 % of
its peak in IB, 3C and 7C water); clear water carrying the beam farther and
murky water keeping it nearer the lamp; and the same scene with the light
named `torch`, not a lamp: Ogre lights the plate exactly as the lamp does
(to 3 %; it is the same to the 8-bit step), and no beam. It waits for the
water optics to publish each new water, not a fixed time, and skips
without a GPU.

`effects_test`, rendered: caustics' contrast at 8 m against 3 m deep is
`exp(−K_d · 5)` within 6 % (time-averaged), none on flat water; the
surface drawn above the seabed not darkening it (caustics off, to the
rounding); open water the veil; marine snow
specks fixed frame to frame, none once the snow is off, the depth image
unchanged; at night with the lamps off, nothing to see. Under a wave of
0.5 m, a camera 0.15 m above the still water is under water in 34 of 80
frames (two periods; 32 expected) and one 0.15 m below it in 46 (48), and
the higher one never while the lower is in air. A particle emitter's
particles 2 m away, in water that fades and veils: 0.3095 in linear light
for 0.3092 from the formula and their colour in air.

`snell_test`, rendered: Snell's window's half-angle 48.5 degrees (Snell's
law: 48.6), from 5 m below near-flat water; a camera above the still water
in a 1 m sea seeing the crests higher than itself from above.

`effects_test` also: a slow camera beside a fast one, each slow frame
the fast one's of the same instant, under the waves; `<quality>` low and
high close to medium, an unknown
value the medium image; a lidar's scan, a thermal image and a bounding box
camera's boxes the same with the GPU path and without (the backdrop is
hidden from those cameras). The rendered tests skip
without a GPU.

`orthographic_test`, rendered, the bridge driven through its C interface
without Gazebo: a perspective camera sees a box through the water as the
model has it, an orthographic one beside it as in air.

`lamp_edges_test`, rendered, in a process of its own: five lamps, one past
the limit, which Ogre lights as in air, and four that light nothing in view
(one whose range lies wholly behind the camera, three aimed behind it) and
draw no beam; a lamp on a vehicle removed while the world runs, its light
and beam gone, then the world shut down.

`underwater_rendering_test`, rendered (needs a GPU): the same scene without
and with the GPU path, in one process. The GPU image against the reference
camera's: mean difference 0.01 of an 8-bit step, 99th percentile 1 (the
criterion is 1 and 3). Depth images byte for byte the same; a camera above
the water byte for byte the same; segmentation labels byte for byte the
same; the reference camera passing its image through.
`underwater_rendering_fallback_test`, in a process of its own: with the
pieces missing, nothing is installed and the reference camera draws the
water, as without the GPU path. `underwater_rendering_runtime_test`, in a
process of its own: turned off on its topic, then on again, then its model
removed; off and removed look as without the GPU path. All three skip
without a GPU.
