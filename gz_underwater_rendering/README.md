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

- **It lights as Ogre would**: Ogre's default BRDF, diffuse and specular
  (metal included), with Ogre's attenuation, spot cone and range fade, so in
  air, or seen by a camera above the water, a lamp looks as any other light,
  but for its shadows: a lamp casts none while the renderer draws the water,
  in air too.
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

## How it works

- **Shader pieces** (`share/hlms/Underwater`): the water in the pass
  buffer (`custom_passBuffer_underwater`) and the model at the end of the
  pixel shader (`custom_ps_posExecution_underwater`), switched on per pass
  by the `gz_underwater` property; never in shadow-caster or depth-only
  passes, nor on gz-rendering's solid colours (segmentation keeps its exact
  labels).
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

- **Only what PBS shades.** The sky (a skybox), the background colour, the
  water surface (its own shader), unlit materials (markers) and terrain are
  drawn as in air. Seen from under water, a camera looking up or out at
  nothing still sees the sky; the reference camera turns those pixels into
  the veil. Planned: the water surface seen from below (§4.5 of the plan),
  and the background through a compositor listener.
- **Depth from the still water level**, not the wave surface, unlike the
  reference camera.
- **Lamps**: lights named `lamp` only, up to four (the others are lit by
  Ogre, with a warning); lit by Ogre's default BRDF without clear coat, and
  with no shadows, in air too; their beams show only where something is
  drawn behind them (the open water's background is not shaded). The
  reference camera has no lamps. A light that is not a lamp is lit by Ogre
  and, under water, fades with the daylight: by its depth, not over its own
  distance, so a torch 20 m down is much dimmer than a lamp there.
- **ogre2 only, and gz-rendering 10.** The bridge chains its pieces into
  the pieces that define Ogre's hooks, in a copy kept in memory. With
  another render engine, its own pieces missing, a bridge of another
  version, or gz-rendering's pieces not as expected (a hook defined twice,
  or defined by code it cannot extend), it does not install, the world is
  not marked, and the reference camera draws the water. Another
  gz-rendering major version does not build.

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

`lamp_edges_test`, rendered, in a process of its own: five lamps, one past
the limit, which Ogre lights as in air, and four that light nothing in view
(one whose range lies wholly behind the camera, three aimed behind it) and
draw no beam; a lamp on a vehicle removed while the world runs, its light
and beam gone, then the world shut down; the renderer's model removed, and
the lamp given back to Ogre, which lights the plate with it.

`lamp_air_test`, rendered, in a process of its own: a camera above the
water and a lamp lighting a metal plate and a glossy one, with and without
the GPU path: the same image, to the 8-bit step, specular included.

`underwater_rendering_test`, rendered (needs a GPU): the same scene without
and with the GPU path, in one process. The GPU image against the reference
camera's: mean difference 0.01 of an 8-bit step, 99th percentile 1 (the
criterion is 1 and 3). Depth images byte for byte the same; a camera above
the water byte for byte the same; segmentation labels byte for byte the
same; the reference camera passing its image through.
`underwater_rendering_fallback_test` and `underwater_rendering_version_test`,
each in a process of its own: with the pieces missing, or a bridge of
another version found first, nothing is installed and the reference camera
draws the water, as without the GPU path.
`underwater_rendering_runtime_test`, in a process of its own: `<enabled>`
false from the start; turned off on its topic, then on again; its model
removed; the next world after that removal, installed again in its own
Ogre; the model removed and added again four times, drawn once. Off and
removed look as without the GPU path. These skip without a GPU; each takes
a transport partition with its process id and steps the world itself.
Without a GPU: `bridge_pieces_test`, the bridge's reading of the pieces
and each case it refuses, and `world_mark_test`, which plugin instance the
world's mark names.
