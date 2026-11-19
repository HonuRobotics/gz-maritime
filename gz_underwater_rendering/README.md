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
world passes its image through: the renderer has already drawn the water.

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
- **An Ogre bridge** (`libunderwater-ogre2-bridge.so`), loaded with dlopen,
  as `gz_waves_rendering` does: linking gz-rendering's ogre2 library into a
  plugin stops gz-rendering loading its engine.

## Limits

- **Only what PBS shades.** The sky (a skybox), the background colour, the
  water surface (its own shader), unlit materials (markers) and terrain are
  drawn as in air. Seen from under water, a camera looking up or out at
  nothing still sees the sky; the reference camera turns those pixels into
  the veil. Planned: the water surface seen from below (§4.5 of the plan),
  and the background through a compositor listener.
- **Depth from the still water level**, not the wave surface, unlike the
  reference camera.
- **ogre2 only.**

## Tests

```bash
colcon test --packages-select gz_underwater_rendering
```

`underwater_rendering_test`, rendered (needs a GPU): the same scene without
and with the GPU path, in one process. The GPU image against the reference
camera's: mean difference 0.01 of an 8-bit step, 99th percentile 1 (the
criterion is 1 and 3). Depth images byte for byte the same; a camera above
the water byte for byte the same; segmentation labels byte for byte the
same; the reference camera passing its image through.
