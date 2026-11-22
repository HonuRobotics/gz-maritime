# Sonar performance

A simulated sonar is one of the more expensive sensors in a world. Consequently, an active sonar should be in a scene only when the scenario needs it. Each ping

1. renders the scene from the sonar,
2. reads the images back from the GPU, and
3. turns tens of thousands of hits into beams and samples on the CPU.

In this document we report one performance evaluation as a reference point. The numbers are anecdotal: each was measured once, on one machine. The results are a guide to where the time goes and how it scales. They are meant to help you predict how a world with sonars will run on your machine, with or without a GPU, and to choose where and how to fit them. 

## The test machine and scenes

Test machine 

- A workstation with 24 CPU cores 
-  NVIDIA RTX 2070 SUPER
- running inside [drydock](https://github.com/HonuRobotics/drydock)'s `maritime` container
- the world headless
- "llvmpipe (CPU only)" is the same machine rendering with Mesa's llvmpipe, a software renderer on the CPU.

Scenes: 
- Small scene: the imaging sonar world (`sonar_imaging.sdf`), a few targets on a flat seabed, with the BlueROV2 and one forward imaging sonar (the Sonoptix ECHO, 256 beams, 768 samples to 15 m, labels on, default sampling), seen by two 604 × 429 cameras.
- Large scene: 2.1 million triangles, used to time the rendering and sampling alone. It is not a shipped world: a test program built it from a procedural seabed and the `sydney_regatta` harbor mesh from [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/sydney_regatta).
- USV scene: the survey world (`sonar_survey.sdf`), the BlueBoat with two sonar sensors: a sidescan (one sensor per side) and a profiler.

## Results

| Scene | Compute | Sonar | RTF |
|---|---|---|---|
| Small | GPU | none | 0.96 |
| Small | llvmpipe | none | 0.95 |
| Small | GPU | imaging, 10 Hz (the default) | 0.81 |
| Small | llvmpipe | imaging, 10 Hz | 0.81 |
| Small | GPU | imaging, 15 Hz | 0.75 |
| Small | llvmpipe | imaging, 15 Hz | 0.67 |
| USV | GPU | sidescan and profiler, 10 Hz | 0.84 |
| Large | GPU | imaging, drawing only | not measured |
| Large | llvmpipe | imaging, drawing only | not measured; estimated well below 0.5 at 10 Hz |

RTF is the real time factor: simulated time over wall-clock time, so 1.0 is real time and 0.81 means a simulated second takes about 1.2 s. Compute is what draws the sonar's images: the GPU, or llvmpipe, Mesa's software renderer on the CPU.


## Where the time goes

A ping passes through five pieces of software, on three threads. Only the rendering uses the GPU.

- Gazebo:
    - gz-sim: steps physics and the world's systems.
    - The Sensors system, a gz-sim system the world loads: owns the rendering thread.% CLAUDE: Is theis gz-sim's Sensors system?   Maybe a link to the Sensors documentation
    - gz-rendering, the rendering engine (Ogre 2): draws images, on the GPU if there is one, otherwise on the CPU (llvmpipe).
- Sonar plugin
    - `gz-maritime-sonar-system`, the sonar plugin, a gz-sim system: places each sonar's cameras, triggers their rendering, and runs each sonar's worker thread.% CLAUDE: How many worker threads are typical and what do thay do?
    - `gz_sonar`, the sonar model, a library the plugin calls on the worker.

A ping, step by step: % CLAUDE: augment this with a flow chart

| Step | Software | Thread | Runs on |
|---|---|---|---|
| Step the world; wait for rendering when a sensor is due | gz-sim | simulation | CPU |
| Bring the scene up to date | Sensors system | rendering | CPU |
| Render the sonar's cameras when a ping is due | sonar plugin | rendering | CPU |
| Draw depth and segmentation, copy them to CPU memory | gz-rendering | rendering | GPU, or CPU (llvmpipe) |
| Hand the images to the worker; at most two wait | sonar plugin | rendering | CPU |
| Turn the images into hits | gz_sonar | worker | CPU, 1 core |
| Beams and samples: the engine | gz_sonar | worker | CPU, all cores |
| Levels, gain, the published message | gz_sonar | worker | CPU, 1 core |

Only the drawing uses the GPU; without a GPU, llvmpipe does it on the CPU, on the same rendering thread.

The segmentation image is always drawn: its label tells each hit which surface it struck, and so which acoustic properties to use. The `labels` option only decides whether those ids are also published as ground truth.

The steps fall on two threads, and the two cost the world differently.

1. The rendering thread. The Sensors system brings the scene up to date, the plugin asks gz-rendering to draw the sonar's cameras, and the images are read back to the CPU. gz-sim waits for this whenever a sensor is due, so it slows the world in proportion to how often the sonar pings, whether or not the rest keeps up.
2. The sonar's worker thread, on the CPU only. This runs alongside the simulation, so it costs the world nothing as long as each ping is finished before the next ones pile up. The plugin lets two rendered pings wait for the worker; if a third is ready before the worker has taken one, the rendering thread stops until it has, and the world waits with it.

One imaging sonar, small scene, per ping:

```{figure} images/sonar-ping-time.svg
:alt: Horizontal bars of one imaging ping's time. With a GPU: drawing and copying back 13 to 15 ms on the rendering thread, then on the worker images to hits 18 ms, the engine 25 ms, levels and message 11 ms, 68 ms in all. With llvmpipe: drawing 18 to 23 ms, the worker's stages the same, 74 ms in all

One imaging sonar, small scene, per ping. Only the first segment, on the rendering thread, holds up the world; the worker's stages run alongside it.
```

Two limits follow.

- The world slows by roughly the ping rate times the render time. At 10 Hz, 10 × 15 ms is 15 % of every second on the rendering thread, which is the drop from 0.96 to 0.81.
- The worker takes about 54 ms of CPU time a ping, so one sonar keeps up to about 18 Hz. The engine is the largest part of it and runs on every core: on a machine with fewer cores, expect the worker, and so the highest rate the sonar keeps up with, to scale down roughly in proportion.

## With a GPU or without one % CLAUDE: FRom here to the rest of the document, revise to be much more succinct and to the point.   I believe the take away is that based on the workflow, the GPU dependency is modest.   The GPU on speads up rendering thread, which is only one of four significant steps.   All the other three must be done on the CPU.

- The GPU only draws the images. Everything after that runs on the CPU and takes the same time either way: more CPU cores help it, a faster GPU does not.
- In a small scene llvmpipe is nearly as fast as a GPU (18 to 23 ms against 13 to 15 ms per ping): drawing costs little geometry.
- In a detailed scene it is not. On llvmpipe the drawing grows with the triangles in view, and in a 2.1-million-triangle scene an imaging sonar's drawing and sampling take 178 ms a ping against 11 ms on a GPU: plan on about 5 pings a second, and a world well below real time at 10 Hz.
- The output is the same either way, to within 4 mm in range.

To try llvmpipe in drydock, start `gz sim` with `__GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1`; the EGL settings alone leave it on the GPU. Ogre's log, `~/.gz/rendering/ogre2.log`, names the renderer it used.

## The cost of the options

- Labels cost little: the segmentation they come from is drawn anyway, so turning them on adds only a labels message per ping, the size of the image.
- The sampling sets the number of hits: `rays_per_beam` first, then `elevation_rays` and `elevation_span` (the advanced parameters in the design's [SDF](sonar.md#sdf) section). Fewer rays mean less work on both threads, at a cost in fidelity to check on the scene at hand.
- The range window and the number of samples set the engine's and the readout's work, and the message's size.
- The rate multiplies everything.

