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

| Scenario | Real time factor | Pings |
|---|---|---|
| Small scene, no sonar | 0.96 | |
| Small scene, one imaging sonar at 10 Hz (the default), GPU | 0.81 | every ping, on time |
| The same, llvmpipe (CPU only) | 0.81 | every ping, on time |
| Small scene, one imaging sonar at 15 Hz, GPU | 0.75 | every ping, on time |
| The same, llvmpipe (CPU only) | 0.67 | every ping, on time |
| USV scene, sidescan and profiler at 10 Hz, GPU | 0.84 | every ping, on time |
| Large scene, imaging sonar, llvmpipe (CPU only) | well below 1 at 10 Hz | rendering alone allows about 5 pings a second |


## Where the time goes

A ping passes through five pieces of software, on three threads. Only the rendering uses the GPU.

- gz-sim, the simulator: steps physics and the world's systems.
- The Sensors system, a gz-sim system the world loads: owns the rendering thread.
- gz-rendering, the rendering engine (Ogre 2): draws images, on the GPU if there is one, otherwise on the CPU (llvmpipe).
- `gz-maritime-sonar-system`, the sonar plugin, a gz-sim system: places each sonar's cameras, triggers their rendering, and runs each sonar's worker thread.
- `gz_sonar`, the sonar model, a library the plugin calls on the worker.

A ping, step by step:

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

So a ping has two halves. % CLAUDE: Confusing with the four steps above vice two halves ?

1. Render, on the rendering thread. The Sensors system brings the scene up to date, the plugin asks gz-rendering to draw the sonar's cameras, and the images are read back to the CPU. gz-sim waits for this whenever a sensor is due, so it slows the world in proportion to how often the sonar pings, whether or not the rest keeps up.
2. Compute the sonar model, on the plugin's worker thread, on the CPU only. This runs alongside the simulation, so it costs the world nothing as long as each ping is finished before the next ones pile up. The plugin lets two rendered pings wait for the worker; if a third is ready before the worker has taken one, the rendering thread stops until it has, and the world waits with it.

One imaging sonar, small scene, per ping:

| Stage | With a GPU | llvmpipe | % CLAUDE: show this as a horizontal bar to show the time spans visually
|---|---|---|
| Draw and copy back, 2 cameras | 13 to 15 ms | 18 to 23 ms |
| Images to hits, about 100,000 | 18 ms | 18 ms |
| Engine, 24 cores | 25 ms | 25 ms |
| Levels and message | 11 ms | 11 ms |

Two limits follow.

- The world slows by roughly the ping rate times the render time. At 10 Hz, 10 × 15 ms is 15 % of every second on the rendering thread, which is the drop from 0.96 to 0.81.
- The worker takes about 54 ms of CPU time a ping, so one sonar keeps up to about 18 Hz. The engine is the largest part of it and runs on every core: on a machine with fewer cores, expect the worker, and so the highest rate the sonar keeps up with, to scale down roughly in proportion.

## With a GPU or without one % CLAUDE: tldr this section.   summarize succinctly. 

The GPU does only the rendering. The sampling, the engine and the readout run on the CPU either way and take the same time, so a faster GPU cannot speed them up, and more or faster CPU cores can. Without a GPU, the CPU also does the rendering, on the same thread that holds up the simulation.

How much the GPU matters depends on the scene:

- In a small scene the render is dominated by fixed costs per camera, not by geometry, so llvmpipe is close to the GPU: 18 to 23 ms against 13 to 15 ms, and the same real time factor at 10 Hz.
- On llvmpipe the render cost grows with the triangles in view; on a GPU it hardly does. In the large scene, rendering depth and segmentation and sampling them into hits take:

| Sonar, per ping | With a GPU | Without (llvmpipe) |
|---|---|---|
| Imaging (1024 × 64 rays) | 10.9 ms | 178 ms |
| Sidescan, one side | 1.8 ms | 94 ms |
| Profiler | 2.1 ms | 95 ms |

So without a GPU, plan on about 5 imaging pings a second in a detailed scene, and expect a 10 Hz sonar to run the world well below real time. With a GPU the render is a small, nearly constant cost, and the worker sets the limit.

The output is the same either way: the GPU and llvmpipe hit the same surfaces, with ranges differing by up to 4 mm.

To try llvmpipe in drydock, start `gz sim` with `__GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1`; it renders through GLX on the forwarded display, so the EGL settings alone leave it on the GPU. Ogre's log, `~/.gz/rendering/ogre2.log`, names the renderer it used.

## The cost of the options

- Labels cost little: the segmentation they come from is drawn anyway, so turning them on adds only a labels message per ping, the size of the image.
- The sampling sets the number of hits: `rays_per_beam` first, then `elevation_rays` and `elevation_span` (the advanced parameters in the design's [SDF](sonar.md#sdf) section). Fewer rays mean less work on both threads, at a cost in fidelity to check on the scene at hand.
- The range window and the number of samples set the engine's and the readout's work, and the message's size.
- The rate multiplies everything.

