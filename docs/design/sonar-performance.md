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

A ping passes through four pieces of software, on three threads. Only the rendering uses the GPU.

| Piece | What it is | What it does for a ping | Thread | Runs on |
|---|---|---|---|---|
| gz-sim | The simulator | Steps physics and the world's systems; waits for the rendering thread whenever a sensor is due | simulation | CPU |
| Sensors system | A gz-sim system the world loads | Owns the rendering thread: brings the scene up to date with the simulation, then signals the plugins that may render | rendering | CPU |
| gz-rendering (Ogre 2) | The rendering engine | Draws each of the sonar's cameras, depth and then segmentation, and reads both images back from GPU memory into CPU memory | rendering | GPU if there is one, otherwise the CPU (llvmpipe) |
| gz-maritime-sonar-system | The sonar plugin, a gz-sim system | Places the sonar's cameras, asks gz-rendering to draw them when a ping is due, and hands the images to the sonar's worker; then, on the worker, runs the sonar model | rendering, then the sonar's own worker | CPU |
| gz_sonar engine | A library the plugin calls on the worker | Turns the images into hits, the hits into intensities per beam and sample, and the intensities into the published levels | worker | CPU: the engine step on all cores, the rest on one |

The segmentation image is always drawn: its label tells each hit which surface it struck, and so which acoustic properties to use. The `labels` option only decides whether those ids are also published as ground truth.

So a ping has two halves.

1. Render, on the rendering thread. The Sensors system brings the scene up to date, the plugin asks gz-rendering to draw the sonar's cameras, and the images are read back to the CPU. gz-sim waits for this whenever a sensor is due, so it slows the world in proportion to how often the sonar pings, whether or not the rest keeps up.
2. Compute the sonar model, on the plugin's worker thread, on the CPU only. This runs alongside the simulation, so it costs the world nothing as long as each ping is finished before the next ones pile up. The plugin lets two rendered pings wait for the worker; if a third is ready before the worker has taken one, the rendering thread stops until it has, and the world waits with it.

One imaging sonar, small scene, per ping:

| Stage | Piece | Runs on | With a GPU | Without (llvmpipe) |
|---|---|---|---|---|
| Render 2 cameras × (depth + segmentation) and read back | gz-rendering, asked by the plugin | GPU (CPU with llvmpipe) | 13 to 15 ms | 18 to 23 ms |
| Sample the images into hits (about 100,000) | gz_sonar, on the worker | CPU, one core | 18 ms | 18 ms |
| Engine: beams and samples | gz_sonar, on the worker | CPU, all 24 cores | 25 ms | 25 ms |
| Readout: levels, gain, the 8-bit frame | gz_sonar, on the worker | CPU, one core | 11 ms | 11 ms |

Two limits follow.

- The world slows by roughly the ping rate times the render time. At 10 Hz, 10 × 15 ms is 15 % of every second on the rendering thread, which is the drop from 0.96 to 0.81.
- The worker takes about 54 ms of CPU time a ping, so one sonar keeps up to about 18 Hz. The engine is the largest part of it and runs on every core: on a machine with fewer cores, expect the worker, and so the highest rate the sonar keeps up with, to scale down roughly in proportion.

## With a GPU or without one

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

## A world without an active sonar

The sonar system costs nothing until a sonar exists. Attaching to the rendering thread makes gz-sim render every physics step, so the system attaches only when the first sonar appears and lets go when the last one is removed. On a light scene with a camera at 30 Hz:

| World | Real time factor |
|---|---|
| The camera, no sonar system | 4.15 |
| The camera and the sonar system, no sonar | 4.11 |
| The camera and one sonar at 10 Hz | 3.02 |

Each sonar's first ping also takes 30 to 140 ms once, to create its textures and shaders.

## Fitting sonars deliberately

- Fit a sonar only where the scenario uses it. Every vehicle world loads the sonar system, which is free until a sonar is fitted, so the choice is in the vehicle's config: the BlueROV2's and BlueBoat's `*_sonar.yaml` configs carry sonars, their default configs do not.
- Budget one active sonar per scenario; that is what the defaults assume. Several sonars on several vehicles in one world have not been measured.
- Keep the default 10 Hz unless the task needs more. Without a GPU, or in a detailed scene, lower it.
- If the world runs too slowly, lower `rays_per_beam` before the rate.

## Not measured

- Several vehicles with sonars in one world.
- Large competition worlds, such as `robotx_2026` with its 159 models, with a sonar.
- Machines with fewer cores, or a smaller GPU.
