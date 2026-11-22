# Sonar performance

A simulated sonar is one of the more expensive sensors in a world. Each ping

1. renders the scene from the sonar,
2. reads the images back from the GPU, and
3. turns tens of thousands of hits into beams and samples on the CPU.

Consequently, an active sonar should be in a scene only when the scenario needs it.

In this document we report one performance evaluation as a reference point. The numbers are anecdotal: each was measured once, on one machine. The results are a guide to where the time goes and how it scales, not specifications. They are meant to help you predict how a world with sonars will run on your machine, with or without a GPU, and to choose where and how to fit them. The model itself is in the [sonar simulation design](sonar.md).

## The test machine and scenes

- A workstation with 24 CPU cores and an NVIDIA RTX 2070 SUPER, inside drydock, the world headless. "llvmpipe (CPU only)" is the same machine rendering with Mesa's llvmpipe, a software renderer on the CPU.
- Small scene: the imaging sonar world (`sonar_imaging.sdf`), a few targets on a flat seabed, with the BlueROV2 and one forward imaging sonar (the Sonoptix ECHO, 256 beams, 768 samples to 15 m, labels on, default sampling), seen by two 604 × 429 cameras.
- Large scene: 2.1 million triangles (a procedural seabed and a harbor mesh), used to time the rendering and sampling alone.
- USV scene: the survey world (`sonar_survey.sdf`), the BlueBoat with a sidescan (one sensor per side) and a profiler.

## What to expect

| Scenario | Real time factor | Pings |
|---|---|---|
| Small scene, no sonar | 0.96 | |
| Small scene, one imaging sonar at 10 Hz (the default), GPU | 0.81 | every ping, on time |
| The same, llvmpipe (CPU only) | 0.81 | every ping, on time |
| Small scene, one imaging sonar at 15 Hz, GPU | 0.75 | every ping, on time |
| The same, llvmpipe (CPU only) | 0.67 | every ping, on time |
| USV scene, sidescan and profiler at 10 Hz, GPU | 0.84 | every ping, on time |
| Large scene, imaging sonar, llvmpipe (CPU only) | well below 1 at 10 Hz | rendering alone allows 4 to 8 pings a second |

A real time factor of 1.0 is real time. The sonar never drops a ping: when the machine cannot keep up, the simulation slows down instead, so the output depends only on the seed and the scene, never on the machine. A slow machine gives the same data, more slowly.

## Where the time goes

A ping has two halves, on two threads.

1. On gz-sim's rendering thread, each of the sonar's cameras renders depth and then segmentation (the truth labels), and both are read back to the CPU. gz-sim's simulation thread waits for rendering whenever a sensor is due, so this half is what slows the world.
2. On the sonar's own worker thread, the images become hits, the hits become intensities per beam and sample (the engine), and the intensities become the readout. This runs alongside the simulation and costs it nothing as long as it keeps up; a queue two pings deep sits between the two halves, and when it is full the rendering thread is blocked by the worker.

One imaging sonar, small scene, per ping:

| Stage | Thread | GPU | llvmpipe (CPU only) |
|---|---|---|---|
| Render and read back, 2 cameras × (depth + segmentation) | rendering | 13 to 15 ms | 18 to 23 ms |
| Sample the images into hits (about 100,000) | worker | 18 ms | 18 ms |
| Engine: beams and samples, on 24 cores | worker | 25 ms | 25 ms |
| Readout: levels, gain, the 8-bit frame | worker | 11 ms | 11 ms |

Two limits follow.

- The world slows by roughly the ping rate times the render time. At 10 Hz, 10 × 15 ms is 15 % of every second on the rendering thread, which is the drop from 0.96 to 0.81.
- The worker takes about 54 ms a ping, so one sonar keeps up to about 18 Hz. The engine is the largest part of it and runs on every core: on a machine with fewer cores, expect the worker, and so the highest rate the sonar keeps up with, to scale down roughly in proportion.

## With a GPU or without one

The GPU affects only the first half, the rendering. The worker's half runs on the CPU either way and takes the same time.

How much the GPU matters depends on the scene:

- In a small scene the render is dominated by fixed costs per camera, not by geometry, so llvmpipe is close to the GPU: 18 to 23 ms against 13 to 15 ms, and the same real time factor at 10 Hz.
- On llvmpipe the render cost grows with the triangles in view; on a GPU it hardly does. In the large scene, rendering and sampling alone take:

| Sonar, per ping | GPU, labels | GPU, no labels | llvmpipe, labels | llvmpipe, no labels |
|---|---|---|---|---|
| Imaging (1024 × 64 rays) | 10.9 ms | 7.6 ms | 178 ms | 129 ms |
| Sidescan, one side | 1.8 ms | 0.9 ms | 94 ms | 44 ms |
| Profiler | 2.1 ms | 1.1 ms | 95 ms | 45 ms |

So without a GPU, plan on 4 to 8 imaging pings a second in a detailed scene, and expect a 10 Hz sonar to run the world well below real time. With a GPU the render is a small, nearly constant cost, and the worker sets the limit.

The output is the same either way: the GPU and llvmpipe hit the same surfaces, with ranges differing by up to 4 mm.

To try llvmpipe in drydock, start `gz sim` with `__GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1`; it renders through GLX on the forwarded display, so the EGL settings alone leave it on the GPU. Ogre's log, `~/.gz/rendering/ogre2.log`, names the renderer it used.

## The cost of the options

- Labels are a second render of every camera: about 40 % more render time for the imaging sonar, and about double for the sidescan and profiler, on either renderer.
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
- Turn labels on only when recording ground truth.
- If the world runs too slowly, lower `rays_per_beam` before the rate.

## Not measured

- Several vehicles with sonars in one world.
- Large competition worlds, such as `robotx_2026` with its 159 models, with a sonar.
- Machines with fewer cores, or a smaller GPU.
