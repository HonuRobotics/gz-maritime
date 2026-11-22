# Sonar performance: what it costs and where the time goes

A simulated sonar is one of the more expensive sensors in a world. Every ping renders the scene from the sonar, reads the images back from the GPU, and turns tens of thousands of hits into beams and samples on the CPU. That is worth being deliberate about: put an active sonar in a scene only when the scenario needs it, and configure it no richer than the task needs.

The numbers here are anecdotal: each was measured once, on one machine, while the sonar was being built. Read them as a guide to where the time goes and how it scales, not as specifications. The model itself is in the [sonar simulation design](sonar.md).

## The machine and the scenes

One workstation with 24 cores and an NVIDIA RTX 2070 SUPER, inside drydock, the world headless unless noted. CPU-only runs use Mesa's llvmpipe software renderer on the same machine.

- Phase 1: the standalone sampler prototype, on a 2.1-million-triangle scene (a procedural seabed plus the `sydney_regatta` mesh).
- Phase 2: the imaging sonar world (`sonar_imaging.sdf`), the BlueROV2 with one forward imaging sonar at the walkthrough's settings (the ECHO with 768 samples to 15 m, labels on), sampled by two 604 × 429 cameras. The rays then reached two half-widths of the vertical beam; the default is now three (see below).
- Phase 3: the survey world (`sonar_survey.sdf`), the BlueBoat with the sidescan (two sensors) and the profiler.

## Where a ping spends its time

A ping has two halves, on two threads.

1. On gz-sim's rendering thread, each of the sonar's cameras renders depth and then segmentation (the labels), and both are read back to the CPU. gz-sim's simulation thread waits for rendering whenever a sensor is due, so this half is what slows the world.
2. On the sonar's own worker thread, the images become hits, the hits become intensities per beam and sample (the engine), and the intensities become the readout. This runs alongside the simulation and costs it nothing as long as it keeps up; a queue two pings deep sits between the two halves, and when it is full the rendering thread waits for the worker.

Imaging sonar, per ping, Phase 2:

| Stage | Thread | GPU | llvmpipe |
|---|---|---|---|
| Render and read back, 2 cameras × (depth + segmentation) | rendering | 13 to 15 ms | 18 to 23 ms |
| Sample the images into hits (68,864) | worker | 12 ms | 12 ms |
| Engine: beams and samples, on 24 threads | worker | 24 ms | 24 ms |
| Readout: levels, gain, the 8-bit frame | worker | 11 ms | 11 ms |

Widening the rays to the current default (`elevation_span` 3) added about 50 % more hits and about 7 ms a ping on the worker (sampling 18 ms, engine 25 ms), with no measurable change to the real time factor. The worker's total is then about 54 ms, so it keeps up to about 18 Hz.

The bottlenecks, in order:

- With a GPU, the engine is the slowest stage, but it is on the worker, so it limits the ping rate (about 18 Hz) rather than slowing the world. Before Phase 2 the engine ran on one thread and took 30 to 100 ms an imaging ping; spreading it across the cores is what brought it to 24 ms.
- The render and readback is what the world pays for, every ping, on the rendering thread.
- Without a GPU, the render grows with the triangles in view and becomes the bottleneck for everything (below).

## What the world pays: real time factor

Real time factor of the imaging world (1.0 is real time), Phase 2:

| | GPU | llvmpipe |
|---|---|---|
| No sonar | 0.96 | 0.95 |
| One sonar at 10 Hz (the default) | 0.81 | 0.81 |
| One sonar at 15 Hz | 0.75 | 0.67 |

The USV scenario, the BlueBoat's sidescan (two sensors) and profiler all at 10 Hz on the GPU, ran at about 0.84 (Phase 3, from the message rates on the ROS side).

In every run the sonar kept its configured rate in sim time, with no ping missing: the cost shows up as a slower world, never as dropped pings, so the output stays a function of the seed and the scene. A rule of thumb follows: the world slows by roughly the ping rate times the render time; at 10 Hz, 10 × 15 ms is 15 % of each second on the rendering thread.

## Without a GPU

In the small imaging scene llvmpipe is close to the GPU, because the render is dominated by fixed per-camera costs, not triangles. In a large scene it is not. The Phase 1 sampler on the 2.1-million-triangle scene, median milliseconds per ping including the CPU post-processing:

| Fan | GPU, labels | GPU, no labels | llvmpipe, labels | llvmpipe, no labels |
|---|---|---|---|---|
| Imaging, 1024 × 64 rays | 10.9 (92 Hz) | 7.6 (131 Hz) | 178 (5.6 Hz) | 129 (7.7 Hz) |
| Sidescan, one side | 1.8 | 0.9 | 94 | 44 |
| Profiler | 2.1 | 1.1 | 95 | 45 |

So an imaging sonar without a GPU manages about 4 to 8 pings a second in a large scene, and the world runs below real time at the default 10 Hz. The labels add about 40 % to the imaging fan's cost and about double the sidescan's and profiler's, on either renderer, because they are a second render of every camera. GPU and llvmpipe hit the same surfaces, with ranges differing by up to 4 mm: the same output within millimeters, not bit for bit.

To force llvmpipe in drydock, `gz sim` needs `__GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1`: it renders through GLX on the forwarded display, and the EGL settings alone leave it on the GPU (the first "llvmpipe" run in Phase 2 was in fact on the GPU). Ogre's log, `~/.gz/rendering/ogre2.log`, names the renderer it used.

## A world with no active sonar

The sonar system costs nothing until a sonar exists. Joining the rendering thread's events makes gz-sim render every physics step, so the system joins them only when the first sonar appears, and leaves them when the last one goes. Measured in Phase 2 on a light scene (a camera at 30 Hz, 42 s of sim time):

| World | Real time factor |
|---|---|
| A camera, no sonar system | 4.15 |
| The same, sonar system joined to the render events from the start, no sonar | 3.30 |
| The same, joining only once a sonar exists (as built), no sonar | 4.11 |
| The camera plus one sonar at 10 Hz | 3.02 |

The first ping of each sonar also costs 30 to 140 ms once, for texture and shader creation.

## Being deliberate about sonar in a scene

- Fit a sonar only where the scenario uses it. Every vehicle world loads the sonar system, which is free until a sonar is fitted, so the choice is in the vehicle's config: the BlueROV2's and BlueBoat's `*_sonar.yaml` configs carry sonars, their default configs do not.
- One active sonar per scenario is the budget the defaults assume. Several sonars on several vehicles in one world has not been measured.
- Rate: 10 Hz is the default and a realistic imaging rate at 15 m. Each extra ping per second costs the world roughly one more render; on a machine without a GPU, or in a large scene, pick a lower rate.
- Labels: turn them on only when recording ground truth; they are a second render of every camera, 40 % to 100 % more render time.
- Sampling: `rays_per_beam` is the first lever for speed, then `elevation_rays` and `elevation_span` (the advanced parameters in the design's [SDF](sonar.md#sdf) section), at a cost in fidelity to check on the scene at hand.
- Range and samples: a shorter range window and fewer samples shrink the engine's work and the message.

## Not measured yet

- Several vehicles with sonars in one world (planned for a dedicated machine).
- Large competition worlds (`robotx_2026`, 159 models) with a sonar.
- The current code after the review's changes (stamped run-time changes, label reuse); none of them is on the per-ping path, so no change is expected.
