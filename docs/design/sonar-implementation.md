# Sonar implementation

How the sonar simulation is built, in five minutes: the pieces, how a ping flows through them, and why each was chosen. What the sonar models and how to use it are in the [sonar simulation design](sonar.md) and the how-to pages.

## The pieces

```{mermaid}
flowchart LR
  E["gz_sonar engine<br/>(library, no Gazebo)"] --> S["gz-maritime-sonar-system<br/>(gz-sim world system)"]
  S -- "gz-transport" --> B["kai_sonar_bridge<br/>(ROS 2 node)"]
  P["bluerobotics_models<br/>sonar parts"] -. "fit sensors,<br/>start the bridge" .-> S
  P -.-> B
```

Two packages in gz-maritime, plus the vehicles that use them:

- `gz_sonar`, in two parts.
  - The library: the sonar model, with no Gazebo dependency. `Config` holds the parameters and the three device presets, `Rays` the fan of rays, `Water` the absorption, `Engine` the sonar equation (hits to intensity per beam and sample), `Readout` the levels, gain and published output, `Random` the seeded draws, `CameraSampler` the step from depth and label images to hits, and `SyntheticScene` analytic shapes for tests.
  - The system plugin, `gz-maritime-sonar-system`: `Sonar` is the world system, `SonarSensor` one sonar with its worker and publishers, `SonarSdf` the reader of `<gz:sonar>` and the acoustic marks. Beside it, the gz-transport messages (`SonarImage`, `SonarDetections`, `SonarLabels`), the walkthrough worlds, target models and example scripts.
- `kai_sonar_bridge`: a ROS 2 node converting the messages to `marine_acoustic_msgs` (`ProjectedSonarImage`, `SonarDetections`), with a viewing image and a point cloud beside them.
- `bluerobotics_models`: the three sonar parts with their frames, one shared xacro for their sensors (`gz_sonar.xacro`) and one table for the bridge's parameters (`sonar.py`), so either vehicle carries any sonar from its config.

Why this split. The model is a library so it can be built, tested and run without a simulator: `render_ping` draws a ping from an analytic scene, and the engine tests check it against known answers. Everything Gazebo-specific is in the plugin, and everything ROS-specific in the bridge, so the plugin never links ROS and a non-ROS user still gets the sonar on gz-transport. The bridge exists because `ros_gz_bridge` does not convert these messages; contributing the conversions there later would retire it.

## One ping

```{figure} images/sonar-ping-flow.svg
:alt: Swimlane chart of one ping on three threads: gz-sim steps the world; on the rendering thread the Sensors system updates the scene, the sonar plugin asks for a render and gz-rendering draws depth and segmentation on the GPU; on the sonar's worker thread gz_sonar turns the images into hits, computes beams and samples, and publishes

One ping, on three threads. Only the shaded step uses the GPU.
```

1. Schedule. Each sonar pings on a fixed grid of simulated time, ping k at k divided by its rate.
2. Render. On gz-sim's rendering thread, owned by the Sensors system, the plugin has gz-rendering draw the sonar's cameras: a depth image (range to every surface) and a segmentation image (which surface each pixel is). An imaging sonar's 120° fan needs two cameras side by side; a sidescan or profiler's thin fan needs one or two narrow ones. Before drawing, the plugin hides what the sonar must not see: the water surface, surfaces marked acoustically transparent, and its own vehicle.
3. Hand over. The images go into a queue, at most two deep, to the sonar's own worker thread.
4. Model. On the worker, `CameraSampler` turns the images into hits (a range, an incidence angle from the neighboring pixels, and the surface's acoustic properties from its label), and `Engine` sums every hit's echo into beams and range samples through the beam pattern, spreading and absorption, backscatter and speckle. `Readout` applies gain and produces what the device publishes: an 8-bit image, a sidescan line in dB, or bottom-detected soundings.
5. Publish, on gz-transport, stamped with the simulated time of the scene that was drawn; the bridge republishes on ROS 2 in the part's frame.

Why geometry from rendering. A sonar needs a range and a surface for tens of thousands of rays per ping. Rendering gives them in one GPU pass, on the scene the cameras already see, with the same meshes and no second geometry representation to keep in sync. Ray casting against the physics geometry would see collision shapes rather than visuals, and would be far slower on detailed meshes. The cost is that rendering happens on gz-sim's rendering thread, which the simulation waits for; that is why only the drawing happens there and the rest is on the worker.

Why segmentation carries the acoustics. Each surface's acoustic properties (Lambert strength, specular strength and width, transparency) are set in SDF, on the visual, the model, or the world's defaults. The plugin resolves them per visual, gives each distinct set a label id, and the segmentation camera writes that id into every pixel. So the label image is both the ground truth that can be published and the lookup that tells the engine how each hit echoes; it is always drawn, and publishing it as `labels` costs only a message.

## Design choices worth knowing

One world system for every sonar. Sonars are gz-sim custom sensors (`<sensor type="custom" gz:type="imaging_sonar">`), and one system in the world finds and drives all of them, including on vehicles spawned while the world runs, as gz-maritime's wind and DVL systems do. It owns the water (sound speed, temperature, salinity) and the scene's acoustic defaults, which are properties of the world, not of each sensor. It joins the rendering thread only while a sonar exists, because joining it makes gz-sim render every physics step; a world without a sonar pays nothing.

Determinism over speed. The output must be a function of the seed and the scene, so a run can be repeated exactly for training data, scoring or debugging. Three things make it so:

- every random draw is seeded from the world's `<seed>`, the sensor's name and the ping number, so two sonars differ and a ping repeats;
- a ping is never dropped: if the worker falls behind, the queue fills and the simulation waits, so a slow computer gives the same data, more slowly;
- changes at run time (a sonar or a target spawned or removed, a reset) are stamped with the simulated time they were made and take effect at the first frame that reaches it, so they land on the same ping in every run.

Two threads, not one. Only drawing needs the rendering thread; the model, about three quarters of a ping's work, runs on the worker alongside the simulation, and its engine step uses every CPU core. This is why a GPU matters little (see [Sonar performance](sonar-performance.md)) and why more cores help more than a faster GPU.

Device presets, open parameters. An empty `<gz:sonar/>` gives the exemplar device for its type (Sonoptix ECHO, Cerulean Omniscan 450 SS, Cerulean Surveyor 240-16); every parameter can be overridden, and values are validated when the sensor loads, so a mistake fails with a message instead of a NaN image.

Messages of our own, standard ones on ROS. The gz-transport messages carry what the model knows, including the gain and time-varied gain of each sidescan line and the per-sample labels; the ROS side uses `marine_acoustic_msgs`, the messages real sonar drivers publish, so a perception stack written for the real device runs unchanged. Images are stamped in an `_acoustic` frame of each part (Z along the beam, X up), which `ProjectedSonarImage` expects; soundings in the sensor's own frame.

Vehicles from config. On the Blue Robotics vehicles a sonar is a part in the vehicle's YAML config, with a `sonar:` dict for its settings. One shared xacro emits its sensors, one table (`sonar.py`) generates the bridge's parameters, and the launch starts the bridge, so a sonar is added, moved or reconfigured without editing a model.

## Where it is tested

- `engine_test`, `sampler_test`, `sdf_test`: the model against analytic scenes, the sampler on synthetic images, the SDF parser and its validation.
- bluerobotics_models' tests: the parts' frames, the generated sensors and bridge parameters, and every shipped config.
- The walkthroughs ([imaging](../how-to/sonar-imaging.md), [USV survey](../how-to/sonar-survey.md), [features](../how-to/sonar-features.md)) run in real worlds and were checked against what each scene predicts; repeatability with run-time spawns and resets was checked by comparing runs byte for byte.
