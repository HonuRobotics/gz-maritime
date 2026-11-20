# Sonar simulation design

Status: first draft, 2026-11-20. Numbered decisions and open questions at the end are for discussion.

## Use cases

The sonar simulation exists so that a team can develop underwater perception and mapping in simulation and run the same pipeline on the real vehicle. The driving case is the Virtual RobotX 2027 competition, but the needs are generic to small marine robots working in shallow harbor water:

1. Detect and classify objects underwater, from a strong, simple target (a hollow sphere) up to lookalike shapes, low-contrast objects and clutter near structures, including when the camera cannot see.
2. Measure the range and bearing to a nearby object precisely enough to report its position in the world.
3. Decide whether a space (a dock bay, a route) is clear of submerged obstructions.
4. Survey an area and produce a georeferenced map product of the seabed and an object on it, such as a shipwreck, from a surface vehicle, an underwater vehicle, or both.

Two properties of the setting shape everything else. The water is shallow: nominally 3 to 12 m, at most 30 m. And the simulation is used for scored competition runs as well as development, so it must be repeatable from a seed, run on the evaluation hardware alongside several vehicles, and expose difficulty through a small set of documented parameters.

## Sonars in scope

| Sonar | Where | Use cases | Exemplar | Native product of the exemplar |
|---|---|---|---|---|
| Imaging sonar (forward-looking multibeam) | UUV forward slot | 1, 2, 3 | Sonoptix ECHO: 400 kHz to 100 m or 700 kHz to 30 m, 256 beams over 90° or 120°, 20° vertical, up to 25 Hz | uint8 matrix, one column per beam and one row per range sample |
| Sidescan | USV, one unit per side | 4 | Cerulean Omniscan 450 SS: 450 kHz, 0.5° along track by 50° vertical, up to 150 m, up to 20 Hz | One line per ping: 200 to 1200 uint16 samples in dB, with range start, length, gain and sound speed |
| Profiling sonar (downward multibeam) | UUV downward slot | 4 | To be chosen | A range (and intensity) per beam |

A UUV has two sonar slots, forward and downward, and a team fills none, one or both. Out of scope: single-beam echosounders and altimeters, interferometric sidescan, synthetic aperture sonar, sub-bottom profilers, mechanically scanned sonars (imaging or profiling), 3D sonars with a two-dimensional receive array, acoustic positioning (USBL, LBL) and acoustic communication, which is a separate design.

The three sonars differ in where they point and how wide the beam is in elevation, not in the underlying physics. An imaging sonar has a beam wide in elevation: every pixel sums echoes from a tall wedge of water, so the image shows the scene's appearance (highlights, shadows) but loses each return's elevation. A profiling sonar has a beam narrow in elevation, so each beam's strongest return fixes a point in 3D, and the vehicle's motion sweeps the profile into bathymetry. A sidescan is a single fan-shaped beam per side, swept along the track by the vehicle's motion.

### Glossary

How this project uses the terms. The field uses several of them loosely, so the synonyms column says what else a reader may meet and what we avoid.

| Term | How we use it | Synonyms and terms we avoid |
|---|---|---|
| Multibeam sonar | A sonar whose array forms many beams at once, side by side across a fan. In this project it is either an imaging sonar or a profiling sonar. | "Multibeam" alone means a profiling sonar to hydrographers and often an imaging sonar in robotics papers; we always say which. |
| Imaging sonar | A multibeam sonar whose beams are wide in elevation. Its product is an intensity image in range and bearing; the elevation of each return is lost. | Forward-looking sonar (FLS), multibeam imaging sonar, 2D sonar, acoustic camera (usually for the highest-frequency units). We avoid FLS as a type name, because it names the pointing direction: a profiler or a 3D sonar can also look forward. |
| Profiling sonar | A multibeam sonar whose beams are narrow in elevation. Its product is a range (and intensity) per beam, a profile of the scene that motion sweeps into bathymetry. | Multibeam echosounder (MBES), bathymetric multibeam, swath sonar, multibeam profiler. Not a sub-bottom profiler, which images layers beneath the seabed. |
| Sidescan sonar | One beam per side, narrow along track and wide in elevation, pointing sideways. Each ping gives one intensity line per side; motion along the track builds the image. | Side-scan, side scan, SSS. It is not a multibeam sonar in this project. |
| Ping | One transmission and the echoes received from it. | Shot, frame (for an imaging sonar's image). |
| Beam | One direction the sonar resolves within a ping. | Channel (in some vendor formats). |
| Sample | One time step of a beam's received echo, which corresponds to one range, at 2R/c. | Range bin, bin, row (in the ECHO's matrix). |
| Azimuth, elevation | The angles across a fan and out of it: for an imaging sonar, horizontal and vertical; for a sidescan, along track and across the swath. | Horizontal and vertical field of view (FOV), beam width and beam height (Cerulean). |
| Beam width | The angle between the −3 dB points of a beam's two-way response. | Beam angle, opening angle. |
| Incidence angle, grazing angle | The angle between the ray and the surface normal (incidence), or between the ray and the surface itself (grazing). They add to 90°. | Angle of incidence. Seabed backscatter is usually quoted against grazing angle. |
| Waterfall, mosaic | A waterfall stacks sidescan lines in ping order, in the sonar's own frame. A mosaic places them in the world from the vehicle's position and heading. | Sidescan image, georeferenced image. |

## Related work

A survey of the field in November 2026 found the following. Its full notes are in the maritime-workspace repository.

| Simulator | Sonar model | Fidelity | What we take from it |
|---|---|---|---|
| DAVE, now [IOES-Lab/POSIM](https://github.com/IOES-Lab/POSIM) (Gazebo Jetty, ROS 2 Lyrical, Apache-2.0) | Point scattering on depth-camera rays, complex Gaussian amplitudes, beam pattern, per-beam time series; CPU, CUDA and WGPU back ends; publishes `marine_acoustic_msgs/ProjectedSonarImage` | Coherent time series on rasterized geometry, the only open model at this level | The output contract, the depth-camera sampling, and possibly the engine. Its absorption constant suits about 100 kHz rather than the MHz heads it ships, its back ends disagree numerically, and it has not been validated quantitatively against real data |
| [HoloOcean](https://github.com/byu-holoocean/HoloOcean) (Unreal) | Octree, ray-cast and GPU back ends; cosine and impedance shading, mean per bin, heuristic noise; imaging, profiling, sidescan, single beam | Geometric ray cast with partial intensity physics | That one engine can serve imaging, profiling and sidescan by configuration. Averaging per bin, instead of summing, removes the dependence on ensonified area |
| [Stonefish](https://github.com/patrykcieslak/stonefish) (OpenGL, GPL-3.0) | Rasterize and bin per beam column; correct ping timing from 2R/c | Image emulation to geometric | Ping timing; the GPL rules out reusing code |
| [OceanSim](https://github.com/umfieldrobotics/OceanSim) (Isaac Sim) | Camera point cloud, cosine and attenuation, normalized per range row | Image emulation | Normalizing per frame or row erases the absolute intensity that classification needs |
| [MARUS](https://github.com/MARUSimulator) (Unity) | CPU ray casts, cosine-squared shading; labeled dataset export | Geometric | Per-return object labels for training data |
| [SMaRC](https://github.com/smarc-project) (Unity, plus auvlib) | Cheap runtime sidescan with the real vehicle's message; differentiable renderers fitted to real surveys offline | Geometric at run time | A sidescan message modeled on the real sensor's packet, so the same consumer runs on both |

Three findings carry into this design. None of the perception simulators puts the geometry (beam angles, range per sample) in its output, so a consumer receives a picture without its coordinate system. Where users have measured transfer to real data, geometric tasks transfer (feature matching trained on HoloOcean beat LightGlue on real tank data) and intensity and noise do not (a HoloOcean-trained denoiser failed on real data). And Gazebo itself has no sonar sensor; gz-sensors issue #19 has been open since 2020.

## Theory of operation

### The shared engine

All three sonars use one engine, an incoherent beam model: per ping, it computes the echo level arriving in each beam as a function of range, from the scene geometry and a sonar equation, then reads that out the way the real device does. In the survey's terms this is physics-based intensity on rasterized geometry: one rung above the geometric ray casts in game-engine simulators, one rung below a coherent time-series model. The competition consumes beamformed intensity and scores decisions, not signal processing, so coherent phase adds cost without adding anything a team's pipeline can use; intensity physics, by contrast, is where classification difficulty lives.

```{mermaid}
flowchart LR
  G["Sample geometry<br/><i>range, incidence, label per ray</i>"] --> E["Echo level per ray<br/><i>sonar equation</i>"]
  E --> B["Bin into beams and range<br/><i>beam pattern, pulse length</i>"]
  B --> S["Speckle and noise<br/><i>seeded</i>"]
  S --> R["Readout<br/><i>TVG, gain, dB, quantize; or detect range</i>"]
```

1. Sample the geometry. For each ping, a fan of rays covers the sonar's field of view at finer resolution than the beams, in azimuth and in elevation. Each ray returns the range to the first surface it hits, the incidence angle there, and the acoustic properties of that surface. Taking only the first hit along each ray produces acoustic shadows without any extra step.
2. Compute the echo level of each ray from the sonar equation, in linear units: the contribution of a ray hitting a surface at range R and incidence θ is proportional to b²(φ, ψ) · σ(θ) · A / R⁴ · 10^(−2αR/10), where b is the one-way beam pattern at the ray's azimuth φ and elevation ψ, A = R² ΔΩ / cos θ is the area the ray's solid angle ΔΩ covers on the surface, and α is the absorption in dB per meter. The backscatter σ(θ) is Lambert's law, μ cos²θ, with μ a property of the surface, plus an optional specular term for smooth objects. Absorption comes from the Francois–Garrison equations at the configured frequency, temperature, salinity and depth, not from a fixed constant.
3. Bin into beams and range. Each ray's contribution is added (not averaged) into the range sample at 2R/c, spread over the pulse length, and into the beams according to the beam pattern, so sidelobes appear around bright targets. Summing over the elevation of a beam reproduces the imaging sonar's elevation ambiguity; for the narrow beams of the profiler and the narrow along-track width of the sidescan it collapses to almost nothing.
4. Speckle and noise. Each sample's intensity is multiplied by an exponentially distributed random factor, the statistics of fully developed speckle from many unresolved scatterers, and an additive noise floor is applied. Every random draw comes from a generator seeded from the world's seed, the sensor's name and the ping number, so a run is reproducible.
5. Readout, which differs by sonar (below).

The shallow, near-isovelocity water lets the engine ignore refraction and propagation modeling: paths are straight, and sound speed is a single configured value. Surface and bottom multipath matter for the sidescan in shallow water and are modeled as one mirror bounce off the mean water surface, as an option.

### Imaging sonar

Configured as a fan of N beams over the horizontal field of view, wide in elevation, pointing forward with a configurable tilt. Readout: time-varied gain and gain as the device applies them, conversion to dB, then quantization to the device's 8-bit beam × sample matrix. The ECHO defaults: 256 beams over 120° at 700 kHz or 90° at 400 kHz, 20° vertical, range resolution better than 8 mm with the number of samples following the range setting.

### Sidescan

Configured as one beam per side, narrow along track and wide in elevation, pointing to the side with a configurable depression angle. One ping produces one line; the line-by-line waterfall and any mosaic are built by the consumer from the vehicle's motion, as with the real sensor, so the vessel's roll, pitch and yaw in waves enter the data without special handling. Readout: dB scaled into uint16 between a minimum and maximum level, as the Omniscan reports it. The Omniscan defaults: 450 kHz, 0.5° by 50°, no tilt, 600 samples, far field beyond 9 m. The nadir gap, the first bottom return, and the grazing-angle dependence of the seabed return all follow from the geometry and the backscatter law.

### Profiling sonar

Configured as a fan of beams narrow in both directions, pointing down. Readout: per beam, the range of the strongest return above a detection threshold (or the leading edge), with range noise that grows as the signal-to-noise ratio falls, and a dropout when nothing exceeds the threshold, as happens at grazing angles and over weak surfaces. The bathymetric work that a large-ocean multibeam needs (sound-speed ray tracing, multi-sector timing) is unnecessary at 30 m or less. Defaults follow the exemplar once chosen.

### Acoustic properties of the scene

Each surface needs a backscatter strength μ (and optionally a specular term). Materials of a part live in its `.glb`, and an SDF `<visual>` must not declare a `<material>`, so acoustic properties cannot come from SDF materials. They are attached the way the project already marks collisions for buoyancy and wind: an attribute on the visual (or a model-level default), for example `gz:acoustic_reflectivity="0.2"`, with a world default for the seabed and for anything unmarked. Ground truth labels come from the same pass: each ray knows which entity it hit.

## Interface

### SDF

Each sonar is a gz-sensors custom sensor, the pattern `gz_wind`'s anemometer already uses: `<sensor type="custom" gz:type="imaging_sonar">`, `gz:type="sidescan_sonar"` or `gz:type="profiling_sonar"`, with the standard `<update_rate>`, `<topic>` and `<pose>`, and a `<gz:sonar>` block:

```xml
<sensor name="forward_sonar" type="custom" gz:type="imaging_sonar">
  <update_rate>15</update_rate>
  <topic>sonar/forward</topic>
  <gz:sonar>
    <frequency>700000</frequency>
    <beams>256</beams>
    <horizontal_fov>2.094</horizontal_fov>
    <vertical_fov>0.349</vertical_fov>
    <range><min>0.2</min><max>30</max></range>
    <samples>512</samples>
    <sound_speed>1500</sound_speed>
    <pulse_length>0.0001</pulse_length>
    <noise_floor>...</noise_floor>
    <gain>...</gain>
  </gz:sonar>
</sensor>
```

The parameters are the same for the three types, with defaults set per type from its exemplar. A world system (the counterpart of the wind system) owns the engine, the water properties (temperature, salinity, sound speed) and the scene's acoustic defaults, and drives every sonar sensor in the world, including those on vehicles spawned at run time.

### Messages

| Sonar | ROS 2 message | Notes |
|---|---|---|
| Imaging | `marine_acoustic_msgs/ProjectedSonarImage` | Beam directions, range per sample, sound speed and the 8-bit data in one message |
| Profiling | `marine_acoustic_msgs/SonarRanges` | Range and direction per beam |
| Sidescan | To be decided: a two-beam `marine_acoustic_msgs/RawSonarImage`, or a sidescan message modeled on the Omniscan `os_mono_profile` packet | `marine_acoustic_msgs` has no sidescan message |
| Ground truth (optional) | Per-sample entity labels on a separate topic | Never on the sensor topic; an imaging sonar cannot resolve elevation, so 3D points from it would be information the real sensor never gives |

No sensor publishes a point cloud except as the profiler's ranges. Topics follow the one-name-per-instance rule (`<vehicle>/sonar/forward`, `<vehicle>/sonar/down`, `<vehicle>/sonar/port`). Frames follow the sensor's link, with the axis convention `marine_acoustic_msgs` documents.

Inside Gazebo the sensors publish gz-transport messages. `ros_gz_bridge` has no conversion for the `marine_acoustic_msgs` sonar types, so the messages reach ROS 2 either through a small conversion node in this repository, through new conversions contributed to `ros_gz_bridge`, or by publishing ROS 2 directly from the sensor, as POSIM does. Decision 3 below.

## Implementation plan

Each phase ends with something a user can run, and gets its own issue.

0. Spike (a few days). Build POSIM in drydock against Jetty and Lyrical. Measure its rate on CPU and GPU, headless, with three vehicles in `marina_bay`; check how far its back ends disagree; check whether gz-rendering's depth camera runs acceptably on a CPU-only renderer (Mesa llvmpipe) in the scoring container. The outcome decides between reusing POSIM's engine (fixing absorption, summing, and the back-end disagreement upstream) and writing a smaller engine of our own on the same depth-camera sampling. Either way the interface above stays.
1. Imaging sonar. The custom sensor, the world system, geometry sampling, the sonar equation, binning and the readout, publishing `ProjectedSonarImage`; ECHO defaults; seeded noise. A demo world with a sphere, a cylinder and a box on a flat seabed in front of a BlueROV2.
2. Acoustic properties. The reflectivity attribute, world defaults, and a small set of target models (glass float, cylinder, box, tire, crate) with documented reflectivities; ground-truth labels.
3. Sidescan. The sidescan configuration and its message, Omniscan defaults, a port and starboard mount on the BlueBoat in `bluerobotics_models`, and a survey demo over a wreck model.
4. Profiling sonar. The downward configuration, range detection, `SonarRanges`, the downward slot on the BlueROV2, and a mapping demo over the same wreck.
5. Competition integration. Difficulty parameters exposed per task, seeds from the scenario, the ground truth the referee needs to score a map product, and performance tuning for the evaluation container.

## Testing plan

Tests run headless in drydock, as gtests for the engine and pytests for whole worlds, following the existing packages. Visual checks (that images look right) are left to a person.

| What | How | Passes when |
|---|---|---|
| Geometry | A single sphere at known range and bearing; a flat seabed at known altitude | The return is at 2R/c within one sample and in the right beam; the seabed's first return is at the altitude |
| Spreading and absorption | The same sphere at several ranges | Level falls as 40 log R + 2αR within a tolerance, with α from Francois–Garrison at the configured frequency |
| Backscatter law | A flat seabed seen at several grazing angles | Level follows the configured law (Lambert's cos²θ) |
| Area dependence | Seabed patches of different size at the same range | Level grows with ensonified area (summing, not averaging) |
| Shadows | A box on the seabed | A shadow of the length the geometry predicts behind it |
| Beam pattern | A point target swept across the fan | Main lobe width matches the configured −3 dB width; sidelobes at the configured level |
| Elevation ambiguity | Two targets at the same range, different elevations, inside the imaging beam | Both fall in the same sample of the same beam |
| Speckle | A uniform seabed patch | Intensity samples are exponentially distributed (coefficient of variation near 1) |
| Profiling | A seabed with a known slope and step | Recovered ranges match the geometry within the configured noise; dropouts appear past the configured grazing limit |
| Determinism | The same world, seed and motion twice | Byte-identical messages |
| Back ends | The same ping on CPU and GPU | Agreement within a stated tolerance |
| Interface | Each message type | Beam directions, range per sample, frame and stamp are present and consistent with the sensor's configuration; topics follow the instance name |
| Performance | Three vehicles with all sonars in `marina_bay`, headless | Each sonar holds its configured rate on the scoring hardware, on GPU, and a stated rate on CPU |
| Real data | Recordings of a known target (a glass float, a tire) with a real ECHO and Omniscan, if they can be obtained | Target-to-background contrast, shadow length and speckle statistics within stated tolerances of the simulation of the same scene |

The last row is the only one that checks the model against the world rather than against itself; no surveyed simulator has done it quantitatively.

## Decisions to make

1. Reuse POSIM's engine or write our own on the same sampling approach. Decided by the Phase 0 spike.
2. How the sidescan message is shaped: a two-beam `RawSonarImage`, or a custom message modeled on the Omniscan packet.
3. How messages reach ROS 2: a conversion node here, conversions contributed to `ros_gz_bridge`, or ROS 2 published from the sensor.
4. CPU and GPU. The target is to run on either, with the GPU as the fast path; whether the CPU path must meet the same rate, and whether a GPU-only design is acceptable if it is much simpler.
5. Whether the imaging sonar output is the matrix before or after the device's dynamic compression and contrast settings, and whether the sidescan output has time-varied gain applied.

## Open questions

1. The exemplar for the downward profiling sonar.
2. Whether the BlueBoat carries both sidescan units or one.
3. The form of the map product and how it is scored.
4. Whether recordings from a real ECHO and Omniscan can be obtained for the real-data test.
