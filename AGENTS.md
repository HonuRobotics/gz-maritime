# AGENTS.md

Operational guide for AI agents working in this repository. Read this first, then
the deeper design docs it points to. This file applies to the whole repo; if a
subdirectory adds its own `AGENTS.md`, the closest one to the file you are
editing wins.

Kai (Hawaiian for sea) is a Gazebo + ROS 2 maritime simulation, continuing
the VRX rewrite. The substantive
subsystem today is the **wave simulation packages** (`gz_waves*`); more
subsystems will be added over time.

## Project layout

| Path | What |
|------|------|
| `gz_waves/` | Engine-agnostic wave **core** (the `IWaveField` contract, engine registry, `Wavefield` ECM component, the `WaveCoupling` tag component, `Eval` facade, `WavesSystemBase`). |
| `gz_waves_provider_gerstner/`, `gz_waves_provider_fft/` | Wave field **engines** (an `IWaveField` implementation + its server source plugin + a GUI registrar). |
| `gz_waves_rendering/` | Engine-agnostic **renderer** (`WaterVisual`) + the Ogre2 C-ABI bridge + the `water_surface` model. The water surface is two-sided, and its fragment shader draws Snell's window seen from below (sky refracted inside 48.6 degrees, the water's mirror outside). |
| `gz_waves_buoyancy/` | The wave field's physics **consumer**: `gz-maritime-wave-buoyancy-system` (`WaveBuoyancy`), run instead of the flat buoyancy in a world whose vehicles should float on the waves. It floats every collision marked `gz:buoyancy="true"` (found with `gz_marked_shapes`) on a plane fitted through the drawn surface around it (`waves::SurfaceHeight`, the field as the wave system last advanced it; it never advances it), with the water and the air each displaced; hydrostatic only. It leaves a link the flat buoyancy floats too. It tags the world with the core's `WaveCoupling` component, so `gz_hydrodynamics` damps against the waves' water velocity too (tested here, `hydrodynamics_waves_test`). Opt-in: no `kai_gazebo` world runs it yet. |
| `gz_buoyancy/` | gz-sim's Buoyancy system, vendored with the upstream marked-collision change (`gz:buoyancy="true"`, `<enable_by_default>`); a bridge until a Gazebo release ships it. |
| `gz_wind/` | The world's wind, like the wave field: a `Windfield` recipe (model name, `<speed>`, `<direction>` it comes from, seeded spectral gusts on both that travel with the wind, a logarithmic profile with height) on the world entity, a model registry, and `WindSampler`/`WindAt` for any system that needs the wind at a point. The system writes the recipe, sets gz-sim's wind entity so rotors and wings feel it, takes changes on the `wind/set` topic (bridged to ROS), publishes `wind_info` ground truth, and pushes on marked collisions (`gz:wind="true"`, optional `gz:wind_cd`): quadratic drag on the part of each shape above the water, with the wind at that part. It also reads `anemometer` custom sensors (`type="custom" gz:type="anemometer"`): the apparent wind in the sensor frame as `gz.msgs.Twist`. |
| `gz_ocean_current/` | The world's ocean current, built like the wind: an `OceanCurrentfield` recipe (model name, `<speed>`, the `<direction>` it sets towards, the `<water_level>`, a seed, and the parameters a model owns, such as `source`, opaque to the system, a generation) on the world entity, an `IOceanCurrentModel` registry, and `OceanCurrentSampler`/`OceanCurrentAt` for any system that needs the current at a point, loaded with the world or spawned later. The built in `standard` model is uniform and horizontal; a new model is one registered class. The system writes the recipe as a component change only, takes changes on the `ocean_current/set` topic (bridged to ROS), publishes `ocean_current_info` ground truth, and pushes on marked collisions (`gz:ocean_current="true"`, optional `gz:ocean_current_cd`): quadratic drag on the part of each shape below the `<water_level>`, relative to the water at that part, for a vehicle without identified coefficients. No sensor. A vehicle with Fossen coefficients feels it through `gz_hydrodynamics`. |
| `gz_water_optics/` | What the world's water does to light, built like the ocean current: a `WaterOptics` recipe (the Jerlov type it came from; per R G B the attenuation β^D, the backscatter β^B, the veil B∞ and the diffuse attenuation K_d; the water level; a generation) on the world entity, the Jerlov table (Solonenko and Mobley 2015, `src/Jerlov.cc`, red 625, green 525, blue 450 nm), `Resolve` (type, `<visibility>` scaling a, b and K_d alike so 4.8 / c_green is the visibility, explicit values overriding), and `WaterOpticsSampler` for any system that sees through water: depth below the wave field's local surface, daylight and veil at a depth, transmittance and veil build-up over a range. The system takes changes on `water_optics/set` (whole or not at all) and publishes `water_optics_info` ground truth. Also `<daylight>` (scales the veil; 0 at night), `<caustics>` and `<marine_snow>`. Drawn by `gz_underwater_camera` (reference, CPU) and `gz_underwater_rendering` (GPU); `kai_gazebo/worlds/underwater_camera_demo.sdf` runs it. |
| `gz_underwater_camera/` | The underwater camera, the reference (CPU) path of `UNDERWATER_CAMERA_PLAN.md`: a plugin (`gz-maritime-underwater-camera-system`) inside an `rgbd_camera` sensor with a `<topic>`, which pairs each colour frame with the depth frame of the same stamp (one render), reads the world's water through `WaterOpticsSampler` at the sensor's world pose, and publishes `<topic>/underwater/image` and `camera_info`. The image library (`UnderwaterImage`, no Gazebo) does, per pixel on linear light: z-depth to range (a ray table from the CameraInfo intrinsics; a Gazebo camera looks along +x, image right is -y, image down -z), the daylight faded to the object's depth, the water only up to the surface for a climbing ray, I = lit e^(-beta^D w) + B-inf(z_cam)(1 - e^(-beta^B w)), then the camera's response (noise on the light caught, then exposure and grey-world white balance), which applies above the water and without water optics too. Invariants: callbacks only fill a mailbox (a `shared_ptr` they capture); one worker of the plugin's own, joined in the destructor, processes frames outside every lock, with the water of the frame's own step; it subscribes to the camera only while its output has a subscriber; noise is drawn per row from generators the frame's one draw seeds, so a frame is the same on any thread count. `kai_gazebo/worlds/underwater_camera_demo.sdf` shows it. Also forward scatter (`<forward_scatter>`) and a flat port's refraction (`<port>`). The validation tools (`tools/make_chart_world.py`, `tools/chart_validation.py`, RGB angular error per ColorChecker patch, OceanSim's metric) compare it with real footage; no real footage has been compared yet. |
| `gz_underwater_rendering/` | The GPU path of `UNDERWATER_CAMERA_PLAN.md`: the underwater camera's model per fragment inside Ogre's PBS shaders, so every ogre2 camera and the GUI see through the world's water. A visual plugin (`gz-maritime-underwater-rendering-system`, on the invisible `underwater_optics` model) reads `WaterOpticsSampler` in PreUpdate and on SceneUpdate dlopens `libunderwater-ogre2-bridge.so` (never link gz-rendering-ogre2 into a plugin; see `gz_waves_rendering`), which checks `share/hlms/Underwater` defines its `<hook>_underwater` pieces, adds one `@insertpiece` to the piece that defines each hook in an in-memory copy of its archive (or defines the hook if nothing does; anything unexpected, or a gz-rendering other than 10, and it does not install), adds its pieces, reloads PBS, and wraps gz-rendering's Hlms listener to append the camera pose and the water to the pass buffer (property `gz_underwater`; not in caster, depth-only or solid-colour passes). Only once installed, and while `<enabled>` and the `underwater_rendering/enable` topic say so, does the plugin mark the world with `UnderwaterRendering`, naming its own entity (so the mark stops holding when its model is removed); `gz_underwater_camera` then applies only its response; otherwise the reference camera draws the water. The plugin and the bridge share `src/systems/UnderwaterOgre2Bridge.hh` (a versioned `UwWater` struct, checked on dlopen; bump `UW_OGRE2_BRIDGE_VERSION` with it); the bridge asks the plugin's `WaterOpticsSampler` for the surface over each camera on the render thread, so depth is below the wave, in the GUI too. Matches the reference camera to 0.01 of a step on average. PBS and unlit (particles) are drawn through the water, open water is the veil (a backdrop box in queue 10), the water surface from below shows Snell's window; terrain is drawn as in air. Lamps: spot/point lights with `lamp` in their own name (up to 4, first by name; the rest and every other light stay with Ogre) are hidden from Ogre's own lighting while the water is drawn (`HideLamps`, rebuilt each frame from gz-rendering's lights; never turn Forward+ off by property, it shifts the pass buffer layout under the listener's data) and lit in the shader by Ogre's own diffuse BRDF, attenuation and range fade (so they look the same in air), faded both ways, plus its beam (single scatter, HG g 0.9, 8, 12 or 16 quadratically spaced steps by `<quality>`, up to 8 attenuation lengths). gz gives Ogre a light's power as intensity × pi. World `<daylight>0` is night. Caustics (`<caustics>`, scaled by the wave height at 64 spiral points around the cameras, from the sampler; phases wrapped in double on the CPU) on up-facing fragments, faded `exp(-K_d z)`; marine snow (`<marine_snow>`) by a 3D DDA over 25 cm hashed cells; every hook is chained as `<hook>_underwater` (or `_underwater_unlit`), never defined by the pieces themselves. GLSL only. |
| `gz_acoustics/` | Underwater acoustics as behaviour, not physics. A model plugin (`gz-maritime-acoustic-pinger-system`) puts a pinger on a link: an `AcousticPinger` component with frequency, period, offset, reach, pulse length and an active flag, changed at run time on `/world/<world>/acoustics/<name>/set`. A world system (`gz-maritime-acoustics-system`) makes each active pinger ping on simulation time, carries the ping at `<speed_of_sound>` and hands it to the `acoustic_receiver` custom sensors (`type="custom" gz:type="acoustic_receiver"`) it reaches: one `gz.msgs.Param` per ping heard, bearing and elevation in the sensor frame, frequency, amplitude, valid and optionally range and emit time, stamped with the arrival time (sent plus distance over the speed of sound, not the step), under the water only, within the reach, the receiver's band and its field of view, with gz-sensors' noise models on each channel (repeating with `gz sim --seed`, as every sensor's), and dropouts and outliers drawn from the receiver's own generator, seeded from `<seed>`, never gz-math's global one; two in-band pings, the later arriving within the earlier one's pulse, are invalid. Ground truth on `/world/<world>/acoustics/pingers`, and the receivers listed on `/world/<world>/acoustics/receivers`. `kai_gazebo`'s `acoustic_demo.sdf` and `kai_custom_vehicle`'s `acoustic_demo.launch.xml` show it. |
| `gz_sonar/` | Imaging, sidescan and profiling sonar: `custom` sensors (`gz:type="imaging_sonar"`, `"sidescan_sonar"`, `"profiling_sonar"`) driven by a world system (`gz-maritime-sonar-system`) that samples each sonar's fan with depth and segmentation cameras on the Sensors system's rendering thread, and a library with no Gazebo in it (rays, the incoherent sonar equation, the published image and soundings, water absorption, seeded randomness) that a per-sonar worker thread runs. Publishes its own gz messages (`gz.maritime.sonar.SonarImage`, `SonarDetections`, `SonarLabels`). Surfaces are marked with `gz:acoustic_*` attributes on a `<visual>` or `<model>`. Its three worlds are walkthrough-only (see the world rules). Design: `docs/design/sonar.md`. |
| `kai_sonar_bridge/` | ROS 2 node converting the sonar's gz messages to `marine_acoustic_msgs` (`ProjectedSonarImage`, `SonarDetections`), with a drawn `/view` and a `/points` cloud; tested conversions in `Conversions.hh`. |
| `gz_marked_shapes/` | The collisions a vehicle marks for a fluid, shared by `gz_wind` and `gz_ocean_current`: shapes and projected areas from the geometry, the cut at the water level, the links that carry them (found on every model whenever it shows up), and the quadratic drag the fluid applies. |
| `gz_hydrodynamics/` | gz-sim's Hydrodynamics system, vendored so it damps against the water: when the world has a `gz_ocean_current` recipe it keeps an `OceanCurrentSampler` and asks it every step at the link origin, or at the plugin's `<center_of_pressure>` when the vehicle names one, so added mass, Coriolis and damping all use the velocity relative to the water (the world owns the current; the plugin's `<default_current>`, `<lookup_current_*>` and ocean current topic are then ignored, with a warning once for a tag set and once at the first message on the topic), in a world tagged `WaveCoupling` (by `gz_waves_buoyancy`) it adds the waves' water velocity at that same point, faded with depth below the current recipe's `<water_level>` (`waves::WaterVelocity`), and it reads an `EnvironmentPreload` current table on a vehicle spawned at run time, which upstream does not, rebuilding its lookup on a reload. Its tests reload and remove the table, and the custom USV's `test_ocean_current.py` drifts a boat, spawns one later and holds one against the current. Library `gz-maritime-hydrodynamics-system`, alias `gz::sim::maritime::Hydrodynamics`; Fossen coefficients unchanged. A fork until upstream has a current component; see its `PROVENANCE.md`. |
| `gz_maritime_gui/` | The Gazebo GUI for the environment: four GUI systems (libraries of these names, installed in `lib/gz_maritime_gui` on `GZ_GUI_PLUGIN_PATH`). `OceanCurrentPanel` and `WindPanel` show their field from its recipe in the GUI's ECM, steer it by hand from a compass (`set` messages, rate limited, `Steering`, which stops when someone else moves the field) or set it exactly, draw it as flow trails (`ParticleField`) or arrows coloured by speed (`Colormap`, one `LINE_LIST` marker per colour bin, `ColorBins`, through `MarkerManager`'s `/marker_array`), and on each vehicle (non-static models with a link, buoys included, `VesselTracker`, the model origin's velocity from poses) the water past the hull at its origin or the apparent wind 2 m up as solid arrows, with the track and heading; the wind panel flies wind socks (`WindSock`). `ScenarioPanel` steers both fields to presets and runs a demo (`Presets`, `Demo`). `AcousticsPanel` lists the pingers (those the acoustics system lists on `/world/<world>/acoustics/pingers`, each as its `AcousticPinger` component in the GUI's ECM has it when it does, `MergePingers`; it never asks the state service) with an on/off switch each, and the receivers the acoustics system lists on `/world/<world>/acoustics/receivers` with their last reading and a retune field (`AcousticListener`), and draws, each switched on its own and in a colour per frequency, the reach where it meets a horizontal plane (the water, a depth, or a receiver's depth following it, `Plane`, `PlaneZ`), or in 3D as wireframe spheres cut at the water (`SphereSegments`, `ReachSpheres`, `PingSpheres`), a ring per ping spreading on that plane at the speed of sound slowed down, and a ray along each receiver's last bearing, from the pose the receiver had when it heard it (`ReachMarkers`, `PingMarkers`, `RayMarkers`, `KeepRays`); every panel's drawing goes through one `MarkerLayer`. A Qt-free core library (`gz_maritime_gui_core`) holds the logic and the gtests: a field's rates, colours and scales once (`FieldSpec`), its control (`FieldControl`: topic, state service, tracking, steering) apart from its drawing (`FieldDrawing`); a pytest runs the four plugins in an offscreen GUI. Every `kai_gazebo` world docks all four, drawing off (the acoustic demo draws its acoustics); `open_water.sdf` flies a sock on the landing pad. |
| `gz_lens_drops/` | Water drops on a camera's lens: the drops behave like water on glass, and the camera sees through them as through small lenses. A core with no Gazebo dependency holds the drop model (`Drops`) and the drop map (`DropMap`). `Drops` keeps up to 512 seeded drops, stepped at a fixed 60 Hz. They land at the rain flux on the lens (`<rain_rate>`, `<exposure>` and an `Environment` of gravity along the lens, apparent air and flux) and stay pinned until gravity and air beat a threshold set by `<slide_radius>`. Then they slide, leave beads and wet trails, merge keeping their water (only moved or new drops are checked) and dry keeping their contact circle (`fill`). `DropMap` draws spherical caps at `<contact_angle>` (clamped to pi/2) into two channels, height and film, with wobbly rims, stretched when sliding, plus the trails' film, blurred by the defocus (box blurs). A system (`gz-maritime-lens-drops-system`) inside a `<sensor type="camera">` fills the `Environment` each step from the sensor's world pose, the world's gravity, the link's velocity and the world's wind (`WindSampler`). It takes changes on `<sensor>/lens_drops/set` (`enabled`, `clear`, and `add_x`, `add_y`, `add_radius` for one drop) and reports `count`, `sliding`, `covered`, `flux`, the gravity, `draws`, `attached` and `drawing` on `<sensor>/lens_drops/state`, with an optional drop mask on `<sensor>/lens_drops/mask`. It names the camera and its topics on the first step that finds the sensor under the world, so a camera spawned at run time works too. On `SceneUpdate` it hands the camera a render pass from a dlopen'd Ogre Next bridge (`lens-drops-ogre2-bridge`, ABI version 4), first in the camera's passes. The camera owns the pass; before each frame the bridge asks the system whether the pass is wanted, then the pass calls the system back, which draws the map only if the drops changed. The bridge uploads it as an RG16F texture for a full screen pass whose GLSL shader refracts the camera ray into and out of the water by Snell's law and the camera's HFOV, with Fresnel loss into the dark housing. Plain cameras, ogre2 and OpenGL only; the GUI's view is not touched. |
| `gz_thruster/` | gz-sim's Thruster system, vendored with a normalized command mode (`<use_normalized_cmd>`: a command in [-1, 1] scaled onto the thrust limits), the interface every vehicle here is driven with; intended to go upstream. |
| `kai_gazebo/` | Worlds (`open_water.sdf` and the four VRX/VORC sites: `sydney_regatta` (and `sydney_regatta_lite`), `benderson_park` (and `benderson_park_lite`), `sand_island` (and `sand_island_lite`), `la_spezia` (and `la_spezia_lite`), the RobotX 2026 venue `marina_bay` (and `marina_bay_lite`), the same venue set up for the competition `robotx_2026` (its course elements and venue dressing from Fuel, its overcast water here), the indoor pool `aquatic_centre`, `acoustic_demo`, open water with three moored pingers, and `underwater_camera_demo`, a camera 3 m down in coastal water (3C) facing colour charts from 0.5 to 16 m; every world is named `default` and sets its water's optics, the site's Jerlov type), the landing pad, the moored `acoustic_pinger`, and each site's water model (the sites themselves from Fuel, `honurobotics`; see `NOTICE`), resource-path hooks and the world tests. |
| `kai_bringup/` | ROS 2 launch + `ros_gz_bridge` config. `simulation.launch.xml` is the simulation part only (server, GUI, `/clock`); `spawn_vehicle.launch.xml` puts one instance of any vehicle in, from its model xacro, bridge template and URDF (or its own generator), through `instantiate_vehicle.py`. Neither names a vehicle. |
| `kai_custom_vehicle/` | The example vehicle, the custom USV: URDF/xacro, the Gazebo model that marks its displacement and drives `gz_thruster`, the bridge template, an RViz config, and thin `sim`/`rviz`/`display` launches over `kai_bringup`. The docs' worked example; one package on purpose. |
| `docs/` | User documentation (Sphinx + MyST). Screenshots are regenerated by the scripts in `docs/_tools/` (Gazebo camera renders, and RViz on a virtual display). |
| `WAVES_DESIGN.md` | Full design + contributor reference for the wave packages. |

## Setup & build

ROS 2 Rolling + Gazebo Jetty (+ matching `ros_gz`). The workspace is `~/kai_ws`,
this repo is `~/kai_ws/src/gz-maritime`.

```bash
cd ~/kai_ws
colcon build --merge-install
source install/setup.bash
```

The FFT wave engine depends on **Ehukai**, an external system library
(installed separately, not vendored). It must be discoverable via
`CMAKE_PREFIX_PATH` (e.g. installed under `~/.local`) or
`gz_waves_provider_fft` will fail to configure.

## Run

```bash
ros2 launch kai_bringup simulation.launch.xml          # boots open_water.sdf with a moving ocean
ros2 launch kai_bringup simulation.launch.xml world:=sand_island.sdf   # a site world instead
ros2 launch kai_custom_vehicle sim.launch.xml          # the example vehicle on it
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=boat_b y:=4 \
    xacro:=$(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
    bridge:=$(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
    urdf:=$(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro   # one more, while it runs
gz sim -r src/gz-maritime/kai_gazebo/worlds/open_water.sdf  # world directly
```

A vehicle package includes `simulation.launch.xml` once and
`spawn_vehicle.launch.xml` once per instance
(`kai_custom_vehicle/launch/sim.launch.xml` is the template); one `name` per
instance drives the model name, topics, ROS namespace and TF prefix.

## Test — run before declaring work done

```bash
colcon test --packages-select <pkg> && colcon test-result --verbose
# or run a gtest binary directly:
./build/gz_waves/wave_core_test
./build/gz_waves_provider_gerstner/gerstner_test
./build/gz_waves_provider_fft/fft_test
# the drawn sea against the wave field, through a depth camera (needs a GPU):
colcon test --packages-select gz_waves_rendering
# marked-collision buoyancy, the normalized thruster, the spawn launch's
# instance script, and the example vehicle end to end (generation for two
# names, two boats through the launches, a command moves one only):
colcon test --packages-select gz_buoyancy gz_hydrodynamics gz_thruster kai_bringup kai_custom_vehicle
# the acoustics: the rules without a world, and the systems in test worlds
colcon test --packages-select gz_acoustics
# the GUI: the core gtests, the four plugins in an offscreen GUI, and a
# camera that must not see the panels' markers (needs a GPU, skips without)
colcon test --packages-select gz_maritime_gui
# the lens drops: the drop model, the system headless, and a camera that
# must show the drops (needs a GPU, skips without)
colcon test --packages-select gz_lens_drops
# the water's optics and the underwater camera, CPU and GPU (the rendered
# tests skip without a GPU; build with --packages-up-to, they need
# gz_waves_rendering):
colcon test --packages-select gz_water_optics gz_underwater_camera gz_underwater_rendering
# docs, strict (needs network for the requirements):
pip install -r docs/requirements.txt && sphinx-build -W docs docs/_build/html
```

Render-path changes cannot be verified without a GPU — they need a real
`gz sim -r`. State plainly when a change is build-only and GUI-unverified. One
part is covered by a test: `gz_waves_rendering`'s `drawn_surface_test` compares
the surface a depth camera sees with the wave field (it has to render too).

## Code style (Gazebo house style)

Match the surrounding code. For C++:

- **`_`-prefix every function/method parameter** (`void Foo(int _x)`); loop and
  local variables are exempt.
- A **50-slash separator** `//////////////////////////////////////////////////`
  before each out-of-line function/method definition **and** before each
  `TEST`/`TEST_F`.
- **Per-member access specifiers** — prefix every member/method with
  `public:` / `private:` / `protected:` (Gazebo style, not one block per group).
- **Doxygen** on declarations: `\brief` above each class/struct member (not a
  trailing `///<`), `\param`/`\return` on functions, `// Documentation inherited`
  on overrides.
- **Copyright header**: new files use the **Honu Robotics** Apache-2.0 header;
  when editing an existing file, keep its existing header.

## Commit & PR conventions

- Branch off the default branch; do not commit directly to it.
- Recommended one-line, imperative commit subject (`package: do the thing`).
- Sign off (`git commit --signoff`) and end the message with the trailer:
  `Co-Authored-By:`

## Critical invariants (do not regress these)

These are non-obvious and have each caused real bugs. See `WAVES_DESIGN.md` for
the full rationale (section refs below).

- **Core and renderer stay engine-agnostic.** `gz_waves` and
  `gz_waves_rendering` must not build-depend on, link, or include any concrete
  engine. Adding an engine must not modify them. (§2, §7)
- **The engine is never serialized.** Replication carries the *recipe* only;
  each consumer rebuilds its engine via `CreateWaveSimulation`. (§3.4)
- **`WaterVisual` owns a private engine.** Never alias the component's
  `simulation` into the renderer — the render thread and ECM thread race it and
  it segfaults. Build a private, mutex-guarded instance. (§5.1, N2)
- **GUI registrars load at the `<visual>` level.** gz-sim's GuiRunner loads
  system plugins only at `<visual>`; model/link/world-level never reach the GUI
  process → flat/white ocean. Put `gz-sim-waves-<engine>-gui` next to
  `WaterVisual` in the `water_surface` model's `<visual>`. (§5.2)
- **Ogre stays behind the dlopen'd C-ABI bridge.** Never link
  `gz-rendering-ogre2` into `WaterVisual` or `gz-maritime-lens-drops-system`
  — the resulting `DT_NEEDED` breaks gz-rendering's own engine loader
  (default-material plane / RTF collapse). Only the bridges link it, loaded
  with `dlopen`. (§5.3, N4) For the lens drops: create and attach the pass
  on the render thread (`SceneUpdate`); never connect to `PreRender`, which
  makes the sensors update the scene every step.
- **Consumers query via `Eval` free functions**, never via `IWaveField`
  directly; the engine type stays opaque. (§3.5, N3)
- **Vehicles mark the collisions that float them; the world names nobody.**
  `open_water.sdf` runs `gz-maritime-buoyancy-system` with
  `<enable_by_default>false</enable_by_default>` and no `<enable>` list. Never
  add a vehicle's links to a world: a model marks its displacement collisions
  `gz:buoyancy="true"` (with a zero `collide_bitmask`), on a link of their own,
  and keeps its contact geometry unmarked. `gz_buoyancy` carries only the
  upstream change plus a gz-math workaround; add nothing else to it, and retire
  it when a Gazebo release with gz-sim 229ec07e reaches ROS.
- **The world owns the wind.** Every world runs `gz-maritime-wind-system`,
  calm, with `<speed>` and `<direction>`, changed at run time with the
  `/world/default/wind/set` topic. A model marks the shapes the wind sees
  with `gz:wind="true"` (a marked buoyancy box can carry both marks) and
  never needs Gazebo's mass based `enable_wind`. A system that needs the wind asks
  `gz::sim::wind::WindAt` (or keeps a `WindSampler`) instead of reading the
  wind entity; do not add Gazebo's `WindEffects`, which would write the same
  wind.
- **The world owns the ocean current.** Every world runs
  `gz-maritime-ocean-current-system`, slack, with `<speed>` and
  `<direction>` (the direction it sets *towards*, the opposite of the wind),
  set in the world file and changed at run time on the
  `/world/default/ocean_current/set` topic. A system that asks the current
  every step, such as the hydrodynamics, keeps an `OceanCurrentSampler`;
  `gz::sim::ocean_current::OceanCurrentAt` builds the model on every call
  and is for an occasional query. Either way the query stays a point query. A new current
  model is a class registered under a name, never a change to the system or
  its consumers; nothing
  publishes on Gazebo's `/ocean_current` topic. A vehicle feels the current
  only through `gz-maritime-hydrodynamics-system`, with its own Fossen
  coefficients; gz-sim's `gz-sim-hydrodynamics-system` does not see it, and
  added mass goes in the plugin's coefficients, never `<fluid_added_mass>`,
  which only DART implements. A change on the topic is a
  step in the current; a vehicle whose hydrodynamics carries plugin added
  mass turns that step into a force spike, so keep added mass out of a
  vehicle that may see one, or add a ramp before relying on it.
- **The world owns the water's optics.** A world sets what its water does
  to light with `gz-maritime-water-optics-system` (a Jerlov type, a
  `<visibility>`, or explicit coefficients), changed at run time on the
  `/world/<world>/water_optics/set` topic. A camera or a renderer that sees
  through the water keeps a `WaterOpticsSampler` and never coefficients of
  its own. The renderer marks the world while it draws the water, so a
  reference camera never applies it twice. A sampler asked off the
  simulation thread (a renderer) answers from a wave engine of its own,
  never the one the simulation advances. Every `kai_gazebo` world runs the
  system with its site's Jerlov type, and the world tests require it.
- **The sonar worlds are walkthrough worlds, outside the world rules.** `gz_sonar/worlds/` (`sonar_imaging`, `sonar_survey`, `sonar_shipwreck`) exist only for the sonar how-tos, and copy the water world of the vehicle each one flies (`bluerobotics_models`' `bluerov2_water.sdf` and `blueboat_water.sdf`): gz-sim's graded buoyancy enabled by name on that vehicle's displacement link, the world named for its scene rather than `default`, and no wind system. They need the vehicles from `bluerobotics_models` and nothing in this repository launches them. Do not use them as a pattern or build on them; a world for general use goes in `kai_gazebo` and follows the rules above.
- **A hull feels the water through one model: Fossen or marks, never
  both.** A model with identified coefficients keeps its Hydrodynamics
  plugin and marks nothing for the current. A model without them marks the
  shapes the water sees with `gz:ocean_current="true"` (a buoyancy box can
  carry the buoyancy and wind marks too; the wind takes the part above the
  water, the current the part below) and carries no linear or heave
  damping: the marks damp more than surge and sway (the plan area damps
  heave, drag below the centre of mass rolls and pitches the hull, and a
  shape off the centre line yaws it), so plugin damping beside them counts
  those axes twice. A Hydrodynamics plugin with yaw terms only (`nR`,
  `nRabsR`) is the most a marked vehicle may add, and only when no marked
  shape is off the centre line. The drag acts at the centre of each wet
  part, so a single centred shape gives no yaw damping at all. The cut is
  at the flat `<water_level>`, not the wave surface, like the windage; so
  is the buoyancy's, except in a world that runs `gz_waves_buoyancy`,
  where the buoyancy alone follows the waves.
- **The GUI reads the world and writes only through topics.** The
  `OceanCurrentPanel` and `WindPanel` plugins read their recipe from the
  GUI's ECM (or, loaded from the plugin menu after the first state, from a
  private copy of the world's state service), never write into the GUI's
  ECM, and change a field only with a `gz.msgs.Param` on its `set` topic, so
  the server's rules stay the only rules. Each keeps its field's convention
  on screen: the wind's direction is where it comes from, the current's
  where it sets towards. A number field takes a point or a comma whatever
  the locale; the plugin parses the text (`ParseNumber`), never Qt's
  `DoubleValidator`, which takes only the locale's separator. A marker they
  draw has a non-zero id: MarkerManager gives id 0 a new random id, which
  would add a marker on every redraw. Gazebo's renderer ignores colours
  given point by point on a line list, so a coloured drawing is one marker
  per colour; a bin that empties is deleted, since a marker sent with no
  points keeps its old ones. Steering is a rate limit on the controls, so
  the arrows and trails turn smoothly: each `set` message is still a step
  in the recipe, so it spares a vehicle with plugin added mass nothing, and
  the rule above (keep added mass out of a vehicle that may see a step)
  stands. A panel's steering stops when the field moves to a value it did
  not send, so two controllers never take turns.
- **The panels draw through the GUI's marker service only.** Every panel
  sends its markers to `/marker` (`/marker_array`), which the GUI's
  MarkerManager serves; a camera sensor never sees them because the
  server's sensors take markers on `/sensors/marker` instead. The markers'
  `visibility: GUI` flag does not keep them out of a camera on its own: sent
  to `/sensors/marker` they show in the image. `markers_camera_test`
  (GPU, skips without one) holds both.
- **A pinger is on a link; a receiver is a sensor; the world names
  neither.** A pinger is the `gz-maritime-acoustic-pinger-system` plugin on
  the model that carries it, one per link, and moves with that link; never
  a position held by the receiving vehicle, as VRX did. A receiver is a
  custom sensor the acoustics system finds on any model; its rate is the
  pings', one reading per ping heard, so it has no `<update_rate>`. The
  system reads `<name>` and other plugin values from child elements: a
  plain `Get("name")` on a plugin element returns the plugin's own `name`
  attribute.
- **No instance name in a URDF, no `/clock` in a vehicle bridge.** The
  simulation launch bridges the clock once; a vehicle's `name` reaches its
  model topics and frame ids through xacro, its nodes through the namespace,
  and TF through `robot_state_publisher`'s `frame_prefix`.
- **One thruster interface.** Vehicles drive `gz-maritime-thruster-system` in
  normalized mode, a command in [-1, 1] on `/<name>/motor_*/cmd`; a docs
  example that sends newtons is wrong.

## Adding a new wave engine

A new engine is one self-contained `gz_waves_provider_<name>` package (engine +
source system + GUI registrar); the core, renderer, and consumers need **zero
changes**. Follow the step-by-step recipe in **`WAVES_DESIGN.md` §9**, using
`gz_waves_provider_gerstner/` as the template.

## Deeper docs

- `WAVES_DESIGN.md` — wave-package requirements, architecture, per-package detail,
  and the "add a new engine" guide.
- `docs/` — user-facing documentation, including the "bring your own vehicle"
  guides.
