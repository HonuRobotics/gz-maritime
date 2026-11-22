# Profiling and sidescan sonar from the USV

The example demonstrates the Blueboat with both 

* downward looking profiling multibeam sonar for bathymetric mapping - sensor parameters to match a Cerulean Surveyor 240-16
* a sidescan sonar - sensor parameters to match a Cerulean Omniscan 450 SS

In the demo, the USV moves  one straight pass over a gently undulating sandy seabed about 8 m down, past a steel drum under its track, a concrete block to port and a glass float to starboard. This page shows what each sonar sees on the same pass. The sonar sensor model is in the [sonar simulation design](../design/sonar.md).

```{figure} images/sonar-survey-scene.svg
:alt: Section and plan views of the survey: a BlueBoat on 8 m of water with a sidescan beam to each side and a downward profiler fan, and a steel drum under the track, a concrete block 12 m to port and a glass float 10 m to starboard

The scene. (a) A section across the track: the sidescan's beam on each side (shaded, its axis dotted 15° down, its lower edge dashed) leaves a nadir gap under the boat, which the profiler's 80° fan of soundings covers; between the two, from about 6.7 to 9.5 m out, neither reaches the seabed within its −3 dB beam. (b) From above: the track, the sidescan swaths, the profiler's swath and the three targets.
```

```{figure} images/sonar-survey-gazebo.png
:alt: Gazebo view from under the water beside the track: the BlueBoat on the surface at the left, and on the seabed the steel drum, the concrete block in the distance and the glass float on the right

The same scene in Gazebo, from a camera under the water beside the track: the boat on the surface, the drum under its track, the block beyond it and the float nearer the camera.
```

Waves do not move the boat yet: gz-maritime's waves are drawn but not coupled to buoyancy, so the boat runs on flat water.

Run everything inside drydock, from `~/maritime_ws`, with the workspace sourced.

## Launch

```bash
ros2 launch blueboat_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share gz_sonar)/worlds/sonar_survey.sdf \
  config_file:=$(ros2 pkg prefix --share blueboat_description)/config/blueboat_sonar.yaml \
  x:=-15
```

The world loads the sonar system and names its seabed; the config fits the sidescan and the profiler under the middle of the boat, and `x:=-15` starts the boat 25 m before the drum.

## Watch the sonars

| Topic | What it is |
|---|---|
| `/blueboat/sidescan/port/image`, `/blueboat/sidescan/stbd/image` | Each side's line per ping, `marine_acoustic_msgs/ProjectedSonarImage`, levels in dB with time-varied gain |
| `/blueboat/sidescan/port/image/view`, `.../stbd/image/view` | Each side's waterfall, for looking at |
| `/blueboat/profiler/detections` | The profiler's soundings, `marine_acoustic_msgs/SonarDetections`: a travel time and an angle each |
| `/blueboat/profiler/detections/points` | The same soundings as a point cloud in the profiler's frame |

```bash
ros2 run rqt_image_view rqt_image_view /blueboat/sidescan/port/image/view
```

For the soundings, add a PointCloud2 display of `/blueboat/profiler/detections/points` in RViz with the fixed frame `blueboat/base_link`.

## Drive the pass

```bash
ros2 run gz_sonar survey_lawnmower.py sonar_survey --x0 -15 --x1 40 --y0 0 --y1 0
```

`survey_lawnmower.py` drives the boat along survey lines on its two propellers, holding each line with a heading and cross-track controller; one line, from x = −15 to 40 along y = 0, is this pass, at about 1.8 m/s, and it stops the propellers at the end. Without it, both propellers at the same command (`ros2 topic pub` on `/blueboat/prop_port/cmd` and `/blueboat/prop_stbd/cmd`) also drive the boat straight ahead, but started a moment apart they can leave it a few degrees off its heading, and the targets then appear at slightly different ranges.

```{figure} images/sonar-survey-pass.webp
:alt: Animation: above, the Gazebo view of the boat crossing the scene; below, the port waterfall on the left and the starboard waterfall on the right, growing as the boat moves, the block appearing on the port side

The pass from both sides, played at about twice its speed: the boat in Gazebo above, and below it the two waterfalls, port on the left and starboard on the right, newest line at the top.
```

## What to look for

```{figure} images/sonar-survey-waterfall.png
:alt: The whole sidescan waterfall of the pass: a dark nadir band down the middle that follows the swells, the seabed's swells as broad light and dark bands on each side, the concrete block on the port side at 20 m along track, and the glass float faintly to starboard at 30 m

The sidescan, the whole pass. Nothing returns before the seabed, so the nadir gap down the middle is the seabed's depth under the boat, wider where the water is deeper; the swells show as bands of brighter and darker seabed, brighter where a slope faces the sonar. The drum is under the track, in the nadir gap, where the sidescan cannot see it.
```

```{figure} images/sonar-survey-targets.png
:alt: Two close-ups of the waterfall: on the port side a bright return at about 14 m slant range with a dark shadow beyond it, and on the starboard side a faint return at about 12.8 m with a small shadow

Close-ups. The concrete block, 12 m to port, returns brightly from the face toward the boat and casts a shadow beyond it. The glass float, 10 m to starboard, returns weakly: glass is smooth, and its glint is seen only head on, which a sidescan never sees. Its shadow shows where it is.
```

```{figure} images/sonar-survey-soundings.png
:alt: Above, the profiler's soundings from above, colored by height, showing the seabed's swells across a 13 m wide swath; below, the soundings along the track on top of the true seabed, following it and rising over the drum

The profiler's soundings, placed in the world with the boat's pose. They follow the seabed to within 0.1 m (95 % of them), and rise by about 0.6 m, the drum's height, over the drum, the only target under the track.
```

## Check

- Each sidescan target shows on its own side, at the range its position gives, with a shadow beyond it.
- The soundings trace the seabed's swells and rise over the drum.
- The sidescan's levels carry its time-varied gain and gain, which the gz-transport message (`/blueboat/sidescan/port/image` in Gazebo) carries per sample and per ping: subtracting both gives back the received level that `tvg: none` publishes, to within 0.01 dB for echoes well above the noise. The ROS message has no field for them; for received levels on the ROS side, set `tvg: none` and a fixed `gain`.
