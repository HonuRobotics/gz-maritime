# Imaging (forward-looking) sonar from the ROV

A BlueROV2 with a forward imaging sonar (a Sonoptix ECHO) hovers 2.5 m above a sandy seabed, with a glass float, a steel cylinder, a concrete block and a tire ahead of it. This page shows what the sonar sees. The model behind it is in the [sonar simulation design](../design/sonar.md).

```{figure} images/sonar-imaging-scene.svg
:alt: Side and plan views of the scene: a BlueROV2 2.5 m down with its imaging sonar pitched 15° down, the 120° × 20° beam out to 15 m, and a glass float, steel cylinder, concrete block and tire on the seabed 6.5 to 9.5 m ahead

The scene. (a) From the side: the sonar's 20° beam, pitched down 15°, meets the seabed from 6.2 m slant range, and the seabed shows faintly from 3.7 m, below the beam's edge. (b) From above: the 120° fan and the four targets in it; the flag is transparent to the sonar.
```

```{figure} images/sonar-imaging-gazebo.png
:alt: Gazebo view from behind and to the side of the ROV: the BlueROV2 on the left facing a concrete block, a steel cylinder, a tire and a glass float on the seabed, with a marker flag standing among them

The same scene in Gazebo, from a camera behind the ROV and to its starboard side: the ROV on the left, facing the block, the cylinder, the tire and the glass float, with the flag among them.
```

Run everything inside drydock, from `~/maritime_ws`, with the workspace sourced.

## Launch

```bash
ros2 launch bluerov2_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share gz_sonar)/worlds/sonar_imaging.sdf \
  config_file:=$(ros2 pkg prefix --share bluerov2_description)/config/bluerov2_sonar.yaml
```

The world loads the sonar system and names its seabed; the config fits the ECHO looking forward, pitched down 15°, with a 15 m range.

## Watch the sonar

```bash
ros2 run rqt_image_view rqt_image_view /bluerov2/sonar_forward/image/view
```

`/view` is the sonar's frame drawn as the fan it covers, forward up, for looking at; the sonar data itself is `/bluerov2/sonar_forward/image`, a `marine_acoustic_msgs/ProjectedSonarImage`.

```{figure} images/sonar-imaging.png
:alt: Sonar fan: seabed from about 4 m, with the targets as small highlights and shadows near the middle

The fan, forward up: nothing close in, then the seabed from about 4 m, where the rays first reach it below the vertical beam's edge, and the four targets as bright returns, each with a shadow behind it. The time-varied gain lifts the near seabed, so the beam's edge shows little in the image.
```

## Turn the ROV

```bash
ros2 topic pub /bluerov2/thruster_1/cmd std_msgs/msg/Float64 "data: -0.15" -1 &
ros2 topic pub /bluerov2/thruster_2/cmd std_msgs/msg/Float64 "data: 0.15" -1 &
ros2 topic pub /bluerov2/thruster_3/cmd std_msgs/msg/Float64 "data: 0.15" -1 &
ros2 topic pub /bluerov2/thruster_4/cmd std_msgs/msg/Float64 "data: -0.15" -1 &
```

A turn to port, so the targets slide across the fan and out of it; publish `data: 0.0` to the same four to stop.

```{figure} images/sonar-imaging-turn.webp
:alt: Animation, side by side: on the left the Gazebo view of the ROV turning to port, on the right its sonar fan, where the targets and their shadows slide to starboard and leave the fan

The turn from both sides: the ROV in Gazebo on the left, its sonar fan on the right, played at about half speed. As the ROV turns to port, the targets slide to starboard across the fan and out of it.
```

