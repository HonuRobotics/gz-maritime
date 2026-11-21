# First look: imaging sonar from the ROV

A BlueROV2 with a forward imaging sonar (a Sonoptix ECHO) hovers 2.5 m above a sandy seabed, with a glass float, a steel cylinder, a concrete block and a tire ahead of it. This page shows what the sonar sees. The model behind it is in the [sonar simulation design](../design/sonar.md).

Run everything inside drydock, from `~/maritime_ws`, with the workspace sourced.

## Launch

```bash
ros2 launch bluerov2_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share gz_sonar)/worlds/sonar_first_look.sdf \
  config_file:=$(ros2 pkg prefix --share bluerov2_description)/config/bluerov2_sonar.yaml
```

The world loads the sonar system and names its seabed; the config fits the ECHO looking forward, pitched down 15°, with a 15 m range.

## Watch the sonar

```bash
ros2 run rqt_image_view rqt_image_view /bluerov2/sonar_forward/image/view
```

`/view` is the sonar's frame drawn as the fan it covers, forward up, for looking at; the sonar data itself is `/bluerov2/sonar_forward/image`, a `marine_acoustic_msgs/ProjectedSonarImage`.

```{figure} images/sonar-first-look.png
:alt: Sonar fan: seabed fading in from about 4 m and bright from about 5 m, with the targets as small highlights and shadows near the middle

The fan, forward up: nothing close in, then the seabed, faint where only the edge of the vertical beam reaches it and full where the main beam does, and the targets as bright returns with shadows behind them.
```

## Turn the ROV

```bash
ros2 topic pub /bluerov2/thruster_1/cmd std_msgs/msg/Float64 "data: -0.15" -1 &
ros2 topic pub /bluerov2/thruster_2/cmd std_msgs/msg/Float64 "data: 0.15" -1 &
ros2 topic pub /bluerov2/thruster_3/cmd std_msgs/msg/Float64 "data: 0.15" -1 &
ros2 topic pub /bluerov2/thruster_4/cmd std_msgs/msg/Float64 "data: -0.15" -1 &
```

A slow turn to port, so the targets slide across the fan; publish `data: 0.0` to the same four to stop.

## Check

The targets and their shadows sit in the fan at the ranges and bearings the world places them, about 6.5 to 9.5 m ahead, and slide across the fan and out of it as the ROV turns. To read one ping's numbers:

```bash
ros2 topic echo --once /bluerov2/sonar_forward/image --field ping_info
```
