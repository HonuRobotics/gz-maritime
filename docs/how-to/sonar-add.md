# Add a sonar to your vehicle

A sonar is a sensor on one of your vehicle's links, driven by one world system and brought into ROS 2 by one bridge. This page adds an imaging sonar; sidescan sonars work the same way with `gz:type="sidescan_sonar"`. Every parameter, with its default and range, is in the [design](../design/sonar.md#sdf).

## The sensor

```xml
<sensor name="sonar_forward" type="custom" gz:type="imaging_sonar">
  <pose>0.25 0 0 0 0.26 0</pose>
  <update_rate>15</update_rate>
  <topic>my_vehicle/sonar_forward/image</topic>
  <gz:sonar>
    <range_max>15</range_max>
  </gz:sonar>
</sensor>
```

Put it in a `<link>`; the sensor looks along its own x axis, so the pose points it (here forward, pitched down 15°). An empty `<gz:sonar/>` gives the Sonoptix ECHO; the model file needs `xmlns:gz="http://gazebosim.org/schema"` on its `<sdf>`.

## The world

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-maritime-sonar-system" name="gz::sim::maritime::Sonar">
  <temperature>29</temperature>
  <salinity>30</salinity>
</plugin>
```

The sonar draws its view on the Sensors system's rendering thread, and the sonar system owns the water; a world with no sonar pays nothing for it.

## The bridge

```bash
ros2 launch kai_sonar_bridge sonar_bridge.launch.xml \
  gz_topics:="[/my_vehicle/sonar_forward/image]" \
  ros_topics:="[/my_vehicle/sonar_forward/image]"
```

`ros_gz_bridge` cannot convert the sonar's message; this bridge publishes `marine_acoustic_msgs/ProjectedSonarImage`, and a `/view` image beside it.

## On the BlueROV2

```yaml
parts:
  - {type: sonoptix_echo_multibeam, name: sonar_forward, xyz: "0.25 0 0.12", rpy: "0 0.26 0",
     sonar: {range_max: 15}}
```

In a BlueROV2 config the part emits the sensor, `sonar:` fills its `<gz:sonar>`, and the launch starts the bridge; `bluerov2_sonar.yaml` in `bluerov2_description` is a complete example.

## Check

```bash
ros2 topic echo --once /my_vehicle/sonar_forward/image --field image.beam_count
```

256, the ECHO's beams.
