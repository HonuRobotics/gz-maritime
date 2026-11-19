# Find an acoustic pinger

RoboNation courses hide acoustic pingers under buoys: a RobotX, RoboBoat or
RoboSub vehicle listens with a hydrophone array and works out which buoy
pings. gz-maritime simulates the **behaviour** of that, not the acoustics:
pingers ping on a schedule at a frequency, and a receiver on your vehicle
reports the direction each ping it hears came from, after the delay of the
speed of sound, within a band and a reach, with noise. There is no
propagation model: no bending, no reverberation, no waveform.

Every world here runs the acoustics system, silent until a pinger shows up.
`marina_bay.sdf` and `marina_bay_lite.sdf` have the two pingers of the
RobotX 2026 survey task on course B, and `acoustic_demo.sdf` is a world
built to try them.

## Run the demo

```bash
ros2 launch kai_custom_vehicle acoustic_demo.launch.xml
```

The open water world with three moored buoys:

| Buoy | Where | Pinger |
|---|---|---|
| green | (40, 15) | `pinger_green`, 30 kHz, pinging |
| red | (40, -15) | `pinger_red`, 30 kHz, silent |
| yellow | (-30, 30) | `pinger_yellow`, 37 kHz, a decoy |

Two custom USVs, each with a receiver tuned to 30 kHz. `custom_usv` starts
at the origin and homes on what it hears (`pinger_homing.py`); `boat_b`, at
(-10, -20), only listens. Neither hears the yellow decoy. `homing:=false`,
`listener:=false` and `gazebo_gui:=false` turn the pieces off.

The same demo runs on the RobotX 2026 course, course B of Marina Bay, whose
two survey pingers sit about 35 m out from the course's start point:

```bash
ros2 launch kai_custom_vehicle acoustic_demo.launch.xml world:=marina_bay.sdf \
    x:=-15 y:=-8 yaw:=0.55 listener_x:=15 listener_y:=-28 listener_yaw:=0.55
```

Watch what a boat hears, and the ground truth:

```bash
ros2 topic echo /custom_usv/acoustic_receiver
ros2 topic echo /world/default/acoustics/pingers
```

Switch the course pinger, and `custom_usv` turns for the red buoy:

```bash
ros2 topic pub -1 /world/default/acoustics/pinger_green/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: active, value: {type: 1, bool_value: false}}]}"
ros2 topic pub -1 /world/default/acoustics/pinger_red/set ros_gz_interfaces/msg/ParamVec \
  "{params: [{name: active, value: {type: 1, bool_value: true}}]}"
```

## See them in the GUI

Every world docks an **Acoustics** panel on the right of the Gazebo GUI.
It lists the pingers, each with a switch to turn it on or off, and the
receivers, each with its last reading and a field to retune it in kHz. It
draws, each with its own check box:

- **Reach:** where each pinger's reach meets the water, grey and dashed
  when the pinger is off, and a lighter dashed ring inside it where the
  receivers start to lose pings;
- **Pings spreading:** a ring per ping, slowed down 50 times by default so
  the eye can follow it;
- **Bearings heard:** a ray from each receiver along the bearing it last
  reported, from where the receiver was when it heard it, so the ray stays
  on the pinger while the boat turns; red when the reading was invalid.

The reach and the pings are drawn on the water by default, where a boat's
hydrophone listens. For a vehicle under the water, pick **a depth** or **a
receiver's depth** in the panel: the circles are then cut at that depth, or
at the receiver's, following it, which is the slice the vehicle really
hears in. A pinger 10 m down with a 20 m reach covers 17 m around it at the
surface but the full 20 m at its own depth. Pick **spheres, in 3D** to see
the whole of it: the reach as a wireframe sphere and each ping as a sphere
growing out of the pinger, cut at the water. Drawn under the water, they are
behind the animated sea: look from an underwater camera, or hide the water.

Colours go with frequency, from orange at 25 kHz to violet at 40 kHz, so the
37 kHz decoy is a different colour from the course pingers. The demo world
draws all three; every other world starts with the drawing off. The
drawings are the GUI's only: a camera on a vehicle does not see them.

## Put a pinger in a world

`kai_gazebo` ships `model://acoustic_pinger`: a spar buoy with the pinger
1.5 m under it, 30 kHz once a second with a 300 m reach. The pinger takes
the name the model is included under, which scopes its `set` topic.
Change its settings, or the buoy's colour, per include:

```{literalinclude} ../../kai_gazebo/worlds/acoustic_demo.sdf
:language: xml
:start-at: <!-- The red buoy
:end-at: </include>
```

`experimental:params` needs SDFormat 1.9 or later, and a world that is also
read by plain XML tools should declare the prefix on its root,
`xmlns:experimental="http://sdformat.org/schemas/experimental"`.

A pinger can also go on any link of your own model, a buoy, a mooring or a
vehicle, with the plugin directly:

```xml
<plugin filename="gz-maritime-acoustic-pinger-system"
        name="gz::sim::maritime::AcousticPinger">
  <name>my_pinger</name>           <!-- default: the model's name -->
  <link_name>link</link_name>      <!-- default: the canonical link -->
  <pose>0 0 -1.5 0 0 0</pose>      <!-- in the link frame -->
  <frequency>30000</frequency>     <!-- Hz -->
  <period>1</period>               <!-- s between pings -->
  <offset>0</offset>               <!-- s into the period -->
  <reach>300</reach>               <!-- m; 0 is no limit -->
  <active>true</active>
</plugin>
```

Pingers on one period with different offsets ping in turn, as the
RoboNation rules have it. A pinger above the water sends nothing.

## Give your vehicle a receiver

The receiver follows the [sensor pattern](sensors.md#the-pattern): a frame
in the URDF, a tiny link in the model, a `<frame_id>` with the instance
name and a bridge entry. The custom USV hangs its hydrophone on a pole,
0.3 m below the waterline (its `base_link` is on the waterline), so it
stays in the water as the boat rolls:

```{literalinclude} ../../kai_custom_vehicle/models/custom_usv/model.sdf.xacro
:language: xml
:start-at: <xacro:sensor_link sensor_name="acoustic_receiver"
:end-at: </xacro:sensor_link>
```

It publishes one message per ping it hears, a `gz.msgs.Param` bridged to
`ros_gz_interfaces/msg/ParamVec`, with these parameters:

| Name | What |
|---|---|
| `bearing` | rad, counter clockwise from the sensor's x |
| `elevation` | rad, positive up; a pinger below gives a negative value |
| `frequency` | Hz of the ping |
| `amplitude` | dB relative to 1 m, -20 log10 of the distance, with the receiver's `<amplitude_noise>` |
| `valid` | false when another ping in band arrived while an earlier one still sounded |
| `range` | m, only with `<range>true</range>` |
| `emit_time` | s of simulation time the pinger sent it, only with `<emit_time>true</emit_time>` |

The stamp of a reading is when the ping arrived: when it was sent plus the
distance over the speed of sound.

A passive receiver hears a direction, not a distance, but four things give
the distance away:

- **`range` and `emit_time`.** Both are off unless asked for.
- **The stamp.** It is the exact arrival, and a pinger pings on a known
  schedule, so the stamp less the time the ping was sent is the distance
  over the speed of sound. A real receiver in step with the pinger can do
  the same, so this one stays.
- **The amplitude.** Without noise, 10^(-amplitude / 20) is the distance
  exactly. Noise around zero only blurs it, since it averages out over a few
  pings. What hides the range is a bias, which stands for a source level the
  receiver does not know: a `<bias_stddev>` in its `<amplitude_noise>`, with
  some `<stddev>`. A receiver with `<range>` off and neither spread nor bias
  on its amplitude warns when it loads.

The custom USV's receiver has 1 dB of amplitude noise and no bias, on
purpose. `pinger_homing.py` stops when the median of the last three
amplitudes is louder than -16 dB, which is a distance only with a known
source level. A receiver that must hide its range needs the bias and
another way to tell it is close, such as the elevation, which steepens as
the boat closes on a pinger that hangs deeper than the hydrophone.

Bridge the readings and the retuning topic:

```{literalinclude} ../../kai_custom_vehicle/config/ros_gz_bridge.yaml.in
:language: yaml
:start-at: "# Acoustic receiver"
:end-before: "- ros_topic_name: /@name@/navsat"
```

A receiver has no `<update_rate>`: it reports when a ping arrives. A ping is
heard when both ends are under the water, the receiver is within the
pinger's reach, the frequency is in the receiver's band, it comes from
inside the receiver's field of view (`<min_elevation>` to `<max_elevation>`,
every direction by default), and it isn't lost to the dropout near the
reach. The water is the flat `<water_level>`, not the waves. The full list
of tags is in the
[gz_acoustics README](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/gz_acoustics/README.md).

## Coming from VRX

VRX's `AcousticPingerPlugin` kept the pinger as a position inside the boat's
plugin and published range, bearing and elevation at a fixed rate. Here:

- the pinger is a model, or a plugin on any link, and moves with it; there
  can be any number, and every vehicle hears the same ones;
- a reading comes per ping, with the ping's frequency, after the travel
  time;
- `bearing` and `elevation` keep their names and meaning, measured from the
  sensor's own frame; `range` is there only when the receiver asks for it,
  since a passive receiver cannot measure it;
- a pinger is switched on or off, and its frequency, period, offset, reach
  and pulse length changed, through its `set` topic. It has no position to
  publish: it moves with its model.

Next: [Use your own world](own-world.md).
