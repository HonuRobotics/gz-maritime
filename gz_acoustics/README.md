# gz_acoustics

Underwater acoustics for Gazebo Sim as behaviour, not physics: pingers that
ping on a schedule at a frequency, and receivers that report the direction
each ping they hear came from. There is no propagation model: no ray
bending, no sound speed profile, no reverberation, no waveform. What is
modelled is the geometry, the delay at the speed of sound and a few rules
that each can be switched off.

It continues VRX's `AcousticPingerPlugin`, which kept the pinger as a
position inside the receiving boat's plugin. Here a pinger is a thing in
the world, on a link, and a receiver is a sensor on a link.

Three plugins:

| Library | Kind | What |
|---|---|---|
| `gz-maritime-acoustic-pinger-system` | model | Puts a pinger on one of the model's links. |
| `gz-maritime-acoustics-system` | world | Makes the pingers ping and reads the receivers. |
| (the same library) | custom sensor | `<sensor type="custom" gz:type="acoustic_receiver">` on any link. |

The world names nobody: the acoustics system finds pingers and receivers on
every model, a model spawned while the world runs included.

## A pinger

```xml
<model name="buoy_green">
  <static>true</static>
  <link name="link"/>
  <plugin filename="gz-maritime-acoustic-pinger-system"
          name="gz::sim::maritime::AcousticPinger">
    <name>pinger_green</name>        <!-- default: the model's name -->
    <link_name>link</link_name>      <!-- default: the canonical link -->
    <pose>0 0 -1.5 0 0 0</pose>      <!-- in the link frame -->
    <frequency>30000</frequency>     <!-- Hz -->
    <period>1</period>               <!-- s between pings -->
    <offset>0</offset>               <!-- s into the period -->
    <reach>300</reach>               <!-- m; 0 is no limit -->
    <pulse_length>0.004</pulse_length>  <!-- s; 0 turns overlap off -->
    <active>true</active>
  </plugin>
</model>
```

The pinger moves with its link. It pings at its offset and then every
period, on simulation time, from where it is at that moment. A pinger above
the water sends nothing. Pingers on one period with different offsets ping
in turn, as the RoboNation rules have it.

Change any of it while the world runs on
`/world/<world>/acoustics/<name>/set`, a `gz.msgs.Param` whose keys are the
names above (`active` takes a boolean or a number):

```bash
gz topic -t /world/default/acoustics/pinger_green/set -m gz.msgs.Param \
  -p 'params { key: "active" value { type: BOOLEAN bool_value: false } }'
```

## A receiver

```xml
<sensor name="acoustic_receiver" type="custom" gz:type="acoustic_receiver">
  <frame_id>my_boat/hydrophone_link</frame_id>
  <topic>my_boat/acoustic_receiver</topic>
  <gz:acoustic_receiver>
    <frequency>30000</frequency>    <!-- Hz, band centre; 0 hears all -->
    <bandwidth>1000</bandwidth>     <!-- Hz, full width -->
    <range>false</range>            <!-- report the slant range too -->
    <bearing_noise><noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise></bearing_noise>
    <elevation_noise><noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise></elevation_noise>
    <range_noise><noise type="gaussian"><mean>0</mean><stddev>1</stddev></noise></range_noise>
    <outlier_probability>0.02</outlier_probability>  <!-- random bearing -->
    <dropout_start>0.8</dropout_start>  <!-- fraction of a pinger's reach -->
  </gz:acoustic_receiver>
</sensor>
```

It publishes one `gz.msgs.Param` per ping it hears on its `<topic>`, with
its `<frame_id>` in the header and the arrival time as the stamp. Bridge it
to `ros_gz_interfaces/msg/ParamVec`. Its keys:

| Key | Type | What |
|---|---|---|
| `bearing` | double | rad, counter clockwise from the sensor's x, in (-pi, pi] |
| `elevation` | double | rad, positive up; a pinger below gives a negative value |
| `frequency` | double | Hz of the ping |
| `amplitude` | double | dB relative to 1 m: -20 log10 of the distance |
| `valid` | bool | false when another ping in band arrived on top of it |
| `emit_time` | double | s of simulation time the pinger sent it |
| `range` | double | m, only with `<range>true</range>` |

`bearing`, `elevation` and `range` are VRX's key names, so a VRX client
reads them unchanged. There is no `<update_rate>`: the rate is the pings'.
Retune it on `<topic>/set` with `frequency` and `bandwidth`.

## Who hears what

A ping reaches a receiver after the distance over the speed of sound, at
the receiver's position then. The receiver hears it when:

1. both ends are below `<water_level>` (`<underwater_only>`),
2. the distance is within the pinger's `<reach>`,
3. the frequency is in the receiver's band,
4. it is not lost to the receiver's dropout, whose chance rises in a
   straight line from `<dropout_start>` of the reach to certain at the
   reach.

The reading then gets the receiver's noise, per channel, and with
`<outlier_probability>` a bearing drawn at random. Two pings in the band
that arrive closer than the pulse length both come out invalid. A
receiver's readings wait one pulse length before they are published, for
that check.

## The world system

```xml
<plugin filename="gz-maritime-acoustics-system"
        name="gz::sim::maritime::Acoustics">
  <speed_of_sound>1500</speed_of_sound>   <!-- m/s; 0 delivers at once -->
  <water_level>0</water_level>
  <underwater_only>true</underwater_only>
  <seed>0</seed>                          <!-- 0 draws a new one -->
  <publish_rate>1</publish_rate>          <!-- Hz, ground truth -->
</plugin>
```

It publishes every pinger on `/world/<world>/acoustics/pingers`: one
`gz.msgs.Param` with a child per pinger, keys `name`, `x`, `y`, `z` (world,
m), `frequency`, `period`, `offset`, `reach`, `pulse_length` and `active`.
Bridged to `ParamVec`, the keys arrive as `child_<i>/<key>`.

At the same rate it lists the receivers on `/world/<world>/acoustics/receivers`,
a child per receiver with `name` (scoped, from the world), `topic`,
`frame_id`, `frequency`, `bandwidth` and `dropout_start`: the GUI's copy of
a custom sensor loses its `gz:type`, so this is how a GUI finds them.

Each receiver draws its dropouts and outliers from its own generator,
seeded from `<seed>` and its scoped name. The noise blocks draw from
gz-math's generator, which a non-zero `<seed>` also seeds.

## Compared with VRX

| | VRX `AcousticPingerPlugin` | gz_acoustics |
|---|---|---|
| Pinger | a point inside the boat's plugin | a plugin on any link, moves with it |
| Pingers | one per boat plugin | any number, ground truth for all |
| Output | sampled at `update_rate` | one reading per ping |
| Frequency, schedule | none | frequency, period, offset |
| Range | always | optional |
| Frame | model pose relative to parent | the sensor's world pose |
| Delay | none | distance / speed of sound |
| Reach, band, water, overlap | none | each a rule |
| Message | `Param`, header grows each publish | `Param`, one header entry |

Not yet: occlusion by hulls or terrain, time differences per hydrophone,
transponders (USBL), and drawing pingers in the GUI.

## Tests

```bash
colcon test --packages-select gz_acoustics && colcon test-result --verbose
```

`acoustics_core_test` checks the schedule, the geometry, the rules and the
serialization without a world; `acoustics_test` runs the systems in
`test/worlds`: one reading per ping from the right bearing and after the
right delay, band, reach and water, the set topics, the ground truth, a
receiver spawned at run time, and two pingers in turn and then on top of
each other.
