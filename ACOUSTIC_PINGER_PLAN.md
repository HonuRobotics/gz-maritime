# Acoustic pinger plan

Issue #20. Built and landing as a series of pull requests:

1. `gz_acoustics`: sections 1 to 7, the pinger, the receiver, the hearing
   rules, the `set` topics and the ground truth (this file comes with it);
2. the worlds of section 8: the acoustics system in every world, the
   `acoustic_pinger` model, a demo world and the `marina_bay` pingers;
3. the custom USV's receiver and the demo launch and test of section 8;
4. the GUI panel of section 9, drawn on the water;
5. the panel's other planes, the 3D view, the dropout ring and the fading
   rays;
6. the docs of section 8.

A richer uncertainty model (noise growing with range, reported standard
deviations) was dropped from this round. Left after the series: the "why
not" label and phase 4.

## Scope and success

We want pingers that a world or a model declares, that ping the way the
RoboNation beacons do (a frequency, a pulse rate, one active at a time), and
a receiver that any vehicle mounts on any link and that reports each ping it
hears as a bearing in its own frame. The demo: the custom USV finds which
of two buoys carries the active pinger, a second boat hears the same pings
from its own position, and the pinger is switched while the simulation
runs, in `acoustic_demo.sdf` and on course B of `marina_bay`. A headless
test runs that demo in both (the second once its Fuel site is cached).

Behaviour only. Out of scope for good: wave propagation, ray bending, sound
speed profiles, reverberation, raw waveforms. Acoustic data links are out
too: gz-sim already ships `AcousticComms`.

## What exists (survey)

Sources read at HEAD in November 2026; items marked (docs) were not read as
code.

| Project | Licence | What it gives | Worth taking |
|---|---|---|---|
| VRX `AcousticPingerPlugin` | Apache-2 | Range, bearing, elevation to a point held inside the receiver's plugin, sampled at a rate, gz-sensors noise | The baseline, the noise blocks, the key names |
| gz-sim `AcousticComms` | Apache-2 | Packet delivery with time of flight, max range, loss from range | The delivery loop: latch the emit position, deliver when the sound has covered the distance |
| LRAUV `RangeBearingPlugin` | Apache-2 | Two way ranging over the comms model, turnaround delay, a named link | Interrogate and reply, if we do transponders |
| DAVE USBL (IOES-Lab, `ros2`) | Apache-2 | Transceiver and transponders, common or individual interrogation, ROS inside the plugin | The vocabulary only: its delay is a blocking wall clock `sleep` |
| UF-MIL `mil_passive_sonar_gazebo`, NaviGator `pinger_plugin` | MIT | The pinger is a model; bearing only, with frequency, amplitude and a valid flag; a pinger that jumps to a random gate | Pinger as an entity, bearing only output, the gate switch |
| RWTH `usbl-simulator` | BSD-3 | Noise on times (round trip, TDOA per hydrophone) then a solved fix, fix loss growing with range, seeds, a switch per effect | Range dependent loss, seeding, TDOA output |
| Stonefish USBL and modem | GPL-3 | Autoping, time of flight, max range, vertical field of view, occlusion by ray, quantised output | Ideas only, no code |
| MOOS `uFldBeaconRangeSensor` | GPL-3 | Many beacons, fixed or random interval, two sided reach, rate limited requests, a ground truth report | Ideas only, no code |
| HoloOcean `AcousticBeaconSensor` (docs) | MIT + Unreal | SeaTrac style message types, delay, max distance with a sigma, line of sight | The soft range limit |

Nothing in gz-sim or gz-sensors is a pinger or a hydrophone. `uuv_simulator`,
Plankton and COLA2 publish the true position plus noise and call it USBL;
`ds_sim`, OceanSim and UNav-Sim have nothing.

The real beacons (RobotX 2024 and 2026, RoboSub 2024, RoboBoat 2023
handbooks): Benthos ALP-365, 25 to 40 kHz in 0.5 kHz steps, 0.5 to 2 pulses
per second, active beacons at least 2 kHz apart and pulsing in turn, one
active per course, frequency and rate changed from day to day. RobotX 2026
hangs a pinger under each of two buoys and only one pings. Pulse length and
source level: not found in a primary source.

## 1. The pinger is a thing in the world

**Problem.** In VRX the pinger is a position stored inside the plugin on the
boat. Each boat has its own copy, and a scoring plugin has to keep them in
step over a topic.

**Example.** Two boats in one world can disagree about where the pinger is,
a pinger hung under a buoy does not move with the buoy, and nothing can list
the pingers in the world.

**Solution.** A pinger is declared on a link of any model: a buoy, a
standalone `acoustic_pinger` model, or a vehicle. A small model plugin,
`gz-maritime-acoustic-pinger-system`, writes an `AcousticPinger` component
on that link: frequency, ping period, offset inside the period, reach, and
whether it is active. Its position is the link's world pose, so it follows
whatever carries it. New package `gz_acoustics`, built like `gz_wind`.

## 2. Pings, not samples

**Problem.** VRX samples the geometry at an `update_rate`. There is no ping,
no frequency and no way to tell two pingers apart.

**Example.** A team cannot rehearse "listen at 27 kHz, ignore the 33 kHz
beacon on the next course", and a 1 Hz pinger and a 2 Hz pinger look the
same.

**Solution.** A pinger emits at its period and offset, on simulation time.
A receiver produces one reading per ping it hears, carrying the ping's
frequency. Two pingers on one period with different offsets ping in turn,
which is the handbook's rule. No `update_rate` on the receiver: its rate is
the pingers'.

## 3. A receiver any vehicle mounts

**Problem.** VRX reads the model's pose relative to its parent, ignores the
hydrophone link, and labels the reading with the pinger's name.

**Example.** A hydrophone at the stern of a 5 m boat reports the bearing
from the model origin, and a nested model gets a wrong bearing altogether.

**Solution.** A custom sensor, like the anemometer:
`<sensor type="custom" gz:type="acoustic_receiver">` on any link, with
`<topic>` and `<frame_id>`. A world system, `gz-maritime-acoustics-system`,
finds receivers and pingers on every model whenever they show up, uses world
poses, and reports azimuth (counter clockwise from the sensor's x) and
elevation (up positive) in the sensor frame. The world names nobody.

## 4. Bearing first, range as an option

**Problem.** VRX reports a range. A passive array listening to a free
running pinger measures a direction, not a distance; VRX's own header says
it assumes some mechanism for it.

**Example.** A team that tunes against a 3 m range reading arrives at the
course with hardware that cannot give one.

**Solution.** The reading is bearing, elevation, frequency, amplitude and a
valid flag. `<range>true</range>` adds the slant range for VRX
compatibility and for vehicles that carry a ranging device. The amplitude
falls with distance, so a team can still tell near from far; it takes a
noise block, since without noise it gives the range back exactly. The time
the ping was sent, which with the stamp also gives the range, is reported
only with `<emit_time>true</emit_time>`.

## 5. What a receiver can and cannot hear

**Problem.** VRX always hears the pinger: at any distance, in any band, out
of the water, perfectly on time.

**Example.** A boat 2 km away gets a clean bearing; so does a hydrophone
lifted clear of the water.

**Solution.** Cheap rules, each one switchable and off in a VRX compatible
setup:

- **Band.** The receiver has a centre frequency and a bandwidth, settable
  on a topic; a ping outside it is not reported.
- **Reach.** Beyond the pinger's reach nothing is heard; near the limit
  pings drop with a probability that grows with range.
- **Water.** Both ends must be below the world's `<water_level>`.
- **Delay.** The ping arrives after distance over the speed of sound
  (1500 m/s, set on the world system), from where the pinger was when it
  emitted; the reading carries both times.
- **Overlap.** Two pings in the receiver's band that arrive inside one pulse
  length give an invalid reading.
- **Outliers.** A small probability of a wrong bearing, standing in for
  multipath.

Noise stays the `<noise>` block of a Gazebo sensor per channel, as in VRX,
through gz-sensors' noise models, with the bearing wrapped and the range
kept positive; it repeats with `gz sim --seed`, as every sensor's does. The
dropouts and outliers come from each receiver's own generator, seeded from
`<seed>`, so that seed changes no other sensor.

Occlusion by hulls or terrain is left out of this phase: gz-sim gives a
system no cheap ray query, and it needs its own look.

## 6. Switching pingers at run time

**Problem.** VRX moves its single pinger with a `Vector3d` on a topic, and
the choice of gate lives in a scoring plugin.

**Example.** "Only the pinger under the green buoy pings" needs custom code
per task.

**Solution.** Each pinger takes a `gz.msgs.Param` on its `set` topic, like
the wind: active, frequency, period, offset. The acoustics system publishes
a ground truth list of pingers and their state. Choosing which one is
active (at random, per run, per task) belongs to the competition server,
which only has to send `set` messages.

## 7. Messages and ROS

**Problem.** VRX publishes a `gz.msgs.Param`, bridged to `ParamVec`, and a
consumer has to search an array by name. A typed message needs a new
message package and its own bridge, since `ros_gz_bridge` only knows its
built in pairs.

**Solution (proposed).** Keep `gz.msgs.Param` to `ParamVec` in this phase,
with VRX's keys (`range`, `bearing`, `elevation`) plus `frequency`,
`amplitude`, `valid` and, when asked for, `emit_time`, so VRX code keeps working and nothing new
is needed to bridge it. Revisit a typed message together with the sonar
issue (#21), which will hit the same question.

## 8. Worlds, tests and documentation

**Solution.** The acoustics system in every world, silent until a pinger
shows up. An `acoustic_pinger` model, a receiver on the custom USV with its
bridge entry, two pingers under buoys in `marina_bay`. Core gtests (ping
schedule, delay, reach, band, overlap, seeded repeat) and a headless test
of the demo. A how to page, a design note, and the `AGENTS.md` line and
invariants. VRX's defects are fixed on the way: the header that grows on
every publish, the unchecked rate, the unchecked pose.

## 9. Seeing it in Gazebo

**Problem.** A pinger is invisible. Nobody can see how far it reaches, which
one is active, or what a receiver heard.

**Example.** A boat gets no reading and the user cannot tell whether it is
out of reach, tuned to the wrong band, or the pinger is off.

**Solution (built).** A fourth GUI system in `gz_maritime_gui`,
`AcousticsPanel`, built like the wind and current panels: it reads the
`AcousticPinger` components from the GUI's ECM, hears the receivers the
acoustics system lists on `/world/<world>/acoustics/receivers` (the GUI's
copy of a custom sensor loses its `gz:type`, so the ECM cannot tell a
receiver from an anemometer), draws with markers through `MarkerManager`,
writes only through the `set` topics, and is docked in every world with
drawing off; the demo world draws.

What it draws, each with its own switch, in a colour per frequency:

- **Reach.** The circle where each pinger's reach sphere cuts the plane
  drawn on, grey and dashed when the pinger is off, and inside it a lighter
  dashed ring where pings start to drop, one per dropout start the
  receivers use. It follows the pinger and its `set` messages.
- **Pings.** A ring per ping spreading on that plane, at the speed of sound
  slowed by a factor (50 by default) so the eye can follow it.
- **Readings.** From each receiver, a ray along the bearing and elevation it
  reported, fading in three steps over 1.5 s. An invalid reading in red.

The plane: the water (default); a depth under it; a receiver's depth,
following it, the slice a vehicle under the water hears in; or 3D, the
reach and each ping as wireframe spheres cut at the water, with the
dropout ring on the water. Wireframe, not translucent: marker transparency
was not verified.

The panel also lists the pingers (frequency, period, offset, reach, active)
and lets the user switch one on or off and retune a receiver.

Why markers and not a `<visual>` on the pinger model: a visual is fixed at
load time, cannot show active or the pings, and every camera sensor on a
vehicle would see it. The markers go to the GUI's marker service,
`/marker`; the server's sensors draw only what comes on their own,
`/sensors/marker`, so a camera never sees them. `markers_camera_test`
renders a camera and confirms it, and shows that `visibility: GUI` alone
would not: the same markers sent to the sensors' service appear in the
image.

Not built: **why not**, over a receiver that heard nothing, the reason (out
of band, out of reach, dropped, out of the water), which needs a ground
truth topic per receiver. For RViz, the ground truth topics are enough to
build a marker display later; not in this plan.

## Phases

1. **Parity, done right** (sections 1, 2, 3, 4, 6, 7, noise and seed from
   5): everything VRX does, with real pingers, real frames and frequencies.
2. **Hearing rules** (the rest of 5) and the worlds, tests and docs of 8.
3. **The GUI panel** (9): reach and pingers first, then pings, then
   readings.
4. **Later, each its own issue:** per hydrophone time differences for teams
   that test their own solver; transponders and USBL for the BlueROV2
   (interrogate and reply, after LRAUV and DAVE); occlusion.

## To decide

1. Message: `Param`/`ParamVec` now (proposed), or a typed message and a
   bridge from the start?
2. Is range off by default acceptable for VRX 2026, or must the default
   match VRX today?
3. Does the active pinger choice live in `vrx-competition` (proposed) or
   in a helper plugin here?
4. Is USBL for the BlueROV2 wanted in M3, or is phase 4 the right place?
5. Names: package `gz_acoustics`, sensor `acoustic_receiver`.

Settled by building it: the GUI panel is part of #20, and every world runs
the acoustics system, silent until a pinger shows up.
