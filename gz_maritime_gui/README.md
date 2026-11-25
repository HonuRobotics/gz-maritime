# gz_maritime_gui

The Gazebo GUI for the maritime environment: four GUI plugins.
`OceanCurrentPanel` and `WindPanel` each steer their field by hand or set it
exactly, draw it on the water as flow trails or arrows coloured by speed,
and show the water and the wind each vehicle meets; the wind panel also
flies wind socks. `ScenarioPanel` steers both fields to named conditions
and runs a demo. `AcousticsPanel` shows the acoustic pingers and what the
receivers hear, and switches the pingers (see below). Planned in
[MARITIME_GUI_PLAN.md](../MARITIME_GUI_PLAN.md); how to use them is in the
docs, [Change and see the wind and the current](../docs/how-to/wind-and-current.md).

## What they do

- **Read** their field from its recipe on the world, the `OceanCurrentfield`
  or `Windfield` component, which the GUI's copy of the ECM holds. A panel
  shows the values whoever changed them.
- **Change** it: Apply sends the keys the user changed as one
  `gz.msgs.Param` on `/world/<world>/ocean_current/set` or
  `/world/<world>/wind/set`. The server applies a message whole or refuses
  it and logs why; the panel sees a refusal as a recipe whose generation did
  not move within a few seconds. A number takes a decimal point or a decimal
  comma, whatever the locale: the panel parses the text itself, since Qt's
  number validator takes only the locale's separator. A field typed in
  keeps its text while the world moves, until Apply sends it, or until the
  field is steered, by this panel's compass or by a scenario, which drops
  it; a field the user never touched follows the world.
- **Steer** it by hand: drag the compass's arrow to turn the flow, slide or
  scroll for its speed. A rate limit on the controls: the panel sends a
  `set` message every 100 ms of wall clock, no faster than a set rate (the
  current 0.5 m/s and 90° a second, the wind 5 m/s and 90° a second), so the
  drawing turns smoothly; only the last step is tracked for the status
  line. Each message is still a step in the recipe, not a ramp in
  simulation time, so it does not spare a vehicle with plugin added mass a
  force spike. When the field moves to a value the panel did not send
  (another panel, a second GUI, `gz topic`, ROS), the panel stops steering
  and says so, so two controllers never take turns.
- **Draw** it, sampled with the same `OceanCurrentSampler` or `WindSampler`
  the vehicles use: as flow trails, a few hundred particles drifting with
  the field in a soft disc, each leaving a trail that fades behind it, or as
  a grid of arrows. Both are coloured by speed (the water deep blue to
  white, the air indigo to yellow) up to a top speed shown on a legend. The
  current is drawn just above the water level, the wind at a height above
  the water.
- **On the vehicles**, every model that is not static and has a link,
  spawned later included, buoys too: the water past the hull at its origin
  (the current there less the velocity of the model's origin over the
  ground), the apparent wind (the wind 2 m up less that velocity),
  solid arrows crossed flat and upright so they read from any angle, and the
  track over the ground with a heading line. The GUI's ECM carries poses,
  not velocities, so the velocity is estimated from the poses
  (`VesselTracker`).
- **Wind socks** at the points a world places, striped, pointing downwind,
  drooping in light air and flying straight out from 7.7 m/s (15 knots).
- **Scenarios**: presets (built in Calm, Tidal ebb, Fresh breeze and Squall,
  or the configuration's own) steered to through the same steering, and a demo
  that steps through them on their dwell in simulation time.
- Gazebo's renderer draws a line list in its material's colour and ignores
  colours given point by point, so a drawing is a set of `LINE_LIST`
  markers, one per colour bin (speed, and the trails' brightness), sent in
  one request to `MarkerManager`'s `/marker_array` service. A bin that
  empties is deleted, since a marker sent with no points keeps its old
  ones.

Each panel keeps its field's convention: the wind's direction is where it
comes from, the current's where it sets towards, both in degrees clockwise
from north. The compass in each draws the flow.

They need no ROS.

## The acoustics panel

`AcousticsPanel` shows `gz_acoustics` in the GUI. It lists the pingers,
each with its frequency, period, offset and reach and a switch that sends
`active` on its `set` topic. The pingers are those the acoustics system
lists on `/world/<world>/acoustics/pingers`, each as its `AcousticPinger`
component in the GUI's ECM has it when the GUI has one, which is newer. A
panel loaded with the GUI has them all in the ECM. One added later from the
plugin menu has only those that changed since, and the list fills in the
rest, at the acoustics system's `<publish_rate>`. It never asks for the
world's state: no pinger is the normal case, in every world but the demo. It lists the receivers, which the acoustics system publishes on
`/world/<world>/acoustics/receivers` (the GUI's copy of a custom sensor
loses its `gz:type`, so the panel cannot find them in the ECM), each with its
last reading and a field to retune it. It draws, each switched on its own:

- **Reach:** the circle where each pinger's reach meets the water, in its
  frequency's colour, grey and dashed when the pinger is off, and inside it
  a lighter dashed ring where pings start to drop, one per dropout start
  the receivers use.
- **Pings:** a ring per ping where its sound meets the water, spreading at
  the speed of sound slowed down (`<slow_down>`, 50 by default, so a ring
  crosses 300 m in 10 s), fading out at the reach. `<max_ring_radius>` ends
  the rings of a pinger with no reach only. Only pings that were sent ring:
  none from a pinger above the water, none from before a pinger was last
  switched on or its schedule changed. At most six rings per pinger: slowed
  down further, every second or third ping is drawn, so what is sent stays
  about 15 kB per pinger and frame.
- **Bearings heard:** a ray from each receiver along its last reading's
  bearing and elevation, fading in three steps over 1.5 s of simulation
  time; red when the reading was invalid. The ray keeps the pose the
  receiver had when the reading came, so it stays on the pinger while the
  boat turns.

The reach and the pings are drawn on a horizontal plane, chosen in the
panel: the water (default); a depth under it; or a receiver's depth,
following it, the slice a vehicle under the water listens in. Or in 3D:
the reach as a wireframe sphere around each pinger and each ping as a
sparser one growing out of it, both cut at the water, where sound stops,
with a ring where they meet it. Under the water any of them lies behind the
drawn sea, so look from an underwater camera or hide the water.

The reach and the rays are sent when they change, not on a timer: a pinger
switched off or a buoy dragged while the world is paused is redrawn at
once, and a circle that stays is not sent again. The rings are sent every
50 ms of simulation time, the heavier 3D shells every 100 ms, and no more
often than that in wall time. While the world runs, a pinger that moves or
a plane that follows a receiver is caught up by those frames; while it is
paused, a drag is redrawn at once.

Colours run with frequency from orange at 25 kHz to violet at 40 kHz.
Configuration: `<reach>`, `<pings>`, `<readings>` (all false by default),
`<plane>` (`water`, `depth`, `receiver` or `3d`), `<plane_depth>` (m, default 5),
`<plane_receiver>` (a receiver's topic, default the first listed; one not
listed is said so, and the water is drawn on until it is),
`<speed_of_sound>`, `<slow_down>`, `<max_ring_radius>`, `<ray_length>`,
`<ray_lifetime>`, `<water_level>` and `<marker_service>`.

No camera sensor sees what the panels draw, because they draw only through
the GUI's marker service, `/marker`: the server's sensors take markers on
their own, `/sensors/marker`. The `visibility: GUI` flag the markers carry
does not do it on its own; `markers_camera_test` renders a camera and shows
both (it needs a GPU and skips without one). Never point `<marker_service>`
at `/sensors/marker`.

## How they reach the recipes

Gazebo creates the GUI runner, loads the plugins in the world's `<gui>`
block, and applies the world's first full state after that. The plugins
link `gz_wind` and `gz_ocean_current`, which register both components, so a
plugin listed in the world gets its recipe in that first state. One added
later from the plugin menu missed it; after about a second without a recipe
it asks the world's `/world/<world>/state` service once, reads the recipe
from a private copy, and switches to the GUI's own ECM on the next change.
It never writes into the GUI's ECM.

## Layout

| Path | What |
|---|---|
| `include/gz/sim/maritime_gui/Fields.hh`, `src/Fields.cc` | Reading the recipes, the `set` messages, the arrow grid and its marker, and the change tracker. |
| `include/gz/sim/maritime_gui/Flow.hh`, `src/Flow.cc` | The colour maps, the colour bins, the flow trails (`ParticleField`) and the steering, a rate limit on the controls (`Steering`). |
| `include/gz/sim/maritime_gui/Vessels.hh`, `src/Vessels.cc` | The vehicles (`VesselTracker`), their solid arrows, tracks and headings. |
| `include/gz/sim/maritime_gui/WindSock.hh`, `src/WindSock.cc` | The wind sock's axis and its markers. |
| `include/gz/sim/maritime_gui/Presets.hh`, `src/Presets.cc` | The presets and the demo. |
| `include/gz/sim/maritime_gui/FieldSpec.hh`, `src/FieldSpec.cc` | Each field defined once: its topic name, steering rates, colours, arrow scale and trails. |
| `include/gz/sim/maritime_gui/FieldControl.hh`, `src/FieldControl.cc` | A panel's control of a field: the world's name, the field's topic, the state service, tracking, steering, the status line and number parsing. |
| `include/gz/sim/maritime_gui/FieldDrawing.hh`, `src/FieldDrawing.cc` | A panel's drawing of a field: arrows or trails, and the vehicles and socks beside them. |
| `include/gz/sim/maritime_gui/MarkerLayer.hh`, `src/MarkerLayer.cc` | One drawing on the scene: what is drawn, replaced and deleted through MarkerManager, for every panel, and the panels' default marker service. |
| `include/gz/sim/maritime_gui/Acoustics.hh`, `src/Acoustics.cc` | The acoustics panel's logic: the pingers from the ECM and from the list, the listener, and the reach, rings and rays as markers. |
| `src/OceanCurrentPanel.*`, `src/WindPanel.*`, `src/ScenarioPanel.*`, `src/AcousticsPanel.*` | The four GUI systems and their QML panels. |
| `src/Compass.qml`, `src/Legend.qml`, `src/NumberField.qml` | The steering compass, the colour legend and the number field both panels use. |
| `test/fields_test.cc` | gtests on the core. |
| `test/test_gui_reads_recipes.py` | The plugins in a running GUI on Qt's offscreen platform, reading the recipes from the GUI's state and, in a world without a wind, asking the state service; the acoustics panel's drawings, as `test/marker_recorder.cc` receives them in MarkerManager's place, redrawn at once while the world is paused (a buoy dragged, a pinger switched off), the panel added late from the plugin menu (`test/MenuLoader.cc` does what the menu does), and a world without pingers. |

The core (`gz_maritime_gui_core`, the first six rows) has no Qt. The
plugins install to `lib/gz_maritime_gui`, a folder of their own, and the
package puts that folder on `GZ_GUI_PLUGIN_PATH`: the GUI's plugin menu
lists every library on that path, and a merged install puts every package's
libraries in `lib`.

## Configuration

All optional, in the plugin's element of a `<gui>` block:

| Element | Plugin | Default | Meaning |
|---|---|---|---|
| `<display>` | both | off | `off`, `arrows` or `trails`. |
| `<max_speed>` | both | 1 (current), 20 (wind) | m/s of the colour map's last colour; the speed slider spans it, and further when the field is past it, and the wheel steps a twentieth of it. |
| `<extent>` | both | 40 | Side of the area, m. |
| `<spacing>` | both | 5 | Between two arrows, m. |
| `<center_x>`, `<center_y>` | both | 0 | Centre of the area. |
| `<scale>` | both | 4 (current), 0.3 (wind) | m of arrow per m/s. |
| `<particles>` | both | 400 | How many trails. |
| `<trail_length>` | both | 8 | Points in a trail. |
| `<lifetime>` | both | 6 (current), 4 (wind) | s a particle lives, at most. |
| `<speedup>` | both | 8 (current), 1 (wind) | The particles move this many times the flow. |
| `<height>` | `WindPanel` | 3 | m above the water the wind is drawn at. |
| `<on_vehicles>` | both | false | Draw the water or the wind each vehicle meets at its origin. |
| `<wind_sock>x y z</wind_sock>` | `WindPanel` | none | A wind sock's pole foot; repeat it. |
| `<sock_height>` | `WindPanel` | 3 | m, the poles' height. |
| `<show_socks>` | `WindPanel` | true | Fly the wind socks. |
| `<preset name dwell>` | `ScenarioPanel` | Calm, Tidal ebb, Fresh breeze, Squall | With `<current speed direction/>` and `<wind speed direction speed_gust direction_gust/>`; replaces the built in presets. |
| `<demo>` | `ScenarioPanel` | false | Start the demo with the world. |
| `<marker_service>` | both | `/marker` | MarkerManager's service. |
