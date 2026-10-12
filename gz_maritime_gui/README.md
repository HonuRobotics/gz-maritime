# gz_maritime_gui

The Gazebo GUI for the maritime environment: three GUI plugins.
`OceanCurrentPanel` and `WindPanel` each steer their field by hand or set it
exactly, draw it on the water as flow trails or arrows coloured by speed,
and show what each vehicle feels; the wind panel also flies wind socks.
`ScenarioPanel` ramps both fields to named conditions and runs a demo. Planned in
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
  number validator takes only the locale's separator.
- **Steer** it by hand: drag the compass's arrow to turn the flow, slide or
  scroll for its speed. The field ramps there, no faster than a set rate
  (the current 0.5 m/s and 90° a second, the wind 5 m/s and 90° a second),
  one `set` message every 100 ms, so it never jumps; only the last step is
  tracked for the status line.
- **Draw** it, sampled with the same `OceanCurrentSampler` or `WindSampler`
  the vehicles use: as flow trails, a few hundred particles drifting with
  the field in a soft disc, each leaving a trail that fades behind it, or as
  a grid of arrows. Both are coloured by speed (the water deep blue to
  white, the air indigo to yellow) up to a top speed shown on a legend. The
  current is drawn just above the water level, the wind at a height above
  the water.
- **On the vehicles**, every model that is not static, spawned later
  included: the water past the hull (the current less the vehicle's velocity
  over the ground), the apparent wind (the wind 2 m up less that velocity),
  solid arrows crossed flat and upright so they read from any angle, and the
  track over the ground with a heading line. The GUI's ECM carries poses,
  not velocities, so the velocity is estimated from the poses
  (`VesselTracker`).
- **Wind socks** at the points a world places, striped, pointing downwind,
  drooping in light air and flying straight out from 7.7 m/s (15 knots).
- **Scenarios**: presets (built in Calm, Tidal ebb, Fresh breeze and Squall,
  or the configuration's own) ramped through the same steering, and a demo
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
| `include/gz/sim/maritime_gui/Flow.hh`, `src/Flow.cc` | The colour maps, the colour bins, the flow trails (`ParticleField`) and the steering ramp (`Steering`). |
| `include/gz/sim/maritime_gui/Vessels.hh`, `src/Vessels.cc` | The vehicles (`VesselTracker`), their solid arrows, tracks and headings. |
| `include/gz/sim/maritime_gui/WindSock.hh`, `src/WindSock.cc` | The wind sock's axis and its markers. |
| `include/gz/sim/maritime_gui/Presets.hh`, `src/Presets.cc` | The presets and the demo. |
| `include/gz/sim/maritime_gui/FieldLink.hh`, `src/FieldLink.cc` | What both panels share: the world's name, the field's topic, the state service, steering, the drawing, the status line and number parsing. |
| `src/OceanCurrentPanel.*`, `src/WindPanel.*`, `src/ScenarioPanel.*` | The three GUI systems and their QML panels. |
| `src/Compass.qml`, `src/Legend.qml`, `src/NumberField.qml` | The steering compass, the colour legend and the number field both panels use. |
| `test/fields_test.cc` | gtests on the core. |
| `test/test_gui_reads_recipes.py` | Both plugins in a running GUI on Qt's offscreen platform, reading the recipes from the GUI's state and, in a world without a wind, asking the state service. |

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
| `<max_speed>` | both | 1 (current), 20 (wind) | m/s of the colour map's last colour, and the steering slider's top. |
| `<extent>` | both | 40 | Side of the area, m. |
| `<spacing>` | both | 5 | Between two arrows, m. |
| `<center_x>`, `<center_y>` | both | 0 | Centre of the area. |
| `<scale>` | both | 4 (current), 0.3 (wind) | m of arrow per m/s. |
| `<particles>` | both | 400 | How many trails. |
| `<trail_length>` | both | 8 | Points in a trail. |
| `<lifetime>` | both | 6 (current), 4 (wind) | s a particle lives, at most. |
| `<speedup>` | both | 8 (current), 1 (wind) | The particles move this many times the flow. |
| `<height>` | `WindPanel` | 3 | m above the water the wind is drawn at. |
| `<on_vehicles>` | both | false | Draw what each vehicle feels. |
| `<wind_sock>x y z</wind_sock>` | `WindPanel` | none | A wind sock's pole foot; repeat it. |
| `<sock_height>` | `WindPanel` | 3 | m, the poles' height. |
| `<show_socks>` | `WindPanel` | true | Fly the wind socks. |
| `<preset name dwell>` | `ScenarioPanel` | Calm, Tidal ebb, Fresh breeze, Squall | With `<current speed direction/>` and `<wind speed direction speed_gust direction_gust/>`; replaces the built in presets. |
| `<demo>` | `ScenarioPanel` | false | Start the demo with the world. |
| `<marker_service>` | both | `/marker` | MarkerManager's service. |
