# gz_maritime_gui

The Gazebo GUI for the maritime environment: two GUI plugins,
`OceanCurrentPanel` and `WindPanel`, each of which changes its field and
draws it as arrows on the water. Planned in
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
- **Draw** it as a grid of arrows, sampled with the same
  `OceanCurrentSampler` or `WindSampler` the vehicles use, as one
  `LINE_LIST` marker through `MarkerManager`: the current just above the
  water level, the wind at a height above the water.

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
| `include/gz/sim/maritime_gui/FieldLink.hh`, `src/FieldLink.cc` | What both panels share: the world's name, the field's topic, the state service, the arrows, the status line and number parsing. |
| `src/OceanCurrentPanel.*`, `src/WindPanel.*` | The two GUI systems and their QML panels. |
| `src/Compass.qml`, `src/NumberField.qml` | The compass and the number field both panels use. |
| `test/fields_test.cc` | gtests on the core. |
| `test/test_gui_reads_recipes.py` | Both plugins in a running GUI on Qt's offscreen platform, reading the recipes from the GUI's state and, in a world without a wind, asking the state service. |

The core (`gz_maritime_gui_core`, the first two rows) has no Qt. The
plugins install to `lib/gz_maritime_gui`, a folder of their own, and the
package puts that folder on `GZ_GUI_PLUGIN_PATH`: the GUI's plugin menu
lists every library on that path, and a merged install puts every package's
libraries in `lib`.

## Configuration

All optional, in the plugin's element of a `<gui>` block:

| Element | Plugin | Default | Meaning |
|---|---|---|---|
| `<show_arrows>` | both | false | Draw the arrows. |
| `<extent>` | both | 40 | Side of the grid, m. |
| `<spacing>` | both | 5 | Between two arrows, m. |
| `<center_x>`, `<center_y>` | both | 0 | Centre of the grid. |
| `<scale>` | both | 4 (current), 0.3 (wind) | m of arrow per m/s. |
| `<height>` | `WindPanel` | 3 | m above the water the wind is drawn at. |
| `<marker_service>` | both | `/marker` | MarkerManager's service. |
