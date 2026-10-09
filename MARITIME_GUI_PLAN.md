# Maritime GUI plan

## Scope and success

The wind and the ocean current are each kept on the world as a recipe, a
component that every process replicates, and each changes at run time on a
`set` topic. Today the only ways to change them are `gz topic` and
`ros2 topic pub`, and the only way to see them is a ground truth at the
world's origin. We want two Gazebo GUI plugins, one per field, in a new
package `gz_maritime_gui`, each of which changes its field from a panel and
draws it as arrows in the 3D scene, next to the vehicles and the waves.

The plugin is a gz-sim GUI system (`gz::sim::GuiSystem`), so it gets the GUI's
replicated ECM every update. It builds the same `WindSampler` and
`OceanCurrentSampler` the vehicles use and samples them on a grid: gusts, the
wind's profile with height and any future current model (a tide, a depth
profile, a NOAA grid) show up with no new topic and no copy of a model's maths.
It changes them on the existing `set` topics, so the server's rules (a message
is applied whole or not at all, `model` cannot change) stay the only rules. It
needs no ROS, so it works with `gz sim` on its own and with every launch,
including the Blue Robotics ones, which do not bridge the current.

The demo: launch the custom USV, open the panel, set a 0.5 m/s current setting
east and a 10 m/s wind from the west with gusts, and watch the arrows turn and
the boat drift along them. Turn the current north from the panel, and the
current arrows and the boat follow. A world that does not load the wind or
the current system shows that part of the panel disabled.

The work lands in three pull requests:

1. **The panel** (§1, §2): the package, the plugin, reading the recipes and
   changing them.
2. **The arrows** (§3): the wind and the current drawn as markers.
3. **Worlds and documentation** (§4): the plugin in every world's `<gui>`, the
   docs, and the Blue Robotics worlds in `bluerobotics_models`.

## 0. Before the first pull request: prove the recipes reach the GUI

**Problem.** Everything rests on the GUI's ECM holding the `Windfield` and
`OceanCurrentfield` components. The GUI can only deserialize a component whose
type is registered in its process, and a recipe is written once, in
`Configure`, and again only on a change.

**What gz-sim 10 does** (`src/gui/Gui.cc`, `src/gui/GuiRunner.cc`): it creates
the `GuiRunner`, which asks for the full state asynchronously, then loads the
plugins in the world's `<gui>` block, and the full state is applied on the Qt
event loop, after those plugins are loaded. A plugin that links `gz_wind` and
`gz_ocean_current` registers both components on load, so a plugin listed in
the world gets the recipes in the first state. A plugin added later from the
plugin menu does not: the full state has already come and gone, the components
were dropped with a "has not been registered" warning, and only the next
change on a `set` topic would bring them.

**Spike.** A throwaway GUI system that prints the two recipes' generations,
loaded (a) from a world's `<gui>` and (b) from the plugin menu after the world
is up. Expected: (a) prints at once, (b) prints nothing until a change. If (a)
fails too, rethink before §1.

**Result** (gz-sim 10.5, the plugin itself with debug lines, under Xvfb):
(a) read both recipes from the GUI's state at load, generation 1; (b), added
from the menu, found no recipe, asked the state service and read both from
it, and switched to the GUI's ECM at the next change. Both are now checked
by the package's tests or were checked by hand, below.

**Solution for (b).** When the plugin has seen no recipe after its first
updates, it asks the world's `/world/<world>/state` service once for the full
state, deserializes it into a private `EntityComponentManager`, and reads the
two recipes from there; from then on the replicated ECM keeps it current. It
never writes into the GUI's ECM.

## 1. The package and the plugin

**Problem.** There is no GUI code in this repository yet.

**Solution.** A package `gz_maritime_gui`:

- two GUI plugins, `OceanCurrentPanel` and `WindPanel`, each a
  `gz::sim::GuiSystem` with a QML panel, libraries of the same names in
  `lib/gz_maritime_gui`, found through a `GZ_GUI_PLUGIN_PATH` hook like the
  system packages' `GZ_SIM_SYSTEM_PLUGIN_PATH` one;
- a plain C++ library, `gz_maritime_gui_core`, with no Qt: reading the
  recipes, building `set` messages, sampling the grid and building the
  markers (`Fields`), and what both panels share, the world's name, the
  field's topic, the state service, the arrows and number parsing
  (`FieldLink`). Each plugin is a thin layer over it, and the gtests cover
  it;
- dependencies: `gz_sim_vendor` (component `gui`), `gz_gui_vendor`,
  `gz_transport_vendor`, `gz_msgs_vendor`, `gz_wind`, `gz_ocean_current`, Qt 6
  through gz-gui.

One plugin per field (first built as one plugin with two sections, then
split), so a world or a user takes only the field it has, each panel docks
and closes on its own, and a panel whose world lacks its field says so.

## 2. The panel

**Problem.** The two fields follow different conventions, which a panel must
not blur: the wind's direction is where it comes from, the current's where it
sets towards. A form that silently converts would teach the wrong number.

**Example.** A wind from 270 and a current setting 090 move a boat the same
way, east.

**Solution.** Two panels, each labelled with its own convention.

- **Current:** speed (m/s), direction it sets towards (degrees clockwise from
  north), water level (m).
- **Wind:** speed (m/s) at its reference height, direction it comes from
  (degrees clockwise from north), and the gusts: speed gust (m/s), direction
  gust (degrees), their correlation times (s). The reference height, roughness
  length and the rest stay in the world file.
- A small compass in each, drawing the flow, where the air or the water
  goes, with the number in its own convention beside it.
- Each field shows the values read from its recipe, refreshed when the
  recipe's generation changes, whoever changed it (this panel, `gz topic`,
  ROS). Edits are held until **Apply**, which sends the changed keys of that
  section as one `gz.msgs.Param` on `/world/<world>/<field>/set`.
- A refused message (out of range, or refused by the model) changes nothing
  on the server, which logs why. The panel knows only that the generation did
  not move: after a few seconds (5, to leave room for a GUI that receives the
  state slowly, as one rendering in software does) without a new generation
  it says the change was not applied and points to the server log. A
  message published with nothing subscribed is not sent at all, and the
  panel says so; the `set` publishers are advertised as soon as the world's
  name is known, since a message published right after advertising is lost
  before the subscriber is discovered. It does not repeat the server's
  range checks, so it cannot disagree with them.
- A number takes a decimal point or a decimal comma, whatever the locale:
  the panel parses the text itself. Qt's `DoubleValidator` follows the
  locale, so with a Spanish one it took "1,5" and refused "1.5".
- A model's own `<parameters>` are listed read only, as text. Editing them is
  out of scope.

## 3. The arrows

**Problem.** A field is a velocity at every point; the ground truth at the
origin says nothing about gusts travelling across the water, the profile with
height, or a current that varies with place.

**Solution.** Each field is drawn as a grid of arrows, sampled from the
recipe through the field's own sampler, as `gz.msgs.Marker` on the GUI's
`/marker` service, which `MarkerManager` serves (every world here loads it).

- One `LINE_LIST` marker per field (namespace `gz_maritime_gui/current` and
  `gz_maritime_gui/wind`, id 1), each arrow a shaft and two head strokes, so
  a grid of hundreds of arrows is one marker and one request. Two traps
  found on the way: gz-gui 10's `/marker` is a one way service, which never
  answers a two way request, and MarkerManager gives a marker with id 0 a
  new random id, so id 0 adds a marker on every redraw instead of replacing
  it.
- The current at the water level, drawn just above it (the water surface
  hides anything below it); the wind at a height set in the panel, 3 m by
  default, sampled at that height so the profile shows.
- A grid centred on the world's origin, with its extent and spacing in the
  panel and in the plugin's configuration; length proportional to speed
  (4 m per m/s of current, 0.3 m per m/s of wind, so the default arrows fit
  the default 5 m spacing), one colour per field, and a legend with the
  scale.
- Redrawn when the recipe's generation changes, and at a few hertz of
  simulation time while the field changes on its own (gusts), not every
  frame. Turning a field off deletes its marker; unloading the plugin deletes
  both.

Marker drawing keeps the plugin out of the render thread. If a dense grid
redrawn for gusts proves too slow through the marker service, move the arrows
to gz-rendering in the render event, behind the same core library.

## 4. Worlds, documentation and tests

**Solution.**

- The plugin in the `<gui>` block of the five `kai_gazebo` worlds, docked on
  the right, arrows off by default so the scene looks as it does today until
  a user turns them on. The Blue Robotics worlds get the same in a pull
  request in `bluerobotics_models`.
- Docs: a how to page ("Change and see the wind and the current"), the plugin
  in the launch and world reference, a row in `AGENTS.md`, the package README.
- Tests: gtests on `gz_maritime_gui_core` (a wind from 270 and a current
  setting 090 both draw arrows towards +x; a refused message is reported; the
  marker for a known grid; a world without a field disables its section), and
  one test that loads the plugin with `QT_QPA_PLATFORM=offscreen` into a
  running world and checks it reads both recipes (the offscreen platform
  cannot run the 3D view, so that GUI holds the plugin only). The GUI itself
  is checked by hand, with the steps in the pull request.

## Out of scope

A RViz view (later, the same arrows forwarded to ROS), editing a model's
`<parameters>`, presets saved to a file, a probe that reads the field at a
clicked point, the anemometer's reading in the panel, the waves' controls, and
a ramp between two values (a change on a `set` topic is a step).
