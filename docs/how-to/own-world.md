# Use your own world

Any launch file here takes `world:=`, with a file name found on Gazebo's
resource path or a full path:

```bash
ros2 launch kai_custom_vehicle sim.launch.xml world:=/path/to/my_world.sdf
```

The easiest start is a copy of `open_water.sdf`:

```bash
cp $(ros2 pkg prefix --share kai_gazebo)/worlds/open_water.sdf my_world.sdf
```

## What a world needs

| Piece | Why | In `open_water.sdf` |
|---|---|---|
| Physics, user commands and scene broadcaster systems | Physics, spawning models, and the GUI view | `gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system` |
| **The gz-maritime buoyancy system** | Floats the collisions vehicles mark | `gz-maritime-buoyancy-system` |
| A wave source | The moving sea: its state, and the `set_parameters` service | `gz-sim-waves-fft-system` (or `gz-sim-waves-gerstner-system`) |
| The `water_surface` model | Draws the sea | `<include><uri>model://water_surface</uri></include>` |
| Sensor systems | IMU, magnetometer, GPS and rendered sensors | see [Add sensors](sensors.md#what-the-world-provides) |
| `<spherical_coordinates>` | GPS origin and magnetic field location | Portuguese Ledge, Monterey Bay |

Leave out what you don't need. A world without waves, for example, still
floats vehicles; it just draws no sea.

## The buoyancy system

```xml
<plugin filename="gz-maritime-buoyancy-system"
        name="gz::sim::maritime::Buoyancy">
  <graded_buoyancy>
    <default_density>1025</default_density>
    <density_change>
      <above_depth>0</above_depth>
      <density>1</density>
    </density_change>
  </graded_buoyancy>
  <enable_by_default>false</enable_by_default>
</plugin>
```

Read it as: seawater (1025 kg/m³) everywhere, except above z = 0, where there
is air (1 kg/m³). `<enable_by_default>false</enable_by_default>` means the
world floats nothing on its own: only the collisions vehicles mark with
`gz:buoyancy="true"` displace water.

Marked collisions float in every mode. The tags only decide what happens to
the **unmarked** ones:

| `<enable>` list | `<enable_by_default>` | Unmarked collisions |
|---|---|---|
| none | not set | **All float.** Every collision of every model, as in stock Gazebo. |
| present | not set | The listed models or links float. |
| none | `false` | **None float.** Only marked collisions do. Recommended. |
| present | `true` | All float; the list changes nothing. |

The recommended mode is the one that needs to know nothing about your
vehicles. The `<enable>` list still works, for a vehicle whose model you can't
edit, and takes names as spawned: `<enable>blueboat::hull_displacement</enable>`.

This is Gazebo's own buoyancy system plus the marked-collision change
([gz-sim 229ec07e](https://github.com/gazebosim/gz-sim/commit/229ec07e673b6317fec57af50fd652bcd9bde0ed)),
carried by `gz_buoyancy` until a Gazebo release ships it. When it does, the
plugin line becomes `gz-sim-buoyancy-system` / `gz::sim::systems::Buoyancy`
and nothing else changes.

## The world name

The wave service includes the world's name:
`/world/<name>/wave/set_parameters`. In `open_water.sdf` the name is
`default`. Commands you type yourself have to use your world's name; the
launch files don't care.

## Adding a seabed

For shallow water, or an underwater vehicle working near the bottom, add a
static model under the water:

```xml
<model name="seabed">
  <static>true</static>
  <pose>0 0 -20 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>200 200 1</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>200 200 1</size></box></geometry>
    </visual>
  </link>
</model>
```

It doesn't float, because nothing marks its collision.

Next: [Troubleshooting](faq.md).
