# Change and see the wind and the current

Every world here docks two panels on the right of the Gazebo GUI, **Ocean
current** and **Wind**. Each shows its field as the world has it now,
changes it, and draws it as arrows on the water. They need no ROS: they
work the same with `gz sim` on its own and with every launch.

```{figure} images/wind-and-current.jpg
:alt: The open water world from above, with a grid of blue current arrows pointing north east and yellow wind arrows pointing east, and the Ocean current and Wind panels on the right

A 0.5 m/s current setting 045 (blue) and a 10 m/s wind from 270 with
direction gusts (yellow), seen from above with east up.
```

## Change them

Each panel keeps its field's own convention:

- **Ocean current:** speed (m/s), the direction it **sets towards** in
  degrees clockwise from north (90 flows east), and the water level (m).
- **Wind:** speed (m/s) at the wind's reference height, the direction it
  **comes from** in degrees clockwise from north (270 blows east), and the
  gusts: the standard deviation of the speed (m/s) and of the direction
  (degrees), and their correlation times (s).

A wind from 270 and a current setting 090 both go east. The compass in each
panel draws the flow, where the air or the water goes, so the two agree on
the screen even though their numbers do not.

Type a decimal with a point or a comma, `1.5` or `1,5`, whatever your
system's language. Edit the fields and press **Apply**. The panel sends only
the values you changed, as one message on `/world/default/ocean_current/set`
or `/world/default/wind/set`, the same topics `gz topic` and ROS use. The
world applies a message whole or not at all, and the line beside **Apply**
says which:

| Status | Meaning |
|---|---|
| Applied. | The world took the change. |
| Not applied | The world kept its values: a value out of range, or refused by the model. The server's log says why. |
| Not sent: ... is not a number | A field holds something that is not a number; nothing was sent. |
| Not sent: nothing is listening | Nothing listened on the topic yet, as just after the GUI starts; press Apply again in a moment. |

The fields always show the world's values, whoever changed them last: the
panel, `gz topic` or ROS. A world without a wind or an ocean current system
says so in that panel. The reference height, the roughness length and the
rest stay in the world file; see the
[wind](../reference/launch-and-world.md#wind-parameters) and
[ocean current](../reference/launch-and-world.md#ocean-current-parameters)
parameters.

## See them

Tick **Arrows** in a panel to draw that field as a grid of arrows on the
water. The arrows are the field as the vehicles feel it, sampled from the
world with the same code, so wind gusts travel across the grid and a current
model that varies with place shows its pattern.

- The current is blue, just above the water level; 1 m/s draws 4 m.
- The wind is yellow, at a height above the water, 3 m by default, and
  sampled at that height, so the profile with height shows; 1 m/s draws
  0.3 m.
- Each grid is centred on the world's origin, 40 m on a side with an arrow
  every 5 m; change both in the panel.

The arrows are off until you tick them, so the scene looks as it always did.

## In your own world

Add either panel, or both, to the world's `<gui>` block; every element but
the `filename` is optional:

```xml
<plugin filename="OceanCurrentPanel" name="Ocean current">
  <gz-gui>
    <title>Ocean current</title>
    <property type="string" key="state">docked</property>
  </gz-gui>
  <show_arrows>true</show_arrows>
  <extent>40</extent>      <!-- side of the grid, m -->
  <spacing>5</spacing>     <!-- between two arrows, m -->
  <center_x>0</center_x>   <!-- centre of the grid -->
  <center_y>0</center_y>
  <scale>4</scale>         <!-- m of arrow per m/s -->
</plugin>

<plugin filename="WindPanel" name="Wind">
  <gz-gui>
    <title>Wind</title>
    <property type="string" key="state">docked</property>
  </gz-gui>
  <show_arrows>true</show_arrows>
  <height>3</height>       <!-- m above the water -->
  <scale>0.3</scale>
</plugin>
```

The wind panel takes the same grid elements as the current panel. The
arrows go through Gazebo's `MarkerManager`, which every world here loads; a
world of your own needs it in its `<gui>` too. Both panels are also in the
GUI's plugin menu (top right, **Ocean Current Panel** and **Wind Panel**)
for a world that does not list them. The package `gz_maritime_gui` installs
them and puts their folder on `GZ_GUI_PLUGIN_PATH` when the workspace is
sourced.
