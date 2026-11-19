# See through the water

Under water, a camera sees the water as much as what is in it: far objects
fade into a veil of the water's colour, red goes first, and everything gets
darker the deeper you go. This page grows with the underwater camera work:
for now it covers the first step, giving a world its water.

## Give a world its water

Add the water optics system to the world and name the kind of water, by its
Jerlov type:

```xml
<plugin filename="gz-maritime-water-optics-system"
        name="gz::sim::maritime::WaterOptics">
  <jerlov>3C</jerlov>
</plugin>
```

The open ocean types run from `I`, the clearest, through `IA`, `IB`, `II` and
`III`; the coastal ones from `1C` to `9C`, the murkiest. Coastal water is
greener and murkier than ocean water. Without the system, the world has
clear water.

| If your water is | Try | Visibility |
|---|---|---|
| Tropical open ocean, very clear | `IB` | about 46 m |
| Open ocean | `II` | about 11 m |
| A clear coast | `1C` | about 10 m |
| A typical harbour or coast | `3C` | about 4 m |
| A murky harbour, a river mouth | `7C` | about 1.7 m |

```{figure} ../design/images/uw-jerlov-presets.svg
Every type: how fast each colour fades, the colour of the veil, and how far
you can see.
```

## Set how far you can see

Divers and pilots describe water by its visibility. Give it in metres and
the system scales the water type to it, keeping its colour:

```xml
<plugin filename="gz-maritime-water-optics-system"
        name="gz::sim::maritime::WaterOptics">
  <jerlov>3C</jerlov>
  <visibility>8</visibility>
</plugin>
```

This is coastal, green water, but clearer than usual: a black target
disappears at about 8 m.

## Change it while the world runs

```bash
gz topic -t /world/default/water_optics/set -m gz.msgs.Param \
  -p 'params {key: "visibility" value {type: DOUBLE double_value: 3}}'
```

Any element of the plugin is a key: `jerlov` takes a string, `visibility`
and `water_level` a number, the per-channel values a string `"r g b"`.

## Check what the world has

```bash
gz topic -e -t /world/default/water_optics_info -n 1
```

prints the water type, the visibility, and per channel the numbers a camera
uses. The [reference](../reference/launch-and-world.md#water-optics-parameters)
lists every element and the presets.

## What comes next

A camera that sees through this water, then the same water in the GUI, then
the vehicle's own lights. The plan is in
[UNDERWATER_CAMERA_PLAN.md](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/UNDERWATER_CAMERA_PLAN.md).
