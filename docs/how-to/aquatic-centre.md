# Dive in the aquatic centre

```{figure} images/aquatic-centre.jpg
:alt: An indoor pool hall: the ten lane competition pool in the foreground with the BlueBoat in it, the diving well and its platform tower beyond, stands along the north side

The aquatic centre at its opening view, the BlueBoat in lane 4.
```

Besides the sea and the competition sites, gz-maritime ships an indoor
pool: an Olympic competition pool, 50 by 25 m and 3 m deep, with ten
2.5 m lanes marked on its floor, and a diving well, 25 by 25 m and 5 m
deep, with a 6 by 6 m pit to 15 m against its west wall, under a platform
tower and four springboards. The hall around them has stands, a glazed
south wall the afternoon sun comes through, and a video board. The water is
clear and still, with hard walls and a floor, so it suits what the open
sea makes hard to see: an underwater vehicle's depth hold, a sonar or a DVL
against a known floor, a surface vehicle in a straight lane. There are no
lane ropes, so a vehicle has the pool's whole width.

| | |
|---|---|
| Launch | `ros2 launch kai_bringup simulation.launch.xml world:=aquatic_centre.sdf` |
| Start points | competition pool, lane 4 `x:=-20 y:=-1.25`; diving well, mid water `x:=45 z:=-2.5`; the pit `x:=36 z:=-12` |
| Water | A still pool, 3, 5 and 15 m deep |
| Sea states | 0 |

The world's origin is the middle of the water, x along the pools; lane 0
is the southern one, its centre at y -11.25, and every lane is 2.5 m north
of the last. The hall lives on
[Gazebo Fuel](https://app.gazebosim.org/honurobotics/fuel/models/aquatic_centre):
the first launch downloads it, about 3 MB, and later ones use the cache.

The Blue Robotics vehicles ([BlueBoat and BlueROV2](../vehicles/bluerobotics.md))
launch straight into it:

```bash
world=$(ros2 pkg prefix --share kai_gazebo)/worlds/aquatic_centre.sdf
ros2 launch bluerov2_gazebo sim.launch.xml world:=$world x:=45 z:=-2.5
ros2 launch blueboat_gazebo sim.launch.xml world:=$world x:=-20 y:=-1.25
```

## What to know

- **It keeps the contract, indoors.** The world runs the systems every
  vehicle needs, so every launch file, spawn command and service works in
  it unchanged. The wind and the ocean current are still, and the wave
  source runs at sea state 0 with no water surface model: the pool's water
  is the hall model's own, and a drawn sea would run through the walls.
  Raising the sea state changes nothing you can see.
- **The water is salt.** A pool is fresh, but buoyancy uses 1025 kg/m³, as
  on the sea: the vehicles are trimmed for sea water, and in fresh water
  the neutral BlueROV2 would sink.
- **The water is a skin.** Gazebo draws no water volume, so seen from
  under the surface the walls and the floor take the water's colour, but
  there is no haze.
- **Every collision is a box.** The pools' walls and floors, the deck, the
  stands, the tower, the springboards, the starting blocks and the hall's
  walls and roof all collide; the water, the glass, the flags and the
  fittings are drawn only.
- **Credits.** The hall is procedural, by Honu Robotics, CC BY 4.0; the sky
  outside the glass is Poly Haven's, CC0. Both are credited in
  `kai_gazebo/NOTICE`.
