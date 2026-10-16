# Sail the competition sites

Besides the open water world, gz-maritime ships four real venues: the three
sites of the Virtual RobotX competition (VRX) and the one of the Virtual
Ocean Robotics Challenge (VORC), adapted to this ocean. They keep the same
world contract as `open_water.sdf`, so every launch file, spawn command and
service works on them unchanged. Pass one to the simulation launch and
spawn a vehicle at the start point the site recommends:

```bash
ros2 launch kai_bringup simulation.launch.xml world:=sand_island.sdf
```

```bash
ros2 launch kai_bringup spawn_vehicle.launch.xml name:=boat_a x:=60 y:=120 yaw:=-2.76 \
  xacro:=$(ros2 pkg prefix --share kai_custom_vehicle)/models/custom_usv/model.sdf.xacro \
  bridge:=$(ros2 pkg prefix --share kai_custom_vehicle)/config/ros_gz_bridge.yaml.in \
  urdf:=$(ros2 pkg prefix --share kai_custom_vehicle)/urdf/custom_usv.urdf.xacro
```

The custom USV's own launch does both in one go: `ros2 launch
kai_custom_vehicle sim.launch.xml world:=sand_island.sdf x:=60 y:=120
yaw:=-2.76`. Any other vehicle spawns the same way, see
[Spawn and drive](spawn-and-drive.md).

Every world is named `default` inside, whatever its file is called, so the
wave service and the other world services take the same path everywhere:

```bash
gz service -s /world/default/wave/set_parameters --reqtype gz.msgs.Param \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'params {key: "sea_state" value {type: INT32 int_value: 2}}'
```

The sea state goes from 0 (glassy) to 9, and each site says below which
range fits its water. A rowing lake never sees more than wind chop, and a
lagoon or a gulf at most a slight sea; higher codes are open sea and look
wrong between shores. Sand Island takes any code: it sets the open ocean,
and its sheltered water gets a share of it.

## Sydney International Regatta Centre

```{figure} images/sydney-regatta.jpg
:alt: A rowing lake seen from its shore, with trees and a boathouse on the far bank and the custom USV on the water

The Sydney Regatta world at its opening view.
```

The VRX venue from 2022 to 2024: the man made rowing course at Penrith
Lakes, near Sydney, with its pontoon, shore and buildings. The terrain is
the VRX model from Fuel, about 140 MB, downloaded the first time the world
loads and cached under `~/.gz/fuel`.

| | |
|---|---|
| Launch | `ros2 launch kai_bringup simulation.launch.xml world:=sydney_regatta.sdf` |
| Start point | `x:=-532 y:=162 yaw:=1` |
| Water | A freshwater lake, flat and sheltered |
| Sea states | 0 to 2 |

A landing pad sits off the start point, for the X500 of the
[multi vehicle demo](several-vehicles.md), which runs here by default.

## Nathan Benderson Park

```{figure} images/benderson-park.jpg
:alt: A long rowing lake lined with trees, with a grandstand building on its shore, seen from above the north bank

The Benderson Park world at its opening view.
```

The RobotX 2022 venue in Sarasota, Florida: a man made rowing lake with
its grandstand and island. The terrain is the VRX model from Fuel, about
220 MB, downloaded on first use like Sydney's. The lake runs along y, so a
vehicle at the origin heading 1.57 looks down the course.

| | |
|---|---|
| Launch | `ros2 launch kai_bringup simulation.launch.xml world:=benderson_park.sdf` |
| Start point | `x:=0 y:=0 yaw:=1.57` |
| Water | A freshwater lake, flat and sheltered |
| Sea states | 0 to 2 |

## Sand Island

```{figure} images/sand-island.jpg
:alt: Honolulu harbour under trade wind clouds, container cranes along the quay, downtown towers and the Koolau range behind

The Sand Island world at its opening view, across the harbour.
```

The RobotX 2016 and 2018 venue in Honolulu, Hawaii: the Kalihi channel off
Sand Island, between Keehi Lagoon and Honolulu Harbor, with the container
terminal's cranes, the marinas, downtown Honolulu and the Koolau range
behind it, and the reef and the ocean to the south. It is built from open
data (OpenStreetMap, satellite imagery and elevation) and its models live
on [Gazebo Fuel](https://app.gazebosim.org/honuRobotics): the first launch
downloads the site, about 9 MB, and later ones use the
cache. The world keeps the VRX world's coordinates, but the start point has
moved onto the water: the VRX one is on Sand Island's shore as the map
draws it.

| | |
|---|---|
| Launch | `ros2 launch kai_bringup simulation.launch.xml world:=sand_island.sdf` |
| Start point | `x:=60 y:=120 yaw:=-2.76` |
| Water | The harbour channel and the open ocean, salt |
| Sea states | any; 0 to 3 in the channel |

The world starts at sea state 3, a slight sea. The sea state sets the open
ocean; a shelter map on the water scales the waves down where land, the
reef and the piers shelter it, so the channel stays calmer and the quays
calm (see below).

The world has an optional seabed, from NOAA's topobathymetric model of
Hawaii, for underwater vehicles, sonars and DVLs, or a hull grounding on
the shallows. It is off by default; uncomment its include at the end of
`sand_island.sdf` and the first launch downloads it too, about 6 MB.

## La Spezia

```{figure} images/la-spezia.jpg
:alt: A marina seen from over the sea, a pier pointing at the viewer, quays and a town behind it and hills above

The La Spezia world at its opening view, from over the gulf.
```

The VORC 2020 venue: the marina in the Gulf of La Spezia, Italy, with its
breakwaters and quays. The terrain mesh comes from the VORC repository and
is fetched by the build. Inside the breakwaters the water is calm; the gulf
beyond them sees a slight sea at most.

| | |
|---|---|
| Launch | `ros2 launch kai_bringup simulation.launch.xml world:=la_spezia.sdf` |
| Start point | `x:=10 y:=-372 yaw:=0.3816` |
| Water | The marina and the gulf |
| Sea states | 0 to 2 in the marina, up to 3 in the gulf |

## What to know

- **Terrain is solid.** Every site's terrain mesh is its collision too, so a
  vehicle driven into the shore grounds on it. The water has no floor: a
  sinking vehicle keeps sinking, as on open water.
- **The water is one model.** Each site includes the water surface once, at
  the centre of its water, and sets `<tiles_radius>` in its wave source to
  say how far the sea is drawn around it, in 200 m tiles; the terrain hides
  it wherever there is land. Buoyancy does not depend on the drawn water:
  vehicles float on the flat z = 0 plane everywhere.
- **First use needs the network.** The Sydney and Benderson Park terrains
  and the Sand Island site come from Fuel the first time they load; after
  that they are cached. The La Spezia mesh is fetched by the build, so
  building gz-maritime needs the network once too.
- **Sand Island's water is sheltered.** Its water model, which ships with
  `kai_gazebo`, carries a shelter map,
  the share of the open sea's waves that reaches each point: the full sea
  state offshore, less in the channel, none against a quay or a beach. The
  waves also fade out over the last tile of the drawn sea, where a static
  water plane carries on. It changes only what is drawn.
- **Credits.** The Sydney, Benderson Park and La Spezia terrains are
  Apache 2.0 assets from VRX and VORC; Sand Island is CC BY 4.0, built from
  OpenStreetMap (ODbL), Sentinel-2 cloudless imagery (CC BY 4.0), the AWS
  Terrain Tiles and a Poly Haven sky (CC0). All are credited in
  `kai_gazebo/NOTICE`.

The worlds table in the
[reference](../reference/launch-and-world.md#worlds) has the same facts in
one place, and [Use your own world](own-world.md) explains the contract
they share if you want to build a site of your own.
