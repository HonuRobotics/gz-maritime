# Vehicles

| Vehicle | What it is | Where it comes from |
|---|---|---|
| [Custom USV](custom-usv.md) | A 1 m catamaran, the worked example of the how-to guides | This repository |
| [BlueBoat and BlueROV2](bluerobotics.md) | Blue Robotics' surface boat and underwater ROV | [bluerobotics_models](https://github.com/HonuRobotics/bluerobotics_models) |
| [Holybro X500](holybro.md) | A quadcopter drone, with its own world: not a maritime vehicle, but built the same way | [holybro_models](https://github.com/HonuRobotics/holybro_models) |

Any other vehicle works too; [Bring your own vehicle](../how-to/own-vehicle.md)
shows how, and [Several vehicles on one ocean](../how-to/several-vehicles.md)
puts one of each on the water at once.

## What a world provides, what a vehicle brings

A vehicle is spawned into a world it has never seen, under a name, and
works because each side keeps to a short contract.

The world provides, and every world shipped here does
([Use your own world](../how-to/own-world.md)):

- the physics, user commands and scene broadcaster systems;
- the sensor systems: IMU, magnetometer, NavSat, air pressure and rendered
  sensors, with `<spherical_coordinates>` for the GPS and the magnetic
  field;
- the gz-maritime buoyancy system, reading marked collisions and naming
  nobody, with the waterline at z = 0;
- a wave source and the drawn sea, and somewhere solid for a vehicle that
  does not float, like the landing pad.

The vehicle brings, for one instance name
([Spawn and drive](../how-to/spawn-and-drive.md#what-the-launch-needs-from-a-vehicle)):

- a model whose name, plugin topics and sensor frames all carry the
  instance name, with its displacement collisions marked
  `gz:buoyancy="true"`;
- a bridge configuration for those topics, without a clock entry, and a
  URDF for `robot_state_publisher`, when it has them;
- either a model xacro that takes `name:=`, or a generator that writes the
  three files for `--name` and `--out-dir`.

```{toctree}
:hidden:
:maxdepth: 1

custom-usv
bluerobotics
holybro
```
