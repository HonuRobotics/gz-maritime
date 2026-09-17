# How to guides

## Bring your own vehicle

Follow these in order the first time. Every step uses the
[custom USV](../vehicles/custom-usv.md) as the worked example, so there is
always a real file to look at, and the snippets on these pages are taken
from its files.

1. [Overview](own-vehicle.md): what the world gives you, what your vehicle
   brings, and how a vehicle starts floating.
2. [Describe the vehicle](vehicle-description.md): the URDF, with its frames,
   mass, collision shapes and waterline.
3. [Compose the Gazebo model](gazebo-composition.md): buoyancy registration,
   thrusters, damping and sensors on top of the URDF.
4. [Spawn and drive](spawn-and-drive.md): the spawn launch, the bridge, and
   driving.
5. [Add sensors](sensors.md): IMU, GPS, magnetometer and cameras.

## More

- [Sail the competition sites](site-worlds.md)
- [Use your own world](own-world.md)
- [Troubleshooting](faq.md)

```{toctree}
:hidden:
:maxdepth: 1

own-vehicle
vehicle-description
gazebo-composition
spawn-and-drive
sensors
site-worlds
own-world
faq
```
