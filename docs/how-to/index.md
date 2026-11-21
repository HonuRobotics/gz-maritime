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

## Sonar

Imaging, sidescan and profiling sonars, from watching one to labeled training data.

1. [Imaging (forward looking) sonar from the ROV](sonar-imaging.md)
2. [Profiling and sidescan sonar from the USV](sonar-survey.md)
3. [Add a sonar to your vehicle](sonar-add.md)
4. [Sonar features: acoustic properties and truth labels](sonar-features.md)
5. [Map a shipwreck](sonar-shipwreck.md)

## More

- [Several vehicles on one ocean](several-vehicles.md)
- [Sail the competition sites](site-worlds.md)
- [Dive in the aquatic centre](aquatic-centre.md)
- [Use your own world](own-world.md)
- [Change and see the wind and the current](wind-and-current.md)
- [Troubleshooting](faq.md)

```{toctree}
:hidden:
:maxdepth: 1

own-vehicle
vehicle-description
gazebo-composition
spawn-and-drive
sensors
several-vehicles
site-worlds
aquatic-centre
own-world
wind-and-current
sonar-imaging
sonar-survey
sonar-add
sonar-features
sonar-shipwreck
faq
```
