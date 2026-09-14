# How to guides

## Bring your own vehicle

Follow these in order the first time. Every step uses the
[tutorial USV](../vehicles/tutorial-usv.md) as the worked example, so there is
always a real file to look at.

1. [Overview](own-vehicle.md): what the world gives you, what your vehicle
   brings, and how a vehicle starts floating.
2. [Describe the vehicle](vehicle-description.md): the URDF, with its frames,
   mass, collision shapes and waterline.
3. [Compose the Gazebo model](gazebo-composition.md): buoyancy registration,
   thrusters, damping and sensors on top of the URDF.
4. [Spawn and drive](spawn-and-drive.md): your launch file, the bridge, and
   driving.
5. [Add sensors](sensors.md): IMU, GPS, magnetometer and cameras.

## More

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
own-world
faq
```
