# gz_waves_buoyancy

Vehicles that float on the waves, not on flat water.

Every world here floats vehicles with `gz-maritime-buoyancy-system` on the
flat plane z = 0, and draws the waves on top: a crest passes through a hull
and moves nothing. This package is the wave field's physics consumer that
`WAVES_DESIGN.md` plans. A world that runs it instead of the flat buoyancy
floats every vehicle on the same wave field the renderer draws.

## Buoyancy on the waves

`gz-maritime-wave-buoyancy-system` (`gz::sim::maritime::WaveBuoyancy`) is a
world system that reads the same marks as the flat buoyancy: every collision
marked `gz:buoyancy="true"` (a box, a cylinder or a sphere) on any model,
spawned later included. No vehicle changes.

Every step, for each marked shape, it:

1. asks the wave field, advanced to that step, for the surface height and
   its normal above the shape's centre;
2. takes the volume of the shape under that tangent plane (gz-math's shapes,
   as `gz_buoyancy` does with its flat plane);
3. pushes the link up with the weight of the water displaced, at the centre
   of that volume, and with the weight of the air displaced by the rest.

A crest lifts a shape, a trough drops it, and the slope tilts it. A shape
wholly under the water feels no change from the waves overhead, as it
should. Without a wave field the water is flat at `<water_level>`, and a
floating body rests at the same draft as with the flat buoyancy.

```xml
<plugin filename="gz-maritime-wave-buoyancy-system"
        name="gz::sim::maritime::WaveBuoyancy">
  <water_density>1025</water_density>   <!-- kg/m^3 -->
  <air_density>1</air_density>          <!-- kg/m^3, above the water -->
  <water_level>0</water_level>          <!-- world z of the still water -->
</plugin>
```

Run it **instead of** `gz-maritime-buoyancy-system`, never beside it, or each
marked shape is pushed twice. One tangent plane per shape is right while the
shape is short against the wavelength; a long hull is better split into
several marked shapes, as the BlueBoat's pontoons already are.

## The world is marked wave coupled

The system also marks the world with the `WaveCoupling` component (from
`gz_waves`), holding the water level: the sign, for any system that damps a
vehicle against the water, that the water here moves with the waves.

## Tests

```bash
colcon test --packages-select gz_waves_buoyancy
```

- `displacement_test`: the volume and centre under flat, raised, lowered and
  sloping water, for a box, a sphere and a cylinder.
- `wave_buoyancy_test`, on one long Gerstner wave:
  - a box on still water rests at the flat buoyancy's draft;
  - on the wave, it follows the surface above it (correlation over 0.9, about
    as far);
  - a neutral sphere 3 m down does not move.

## Not yet

- Wave-coupled physics is opt-in per world: no world in `kai_gazebo` runs it
  yet.
- The waves lift a vehicle but do not carry it: its hydrodynamics still damps
  against still water (or the current), not water moving with the waves.
