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

## Water that moves with the waves

The system also marks the world with the `WaveCoupling` component (from
`gz_waves`), holding the water level. `gz_hydrodynamics` sees it and damps a
vehicle against the current **plus** the waves' water velocity at its centre
of mass: the surface particle velocity, faded with depth as e^(-k d), with k
the deep-water wavenumber of the sea's peak period
(`waves::WaterVelocity`). A vehicle near the surface is carried back and
forth by the waves; one deep down hardly feels them. In a world without the
wave buoyancy, nothing changes.

## Tests

```bash
colcon test --packages-select gz_waves_buoyancy gz_hydrodynamics
```

- `displacement_test`: the volume and centre under flat, raised, lowered and
  sloping water, for a box, a sphere and a cylinder.
- `wave_buoyancy_test`, on one long Gerstner wave:
  - a box on still water rests at the flat buoyancy's draft;
  - on the wave, it follows the surface above it (correlation over 0.9, about
    as far);
  - a neutral sphere 3 m down does not move.
- `gz_hydrodynamics`' `hydrodynamics_waves_test`: two damped neutral spheres
  under the same wave. The one 0.6 m down swings 0.37 m, the one 5 m down
  0.12 m, as e^(-k d) predicts. With the waves only drawn, neither moves.

## Not yet

- Wave-coupled physics is opt-in per world: no world in `kai_gazebo` runs it
  yet.
- The water's motion fades with depth at one rate, the sea's peak wave's.
  A real sea fades its short waves faster, so deep down the motion is
  overstated: at 30 m, 40 to 80 % more than linear theory at sea states 5
  to 9.
- Added mass sees the waves' velocity but not their acceleration (the
  Froude–Krylov force); vehicles here carry no added mass.
- The Gerstner engine's particle velocity has its phase wrong (#15, fixed in
  #70). The FFT engine, which every world uses, is not affected.
