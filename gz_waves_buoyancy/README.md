# gz_waves_buoyancy

Vehicles that float on the waves, not on flat water.

Every world here floats vehicles with `gz-maritime-buoyancy-system` on the
flat plane z = 0, and draws the waves on top: a crest passes through a hull
and moves nothing. This package is the wave field's physics consumer that
`WAVES_DESIGN.md` plans. A world that runs it instead of the flat buoyancy
floats every vehicle on the surface the renderer draws.

## Buoyancy on the waves

`gz-maritime-wave-buoyancy-system` (`gz::sim::maritime::WaveBuoyancy`) is a
world system that reads the same marks as the flat buoyancy: every collision
marked `gz:buoyancy="true"` (a box, a cylinder, a sphere, a capsule, an
ellipsoid or a cone) on any model, spawned later included, found by
`gz_marked_shapes` as the wind and the current find theirs. No vehicle
changes.

Every step, for each marked shape, it:

1. fits a plane through the height of the drawn surface at the shape's
   centre and at the ends of its footprint, along its heading and across it
   (`waves::SurfaceHeight`, which follows the sideways displacement of a
   choppy sea, so the height is that of the surface as drawn);
2. takes the volume of the shape under that plane (gz-math's shapes, as
   `gz_buoyancy` does with its flat plane);
3. pushes the link up with the weight of the water displaced, at the centre
   of that volume, and with the weight of the air displaced by the rest, at
   its centre.

A crest lifts a shape, a trough drops it, and the slope tilts it. Without a
wave field the water is flat at `<water_level>`, and a floating body rests
at the same draft as with the flat buoyancy.

It reads the wave field as the wave system last advanced it, at that
system's `<update_rate>`, and never advances it itself. The analytic
Gerstner engine answers at the step's time whatever the rate; the FFT
engine's surface moves in steps of its update. For smoother forces on the
FFT sea, set `<update_rate>0</update_rate>` on the wave system, which then
advances every step, at a cost in speed.

```xml
<plugin filename="gz-maritime-wave-buoyancy-system"
        name="gz::sim::maritime::WaveBuoyancy">
  <water_density>1025</water_density>   <!-- kg/m^3 -->
  <air_density>1</air_density>          <!-- kg/m^3, above the water -->
  <water_level>0</water_level>          <!-- world z of the still water -->
</plugin>
```

Run it **instead of** `gz-maritime-buoyancy-system`, never beside it. If
both run, the wave buoyancy leaves every link the flat buoyancy floats (it
gives each one a `Volume` component), with an error, so nothing floats
twice.

## The model, and its limits

The force is **hydrostatic only**: the weight of the water under the fitted
plane, straight up. Not modelled:

- the wave's pressure changing with depth (Froude-Krylov): a body wholly
  under the water feels still water, where a neutral body 3 m under the
  test wave would orbit about 0.14 m;
- any horizontal wave force: the waves lift and tilt a hull but do not push
  it along;
- the pressure field tilting with the wave: the water pushes straight up,
  not along the surface normal, so a hull heels more than the slope, by
  BM / GM (the test box, 1.3 times);
- water moving with the waves: the hydrodynamics still damps against still
  water (or the current).

One plane per shape is right while the shape is short against the
wavelength; the plane averages a chop shorter than the shape. A long hull is
better split into several marked shapes, as the BlueBoat's pontoons already
are. A marked mesh does not float here (it warns once), as the flat
buoyancy floats none.

## Tests

```bash
colcon test --packages-select gz_waves_buoyancy
```

- `displacement_test`: the volume and centre under, and over, flat, raised,
  lowered and sloping water for every shape that floats, the air's centre of
  a cone, and the fitted plane on still water, on a slope at any heading and
  on a ripple as long as the hull.
- `wave_buoyancy_test`:
  - on still water: a box rests level at the flat buoyancy's draft; a box
    heavy forward trims by the angle its metacentric height gives; a box
    whose shape and centre of mass are both off its link origin floats
    level; a body as heavy as the air it displaces hangs in the air;
  - with the flat buoyancy also running, the box floats once, at the flat
    draft;
  - on one long Gerstner wave, the box follows the surface (correlation
    over 0.9, about as far) and pitches with its slope, by the gain the
    hydrostatic model predicts; a neutral sphere 3 m down stays put (the
    limit above);
  - on the FFT engine's slight sea, the box follows the drawn surface.

## Not yet

- Wave-coupled physics is opt-in per world: no world in `kai_gazebo` runs
  it yet.
- The wind's and the current's marks still cut a shape at the flat
  `<water_level>`, not at the waves.
