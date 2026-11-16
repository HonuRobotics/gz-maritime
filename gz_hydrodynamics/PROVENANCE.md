`src/Hydrodynamics.cc`, `src/Hydrodynamics.hh` and `src/HydrodynamicsUtils.hh`
are vendored from gz-sim:

    https://github.com/gazebosim/gz-sim
    tag gz-sim10_10.5.0, path src/systems/hydrodynamics/

The files are identical on gz-sim `main` at the time of vendoring. Vendor them
verbatim in the commit that adds them, so `git log -p src/` shows our delta
and nothing else. To see it against a newer upstream:

    curl -sSL https://raw.githubusercontent.com/gazebosim/gz-sim/<newtag>/src/systems/hydrodynamics/Hydrodynamics.cc \
      | diff -u - src/Hydrodynamics.cc

## The delta

1. **The world's ocean current.** When the world carries the
   `OceanCurrentfield` recipe of `gz_ocean_current`, the plugin keeps an
   `OceanCurrentSampler`, synced every `PreUpdate`, asks it at the link's
   centre of mass (`WorldInertialPose`), or at the centre of pressure of
   delta 6 when the model names one, and damps against the velocity
   relative to that water, ν − ν_c, in added mass, Coriolis and damping:
   Fossen's model as written. The world owns the current, so the plugin's
   `<default_current>`, `<lookup_current_*>` and its ocean current topic
   (`/ocean_current`, or `/model/<namespace>/ocean_current`) are then
   ignored: a warning once if the plugin set one of the tags, and once at the
   first message on the topic. Without a recipe, upstream's behaviour is
   unchanged.
2. **A current table for a vehicle spawned at run time.** Upstream discovers
   the table loaded by `EnvironmentPreload` with `EachNew<Environment>` in
   `PostUpdate`, which only matches the world's component in the iteration
   it is created; a plugin configured after that, on a model spawned at run
   time, never sees the table, and the model sits still in a current loaded
   from a file. That discovery is the whole bug. The world's Environment
   component (on `worldEntity(_ecm)`) is read every `PreUpdate` instead; the
   lookup sessions are rebuilt only when the data's shared pointer changes
   (a reload through `EnvironmentPreload`'s topic), and cleared when the
   component goes. `ISystemPostUpdate`, whose only job was that discovery,
   goes. Its own commit, meant for upstream as the first of the ocean current
   fixes.
3. **The leaf namespace** `systems` → `maritime`, in the plugin and in
   `HydrodynamicsUtils.hh`, so the registered alias
   `gz::sim::maritime::Hydrodynamics` does not collide with gz-sim's own when
   both libraries are on `GZ_SIM_SYSTEM_PLUGIN_PATH`.

4. **The sign of the added mass Coriolis term.** Upstream applies
   +C_A(ν)ν to the link where Fossen's equation has −C_A(ν)ν:
   `buildCoriolisMatrix` returns C_A for the negative coefficients the
   plugin takes, not −C_A as its comment said, and `PreUpdate` negated it
   once too often. A body with more added mass in sway than in surge was
   turned nose on to its motion instead of broadside (the Munk moment), by
   the right amount in the wrong direction. Fixed, with a test against
   Kirchhoff's equations and against DART's `<fluid_added_mass>`, which
   agree with each other. Meant for upstream.

5. **The link's `<fluid_added_mass>` is seen.** Upstream reads it in
   `Configure` before it creates the link's world pose component, which the
   read needs. Unless another system made that component first the added
   mass is not found, and neither the Coriolis correction in a current nor
   the error for added mass given twice ever runs. Here the components come
   first. With the correction at work a body with `<fluid_added_mass>` in a
   current stays within centimetres of one with the same added mass in the
   plugin, against a quarter of a metre without. The plugin's own Coriolis
   force is not applied to such a link, since the physics engine already
   applies it: with the old sign the two cancelled, with the fix they would
   double.

   Deltas 4 and 5 are proposed upstream in gazebosim/gz-sim#4209.

6. **A centre of pressure.** Upstream takes the velocity at the link origin
   and applies the wrench there, so a link whose origin is off the thrust
   line pitches or rolls under way: thrust and drag make a couple.
   `<center_of_pressure>`, a point in the link frame, moves both: the
   velocity is the point's, the current is asked there and the wrench acts
   at it. That is Fossen's change of origin, so the model is the one a link
   frame at that point would give; a test compares the two. Without the tag
   the point is the link origin and the current is asked at the centre of
   mass, upstream's behaviour, bit for bit. The Coriolis correction for a
   link's `<fluid_added_mass>` keeps the link origin, where that matrix is
   defined. Meant for upstream.

This package is a fork until upstream has a current component its
Hydrodynamics reads, and deltas 4 to 6. Retire it when a Gazebo release that
ROS ships carries them all, and switch the vehicles back to
`gz-sim-hydrodynamics-system`.
