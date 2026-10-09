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
   centre of mass (`WorldInertialPose`) and damps against the velocity
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

This package is a fork until upstream has a current component and its
Hydrodynamics reads it. Retire it when a Gazebo release that ROS ships
carries both, and switch the vehicles back to `gz-sim-hydrodynamics-system`.
