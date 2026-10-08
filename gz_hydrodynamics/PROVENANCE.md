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
   `OceanCurrentfield` recipe of `gz_ocean_current`, the plugin asks
   `OceanCurrentSampler::At` at the link's centre of mass every step and
   damps against the velocity relative to that water, Fossen's model as
   written. The world owns the current, so the plugin's `<default_current>`,
   `<lookup_current_*>` and the `/ocean_current` topic are then ignored, with
   a warning if the plugin set any. Without a recipe, upstream's behaviour is
   unchanged.
2. **A current table for a vehicle spawned at run time.** Upstream finds the
   table loaded by `EnvironmentPreload` with `EachNew<Environment>`, which
   only matches the world entity in its first iteration, so a model spawned
   later never sees it and sits still in a current loaded from a file. The
   world's Environment component is read every `PreUpdate` instead, and the
   lookup sessions are rebuilt when its data set changes (a reload). The
   world entity is found by its component, at the first `PreUpdate`, not by
   walking up from the model in `Configure`: in gz-sim 10 a model spawned at
   run time is not parented yet when it is configured.
   `ISystemPostUpdate`, whose only job was that discovery, goes. Sent upstream
   as the first of the ocean current fixes.
3. **The leaf namespace** `systems` → `maritime`, in the plugin and in
   `HydrodynamicsUtils.hh`, so the registered alias
   `gz::sim::maritime::Hydrodynamics` does not collide with gz-sim's own when
   both libraries are on `GZ_SIM_SYSTEM_PLUGIN_PATH`.

This package is a fork until upstream has a current component and its
Hydrodynamics reads it. Retire it when a Gazebo release that ROS ships
carries both, and switch the vehicles back to `gz-sim-hydrodynamics-system`.
