`src/Buoyancy.cc` and `src/Buoyancy.hh` are vendored from gz-sim:

    https://github.com/gazebosim/gz-sim
    tag gz-sim10_10.5.0, path src/systems/buoyancy/

Vendored verbatim in the commit that added them, so `git log -p src/Buoyancy.*`
shows our delta and nothing else. To see it against a newer upstream:

    curl -sSL https://raw.githubusercontent.com/gazebosim/gz-sim/<newtag>/src/systems/buoyancy/Buoyancy.cc \
      | diff -u - src/Buoyancy.cc

Keep the delta small and additive. It is intended to go upstream, and the file
has been nearly static across releases.

`src/BuoyancyEnable.cc` and `src/BuoyancyEnable.hh` are not vendored: they are
this package's own model-side plugin, which calls the enable service the delta
above adds.
