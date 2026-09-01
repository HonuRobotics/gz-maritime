`src/Thruster.cc` and `src/Thruster.hh` are vendored from gz-sim:

    https://github.com/gazebosim/gz-sim
    tag gz-sim10_10.5.0, path src/systems/thruster/

Vendored verbatim in the commit that added them, so `git log -p src/` shows
our delta and nothing else. To see it against a newer upstream:

    curl -sSL https://raw.githubusercontent.com/gazebosim/gz-sim/<newtag>/src/systems/thruster/Thruster.cc \
      | diff -u - src/Thruster.cc

Keep the delta small and additive. It is intended to go upstream, and the
file has been nearly static across releases — one line changed between
10.4.0 and 10.5.0.
