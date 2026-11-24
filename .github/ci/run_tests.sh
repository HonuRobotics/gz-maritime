#!/usr/bin/env bash
# Copyright 2026 Honu Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Runs the workspace's tests inside the CI image, and copies the results to
# RESULTS_DIR (default /results) whatever happens.
#
#   run_tests.sh cpu   every test without the ctest label gpu
#   run_tests.sh gpu   the gpu-labelled tests, which must render on an NVIDIA
#                      GPU: a run with no test, a skipped test or another
#                      renderer fails
#
# Sourcing this file only defines gpu_verdict, for test_run_tests.sh.

WS=${WS:-/ws}
RESULTS_DIR=${RESULTS_DIR:-/results}
OGRE_LOG="$HOME/.gz/rendering/ogre2.log"
NVIDIA_EGL_VENDOR=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# gpu_verdict TEST_RESULT_FILE OGRE2_LOG
# Prints one line per failed check and returns 1 if any failed. On success,
# prints the renderer Ogre opened.
gpu_verdict() {
  local result_file=$1 ogre_log=$2 failed=0
  local summary_re='^Summary: ([0-9]+) tests?, ([0-9]+) errors?, ([0-9]+) failures?, ([0-9]+) skipped$'
  local summary tests=0 errors=0 failures=0 skipped=0
  summary=$(grep -E "$summary_re" "$result_file" 2>/dev/null | tail -n 1)
  if [[ $summary =~ $summary_re ]]; then
    tests=${BASH_REMATCH[1]} errors=${BASH_REMATCH[2]}
    failures=${BASH_REMATCH[3]} skipped=${BASH_REMATCH[4]}
  fi
  if ((tests == 0)); then
    echo "run_tests.sh: FAIL: no gpu-labelled tests ran"
    failed=1
  fi
  if ((errors + failures > 0)); then
    echo "run_tests.sh: FAIL: tests failed ($errors errors, $failures failures)"
    failed=1
  fi
  if ((skipped > 0)); then
    echo "run_tests.sh: FAIL: $skipped GPU test(s) skipped"
    failed=1
  fi

  local vendor="" reason=""
  if [[ ! -f $ogre_log ]]; then
    reason="no ogre2.log"
  else
    vendor=$(grep -oE 'GL_VENDOR = .*' "$ogre_log" | tail -n 1 | sed 's/[[:space:]]*$//')
    if [[ -z $vendor ]]; then
      reason="no GL_VENDOR line"
    elif [[ $vendor != "GL_VENDOR = NVIDIA Corporation" ]]; then
      reason=$vendor
    fi
  fi
  if [[ -n $reason ]]; then
    echo "run_tests.sh: FAIL: did not render on the NVIDIA GPU ($reason)"
    failed=1
  fi

  if ((failed)); then
    return 1
  fi
  local renderer
  renderer=$(grep -oE 'GL_RENDERER = .*' "$ogre_log" | tail -n 1 | sed -e 's/^GL_RENDERER = //' -e 's/[[:space:]]*$//')
  echo "run_tests.sh: GPU renderer: $renderer"
}

# Copies what the run left behind, pass or fail. Called on exit.
collect_results() {
  local mode=$1 dir
  mkdir -p "$RESULTS_DIR/test_results"
  for dir in "$WS"/build/*/test_results; do
    if [[ -d $dir ]]; then
      cp -r "$dir/." "$RESULTS_DIR/test_results/"
    fi
  done
  if [[ -e $WS/log/latest_test ]]; then
    cp -rL "$WS/log/latest_test" "$RESULTS_DIR/log"
  fi
  if [[ -f $OGRE_LOG ]]; then
    cp "$OGRE_LOG" "$RESULTS_DIR/ogre2.log"
  elif [[ $mode == gpu ]]; then
    : > "$RESULTS_DIR/ogre2.log.missing"
  fi
  # The caller made RESULTS_DIR; give it the files too.
  chown -R --reference="$RESULTS_DIR" "$RESULTS_DIR" ||
    echo "run_tests.sh: could not chown $RESULTS_DIR" >&2
}

main() {
  local mode=${1:-}
  local labels
  case $mode in
    cpu) labels=(-LE gpu) ;;
    gpu) labels=(-L gpu) ;;
    *)
      echo "usage: run_tests.sh cpu|gpu" >&2
      return 2
      ;;
  esac
  mkdir -p "$RESULTS_DIR"
  # shellcheck disable=SC2064  # expand mode now
  trap "collect_results $mode" EXIT

  if [[ $mode == gpu ]]; then
    export GZ_MARITIME_REQUIRE_RENDERING=${GZ_MARITIME_REQUIRE_RENDERING-1}
    # Only NVIDIA's EGL: libglvnd must never fall back to Mesa's software
    # renderer, which needs no GPU.
    export __EGL_VENDOR_LIBRARY_FILENAMES=${__EGL_VENDOR_LIBRARY_FILENAMES-$NVIDIA_EGL_VENDOR}
    rm -f "$OGRE_LOG"
    if command -v nvidia-smi >/dev/null; then
      nvidia-smi -L || true
    fi
  fi

  # The setup scripts read unset variables.
  set +u
  # shellcheck disable=SC1090,SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  cd "$WS"

  local status=0
  colcon test --merge-install --executor sequential --event-handlers console_cohesion+ \
    --ctest-args "${labels[@]}" --output-on-failure || status=$?
  if ((status != 0)); then
    echo "run_tests.sh: FAIL: colcon test exited $status"
  fi
  colcon test-result --test-result-base "$WS/build" --verbose > "$RESULTS_DIR/test-result.txt" 2>&1 ||
    status=1
  cat "$RESULTS_DIR/test-result.txt"
  if [[ $mode == gpu ]]; then
    gpu_verdict "$RESULTS_DIR/test-result.txt" "$OGRE_LOG" || status=1
  fi
  return "$status"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  set -euo pipefail
  main "$@"
fi
