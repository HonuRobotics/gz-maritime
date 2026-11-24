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

WS=${WS:-/ws}
RESULTS_DIR=${RESULTS_DIR:-/results}
OGRE_LOG="$HOME/.gz/rendering/ogre2.log"

# Copies what the run left behind, pass or fail. Called on exit.
collect_results() {
  local dir
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
    *)
      echo "usage: run_tests.sh cpu" >&2
      return 2
      ;;
  esac
  mkdir -p "$RESULTS_DIR"
  trap collect_results EXIT

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
  return "$status"
}

set -euo pipefail
main "$@"
