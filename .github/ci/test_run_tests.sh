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

# Unit test for run_tests.sh's verdict on a GPU run: every way the run could
# pass without rendering on the NVIDIA GPU must fail it. Needs only bash:
#
#   bash .github/ci/test_run_tests.sh

set -uo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=run_tests.sh
source "$here/run_tests.sh" || exit 1

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

# colcon test-result --verbose, as it prints one result file and the summary.
result() {
  printf '%s\n' \
    "build/gz_maritime_gui/test_results/gz_maritime_gui/markers_camera_test.gtest.xml: $1" \
    "" \
    "Summary: $1"
}

# Ogre's log, as gz-rendering's ogre2 engine writes it. Ogre also probes
# Mesa's software device on a GPU machine; only the context it opens counts.
NVIDIA_LOG='13:20:33: EGL Device: EGL_MESA_device_software, Mesa Project
13:20:33: Created GL 4.5 context for device EGL_NV_device_cuda
13:20:33: GL_VERSION = 4.6.0 NVIDIA 580.95.05
13:20:33: GL_VENDOR = NVIDIA Corporation
13:20:33: GL_RENDERER = Tesla T4/PCIe/SSE2'
MESA_LOG='13:20:33: GL_VERSION = 4.5 (Core Profile) Mesa 25.2.3
13:20:33: GL_VENDOR = Mesa
13:20:33: GL_RENDERER = llvmpipe (LLVM 20.1.8, 256 bits)'

failed=0

# case NAME WANT_RC TEST_RESULT OGRE_LOG|absent [TEXT...]
# Each TEXT must be in the output; a TEXT starting with ! must not be.
case_() {
  local name=$1 want_rc=$2 test_result=$3 ogre_log=$4
  shift 4
  printf '%s' "$test_result" > "$tmp/$name.result"
  [[ $ogre_log == absent ]] || printf '%s\n' "$ogre_log" > "$tmp/$name.ogre2.log"
  local out rc=0 ok=1 text
  out=$(gpu_verdict "$tmp/$name.result" "$tmp/$name.ogre2.log") || rc=$?
  [[ $rc == "$want_rc" ]] || ok=0
  for text in "$@"; do
    if [[ $text == !* ]]; then
      [[ $out != *"${text#!}"* ]] || ok=0
    else
      [[ $out == *"$text"* ]] || ok=0
    fi
  done
  if ((ok)); then
    echo "ok $name"
  else
    echo "not ok $name: exit $rc, wanted $want_rc; output:"
    printf '    %s\n' "$out"
    failed=1
  fi
}

case_ zero 1 "Summary: 0 tests, 0 errors, 0 failures, 0 skipped" absent \
  "run_tests.sh: FAIL: no gpu-labelled tests ran" \
  "run_tests.sh: FAIL: did not render on the NVIDIA GPU (no ogre2.log)"
case_ empty 1 "" "$NVIDIA_LOG" \
  "run_tests.sh: FAIL: no gpu-labelled tests ran" "!did not render"
case_ skip 1 "$(result "1 test, 0 errors, 0 failures, 1 skipped")" "$NVIDIA_LOG" \
  "run_tests.sh: FAIL: 1 GPU test(s) skipped" "!no gpu-labelled tests ran"
case_ fail 1 "$(result "1 test, 0 errors, 1 failure, 0 skipped")" "$NVIDIA_LOG" \
  "run_tests.sh: FAIL: tests failed (0 errors, 1 failures)"
case_ mesa 1 "$(result "1 test, 0 errors, 0 failures, 0 skipped")" "$MESA_LOG" \
  "run_tests.sh: FAIL: did not render on the NVIDIA GPU (GL_VENDOR = Mesa)"
case_ pass 0 "$(result "1 test, 0 errors, 0 failures, 0 skipped")" "$NVIDIA_LOG" \
  "run_tests.sh: GPU renderer: Tesla T4/PCIe/SSE2" "!FAIL"

exit "$failed"
