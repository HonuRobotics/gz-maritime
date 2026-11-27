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

# Checks that bake tags exactly the image the test jobs pull, IMAGE, plus the
# EXTRA_TAG tag when there is one. Bake takes any environment variable that
# shares a name with one of its variables, so a job's environment can change
# the tags. The image job runs this before the build, in its own environment;
# locally:
#
#   IMAGE=ghcr.io/honurobotics/gz-maritime-ci:sha-test TAG=sha-test \
#     EXTRA_TAG=lyrical bash .github/ci/check_image_tags.sh

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

want=$(jq -cn --arg image "$IMAGE" --arg extra "${EXTRA_TAG:-}" '
  [$image] + if $extra == "" then [] else [($image | sub(":[^:/]*$"; "")) + ":" + $extra] end')
got=$(docker buildx bake -f "$here/docker-bake.hcl" ci --print --progress=quiet | jq -c '.target.ci.tags')

if [[ $got != "$want" ]]; then
  echo "check_image_tags.sh: FAIL: bake would tag $got, not $want" >&2
  exit 1
fi
echo "check_image_tags.sh: bake tags $got"
