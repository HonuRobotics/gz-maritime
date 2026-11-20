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

# Deletes the CI image's versions that nothing needs any more: older than
# MAX_AGE_DAYS, and untagged or tagged only sha-<commit>. A version tagged
# lyrical is never deleted.
#
#   DRY_RUN=1          print what would be deleted, delete nothing
#   VERSIONS_JSON=FILE read the versions from FILE instead of the API
#   MAX_AGE_DAYS=N     default 30
#   NOW=DATE           measure ages from DATE instead of now
#
# Needs gh and jq. CI passes GH_TOKEN; run locally, gh uses its own login.

set -euo pipefail

readonly VERSIONS_API=/orgs/HonuRobotics/packages/container/gz-maritime-ci/versions

max_age_days=${MAX_AGE_DAYS:-30}
now=$(date -u -d "${NOW:-now}" +%Y-%m-%dT%H:%M:%SZ)

if [[ -n ${VERSIONS_JSON:-} ]]; then
  versions=$(cat "$VERSIONS_JSON")
else
  # One array, however many pages.
  versions=$(gh api --paginate "$VERSIONS_API" | jq -s 'add // []')
fi

# One line per version to delete: id, updated_at, tags or "untagged".
selected=$(jq -r --arg now "$now" --argjson max_age_days "$max_age_days" '
  ($now | fromdateiso8601) as $now_s
  | .[]
  | (.metadata.container.tags // []) as $tags
  | select($now_s - (.updated_at | sub("\\.[0-9]+Z$"; "Z") | fromdateiso8601)
           > $max_age_days * 86400)
  | select(all($tags[]; startswith("sha-")))
  | "\(.id) \(.updated_at) \(if $tags == [] then "untagged" else $tags | join(",") end)"
' <<< "$versions")

while read -r id updated_at tags; do
  if [[ -z $id ]]; then
    continue
  fi
  if [[ ${DRY_RUN:-} == 1 ]]; then
    echo "would delete $id $updated_at $tags"
  else
    echo "delete $id $updated_at $tags"
    gh api -X DELETE "$VERSIONS_API/$id" --silent
  fi
done <<< "$selected"
