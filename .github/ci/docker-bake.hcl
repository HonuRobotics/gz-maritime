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

# The CI image. CI adds the caches and the output with --set; a local build
# is: docker buildx bake -f .github/ci/docker-bake.hcl ci --load

# Not IMAGE: the image job's IMAGE (repository and tag) would override it.
variable "IMAGE_REPO" {
  default = "ghcr.io/honurobotics/gz-maritime-ci"
}

variable "TAG" {
  default = "local"
}

# A second tag, or empty for none.
variable "EXTRA_TAG" {
  default = ""
}

# Rebuilds the dependency layer when it changes; CI passes the ISO week.
variable "DEPS_EPOCH" {
  default = "local"
}

# Compiler processes; empty means one per CPU.
variable "BUILD_JOBS" {
  default = ""
}

target "ci" {
  context    = "."
  dockerfile = ".github/ci/Dockerfile"
  args = {
    DEPS_EPOCH = DEPS_EPOCH
    BUILD_JOBS = BUILD_JOBS
  }
  tags = compact(["${IMAGE_REPO}:${TAG}", EXTRA_TAG == "" ? "" : "${IMAGE_REPO}:${EXTRA_TAG}"])
  # One manifest per tag, so the cleanup job never deletes a child manifest
  # that a tagged index still points to.
  attest = ["type=provenance,disabled=true", "type=sbom,disabled=true"]
}
