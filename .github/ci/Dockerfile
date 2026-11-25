# syntax=docker/dockerfile:1
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

# The CI image: the workspace built in /ws, with every dependency installed.
# The test jobs run .github/ci/run_tests.sh inside it. Build it with
# docker buildx bake -f .github/ci/docker-bake.hcl ci

# Pinned on purpose: a moving tag would change the base under a running week
# and throw away every cached layer. Bump the digest deliberately.
FROM ros:lyrical-ros-base@sha256:c69a7a39142b0a19c8cc5e80a35e9087e2ae46080dfa0a59961ff72287fdc5f6 AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN printf 'Acquire::Retries "3";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n' \
      > /etc/apt/apt.conf.d/80-network-hardening

# Only the manifests. A source edit leaves this stage, and the deps layer
# after it, cached.
FROM base AS manifests
COPY --parents */package.xml /ws/src/gz-maritime/

FROM base AS deps
# The ISO week (e.g. 2026-W47). Changing it is the only thing, besides a
# package.xml or the base digest, that rebuilds this layer. It is unused in
# the command on purpose: a build argument is part of every later RUN's
# cache key.
ARG DEPS_EPOCH
# The upgrade brings the ROS packages of the base image to the same sync as
# the ones rosdep adds. --with-new-pkgs installs the new dependencies of a
# newer version, as apt upgrade does; apt-get upgrade alone keeps such a
# package back. A package whose upgrade removes another is still kept back.
RUN --mount=type=bind,from=manifests,source=/ws/src,target=/ws/src \
    apt-get update && apt-get upgrade -y --with-new-pkgs && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install -y --from-paths /ws/src --ignore-src && \
    apt-get install -y --no-install-recommends ccache libegl1 && \
    rm -rf /var/lib/apt/lists/*

FROM deps AS ci
ENV CCACHE_DIR=/ccache CCACHE_MAXSIZE=1G
# Compiler processes; empty means one per CPU.
ARG BUILD_JOBS
WORKDIR /ws
COPY . src/gz-maritime/
# Make, not Ninja: colcon adds no -j of its own when MAKEFLAGS has one, so
# BUILD_JOBS bounds the compilers, and Ninja would ignore it. ctest needs
# the build tree, not the object files.
RUN --mount=type=cache,id=ccache,target=/ccache \
    source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    ccache --zero-stats && \
    MAKEFLAGS="-j${BUILD_JOBS:-$(nproc)}" colcon build --merge-install --executor sequential \
      --event-handlers console_cohesion+ \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
    ccache --print-stats && \
    find build -name '*.o' -delete
