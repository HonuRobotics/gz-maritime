#!/usr/bin/env bash
# Rebuild the shipwreck model from build_shipwreck.py and commission.yaml: the
# parts (meshes/), model.sdf and ground_truth/features.yaml, then check every
# part against the Honu glTF Asset Profile. Runs on the host, which has
# Blender and gltf-robotics' command-line tools (gltf-manifest, gltf-check);
# the .blend and the point cloud go to $SHIPWRECK_BUILD_DIR, outside the tree.
set -euo pipefail
cd "$(dirname "$0")"
blender --background --factory-startup --python build_shipwreck.py -- . 2>&1 | grep '^==='
gltf-manifest commission.yaml meshes/*_visual.glb >/dev/null
for f in meshes/*_visual.glb; do gltf-check "$f" | head -1; done
rm -f gltf-check-output.md
