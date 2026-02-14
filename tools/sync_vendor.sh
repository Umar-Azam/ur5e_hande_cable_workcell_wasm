#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if [[ ! -d node_modules ]]; then
  echo "node_modules not found. Run: npm install"
  exit 1
fi

mkdir -p vendor/three/build
mkdir -p vendor/three/examples/jsm/controls
mkdir -p vendor/three/examples/jsm/libs
mkdir -p vendor/mujoco-js

cp node_modules/three/build/three.module.js vendor/three/build/
cp node_modules/three/build/three.core.js vendor/three/build/
cp node_modules/three/examples/jsm/controls/OrbitControls.js vendor/three/examples/jsm/controls/
cp node_modules/three/examples/jsm/libs/lil-gui.module.min.js vendor/three/examples/jsm/libs/

cp node_modules/mujoco-js/dist/mujoco_wasm.js vendor/mujoco-js/
# mujoco-js@0.0.7 embeds the wasm payload directly in mujoco_wasm.js.
# Keep this script resilient for both embedded and split-js/wasm package variants.
if [[ -f node_modules/mujoco-js/dist/mujoco_wasm.wasm ]]; then
  cp node_modules/mujoco-js/dist/mujoco_wasm.wasm vendor/mujoco-js/
fi

echo "Vendor sync complete."
