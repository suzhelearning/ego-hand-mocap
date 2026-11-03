#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
deps=$(realpath "${CONDA_PREFIX:-$repo/.pixi/envs/default}")
cmake --fresh -S "$repo" -B "$repo/build" \
  -DCMAKE_BUILD_TYPE=Release -DUSE_NN=OFF -DHAVE_LIBREALSENSE=OFF \
  -DBUILD_ROS2=OFF -DUSE_COLIDMAP=OFF -DUSE_CUDA=OFF \
  -DCMAKE_PREFIX_PATH="$deps" \
  -DCMAKE_MODULE_PATH="$deps/share/cmake/geographiclib"
cmake --build "$repo/build" --target okvis_app_synchronous -j"${OKVIS_BUILD_JOBS:-2}"
cp "$repo/resources/small_voc.yml.gz" "$repo/build/small_voc.yml.gz"
