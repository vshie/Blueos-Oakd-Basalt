#!/bin/bash
# Builds the arm64 extension image locally and saves it as a .tar for BlueOS's extension upload.
# Uses the prebuilt builder image, so only the slim runtime stage is built (a few minutes).
#   TAG=mytest ./build_tar.sh
set -euo pipefail

TAG=${TAG:-local}
IMAGE=${IMAGE:-blueos-oakd-basalt}
BUILDER_IMAGE=${BUILDER_IMAGE:-williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap}
OUT="oakd-vio-arm64-${TAG}.tar"

cd "$(dirname "$0")"
docker buildx build --platform linux/arm64 \
  --build-arg BUILDER_IMAGE="$BUILDER_IMAGE" \
  -t "$IMAGE:$TAG" --load .
docker save "$IMAGE:$TAG" -o "$OUT"
echo "Saved $OUT ($(du -h "$OUT" | cut -f1)). Upload it in BlueOS: Extensions -> Installed -> + -> upload."
