#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$ROOT_DIR/vendor/mujoco_menagerie"

if [ -d "$DEST/.git" ]; then
  echo "Repositório Menagerie já existe em: $DEST"
  git -C "$DEST" sparse-checkout set ufactory_xarm7
  git -C "$DEST" pull --ff-only
else
  mkdir -p "$ROOT_DIR/vendor"
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/google-deepmind/mujoco_menagerie.git "$DEST"
  git -C "$DEST" sparse-checkout set ufactory_xarm7
fi

test -f "$DEST/ufactory_xarm7/xarm7.xml"
test -f "$DEST/ufactory_xarm7/LICENSE"
test -d "$DEST/ufactory_xarm7/assets"
echo "Modelo oficial xArm7 preparado em: $DEST/ufactory_xarm7"
