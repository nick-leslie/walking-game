#!/usr/bin/env bash
set -eu

# This creates a build that is similar to a release build, but it is debuggable.
# There is no hot reloading and no separate game library.
#-strict-style  -vet
OUT_DIR="build/debug"
mkdir -p "$OUT_DIR"
odin build source/main_release -out:$OUT_DIR/game_debug.bin -debug
cp -R assets $OUT_DIR
echo "Debug build created in $OUT_DIR"
