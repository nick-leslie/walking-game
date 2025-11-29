#!/usr/bin/bash

CONFIG=${1:-Debug}

echo "Building joltc with config: $CONFIG"

cmake -S joltc -B joltc/build \
    -DJPH_SAMPLES=OFF \
    -DJPH_BUILD_SHARED=ON \
    -DCMAKE_BUILD_TYPE=$CONFIG

cmake --build joltc/build --config $CONFIG
echo "copying lib and h file"
cp joltc/include/joltc.h libs/jolt/jolt.h
cp joltc/build/lib/libjoltc.so libs/jolt
echo "generating bindigns"
bindgen.bin ./libs/jolt
