#!/bin/env bash
mkdir build &>/dev/null
cd build

cmake .. -G Ninja \
       	-DCMAKE_BUILD_TYPE=Debug

cmake --build . && src/MyApplication
