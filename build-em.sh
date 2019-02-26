#!/bin/env bash
pushd /tmp/emsdk
source emsdk_env.sh
popd


mkdir build-em &>/dev/null
cd build-em

       	#-DCMAKE_INSTALL_PREFIX=/usr/lib/emscripten/system 
cmake .. -G Ninja \
       	-DCMAKE_INSTALL_PREFIX=dist \
       	-DCMAKE_BUILD_TYPE=Debug \
	-DCMAKE_TOOLCHAIN_FILE="../toolchains/generic/Emscripten-wasm.cmake" \
       	-DEMSCRIPTEN_PREFIX="/usr/lib/emscripten"

cmake --build . --target install
