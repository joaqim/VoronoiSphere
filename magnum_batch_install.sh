#!/bin/env bash

cd /opt
git clone https://github.com/juj/emsdk
cd emsdk
./emsdk update
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
cd /tmp

git clone --recurse-submodules git://github.com/mosra/corrade
git clone --recurse-submodules git://github.com/mosra/magnum
git clone --recurse-submodules git://github.com/mosra/magnum-plugins
git clone --recurse-submodules git://github.com/mosra/magnum-integration
git clone --recurse-submodules git://github.com/mosra/magnum-extras

BUILD_DIR=build-emc
mkdir corrade/$BUILD_DIR
mkdir magnum/$BUILD_DIR
mkdir magnum-plugins/$BUILD_DIR
mkdir magnum-extras/$BUILD_DIR
mkdir magnum-examples/$BUILD_DIR

# This is the path magnum example apps will be deployed to.
DEPLOY_PREFIX=$PWD"/deploy"
mkdir $DEPLOY_PREFIX

EMSCRIPTEN_PREFIX="/usr/lib/emscripten"

CMAKE_CMD="cmake .. -GNinja -DCMAKE_PREFIX_PATH=$PREFIX_PATH -DCMAKE_INSTALL_PREFIX=$EMSCRIPTEN_PREFIX/system -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../toolchains/generic/Emscripten-wasm.cmake -DEMSCRIPTEN_PREFIX=$EMSCRIPTEN_PREFIX"

CMAKE_BUILD_CMD="cmake --build . --target install"

cd corrade/$BUILD_DIR
$CMAKE_CMD -DWITH_TESTSUITE=ON -DBUILD_DEPRECATED=OFF
$CMAKE_BUILD_CMD

cd ../../magnum/$BUILD_DIR
$CMAKE_CMD -DWITH_SDL2APPLICATION=ON -DWITH_MESHTOOLS=ON -DWITH_PRIMITIVES=ON -DWITH_OBJIMPORTER=ON -DWITH_TGAIMPORTER=ON -DWITH_SHADERS=ON -DWITH_TEXT=ON -DWITH_AUDIO=ON -DWITH_WAVAUDIOIMPORTER=ON -DBUILD_DEPRECATED=OFF -DWITH_ANYIMAGEIMPORTER=ON
$CMAKE_BUILD_CMD

cd ../../magnum-plugins/$BUILD_DIR
$CMAKE_CMD -DWITH_OPENGEXIMPORTER=ON -DWITH_STBTRUETYPEFONT=ON -DBUILD_DEPRECATED=OFF -DWITH_TINYGLTFIMPORTER=ON
$CMAKE_BUILD_CMD

cd ../../magnum-extras/$BUILD_DIR
$CMAKE_CMD -DWITH_UI=OFF -DWITH_UI_GALLERY=ON -DMAGNUM_DEPLOY_PREFIX=$DEPLOY_PREFIX -DBUILD_DEPRECATED=OFF
$CMAKE_BUILD_CMD

cd ../../magnum-examples/$BUILD_DIR
$CMAKE_CMD -DWITH_AUDIO_EXAMPLE=OFF -DWITH_TEXTUREDTRIANGLE_EXAMPLE=OFF -DWITH_WEBVR_EXAMPLE=ON -DMAGNUM_DEPLOY_PREFIX=$DEPLOY_PREFIX
$CMAKE_BUILD_CMD
cd /tmp

rm -r magnum magnum-examples magnum-plugins magnum-extras corrade
