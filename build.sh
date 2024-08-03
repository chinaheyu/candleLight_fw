#!/bin/bash

set -e

# prepare directory
rm -rf build bin
mkdir -p build bin

# build firmware
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi-8-2019-q3-update.cmake
cmake --build . --target SCUT_candleLightFD_fw
cd ..
cp ./build/SCUT_candleLightFD_fw.bin ./bin
