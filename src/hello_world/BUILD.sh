#!/bin/bash

export CROSS_COMPILE=${PWD}../../../kendryte-toolchain/bin
export PLATFORM=k210

cd ../../build
cmake ../ -DPROJ=hello_world -DTOOLCHAIN=${CROSS_COMPILE}
make -j8
