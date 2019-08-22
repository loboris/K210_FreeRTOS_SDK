#!/bin/bash

export CROSS_COMPILE=${PWD}../../../kendryte-toolchain/bin
export PLATFORM=k210

cd ../../build
cmake ../ -DPROJ=uart_example -DTOOLCHAIN=${CROSS_COMPILE}
make -j8
