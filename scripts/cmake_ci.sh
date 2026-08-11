#!/bin/bash

# Set warnings are errors flag
if [ $1 = "gcc" ]
then
    export CC=gcc
    export CXX=g++
elif [ $1 = "clang" ]
then
    export CC=clang-10
    export CXX=clang++-10
fi

GIT_DIR=$(git rev-parse --show-toplevel)

cd $GIT_DIR/agilib/build && cmake -DBUILD_TEST=ON -DEIGEN_FROM_SYSTEM=OFF ..
