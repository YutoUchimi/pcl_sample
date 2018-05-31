#!/bin/bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set -x

mkdir -p $HERE/bin
mkdir -p $HERE/build
cd $HERE/build
cmake ..
make
cd $HERE

set +x
