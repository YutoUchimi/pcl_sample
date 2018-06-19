#!/bin/bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set -x

mkdir -p $HERE/bin
mkdir -p $HERE/build
cd $HERE/build

set +x
if [ $(which deactivate) ]; then
    source $(which deactivate)
fi
set -x

cmake ..
make
cd $HERE

set +x
