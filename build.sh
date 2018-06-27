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

sudo apt-get install -y libopencv-dev=2.4.8+dfsg1-2ubuntu1
sudo apt-get install -y libeigen3-dev=3.2.0-8
sudo apt-get install -y libboost-dev=1.54.0.1ubuntu1

cmake ..
make
cd $HERE

set +x
