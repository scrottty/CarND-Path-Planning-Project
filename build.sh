#!/bin/sh

#  build.sh
#  MPC
#
#  Created by Ollie Steiner on 15/09/17.
#
rm -rf build
mkdir build
cd build
cmake .. 
make
open ../../term3_sim_mac/term3_sim.app
./path_planning