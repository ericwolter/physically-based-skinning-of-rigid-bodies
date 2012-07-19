#!/usr/bin/env bash

# execute build
./build.sh
rc=$?
if [[ $rc != 0 ]] ; then
    exit $rc
fi

# run programm
./build/Simulation/SimulationTest
