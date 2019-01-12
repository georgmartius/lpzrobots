#!/bin/bash

std="-r 1 -noise 0.01 -f 100 ntst -nographics -simtime 40 -power 0.5 -loadcontroller bungee_medium1.ctrl"
./start_opt -r 1 $std -tilt 0     -name "control" &
./start_opt -r 1 $std -tilt 0     -name "pos1" &
./start_opt -r 1 $std -tilt 0.01  -name "pos3"
./start_opt -r 1 $std -tilt 0.1   -name "pos2"
./start_opt -r 1 $std -tilt 0     -name "pos1" -rescue &
./start_opt -r 1 $std -tilt 0.01  -name "pos3" -rescue
./start_opt -r 1 $std -tilt 0.1   -name "pos2" -rescue
./start_opt -r 1 $std -tilt 0     -name "pos1" -bungee &
./start_opt -r 1 $std -tilt 0.01  -name "pos3" -bungee
./start_opt -r 1 $std -tilt 0.1   -name "pos2" -bungee
./start_opt -r 1 $std -tilt 0     -name "pos1" -reck &
./start_opt -r 1 $std -tilt 0.01  -name "pos3" -reck
./start_opt -r 1 $std -tilt 0.1   -name "pos2" -reck
