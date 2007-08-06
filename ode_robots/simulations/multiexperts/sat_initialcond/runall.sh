#!/bin/bash

for F in $1/*.net; do 
    ./start_opt "$F" -nographics;
done