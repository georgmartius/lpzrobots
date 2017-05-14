#!/bin/bash

xhost local:root
docker run -e DISPLAY=$DISPLAY --net=host -v ~/.cache/Xauthority -it lpzrobots /bin/bash
xhost -local:root
