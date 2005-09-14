#/bin/bash

if test -z "$1"; then 
    echo "USAGE: $0: BaseName";
    exit 1;
fi

NAME=$1;

for F in $NAME*.ppm; do echo "convert $F"; convert "$F" "${F%ppm}sgi"; rm "$F"; done

echo -e "*********************** mjpeg encoding:";
echo "mencoder mf://$NAME*.sgi -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o $NAME.mjpeg";
echo -e "*********************** mpeg2 encoding:";
echo "mencoder mf://$NAME*.sgi -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mpeg2video -oac copy -o $NAME.mpg";
echo -e "*********************** mpeg4 DIVX encoding:";
echo "mencoder mf://$NAME*.sgi -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mpeg4 -oac copy -ffourcc DX50 -o $NAME.mpg";



