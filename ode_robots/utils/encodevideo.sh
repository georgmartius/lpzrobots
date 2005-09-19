#/bin/bash

if test -z "$1"; then 
    echo "USAGE: $0: BaseName";
    exit 1;
fi

NAME=$1;

for F in $NAME*.ppm; do echo "convert $F"; convert "$F" "${F%ppm}sgi"; rm "$F"; done

echo -e "*********************** mjpeg encoding **************************";
mencoder mf://$NAME*.sgi -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o $NAME.mjpeg
echo -e "*********************** to mpeg2 **************************";
transcode -i $NAME.mjpeg -o $NAME.mpg -y mpeg2enc 
echo -e "*********************** to mpeg4 xvid 4 **************************";
transcode -i $NAME.mjpeg -o $NAME.avi -y xvid4 



