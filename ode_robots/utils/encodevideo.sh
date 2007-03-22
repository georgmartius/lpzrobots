#/bin/bash

if test -z "$1"; then 
    echo "USAGE: $0: BaseName";
    exit 1;
fi

NAME=$1;
#NAME="frame";

# for F in $NAME*.bmp; do echo "convert $F"; convert "$F" "${F%bmp}sgi"; rm "$F"; done

echo -e "*********************** mjpeg encoding **************************";
#mencoder mf://$NAME*.jpg -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o $NAME.mjpeg
mencoder mf://frame*.jpg -mf fps=25:type=jpg -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o $NAME.mjpeg
echo -e "*********************** to mpeg2 **************************";
transcode -i $NAME.mjpeg -o $NAME -y mpeg2enc,null -F 3 -w 800 

echo -e "*********************** to mpeg4 xvid 4 **************************";
transcode -i $NAME.mjpeg -o $NAME.avi -y xvid4,null -w 400 




