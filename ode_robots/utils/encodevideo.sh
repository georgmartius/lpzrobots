#/bin/bash

if test -z "$1"; then 
    echo -e "USAGE: $0: BaseName [Target]\n\tExample: $0 frame00 SuperVideo";
    exit 1;
fi

NAME=$1;
TARGET=$1;
if test -n "$2"; then 
    TARGET=$2;
fi

# for F in $NAME*.bmp; do echo "convert $F"; convert "$F" "${F%bmp}sgi"; rm "$F"; done
# copy first frame as screenshot
FRAME=`ls $NAME* -1 | head -1`;
cp "$FRAME" "$TARGET.jpg";

echo -e "*********************** mjpeg encoding **************************";
#mencoder mf://$NAME*.jpg -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o $NAME.mjpeg
mencoder mf://$NAME*.jpg -mf fps=25:type=jpg -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o "$TARGET.mjpeg"
echo -e "*********************** to mpeg2 **************************";
transcode -i "$TARGET.mjpeg" -o "$TARGET" -y mpeg2enc,null -F 3 -w 800 

echo -e "*********************** to mpeg4 xvid 4 **************************";
transcode -i "$TARGET.mjpeg" -o "$TARGET.avi" -y xvid4,null -w 400 




