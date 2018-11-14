#!/bin/bash
#**************************************************************************
#   Copyright (C) 2005 by Robot Group Leipzig                             *
#    martius@informatik.uni-leipzig.de                                    *
#    fhesse@informatik.uni-leipzig.de                                     *
#    der@informatik.uni-leipzig.de                                        *
#    guettler@informatik.uni-leipzig.de                                   *
#                                                                         *
#   This program is free software; you can redistribute it and/or modify  *
#   it under the terms of the GNU General Public License as published by  *
#   the Free Software Foundation; either version 2 of the License, or     *
#   (at your option) any later version.                                   *
#                                                                         *
#   This program is distributed in the hope that it will be useful,       *
#   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#   GNU General Public License for more details.                          *
#                                                                         *
#   You should have received a copy of the GNU General Public License     *
#   along with this program; if not, write to the                         *
#   Free Software Foundation, Inc.,                                       *
#   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
#**************************************************************************

if test -z "$1"; then
    echo -e "USAGE: $0: BaseName [Target]\n\tExample: $0 frame_00 SuperVideo [EXTENSION]";
    exit 1;
fi

NAME=$1;
TARGET=$1;
if test -n "$2"; then
    TARGET=$2;
fi
EXTENSION="jpg"
if test -n "$3"; then
    EXTENSION=$3;
fi

# copy 20th frame as screenshot
FRAME=`ls $NAME* -1 | head -20 | tail -n 1`;
cp "$FRAME" "$TARGET.jpg";

#echo -e "*********************** mjpeg encoding **************************";
#mencoder mf://$NAME*.jpg -mf fps=25:type=sgi -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o $NAME.mjpeg
#nice -10 mencoder mf://$NAME*.jpg -mf fps=25:type=jpg -ovc lavc -lavcopts vcodec=mjpeg -oac copy -o "$TARGET.mjpeg"

if which ffmpeg; then
    cat $NAME*.${EXTENSION} | ffmpeg -y -f image2pipe -r 25 -i - -vcodec libx264 -crf 15 -pix_fmt yuv420p "${TARGET}_hq.mp4"
#    cat $NAME*.${EXTENSION} | ffmpeg -y -f image2pipe -r 25 -vcodec mjpeg -i - -vcodec libx264 -crf 15 -pix_fmt yuv420p  "${TARGET}_hq.flv"
else
    if which avconv; then
        cat $NAME*.${EXTENSION} | avconv -y -f image2pipe -r 25 -i - -vcodec libx264 -pix_fmt yuv420p -crf 15 "${TARGET}_hq.mp4"
#        cat $NAME*.${EXTENSION} | avconv -y -f image2pipe -r 25 -vcodec mjpeg -i - -vcodec libx264 -pix_fmt yuv420p -crf 15 "${TARGET}_hq.flv"

    else
        echo "cannot find ffmpeg or avconv! Install ffmpeg or libav-tools.";
    fi
fi

#echo -e "*********************** to mpeg4 xvid 4 **************************";
#transcode -i "$TARGET.mjpeg" -o "$TARGET.avi" -y xvid4,null -w 800
# if which x264; then
#     echo -e "******************** to high quality mp4 and flv (h264) ***************";
#     nice -10 x264 --crf 15 -o "${TARGET}_hq.mp4" "$TARGET.mjpeg";
#     nice -10 x264 --crf 15 -o "${TARGET}_hq.flv" "$TARGET.mjpeg";
# else
#     echo -e "******************** to high quality avi (msmpeg4) ***************";
#     nice -10 ffmpeg -i "$TARGET.mjpeg" -vcodec msmpeg4 -sameq  "${TARGET}_hq.avi"
#     echo -e "******************** to flash video (high quality) ***************";
#     nice -10 ffmpeg -i "$TARGET.mjpeg" -b 2500k  "${TARGET}_hq.flv"
# fi

# echo -e "******************** to flash video (web version)  ***************";
# nice -10 ffmpeg -i "$TARGET.mjpeg" -b 800k  "${TARGET}.flv"

#echo -e "******************** to mpeg4 xvid 4 small variant ***************";
#transcode -i "$TARGET.mjpeg" -o "$TARGET"_small.avi -y xvid4,null -w 100 -r 2

#echo -e "******************** to wmv small variant ***************";
#transcode -i "$TARGET.mjpeg" -o "$TARGET"_small.wmv.avi -y ffmpeg,null -F wmv2 -w 100 -r 2

echo "#!/bin/bash" > __cleanup.sh
echo "rm -f $NAME*.$EXTENSION" >> __cleanup.sh

# copy src and log files into src folder and tar them
if test -e ../main.cpp; then
    DATAFOLDER=${TARGET}_src
    echo "create $DATAFOLDER for log files and sources";
    mkdir "$DATAFOLDER";
    cp ../*.cpp ../*.h ../Makefile.conf "$DATAFOLDER"
    mv *.log *.agent "$DATAFOLDER"
    tar -cjf "${DATAFOLDER}.tar.bz2" "$DATAFOLDER";
    echo "rm -rf $DATAFOLDER" >> __cleanup.sh
fi

#echo "rm -f $TARGET.mjpeg" >> __cleanup.sh
echo "rm -f __cleanup.sh" >> __cleanup.sh
chmod u+x __cleanup.sh
