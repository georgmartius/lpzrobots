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
#                                                                         *
#  DESCRIPTION                                                            *
#                                                                         *
#   $Log$
#   Revision 1.15  2010-01-12 15:18:24  martius
#   added flash video
#
#   Revision 1.14  2010/01/04 15:30:47  der
#   added high quality video im MS Mpeg4 format
#
#   Revision 1.13  2009/03/13 09:19:53  martius
#   changed texture handling in osgprimitive
#   new OsgBoxTex that supports custom texture repeats and so on
#   Box uses osgBoxTex now. We also need osgSphereTex and so on.
#   setTexture has to be called before init() of the primitive
#
#   Revision 1.12  2009/02/03 18:12:09  martius
#   added wmv to encoding and provided new script to convert existing movies
#
#   Revision 1.11  2008/02/22 06:51:46  der
#   added small xvid variant
#
#                                                                         *
#**************************************************************************

if test -z "$1"; then 
    echo -e "USAGE: $0: BaseName [Target]\n\tExample: $0 frame_00 SuperVideo";
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
echo -e "*********************** to wmv **************************";
mencoder mf://$NAME*.jpg -mf fps=25:type=jpg -ovc lavc -lavcopts vcodec=wmv2:vbitrate=600 -oac copy -o "${TARGET}.wmv.avi"
echo -e "*********************** to mpeg4 xvid 4 **************************";
transcode -i "$TARGET.mjpeg" -o "$TARGET.avi" -y xvid4,null -w 600

echo -e "******************** to mpeg4 xvid 4 small variant ***************";
transcode -i "$TARGET.mjpeg" -o "$TARGET"_small.avi -y xvid4,null -w 100 -r 2

echo -e "******************** to wmv small variant ***************";
transcode -i "$TARGET.mjpeg" -o "$TARGET"_small.wmv.avi -y ffmpeg,null -F wmv2 -w 100 -r 2

echo -e "******************** to high quality avi (msmpeg4) ***************";
ffmpeg -i "$TARGET.mjpeg" -vcodec msmpeg4 -sameq  "${TARGET}_hq.avi"

echo -e "******************** to flash video  ***************";
ffmpeg -i "$TARGET.avi" -sameq  "${TARGET}.flv"


