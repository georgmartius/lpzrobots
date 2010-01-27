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
#   Revision 1.4  2010-01-27 10:55:04  martius
#   cosmetics
#
#   Revision 1.3  2010/01/27 10:35:08  martius
#   fixed framerate for hq flash
#   multiple files are posible at once
#   endings (file extensions) are ignored
#
#   Revision 1.2  2010/01/27 10:23:20  martius
#   cosmetics
#
#   Revision 1.1  2010/01/26 12:03:57  martius
#   a script that converts videos to all available formats.
#
#
#                                                                         *
#**************************************************************************

if test -z "$1"; then 
    echo -e "USAGE: $0: VideoFileStem[.ext] [VideoFileStem2 ....]\n\tExample: $0 SuperVideo\n\t where SuperVideo.mjpeg or so exists\n\tGiven filename extentions are ignored";
    exit 1;
fi

for FILE in $@; do
STEM=${FILE%.*};
echo "Consider: $STEM!";
LEVEL=0;
if test -e "${STEM}.mjpeg"; then 
    SRC="${STEM}.mjpeg";
elif test -e "${STEM}_hq.avi"; then 
    SRC="${STEM}_hq.avi";
    LEVEL=1;
elif test -e "${STEM}.avi"; then 
    echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
    echo "WARNING: Found only low quality avi!";
    echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
    SRC="${STEM}.avi";
    LEVEL=2;
else
    echo -e "Found neither mjpeg, _hq.avi nor .avi file!";
    exit 1;
fi


if test $LEVEL -lt 1; then
    if test ! -e "${STEM}_hq.avi"; then
        echo -e "******************** to high quality avi (msmpeg4) ***************";
        ffmpeg -i "$SRC" -vcodec msmpeg4 -sameq  "${STEM}_hq.avi"
    fi
fi

if test $LEVEL -lt 2; then
    if test ! -e "${STEM}.avi"; then
        echo -e "*********************** to mpeg4 xvid 4 **************************";
        transcode -i "$SRC" -o "${STEM}.avi" -y xvid4,null -w 800
    fi
    if test ! -e "${STEM}_hq.flv"; then
        echo -e "******************** to flash video (high quality) ***************";
        ffmpeg -i "$SRC" -b 2500k "${STEM}_hq.flv"
    fi
fi

if test $LEVEL -lt 3; then
    if test ! -e "${STEM}.flv"; then
        echo -e "******************** to flash video (web version)  ***************";
        ffmpeg -i "$SRC" -b 600k  "${STEM}.flv"
    fi
    if test ! -e "${STEM}.jpg"; then
        echo -e "******************** get screenshot ***************";
        ffmpeg -i "$SRC" -vframes 1 -f mjpeg "${STEM}.jpg"
    fi
fi


done