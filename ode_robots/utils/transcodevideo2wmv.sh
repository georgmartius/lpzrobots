#!/bin/bash
#**************************************************************************
#   Copyright (C) 2009 by Robot Group Leipzig                             *
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
#   Revision 1.2  2009-03-13 09:19:53  martius
#   changed texture handling in osgprimitive
#   new OsgBoxTex that supports custom texture repeats and so on
#   Box uses osgBoxTex now. We also need osgSphereTex and so on.
#   setTexture has to be called before init() of the primitive
#
#   Revision 1.1  2009/02/03 18:12:09  martius
#   added wmv to encoding and provided new script to convert existing movies
#
#
#                                                                         *
#**************************************************************************

if test -z "$1"; then 
    echo -e "USAGE: $0: source [Target]\n\tExample: $0 video.mjpeg";
    exit 1;
fi

NAME=$1;
TARGET=${1%.*}.wmv;
TARGET_S=${1%.*}_small.wmv;
if test -n "$2"; then 
    TARGET=$2;
    TARGET_S=${2%.*}_small.${2#*.};
fi

 
echo -e "******************** to WMV2 normal ***************";
transcode -i "$NAME" -o "$TARGET" -y ffmpeg,null -F wmv2 -w 600

echo -e "******************** to WMV2 small variant ***************";
transcode -i "$NAME" -o "$TARGET_S" -y ffmpeg,null -F wmv2 -w 100 -r 2

echo "Created $TARGET and $TARGET_S"


