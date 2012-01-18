#!/bin/bash

VERSION=0.5
OS=`uname -s`;
ARCH=`uname -m`;
NAME=lpzrobots-$VERSION-extra-lib-devel_$OS.$ARCH
BASE=../..
TEMPLATE=lpzrobots-extra-lib-devel-template

DIR=$BASE/$NAME
INCDIR=$DIR/include
LIBDIR=$DIR/lib
BINDIR=$DIR/bin

if [ -d $DIR ]; then
    echo -n "The target directory $DIR exits, shall I delete it? [N]: ";
    read YN;
    if [ "$YN" = "Y" -o  "$YN" = "y" ]; then
	rm -rf "$DIR";
    else
	echo "Remove it youself!";
        exit 1;
    fi
fi
cp -r $TEMPLATE $DIR || { echo "Error: copy template"; exit 1;}
rm -r $DIR/CVS
mkdir $INCDIR || { echo "Error: mkdir"; exit 1;}
mkdir $LIBDIR || { echo "Error: mkdir"; exit 1;}
mkdir $BINDIR || { echo "Error: mkdir"; exit 1;};

echo -n "Where is your ode located? [/usr/local]: "
read PREFIX;
[ -z $PREFIX ] && PREFIX='/usr/local';
[ ! -e $PREFIX/lib/libode.so ] && { echo "Error: $PREFIX/lib/libode.so not found"; exit 1;}

echo "Copy ode stuff";
cp $PREFIX/lib/libode.* $LIBDIR/;
[ -e $PREFIX/lib/libdrawstuff.a ] && cp $PREFIX/lib/libdrawstuff.* $LIBDIR/;
cp -R $PREFIX/include/ode $INCDIR/;

echo -n "Where is your OSG located? [$PREFIX]:"
read PREFIX_;

[ -n "$PREFIX_" ] && PREFIX=$PREFIX_;
[ ! -e $PREFIX/lib/libosg.so ] && { echo "Error: $PREFIX/lib/libosg.so not found"; exit 1;}

echo "Copy OSG";
cp -P $PREFIX/lib/libosg* $LIBDIR/;
cp -P $PREFIX/lib/libOpenThreads* $LIBDIR/;
cp -RP $PREFIX/lib/osgPlugins* $LIBDIR/;
cp -RP $PREFIX/include/osg* $INCDIR/;
cp -RP $PREFIX/include/OpenThreads $INCDIR/;
ls $PREFIX/bin/osg* 2>&1 > /dev/null && cp  $PREFIX/bin/osg* $BINDIR/;

echo "Successful";
