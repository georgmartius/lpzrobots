#!/bin/bash

VERSION=0.1
NAME=lpzrobots
BASE=../..

DIR=$BASE/$NAME-$VERSION
SIMDIR=$DIR/ode_robots/simulations/
SRCDIR=$BASE/lpzrobots

mkdir $DIR || exit 1;
echo "Copy top level files";
for F in `find $SRCDIR -type f -maxdepth 1`; do
    cp "$F" $DIR/;
done

echo "Copy matrix lib";
cp -r  $SRCDIR/matrixlib $DIR/;
echo "Copy guilogger";
cp -r  $SRCDIR/guilogger $DIR/;
echo "Copy gnuplot";
cp -r  $SRCDIR/gnuplot $DIR/;

echo "Copy controller";
mkdir $DIR/controller || exit 1;
for F in `cat controllers`; do
cp -r  $SRCDIR/controller/$F $DIR/controller/;
done 

echo "Copy ode_robots";
cp -r $SRCDIR/ode_robots $DIR/;
pushd `pwd`;

echo -en "# subdirectories which contain libraries or binaries needed some of the other project in this tree\nSIMULATIONS=" > $SIMDIR/Makefile.conf

cd $SIMDIR  || exit 1;
SIMS=`find . -type d -mindepth 1 -maxdepth 1`
popd;

echo "Keeping the following simulation:";
for S in $SIMS; do
    S="${S#./}";
    if grep "$S" simulations; then
    	echo -n  "$S " >>  $SIMDIR/Makefile.conf
    else
	rm -r $SIMDIR/$S;
    fi
done 

echo "Removing CVS dirs";
find $DIR/ -type d -name CVS | xargs rm -r;
