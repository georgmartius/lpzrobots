#!/bin/bash

VERSION=0.2
NAME=lpzrobots
BASE=../..

DIR=$BASE/$NAME-$VERSION
SIMDIR=$DIR/ode_robots/simulations/
SRCDIR=$BASE/lpzrobots

mkdir $DIR || exit 1;
echo "Copy top level files";
for F in `find $SRCDIR -maxdepth 1 -type f`; do
    cp "$F" $DIR/;
done

echo "Copy matrix lib";
cp -r  $SRCDIR/matrixlib $DIR/;
echo "Copy guilogger";
cp -r  $SRCDIR/guilogger $DIR/;
echo "Copy gnuplot";
cp -r  $SRCDIR/gnuplot $DIR/;
echo "Copy neuronviz";
cp -r  $SRCDIR/neuronviz $DIR/;
echo "Copy webots projects";
cp -r  $SRCDIR/webots $DIR/;

echo "Copy selforg without controllers";
cp -r  $SRCDIR/selforg $DIR/;
rm -r  $DIR/selforg/controller;
echo "Copy  controllers";
mkdir $DIR/selforg/controller || exit 1;
for F in `cat controllers`; do
cp -r  $SRCDIR/selforg/controller/$F $DIR/selforg/controller/;
done 

echo "Copy ode_robots";
cp -r $SRCDIR/ode_robots $DIR/;
pushd `pwd`;

echo -en "# subdirectories which contain libraries or binaries needed some of the other project in this tree\nSIMULATIONS=" > $SIMDIR/Makefile.conf

cd $SIMDIR  || exit 1;
SIMS=`find . -mindepth 1 -maxdepth 1 -type d`
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
