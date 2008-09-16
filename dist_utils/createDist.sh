#!/bin/bash

VERSION=0.4
NAME=lpzrobots
BASE=../..

DIR=$BASE/$NAME-$VERSION
SOSIMDIR=$DIR/selforg/simulations/
ODESIMDIR=$DIR/ode_robots/simulations/
SRCDIR=$BASE/lpzrobots

mkdir $DIR || exit 1;
echo "Copy top level files";
for F in `find $SRCDIR -maxdepth 1 -type f`; do
    cp "$F" $DIR/;
done

echo "Copy opende lib";
cp -r  $SRCDIR/opende $DIR/;
echo "Copy guilogger";
cp -r  $SRCDIR/guilogger $DIR/;
echo "Copy javacontroller";
cp -r  $SRCDIR/javacontroller $DIR/;
echo "Copy neuronviz";
cp -r  $SRCDIR/neuronviz $DIR/;
echo "Copy soundman";
cp -r  $SRCDIR/soundman $DIR/;
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

pushd `pwd`;
echo -en "# subdirectories with simulations\nSIMULATIONS=" > $SOSIMDIR/Makefile.conf

cd $SOSIMDIR || exit 1;
SIMS=`find . -mindepth 1 -maxdepth 1 -type d`
popd;

echo "Keeping the following simulation:";
for S in $SIMS; do
    S="${S#./}";
    if grep "$S" selforg_simulations; then
    	echo -n  "$S " >>  $SOSIMDIR/Makefile.conf
    else
	rm -r $SOSIMDIR/$S;
    fi
done 


echo "Copy ode_robots";
cp -r $SRCDIR/ode_robots $DIR/;
pushd `pwd`;
echo -en "# subdirectories with simulations\nSIMULATIONS=" > $ODESIMDIR/Makefile.conf

cd $ODESIMDIR  || exit 1;
SIMS=`find . -mindepth 1 -maxdepth 1 -type d`
popd;

echo "Keeping the following simulation:";
for S in $SIMS; do
    S="${S#./}";
    if grep "$S" simulations; then
    	echo -n  "$S " >>  $ODESIMDIR/Makefile.conf
    else
	rm -r $ODESIMDIR/$S;
    fi
done 

echo "Removing Makefile.conf";
rm -f $DIR/Makefile.conf

echo "Removing CVS dirs";
find $DIR/ -type d -name CVS | xargs rm -r;
find $DIR/ -type f -name ".cvsignore" | xargs rm -r;
