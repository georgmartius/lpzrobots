#!/bin/bash

VERSION=${1:-0.7}
NAME=lpzrobots
BASE=../..

DIR=$BASE/$NAME-$VERSION
SOSIMDIR=$DIR/selforg/simulations/
ODESIMDIR=$DIR/ode_robots/simulations/
GASIMDIR=$DIR/ga_tools/simulations/
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
echo "Copy matrixviz";
cp -r  $SRCDIR/matrixviz $DIR/;
echo "Copy soundman";
cp -r  $SRCDIR/soundman $DIR/;
echo "Copy configurator";
cp -r  $SRCDIR/configurator $DIR/;
echo "Copy doc folder";
cp -r  $SRCDIR/doc $DIR/;

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

echo "Copy ga_tools";
cp -r $SRCDIR/ga_tools $DIR/;
pushd `pwd`;
echo -en "# subdirectories with simulations\nSIMULATIONS=" > $GASIMDIR/Makefile.conf

cd $GASIMDIR  || exit 1;
SIMS=`find . -mindepth 1 -maxdepth 1 -type d`
popd;

echo "Keeping the following simulation:";
for S in $SIMS; do
    S="${S#./}";
    if grep "$S" ga_tools_simulations; then
    	echo -n  "$S " >>  $GASIMDIR/Makefile.conf
    else
	rm -r $GASIMDIR/$S;
    fi
done 

echo "Removing Makefile.conf";
rm -f $DIR/Makefile.conf

echo "Removing git dirs";
find $DIR/ -type f -name ".gitignore" | xargs rm -r;


pushd `pwd`;
cd $BASE;
echo "tar $NAME-$VERSION.tgz"
tar -czf $NAME-$VERSION.tgz $NAME-$VERSION;
popd;
