#!/bin/bash

if [ -z "$1" -o  -z "$2" -o "$1" = "-h" ]; then
    echo "Usage: `basename $0` [-h] simulation destname"
    echo "       copies simulation executable to binary executable distribution under the name destname"
    echo "       -h   help"
    echo "Example: `basename $0` ~/lpzrobot/oderobots/simulations/template_onerobot/start onerobot"
    exit 0;
fi

DIRNAME=lpzrobots-`uname -m`
if [ ! -e ${DIRNAME} ]; then
    echo "cannot find binary dist directory: $DIRNAME"
fi

SIMDIR=`dirname $1`
if [ ! -e ${SIMDIR} ]; then
    echo "cannot find simulation directory"
fi

DEST=$2
DESTDIR=$DIRNAME/simulations/$DEST
mkdir -p $DESTDIR

cp $1 $DESTDIR/

if [ -e $SIMDIR/guilogger.cfg ]; then
    cp $SIMDIR/guilogger.cfg $DESTDIR/
fi

DESTSCRIPT=$DIRNAME/$DEST
cat $DIRNAME/simulationtemplate | sed "s/mysim/$DEST/" > $DESTSCRIPT

chmod u+x $DESTSCRIPT





