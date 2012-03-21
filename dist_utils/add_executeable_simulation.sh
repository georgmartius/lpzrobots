#!/bin/bash

if [ -z "$1" -o  -z "$2" -o "$1" = "-h" ]; then
    echo "Usage: `basename $0` [-h] simulation destname"
    echo "       copies simulation executable to binary executable distribution under the name destname"
    echo "       -h   help"
    echo "Example: `basename $0` ~/lpzrobot/oderobots/simulations/template_onerobot/start onerobot"
    exit 0;
fi

shopt -s extglob
MACHINE=`uname -m`
KERNEL=`uname -r`
DIRNAME=$(ls -dt1 *-${MACHINE}-kernel-${KERNEL%%.*} | head -n 1)
echo "$DIRNAME";
if [ ! -e "${DIRNAME}" ]; then
    echo "cannot find binary dist directory: $DIRNAME"
    exit 1;
fi

SIMDIR=`dirname $1`
if [ ! -e "${SIMDIR}" ]; then
    echo "cannot find simulation directory"
    exit 1;
fi

DEST=$2
DESTSIMDIR=`cat $DIRNAME/simulationsdir`
if [ ! -e "$DIRNAME/$DESTSIMDIR" ]; then
    echo "Simulations dir $DIRNAME/$DESTSIMDIR does not exist"
    exit 1;
fi

DESTDIR=$DIRNAME/$DESTSIMDIR/$DEST
mkdir -p $DESTDIR

cp $1 $DESTDIR/

if [ -e $SIMDIR/guilogger.cfg ]; then
    cp $SIMDIR/guilogger.cfg $DESTDIR/
fi

if ls $SIMDIR/*.txt 2>/dev/null; then
    cp $SIMDIR/*.txt $DESTDIR/
fi

if ls $SIMDIR/*.fig 2>/dev/null; then
    cp $SIMDIR/*.fig $DESTDIR/
fi

#if ls $SIMDIR/*.xml 2>/dev/null; then
#    cp $SIMDIR/*.xml $DESTDIR/
#fi


DESTSCRIPT=$DIRNAME/$DEST
cp $DIRNAME/simulationtemplate $DESTSCRIPT

chmod u+x $DESTSCRIPT





