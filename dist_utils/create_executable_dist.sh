#!/bin/bash

if [ ! -x ./start ]; then
   echo "run this script in a directory where a simulation (./start) is found."
   exit -1
fi

if [ "$1" = "-h" ]; then
    echo "Usage: $0 [-h] [-n] [NameOfDist]"
    echo "       -h   help"
    echo "       -n   use the exisiting package"
    exit 0;
fi

THISSIM=$(basename `pwd`)

if [ ! "$1" = "-n" ]; then
  #check for cde
  if ! type -p cde; then
    echo "cde not in path. Go to http://www.stanford.edu/~pgbovine/cde.html and install it"
    exit 1;
  fi

  echo "The simulation in this directory is started now. Start also the guilogger and"
  echo " the matrixviz (Ctrl+G and Ctrl+M) (make sure gnuplot is opened) and close all."
  echo " Press Enter to continue."
  read TEST

  cde ./start

else
    shift;
    if [ -e cde-package ]; then
        echo " I am useing the exising cde-package."
    else
        echo " cannot find the cde-package. run without -n!"
        exit -1;
    fi
fi

BASENAME=lpzrobots
if [ -n "$1" ]; then
    BASENAME=$1
fi

MACHINE=`uname -m`
KERNEL=`uname -r`
DIRNAME=$BASENAME-${MACHINE}-kernel-${KERNEL%%.*}
if [ -d $DIRNAME ]; then rm -r $DIRNAME; fi
cp -r cde-package $DIRNAME


# does not work because pkg information is wrong
#OSGLIBPATH=`pkg-config --libs openscenegraph | sed "s/.*-L\([^ ]*\) .*/\1/"`
OSGPLUGINGPATH=$(find $DIRNAME -type d -name "osgPlugin*" | head -n 1)
if [ -n "$OSGPLUGINGPATH" ]; then
    SYSP=${OSGPLUGINGPATH#$DIRNAME/cde-root}; 
#all   
#    cp -r -v $SYSP $OSGPLUGINGPATH
# select some
    for F in osgdb_gif.so osgdb_jpeg.so osgdb_osgshadow.so osgdb_3ds.so osgdb_lwo.so osgdb_osg.so osgdb_osgterrain.so osgdb_png.so osgdb_bmp.so osgdb_osga.so osgdb_osgtext.so osgdb_pnm.so osgdb_rgb.so osgdb_tiff.so osgdb_freetype.so osgdb_osgviewer.so osgdb_cfg.so osgdb_scale.so; do
        cp -v $SYSP/$F $OSGPLUGINGPATH/$F;
    done
else
    echo "Cannot find osgPlugins!"
fi

GUILOGGER=`which guilogger`
BINDIR=`dirname $GUILOGGER`
PREFIX=${BINDIR%/bin}

echo "Found guilogger here: ${GUILOGGER}."
echo "The binary directory is ${BINDIR}"
echo " and the prefix is ${PREFIX}."

for F in encodevideo.sh  feedfile.pl  selectcolumns.pl; do
    cp -v ${BINDIR}/$F $DIRNAME/cde-root${BINDIR}/
done

if [ -e ${PREFIX}/share/lpzrobots/data ]; then
   cp -r ${PREFIX}/share/lpzrobots/data $DIRNAME/cde-root${PREFIX}/share/lpzrobots/
else
   echo "could not find DATA in ${PREFIX}/share/lpzrobots/data"
fi

(
cat <<EOF
This is a binary package of lpzrobots (see http://robot.informatik.uni-leipzig.de)
with a selection of simulations.
---------------
How to install:
Save the $DIRNAME.tar.gz file in a directory of your choice and unpack it, 
e.g. open a terminal and go to this directory and call
> tar -xvzf $DIRNAME.tar.gz
That's it.

-------------
How to start:
The enter the new directory $DIRNAME in the terminal window
 and start one of simulations, for instance
./OutlineDemo

------------------------
FAQ and Troubleshooting:
Q: The simulation aborts with a segmentation fault.
A: try ./SIMNAME -noshadow 
   where SIMNAME is the particular simulation if this works you can try 
   which shadow types work, see ./SIMNAME -h and save later with 
   ./SIMNAME -shadow X -savecfg

Q: What options and which ways of interaction do I have? 
A: press 'h' in the graphical window to get an overview. 
   Important are the mouse actions. Use the mouse buttons to move the camera. 
   Try different camera modes (1,2,3). Press <Ctrl> plus mouse button to drag a robot.
   The console can be entered by pressing <Ctrl>+C in the terminal. 
   Type 'help' there. 
EOF
) > $DIRNAME/README


pushd -n `pwd`
cd ../
SIMPATHS=`pwd`;
popd;

echo "cde-root$SIMPATHS" > $DIRNAME/simulationsdir

(
cat <<EOF
#!/bin/bash
HERE="\$(dirname "\$(readlink -f "\${0}")")"

SIM=cde-root$SIMPATHS/\`basename \$0\`

echo "Entering directory \$SIM"
cd \$SIM && \${HERE}/cde-exec ./start \$@
EOF
) > $DIRNAME/simulationtemplate

cp $DIRNAME/simulationtemplate $DIRNAME/$THISSIM
chmod u+x $DIRNAME/$THISSIM

# add also template for call from browser
(
cat <<EOF
#!/bin/bash
HERE="\$(dirname "\$(readlink -f "\${0}")")"

SIM=cde-root$SIMPATHS/\`basename \$0\`

echo "Entering directory \$SIM"
cd \$SIM && xterm -geom 100x50 -e \${HERE}/cde-exec ./start \$@
EOF
) > $DIRNAME/simulationbrowsertemplate

