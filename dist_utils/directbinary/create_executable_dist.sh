#!/bin/bash

if [ ! -x ./start ]; then
   echo "run this script in a directory where a simulation (./start) is found."
   exit -1
fi

if [ "$1" = "-h" ]; then
    echo "Usage: $0 [-h] [-n] [NameOfDist]"
    echo "       -h   help"
    echo "       -n   use the exisiting traces (tracesim and traceguil tracematviz)"
    exit 0;
fi

if [ ! "$1" = "-n" ]; then
    echo "The simulation in this directory is started now. Close it manually."
    echo "Afterwards the guilogger opens, also close it. Press Enter to continue."
    read TEST
    
# trace all open calls
    strace -e trace=open -o tracesim ./start
    
    strace -e trace=open -o traceguil  guilogger

    echo "#C foo bar" | strace -e trace=open -o tracematviz  matrixviz

else
    shift;
    if [ -e tracesim -a -e traceguil -a -e tracematviz ]; then
        echo " I am useing the exising tracesim traceguil."
    else
        echo " cannot find the trace files (strace -e trace=open ...) tracesim traceguil tracematviz."
        exit -1;
    fi
fi

# get all so files that are loaded
cat tracesim traceguil tracematviz | grep ".*\.so" | grep -v "ENOENT" | grep -v "cache" | grep -v "nvidia" | sed "s/.*\"\(.*\)\".*/\1/" | sort | uniq > libs

BASENAME=lpzrobots
if [ -n "$1" ]; then
    BASENAME=$1
fi

DIRNAME=$BASENAME-`uname -m`
if [ -d $DIRNAME ]; then rm -r $DIRNAME; fi
mkdir $DIRNAME
mkdir $DIRNAME/lib
for F in `grep -v "osgPlugins" libs`; do 
    cp -v $F $DIRNAME/lib/
done

# does not work because pkg information is wrong
#OSGLIBPATH=`pkg-config --libs openscenegraph | sed "s/.*-L\([^ ]*\) .*/\1/"`
OSGPLUGINGPATH=$(dirname $(grep osgPlugins libs | head -n 1))
if [ -n "$OSGPLUGINGPATH" ]; then
    cp -v -r $OSGPLUGINGPATH $DIRNAME/lib/
else
    echo "Cannot find osgPlugins!"
fi

mkdir $DIRNAME/bin
GUILOGGER=`which guilogger`
MATRIXVIZ=`which matrixviz`
BINDIR=`dirname $GUILOGGER`
PREFIX=${BINDIR%/bin}

echo "Found guilogger here: ${GUILOGGER}."
echo "Found matrixviz here: ${MATRIXVIZ}."
echo "The binary directory is ${BINDIR}"
echo " and the prefix is ${PREFIX}."

cp ${GUILOGGER} $DIRNAME/bin/
cp ${MATRIXVIZ} $DIRNAME/bin/
for F in encodevideo.sh  feedfile.pl  selectcolumns.pl; do
    cp ${BINDIR}/$F $DIRNAME/bin/
done
if [ -e ${PREFIX}/share/lpzrobots/data ]; then
   cp -r ${PREFIX}/share/lpzrobots/data $DIRNAME/
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

(
cat <<'EOF'
export LD_LIBRARY_PATH=`pwd`/lib:$LD_LIBRARY_PATH
export PATH=`pwd`/bin:$PATH
export ODEROBOTSDATA=`pwd`/data
export OSG_LIBRARY_PATH=`pwd`/lib
EOF
) > $DIRNAME/setupenv.sh

(
cat <<'EOF'
#!/bin/bash
SIM=simulations/`basename $0`

. ./setupenv.sh
echo "Entering directory $SIM"
cd $SIM && ./start $@
EOF
) > $DIRNAME/simulationtemplate

# add also template for call from browser
(
cat <<'EOF'
#!/bin/bash
SIM=mysim
DIR=`dirname $0`
cd $DIR;
xterm -geom 100x50 -e ./$SIM
EOF
) > $DIRNAME/simulationbrowsertemplate


mkdir $DIRNAME/simulations
