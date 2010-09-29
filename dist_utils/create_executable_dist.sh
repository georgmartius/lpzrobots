#!/bin/bash

if [ ! -x ./start ]; then
   echo "run this script in a directory where a simulation (./start) is found."
   exit -1
fi

if [ "$1" = "-h" ]; then
    echo "Usage: $0 [-h] [-n]"
    echo "       -h   help"
    echo "       -n   use the exisiting traces (tracesim and traceguil)"
    exit 0;
fi

if [ ! "$1" = "-n" ]; then
    echo "The simulation in this directory is started now. Close it manually."
    echo "Afterwards the guilogger opens, also close it. Press Enter to continue."
    read TEST
    
# trace all open calls
    strace -e trace=open -o tracesim ./start
    
    strace -e trace=open -o traceguil  guilogger
else
  if [ -e tracesim -a -e traceguil ]; then
      echo " I am useing the exising tracesim traceguil."
  else
      echo " cannot find the trace files (strace -e trace=open ...) tracesim traceguil."
      exit -1;
  fi
fi

# get all so files that are loaded
cat tracesim traceguil | grep ".*\.so" | grep -v "ENOENT" | grep -v "cache" | grep -v "nvidia" | sed "s/.*\"\(.*\)\".*/\1/" | sort | uniq > libs


DIRNAME=lpzrobots-`uname -m`
rm -r $DIRNAME
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
BINDIR=`dirname $GUILOGGER`
PREFIX=${BINDIR%/bin}

echo "Found guilogger here: ${GUILOGGER}."
echo "The binary directory is ${BINDIR}"
echo " and the prefix is ${PREFIX}."

cp ${GUILOGGER} $DIRNAME/bin/
for F in encodevideo.sh  feedfile.pl  selectcolumns.pl; do
    cp ${BINDIR}/$F $DIRNAME/bin/
done
if [ -e ${PREFIX}/share/lpzrobots/data ]; then
   cp -r ${PREFIX}/share/lpzrobots/data $DIRNAME/
else
   echo "could not find DATA in ${PREFIX}/share/lpzrobots/data"
fi

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
SIM=simulations/mysim

. ./setupenv.sh
echo "Entering directory $SIM"
cd $SIM && ./start $@
EOF
) > $DIRNAME/simulationtemplate

mkdir $DIRNAME/simulations
