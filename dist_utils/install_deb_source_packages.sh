#!/bin/bash

set -e;

wget -r -l 1 -nd http://robot.informatik.uni-leipzig.de/software/packages/deb/current/

PACKS=guilogger lpzrobots-oderobots lpzrobots-selforg  ode-dbl #lpzrobots-gatools
for P in $PACKS; do
    # check for multiple version
    NUM=`echo $P*.dsc | wc -w`
    NEWEST=`echo $P*.dsc | cut -d " " -f $NUM`
    dpkg-source -x $NEWEST
    cd $(find -type d -name "guilogger*")
    dpkg-buildpackage -rfakeroot -b
    cd ../
done

#make sure that we are root
if [ ! $USER = "root" ]; then
  echo "We need to become root";
  su
fi

dpkg -i *.deb
