#!/bin/bash

################################################################################
# Install script
# 
# Project: lpzrobots extra libs and includes
# Author: Georg Martius <georg@nld.ds.mpg.de>
# Copyright: 2008 Georg Martius
#
################################################################################

echo  "Welcome to the install script of lpzrobots-extra-lib-devel.";
echo  "This package contains libaries and headerfiles for ODE and OSG required by lpzrobots";
echo  "For updates and documentation visit http://robot.informatik.uni-leipzig.de";
echo  "Press CTRL + C to abort";

echo -e "\n-- Installation base directory DIR --";
echo -e "\tLibaries go to \$DIR/lib/ ,binary files to \$DIR/bin/ and includes to \$DIR/include/";
echo -e "\tFor system wide install keep the default (requires root permissions)";
echo -e "\tFor a user installions use /home/yourloginname and follow the instructions below";
echo -en "\t**Enter base directory [/usr/local]: ";
read dir;
if [ -z "$dir" ]; then
   dir='/usr/local';
else
    echo "Make sure that the following environment variables are set: PATH, LD_LIBRARY_PATH"
    echo " if you use non-standard paths."
    echo " for bash users add the following to your ~/.bash.bashrc:"
    echo -e "\texport PATH=\$PATH:$dir/bin"
    echo -e "\texport LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$dir/lib"    
    echo -e "\tyou might have to modify the Makefiles by adding to INC and LIBS variables your paths" 
fi

if ! cp -r include lib bin $dir/; then
 echo "ERROR: cannot copy the files to $dir/!"; 
 exit 1;
fi
echo "Have Fun!";

