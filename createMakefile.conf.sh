#!/bin/bash


echo "Generating Makefile.conf (configuration makefile)"
echo " You can change your preference by editing Makefile.conf"
echo " or just delete it and run make again"

echo  "Where do you want to install the simulator?";
echo -n "e.g. (/home/yourlogin) (don' use ~): [/usr/local] ";
read prefix 
[ -z "$prefix" ] && prefix='/usr/local'  # $(HOME)'

# system autodetection
# some muggling for the achitecture
OS=`uname -a | sed 's/\(\w*\).*/\1/'` # selects first word (e.g Linux or Darwin)

if [ "$OS" = "Linux" ]; then
  #linux
  System="linux"
else
  # mac
  System="mac"
fi


# check for CVS, if so then no user installation! 
if [ -d "CVS" ]; then 
    echo "You work with the CVS, such that you can only do a development installation."
    choice="d";
else 
    echo -en "Installation type (user or development):\n\
 Choose user  (u) if you are a user and only program your own simulations (default)\n\
 Choose devel (d) if you develop the simulator\n\
Our choice [U/d] "
    read choice 
    if [ -z "$choice" -o "$choice" = "U" ]; then choice='u'; fi
fi
echo -e "Check your settings:\n Installation to $prefix";
if [ "$choice" = "u" ]; then echo " (u) user installation with libaries and include files."
else echo " (d) development installation without libaries and include files, only utilities."
fi
echo -n "All right? [y/N] "
read okay 
if [ ! "$okay" = "y" ]; then
    echo "Since you didn't say yes I better quit. Run again!"
    exit 1;
fi


echo -e "# configuration file for lpzrobots\n\
# Where to install the simulator and utils"  > Makefile.conf
echo "PREFIX=$prefix/" >> Makefile.conf
echo -e "\n# user or developement installation\n\
# Options:\n\
#  devel: only install the utility functions,\n\
#   which is useful for development on the simulator\n\
#  user: install also the ode_robots and selforg libaries and include files\n\
#   this is recommended for users" >> Makefile.conf
if [ -e  ode_robots/install_prefix.conf ]; then
    sed -e "s/^#define.*//" -ibak ode_robots/install_prefix.conf;
fi
echo -e "#define PREFIX \"$prefix\"" >> ode_robots/install_prefix.conf
if [ "$choice" = "u" ]; then 	
  echo "INSTALL_TYPE=user" >> Makefile.conf
  echo "copy the user Makefiles";
  for Folder in ode_robots/simulations selforg/simulations selforg/examples ga_tools/simulations; do
    for F in `find $Folder -mindepth 2 -name Makefile.conf`; do
#        if [ ! -e ${F}.devel ]; then cp $F ${F}.devel; fi # backup development Makefile
        cp $Folder/Makefile.$System.user ${F%.conf};
    done
  done
else 
  echo "INSTALL_TYPE=devel" >> Makefile.conf
  echo "copy the development Makefiles";
  for Folder in ode_robots/simulations selforg/simulations selforg/examples ga_tools/simulations; do
    for F in `find $Folder -mindepth 2 -name Makefile.conf`; do
        cp $Folder/Makefile.$System.devel ${F%.conf};
    done
  done
fi

