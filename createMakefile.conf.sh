#!/bin/bash



echo "Generating Makefile.conf (configuration makefile)"

echo -n "Where do you want to install the simulator? [$HOME]"
read prefix 
[ -z "$prefix" ] && prefix='$(HOME)'

echo -en "Installation type (user or development):\n\
 Choose user (u) if you are a users and only program your own simulations (default)\n\
 Choose devel  (d)  if you develop the simulator\n\
Our choice [U/d]"
read choice 
[ -z "$choice" ] && choice='u'

echo -e "Check your settings:\n Installation to $prefix";
if [ "$choice" = "u" ]; then echo " user installation with libaries and include files."
else echo " developement installation without libaries and include files, only utilities."
fi
echo -n "All right? [y/N]"
read okay 
if [ ! "$okay" = "y" ]; then
    echo "Since you said no I better exit. Run again!"
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
if [ "$choice" = "u" ]; then 	
echo "INSTALL_TYPE=user" >> Makefile.conf
else 
echo "INSTALL_TYPE=devel" >> Makefile.conf
fi

echo "You can change your preference by editing Makefile.conf"

