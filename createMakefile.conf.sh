#!/bin/bash

echo "Generating Makefile.conf (configuration makefile)"
echo " You can change your preference by editing Makefile.conf"
echo " or just delete it and run make again"

defprefix=${1:-/usr/local}
deftype=${2:-DEVEL}
echo  "Where do you want to install the simulator?";
echo  "Please use either /usr, /usr/local  or you home directory"
echo  "unless you know what you are doing. (no tailing slash)";
echo -n "e.g. (/home/yourlogin) (don' use ~): [$defprefix] ";
read prefix 
[ -z "$prefix" ] && prefix=$defprefix  # $(HOME)'
echo  "Checking PATH variable: "
if ! echo $PATH | grep "$prefix/bin"; then
    echo "Cannot find "$prefix/bin" in PATH variable!" 
    echo "The installation will not work if you do not add it!" 
    echo " use export PATH=$prefix/bin:\$PATH in your ~/.bashrc and " 
    echo " type: source ~/.bashrc to reload the settings!" 
    cp -f Makefile.conf.bak Makefile.conf
    exit 1;
else 
echo  " ... Okay!"
fi


# system autodetection
# some muggling for the achitecture
OS=`uname -a | sed 's/\(\w*\).*/\1/'` # selects first word (e.g Linux or Darwin)

if [ "$OS" = "Linux" ]; then
  #linux
  System="LINUX"
else
  # mac
  System="MAC"
fi

if [ $deftype = "DEVEL" ]; then
    defchoice="d";
else
    defchoice="u";
fi
echo -en "Installation type (user or development):\n\
 Choose user  (u) if you are a user and only program your own simulations (default)\n\
 Choose devel (d) if you develop the simulator\n\
Our choice (u/d): [$defchoice] "
read choice 
if [ -z "$choice" ]; then
    choice=$defchoice;
fi
if [ "$choice" = "U" ]; then choice='u'; fi
if [ "$choice" = "D" ]; then choice='d'; fi

echo -e "Check your settings:\n Installation to $prefix";
if [ "$choice" = "u" ]; then 
 echo " (u) user installation"
 Type=USER
else 
 echo " (d) development installation"
 Type=DEVEL
fi
echo -n "All right? [y/N] "
read okay 
if [ ! "$okay" = "y" ]; then
    echo "Since you didn't say yes I better quit. Run \"make conf\" again!"
    cp -f Makefile.conf.bak Makefile.conf
    exit 1;
fi


echo -e "# configuration file for lpzrobots (automatically generated!)\n\
# Where to install the simulator and utils"  > Makefile.conf
echo "PREFIX=$prefix" >> Makefile.conf
echo -e "\n# user or developement installation\n\
# Options:\n\
#  DEVEL: only install the utility functions,\n\
#   which is useful for development on the simulator\n\
#  USER: install also the ode_robots and selforg libaries and include files\n\
#   this is recommended for users" >> Makefile.conf
echo "TYPE=$Type" >> Makefile.conf

echo "// Automatically generated file! Use make conf in lpzrobots." > ode_robots/install_prefix.conf
echo "#define PREFIX \"$prefix\"" >> ode_robots/install_prefix.conf

for Folder in selforg ode_robots configurator; do
    echo "call $Folder/configure --prefix=\"$prefix\" --system=\"$System\" --type=\"$Type\"";  
    if ! $Folder/configure --prefix="$prefix" --system="$System" --type="$Type"; then
        exit 1;
    fi
done


for Folder in ode_robots/simulations ode_robots/examples selforg/simulations selforg/examples ga_tools/simulations; do
  echo "call: m4 -D \"$System\" -D \"$Type\" $Folder/Makefile.4sim.m4";
  if m4 -D "$System" -D "$Type" "$Folder/Makefile.4sim.m4" > "$Folder/Makefile.4sim"; then
    for F in `find "$Folder" -mindepth 2 -name Makefile.conf`; do
        cp "$Folder/Makefile.4sim" "${F%.conf}";
    done
  fi
done

