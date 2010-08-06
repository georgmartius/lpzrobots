# Makefile to download and install debian source packages of lpzrobots

##! all		check for empty directory
all: checkemptydir deps download alldebs

checkemptydir:
	@if [ `ls | wc -l` -gt 1 ]; then \
 echo "the current directory should be empty (except of the makefile)";	exit 1; fi

##! deps	installs all dependencies
#this has to be updated when the Dependencies file changes
deps:
	su -c "apt-get install \
 g++  make  xutils-dev m4  libreadline-dev  libgsl0-dev \
 libglu-dev libgl1-mesa-dev libglut3-dev  libopenscenegraph-dev \
 libqt4-dev  qt4-qmake  libqt4-qt3support gnuplot \
 docbook-to-man automake libtool"

##! download 	download packages
download:
	wget -r -l 1 -nd http://robot.informatik.uni-leipzig.de/software/packages/deb/current/

##! alldebs	build all debian packages (see below for the individual ones)
alldebs: guilogger_ ode-dbl_ lpzrobots-selforg_ lpzrobots-oderobots_ #lpzrobots-gatools_

##! guilogger_ 
guilogger_: 
	$(MAKE) P=guilogger guilogger
##! ode-dbl_
ode-dbl_: 
	$(MAKE) P=ode-dbl ode-dbl

##! lpzrobots-selforg_
lpzrobots-selforg_: 
	$(MAKE) P=lpzrobots-selforg lpzrobots-selforg

##! lpzrobots-oderobots_
lpzrobots-oderobots_: 
	$(MAKE) P=lpzrobots-oderobots lpzrobots-oderobots

$(P): 
#check for multiple version
	NUM=`echo $(P)*.dsc | wc -w`; \
 NEWEST=`echo $(P)*.dsc | cut -d " " -f $$NUM`; \
 dpkg-source -x $$NEWEST
	cd `find -type d -name "$(P)*"` && dpkg-buildpackage -rfakeroot -b -uc
#make sure that we are root
	if [ ! $(USER) = "root" ]; then su -c "dpkg -i *$(P)*.deb"; else dpkg -i $P*.deb;        fi


##! help	print this message
help:
	@grep -E "^\#\#\!.*" install_deb_source.makefile | sed -e "s/##!/   /"
