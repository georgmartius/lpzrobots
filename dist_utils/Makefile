# this is a makefile for creating a lpzrobots distribution 
#  with packages and so on
##! Copy this makefile to a folder where you want to create
##!  the release. Simply take an empty folder than things go smooth.
##!
##! "Distribution Makefile Targets:"
# use make help to get a nice list



#### EDIT THE FOLLOWING LINES!
VERSION=0.6.1
ODEVERSION=0.11.2

export CVSROOT=:ext:martius@robotcvs.informatik.uni-leipzig.de:/home/robot/cvsroot
export DEBEMAIL=Georg Martius <georg.martius@web.de>
export GPGKEY=0CFA2813

#### END EDITING


DISTDIR = lpzrobots-$(VERSION)
PACKDIR = packages

DCH = dch --force-distribution -D unstable 

# do not set these variables, they are set below with some magic
PackName=non
PackVersion=$(VERSION)
PackFolder=non
DebFull=  # this can be set from the commandline

##!help		show this help text (default)
help: 
	@grep -E "^\#\#\!.*" Makefile | sed -e "s/##!/   /"


##!createDist (1) fetches lpzrobots from the cvs and 
##!               creates the distribution folder and the tar files
# this filter-out thing causes make to not use this dependency if the dir exists
createDist: $(filter-out $(wildcard lpzrobots), lpzrobots) $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR))

##!deb-packs  (2) create debian and ubuntu packages
deb-packs: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) 
	$(MAKE) guilogger-deb
	$(MAKE) install-guilogger
	$(MAKE) selforg-deb 
	$(MAKE) install-selforg
	$(MAKE) ode-deb 
	$(MAKE) install-ode
	$(MAKE) oderobots-deb 
	$(MAKE) install-oderobots

##! ------ fine grain targets, which are called by the above targets ----


##!lpzrobots        fetches lpzrobots from the cvs
lpzrobots: 
	cvs "-d$(CVSROOT)" co lpzrobots;

##!lpzrobots-0.X    create distribution folder
$(DISTDIR): $(filter-out $(wildcard lpzrobots), lpzrobots)
	cd lpzrobots && cvs "-d$(CVSROOT)" update -dP 2> ../cvs.log
# advance version numbers # also add lpzrobots/ga_tools 
	for dir in lpzrobots/selforg lpzrobots/ode_robots lpzrobots/guilogger; do pwd; echo $$dir; cd $$dir && $(DCH) -v $(VERSION)-1 "New Release"; cd ../../; done
	cd lpzrobots/opende && $(DCH) -v $(ODEVERSION)-1 "New Release"
	cd lpzrobots/dist_utils && ./createDist.sh $(VERSION)



##!selforg-deb        create debian and ubuntu packages for selforg
.PHONY: selforg-deb
selforg-deb: PackName=lpzrobots-selforg
selforg-deb: PackVersion=$(VERSION)
selforg-deb: PackFolder=selforg
selforg-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)


##!ode-deb        create debian and ubuntu packages for ode
.PHONY: ode-deb
ode-deb: PackName=ode-dbl
ode-deb: PackVersion=$(ODEVERSION)
ode-deb: PackFolder=opende
ode-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!guilogger-deb        create debian and ubuntu packages for guilogger
.PHONY: guilogger-deb
guilogger-deb: PackName=guilogger
guilogger-deb: PackVersion=$(VERSION)
guilogger-deb: PackFolder=guilogger
guilogger-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!oderobots-deb        create debian and ubuntu packages for ode_robots
.PHONY: ode_robots-deb
oderobots-deb: PackName=lpzrobots-oderobots
oderobots-deb: PackVersion=$(VERSION)
oderobots-deb: PackFolder=ode_robots
oderobots-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh 
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!gatools-deb        create debian and ubuntu packages for ga_tools
.PHONY: ode_robots-deb
gatools-deb: PackName=lpzrobots-gatools
gatools-deb: PackVersion=$(VERSION)
gatools-deb: PackFolder=ga_tools
gatools-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh 
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)


##!install-guilogger        install the created guilogger package
install-guilogger:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/guilogger*.deb

##!install-ode        install the created ode package
install-ode:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/ode-dbl*.deb

##!install-selforg        install the created selforg package
install-selforg:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/lpzrobots-selforg*.deb

##!install-oderobots        install the created oderobots package
install-oderobots:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/lpzrobots-oderobots*.deb

##!install-gatools        install the created gatools package
install-gatools:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/lpzrobots-gatools*.deb



##### helper targets, not to be used from outside

# tar the packages
$(PACKDIR)/$(PackFolder)-$(PackVersion).tgz:
	mkdir -p $(PACKDIR)
	cd $(DISTDIR) && tar -czf "../$(PACKDIR)/$(PackFolder)-$(PackVersion).tgz" "$(PackFolder)"

# names them as package_version.orig.tar.gz
$(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz: $(PACKDIR)/$(PackFolder)-$(PackVersion).tgz
	cd $(PACKDIR) && cp "$(PackFolder)-$(PackVersion).tgz" "$(PackName)_$(PackVersion).orig.tar.gz"


create_deb_packages.sh: Makefile
	@echo "Create create_deb_packages.sh from Makefile"
	@grep  "^#>.*" Makefile | sed -e "s/^#>\(.*\)/\1/" > create_deb_packages.sh
	@chmod u+x create_deb_packages.sh


## embedded script 
## perl -e 'foreach $f (<>){ print "#>" . $f;}' < create_deb_packages.sh
#>#!/bin/bash
#>
#>set -e # exit if command fails
#>if [ $# -le 3 ]; then
#>   echo "Usage $0 packdir packname version folder [onlysource]"
#>   echo "    onlysource means that the debian package is not build completetly"
#>   echo "    this has not influence on the ubuntu packages anyway"
#>   exit 1
#>fi 
#>Packdir=$1
#>PackName=$2
#>Version=$3
#>Folder=$4
#>OnlySource=$5
#>
#>pushd `pwd`
#>
#>ORIGTGZ="${PackName}_${Version}.orig.tar.gz"
#>if [ -e "$Packdir/${ORIGTGZ}" ]; then
#>    echo "found $ORIGTGZ";
#>else
#>    echo "$ORIGTGZ not found in $Packdir!";
#>    exit 1;
#>fi
#>    
#># untar them 
#>cd "$Packdir"
#>tar -xzf ${ORIGTGZ}
#>
#>mkdir -p debian-unstable
#>mkdir -p debian-unstable-bin
#># first make debian unstable package
#>if [ -z "$OnlySource" ]; then # full package
#>    pushd `pwd`
#>    cd ${Folder}
#>    debuild
#>    popd
#>    mv *.build *.changes *.deb debian-unstable-bin
#>    find -maxdepth 1 -name "*.debian.tar.gz" -exec mv \{\} debian-unstable-bin \;
#>else
#>    echo "########## BUILD ONLY SOURCE DEBIAN PACKAGE ########### "
#>fi
#>pushd `pwd`
#>cd ${Folder}
#>debuild -S 
#>popd
#>
#># copy debian packages
#>mv *.build *.changes *.dsc debian-unstable
#>find -maxdepth 1 -name "*.debian.tar.gz" -exec mv \{\} debian-unstable \;
#>find -maxdepth 1 -name "*.diff.gz" -exec mv \{\} debian-unstable \;
#>cp ${ORIGTGZ} debian-unstable
#># make ubuntu packages for launchpad
#>mkdir -p ubuntu
#>pushd `pwd`
#>cd ${Folder}
#>FIRST=""
#>for Dist in jaunty karmic lucid maverick; do
#>    # change changelog for all ubuntu distributions  
#>    dch --increment -D $Dist "$Dist version"
#>    # make only source package
#>    if [ -z "$FIRST" ]; then
#>        debuild -S -sa
#>    else
#>        debuild -S
#>    fi
#>    FIRST="no"
#>done
#>popd
#>cp ${ORIGTGZ} ubuntu
#>mv *.build *.changes *.dsc ubuntu
#>find -maxdepth 1 -name "*.debian.tar.gz" -exec mv \{\} ubuntu \;
#>find -maxdepth 1 -name "*.diff.gz" -exec mv \{\} ubuntu \;
