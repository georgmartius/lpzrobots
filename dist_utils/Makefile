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
GUILOGGERVERSION=$(VERSION)

export CVSROOT=:ext:martius@robotcvs.informatik.uni-leipzig.de:/home/robot/cvsroot
export DEBEMAIL="Georg Martius <georg.martius@web.de>"
export GPGKEY=0CFA2813

#### END EDITING


DISTDIR = lpzrobots-$(VERSION)
PACKDIR = packages


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
deb-packs: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) \
 selforg-deb ode-deb guilogger-deb ode_robots-deb 

##! ------ fine grain targets, which are called by the above targets ----


##!lpzrobots        fetches lpzrobots from the cvs
lpzrobots: 
	cvs "-d$(CVSROOT)" co lpzrobots;

##!lpzrobots-0.X    create distribution folder
$(DISTDIR): $(filter-out $(wildcard lpzrobots), lpzrobots)
	cd lpzrobots && cvs "-d$(CVSROOT)" update -dP 2> ../cvs.log
	cd lpzrobots/dist_utils && ./createDist.sh $(VERSION)



##!selforg-deb        create debian and ubuntu packages for selforg
.PHONY: selforg-deb
selforg-deb: PackName=lpzrobots-selforg
selforg-deb: PackVersion=$(VERSION)
selforg-deb: PackFolder=selforg
selforg-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)


##!ode-deb        create debian and ubuntu packages for ode
.PHONY: ode-deb
ode-deb: PackName=ode-dbl
ode-deb: PackVersion=$(ODEVERSION)
ode-deb: PackFolder=opende
ode-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!guilogger-deb        create debian and ubuntu packages for guilogger
.PHONY: guilogger-deb
guilogger-deb: PackName=guilogger
guilogger-deb: PackVersion=$(GUILOGGERVERSION)
guilogger-deb: PackFolder=guilogger
guilogger-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!ode_robots-deb        create debian and ubuntu packages for ode_robots
.PHONY: ode_robots-deb
ode_robots-deb: PackName=lpzrobots-ode_robots
ode_robots-deb: PackVersion=$(VERSION)
ode_robots-deb: PackFolder=ode_robots
ode_robots-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)


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
#>   echo "Usage $0 packdir packname version folder [debfull]"
#>   echo "    debfull means that the debian package is make completetly"
#>   echo "    with source and binary but no ubuntu packages are build"
#>   exit 1
#>fi 
#>Packdir=$1
#>PackName=$2
#>Version=$3
#>Folder=$4
#>DebFull=$5
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
#>pushd `pwd`
#>
#>cd ${Folder}
#># first make debian unstable package
#>if [ -n "$DebFull" ]; then # full package
#>    echo "########## BUILD FULL DEBIAN PACKAGE ########### "
#>    debuild
#>else
#>    debuild -S 
#>fi
#>
#># copy debian packages
#>popd
#>pushd `pwd`
#>mkdir -p debian-unstable
#>mv *.diff.gz *.build *.changes *.dsc debian-unstable
#>cp ${ORIGTGZ} debian-unstable
#>if [ -n "$DebFull" ]; then 
#>  mv *.deb debian-unstable
#>else # make the ubuntu packages as well (only source)
#>    # make ubuntu packages for launchpad
#>    mkdir -p ubuntu
#>    cd ${Folder}
#>    FIRST=""
#>    for Dist in jaunty karmic lucid maverick; do
#>        # change changelog for all ubuntu distributions  
#>        dch --increment -D $Dist "$Dist version"
#>        # make only source package
#>        if [ -z "$FIRST" ]; then
#>            debuild -S -sa
#>        else
#>            debuild -S
#>        fi
#>        FIRST="no"
#>    done
#>    popd
#>    cp ${ORIGTGZ} ubuntu
#>    mv *.diff.gz *.build *.changes *.dsc ubuntu
#>fi
#>popd # return to place where we started
