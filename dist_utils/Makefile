# this is a makefile for creating a lpzrobots distribution 
#  with packages and so on
# you may need to update the ubuntu release names see vvvvvv below.
#  
##! Copy this makefile to a folder where you want to create
##!  the release. Simply take an empty folder than things go smooth.
##!
##! "Distribution Makefile Targets:"
# use make help to get a nice list



#### EDIT THE FOLLOWING LINES!
# make sure that you create the directories on the webserver and change the "current" links
#  if the major version changes
MAJOR=0.7
VERSION=$(MAJOR).2

ODEVERSION=0.11.3.3

# comment out if not wanted:
BUILDODE=1
BUILDGUILOGGER=1
BUILDMATRIXVIZ=1

#### END of Editing ################################

SHELL=/bin/bash
# make sure the right -config scripts are used 
# /usr/local/bin is included for ccache (optional)
export PATH=/usr/local/bin:/usr/bin:/bin

ROBOTSERVER=martius@robotcvs.informatik.uni-leipzig.de
#GITROOT=/home/georg/local/lpzrobots/.git
GITROOT=git://github.com/georgmartius/lpzrobots.git
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

##!createDist (1) fetches lpzrobots from the git and 
##!               creates the distribution folder and the tar files
# this filter-out thing causes make to not use this dependency if the dir exists
createDist: $(filter-out $(wildcard lpzrobots), lpzrobots) $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR))

##!deb-packs  (2) create debian and ubuntu packages
deb-packs: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) 
ifdef BUILDGUILOGGER
	$(MAKE) guilogger-deb
	$(MAKE) install-guilogger
endif
ifdef BUILDMATRIXVIZ
	$(MAKE) matrixviz-deb
	$(MAKE) install-matrixviz
endif
#	$(MAKE) lpzrobots-configurator-deb
#	$(MAKE) install-lpzrobots-configurator
	$(MAKE) selforg-deb 
	$(MAKE) install-selforg
ifdef BUILDODE
	$(MAKE) ode-deb 
	$(MAKE) install-ode
endif
	$(MAKE) oderobots-deb 
	$(MAKE) install-oderobots
	$(MAKE) doc

##!upload    (3) upload packages to launchpad this will sleep for 1h in between 
##!              in order to give launchpad enough time to generate the packages
upload: 
	$(MAKE) upload-lpzrobots
ifdef BUILDGUILOGGER
	-$(MAKE) upload-guilogger
endif
ifdef BUILDMATRIXVIZ
	-$(MAKE) upload-matrixviz
endif
	$(MAKE) upload-selforg
ifdef BUILDODE
	-$(MAKE) upload-ode
endif
	sleep 3600
	$(MAKE) upload-oderobots
	$(MAKE) changepermissions

##! ------ fine grain targets, which are called by the above targets ----


##!lpzrobots        fetches lpzrobots from the cvs
lpzrobots: 
	git clone $(GITROOT) lpzrobots

##!dist             create distribution folder
dist: $(DISTDIR)

# lpzrobots-0.X    create distribution folder
$(DISTDIR): $(filter-out $(wildcard lpzrobots), lpzrobots)
	cd lpzrobots
	cd lpzrobots/dist_utils && ./createDist.sh $(VERSION)
# call make to config
	cd $(DISTDIR) && rm -rf Makefile.conf && echo -e "/usr\nu\ny\n" | \
		make Makefile.conf
	cd $(DISTDIR) && make MODULE=selforg confsubmodule && make PATH=selforg:$$PATH MODULE=ode_robots confsubmodule && make PATH=selforg:ode_robots:$$PATH MODULE=ga_tools confsubmodule
# advance version numbers # also add lpzrobots/ga_tools 
	for dir in $(DISTDIR)/selforg $(DISTDIR)/ode_robots $(DISTDIR)/guilogger $(DISTDIR)/matrixviz; \
		do pushd $$dir && $(DCH) -v $(VERSION)-1 "New Release"; \
		popd; done
	cd $(DISTDIR)/opende && $(DCH) -v $(ODEVERSION)-1 "New Release"


##!deb-packs-noutils  create debian and ubuntu packages without utils (guilogger and ode)
deb-packs-noutils: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) 
	$(MAKE) selforg-deb 
	$(MAKE) install-selforg
	$(MAKE) oderobots-deb 
	$(MAKE) install-oderobots

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
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz  $(PACKDIR)/$(PackFolder)
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!guilogger-deb        create debian and ubuntu packages for guilogger
.PHONY: guilogger-deb
guilogger-deb: PackName=guilogger
guilogger-deb: PackVersion=$(VERSION)
guilogger-deb: PackFolder=guilogger
guilogger-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz  $(PACKDIR)/$(PackFolder)
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!matrixviz-deb        create debian and ubuntu packages for matrixviz
.PHONY: matrixviz-deb
matrixviz-deb: PackName=matrixviz
matrixviz-deb: PackVersion=$(VERSION)
matrixviz-deb: PackFolder=matrixviz
matrixviz-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz  $(PACKDIR)/$(PackFolder)
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!oderobots-deb        create debian and ubuntu packages for ode_robots
.PHONY: oderobots-deb
oderobots-deb: PackName=lpzrobots-oderobots
oderobots-deb: PackVersion=$(VERSION)
oderobots-deb: PackFolder=ode_robots
oderobots-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh 
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz $(PACKDIR)/$(PackFolder)
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)

##!gatools-deb        create debian and ubuntu packages for ga_tools
.PHONY: gatools-deb
gatools-deb: PackName=lpzrobots-gatools
gatools-deb: PackVersion=$(VERSION)
gatools-deb: PackFolder=ga_tools
gatools-deb: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) create_deb_packages.sh 
	$(MAKE) PackName=$(PackName) PackVersion=$(PackVersion) PackFolder=$(PackFolder) $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz  $(PACKDIR)/$(PackFolder)
	./create_deb_packages.sh $(PACKDIR) $(PackName) $(PackVersion) $(PackFolder) $(DebFull)


##!doc                      creates documentation bundle
.PHONY: doc
doc: $(filter-out $(wildcard $(DISTDIR)), $(DISTDIR)) 
	cp -r $(DISTDIR)/doc $(DISTDIR)/doc.orig
	$(MAKE) -C $(DISTDIR) doc
	mv $(DISTDIR)/doc lpzrobots-$(MAJOR)-doc
	mv $(DISTDIR)/doc.orig $(DISTDIR)/doc
	tar -czf lpzrobots-$(MAJOR)-doc.tgz lpzrobots-$(MAJOR)-doc
	rm -r lpzrobots-$(MAJOR)-doc


##!install-matrixviz        install the created matrixviz package
install-matrixviz:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/matrixviz*$(VERSION)*.deb


##!install-guilogger        install the created guilogger package
install-guilogger:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/guilogger*$(VERSION)*.deb

##!install-ode        install the created ode package
install-ode:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/libode-dbl*$(ODEVERSION)*.deb

##!install-selforg        install the created selforg package
install-selforg:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/lpzrobots-selforg*$(VERSION)*.deb

##!install-oderobots      install the created oderobots package
install-oderobots:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/lpzrobots-oderobots*$(VERSION)*.deb

##!install-gatools        install the created gatools package
install-gatools:
	sudo dpkg -i $(PACKDIR)/debian-unstable-bin/lpzrobots-gatools*$(VERSION)*.deb


##!uninstall-all          uninstall all packages
uninstall-all:
	sudo dpkg -r guilogger matrixviz lpzrobots-selforg libode-dbl libode-dbl-dev lpzrobots-oderobots


##!upload-lpzrobots       uploads master src tar-ball and documentation
upload-lpzrobots:
	scp lpzrobots-*.tgz $(ROBOTSERVER):/home/robot/htdocs/software/packages/src/current/
	-ssh $(ROBOTSERVER) "cd /home/robot/htdocs/software/packages/src/current/ && rm -rf lpzrobots-$(MAJOR)-doc && tar -xzf lpzrobots-$(MAJOR)-doc.tgz"

##!upload-guilogger       uploads the guilogger packages to launchpad
upload-guilogger: STEM=guilogger
upload-guilogger:
	cd $(PACKDIR)/ubuntu && dput -f ppa:georg-martius/lpzrobots guilogger*$(VERSION)*.changes
	-ssh $(ROBOTSERVER) "rm /home/robot/htdocs/software/packages/deb/current/$(STEM)*;"
	scp $(PACKDIR)/debian-unstable/$(STEM)*.gz $(PACKDIR)/debian-unstable/$(STEM)*.dsc $(ROBOTSERVER):/home/robot/htdocs/software/packages/deb/current/
	scp $(PACKDIR)/$(STEM)*.tar.gz  $(ROBOTSERVER):/home/robot/htdocs/software/packages/src/current/

##!upload-matrixviz       uploads the matrixviz packages to launchpad
upload-matrixviz: STEM=matrixviz
upload-matrixviz:
	cd $(PACKDIR)/ubuntu && dput -f ppa:georg-martius/lpzrobots matrixviz*$(VERSION)*.changes
	-ssh $(ROBOTSERVER) "rm /home/robot/htdocs/software/packages/deb/current/$(STEM)*;"
	scp $(PACKDIR)/debian-unstable/$(STEM)*.gz $(PACKDIR)/debian-unstable/$(STEM)*.dsc $(ROBOTSERVER):/home/robot/htdocs/software/packages/deb/current/
	scp $(PACKDIR)/$(STEM)*.tar.gz  $(ROBOTSERVER):/home/robot/htdocs/software/packages/src/current/

##!upload-ode             uploads the ode packages to launchpad
upload-ode: STEM=ode-dbl
upload-ode:
	cd $(PACKDIR)/ubuntu && dput -f ppa:georg-martius/lpzrobots ode-dbl*$(ODEVERSION)*.changes
	-ssh $(ROBOTSERVER) "rm /home/robot/htdocs/software/packages/deb/current/$(STEM)*;"
	scp $(PACKDIR)/debian-unstable/$(STEM)*.gz $(PACKDIR)/debian-unstable/$(STEM)*.dsc $(ROBOTSERVER):/home/robot/htdocs/software/packages/deb/current/
	scp $(PACKDIR)/$(STEM)*.tar.gz  $(ROBOTSERVER):/home/robot/htdocs/software/packages/src/current/

##!upload-selforg         uploads the selforg packages to launchpad
upload-selforg: STEM=lpzrobots-selforg
upload-selforg:
	cd $(PACKDIR)/ubuntu && dput -f ppa:georg-martius/lpzrobots lpzrobots-selforg*$(VERSION)*.changes
	-ssh $(ROBOTSERVER) "rm /home/robot/htdocs/software/packages/deb/current/$(STEM)*;"
	scp $(PACKDIR)/debian-unstable/$(STEM)*.gz $(PACKDIR)/debian-unstable/$(STEM)*.dsc $(ROBOTSERVER):/home/robot/htdocs/software/packages/deb/current/
	scp $(PACKDIR)/$(STEM)*.tar.gz  $(ROBOTSERVER):/home/robot/htdocs/software/packages/src/current/


##!upload-oderobots       uploads the oderobots packages to launchpad
upload-oderobots: STEM=lpzrobots-oderobots
upload-oderobots:
	cd $(PACKDIR)/ubuntu && dput -f ppa:georg-martius/lpzrobots lpzrobots-oderobots*$(VERSION)*.changes
	-ssh $(ROBOTSERVER) "rm /home/robot/htdocs/software/packages/deb/current/$(STEM)*;"
	scp $(PACKDIR)/debian-unstable/$(STEM)*.gz $(PACKDIR)/debian-unstable/$(STEM)*.dsc $(ROBOTSERVER):/home/robot/htdocs/software/packages/deb/current/
	scp $(PACKDIR)/$(STEM)*.tar.gz  $(ROBOTSERVER):/home/robot/htdocs/software/packages/src/current/

##!upload-oderobots       uploads the oderobots packages to launchpad
changepermissions: 
	-ssh $(ROBOTSERVER) "chmod -R g+r /home/robot/htdocs/software/packages/deb/;"

##### helper targets, not to be used from outside

# tar the packages
$(PACKDIR)/$(PackFolder)-$(PackVersion).tgz:
	mkdir -p $(PACKDIR)
	cd $(DISTDIR) && tar -czf "../$(PACKDIR)/$(PackFolder)-$(PackVersion).tgz" "$(PackFolder)"

# names them as package_version.orig.tar.gz
$(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz: $(PACKDIR)/$(PackFolder)-$(PackVersion).tgz
	cd $(PACKDIR) && cp "$(PackFolder)-$(PackVersion).tgz" "$(PackName)_$(PackVersion).orig.tar.gz"

$(PACKDIR)/$(PackFolder): $(PACKDIR)/$(PackName)_$(PackVersion).orig.tar.gz
	cd $(PACKDIR) && tar -xzf $(PackName)_$(PackVersion).orig.tar.gz

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
#>if [ -d "$Folder" ]; then
#>    echo "found $Folder";
#>else
#>    echo "$Folder does not exists-> untar $ORIGTGZ!";
#>    tar -xzf ${ORIGTGZ}
#>fi
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
# vvvvvvvvvvvvvvvvv UBUNTU DISTRIBUTIONS here vvvvvvvvvvvvvvvvvvvvvvvvv
#># ... precise
#>for Dist in lucid maverick natty oneiric precise quantal; do
#>    # change changelog for all ubuntu distributions  
#>    dch --increment --force-distribution -D $Dist "$Dist version"
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
