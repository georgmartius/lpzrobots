
#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>

include Makefile.conf

USAGE   = "Try 'make help' for more..."
USAGE2  = "lpzrobots Makefile Targets:"
USAGE3  = " Usually you do:\n  make prepare\n  \#follow instructions to compile ODE\n  make libs \# get a cup of tea\n  sudo make install"

##!help		show this help text (default)
help: 
	@cat logo.txt
	@echo $(USAGE2)
	@grep -E "^\#\#\!.*" Makefile | sed -e "s/##!/   /"
	@echo -e $(USAGE3)

.PHONY: prepare
##!prepare	build tools and create dependency files and check for ODE
prepare: usage 
	-$(MAKE) guilogger
	-$(MAKE) neuronviz
	-$(MAKE) soundman
	-$(MAKE) javacontroller
	cd selforg && $(MAKE) depend
	cd ode_robots && $(MAKE) depend
	-$(MAKE) tags
	@if test ! -e opende/Makefile && ld -lode -L$HOME/lib 2>/dev/null; then echo -e "You need to setup ODE from first!\nYou have 2 options: use a precompiled one from our webpage or compile the one in opende\nFor compiling please run:\ncd opende; sh autogen.sh\n./configure --enable-release --enable-double-precision\nmake\nsudo make install\n\nOn most SUSE linux computers it's necessary to run thereafter\n\nsudo ldconfig\n\nfor a correct linking of the libode.so!\n"; exit; fi
	@echo "********************************************************************************"
	@echo "Don't worry if you have seen a lot of errors above."
	@echo "This is all optional stuff which is not stricly required."
	@echo "Probably you want to install the ODE and OSG now "
	@echo " and then do \"make libs\" and \"(sudo) make install\""
	@echo "Usually you can use make -j 2 on multicore machines but not for the installation target."

.PHONY: conf
##!conf		configure the installation prefix and type(again)
conf: usage
	-mv Makefile.conf Makefile.conf.bak
	$(MAKE) Makefile.conf	

.PHONY: libs
##!libs		compile libaries in optimised and debug version
libs: usage
	@echo "*************** Compile selforg (optimized) *****************"
	cd selforg && $(MAKE) clean && $(MAKE) opt	
	@echo "*************** Compile ode_robots (optimized) **************"
	cd ode_robots && $(MAKE) clean && $(MAKE) opt	
	@echo "*************** Compile selforg (debug) *********************"
	cd selforg && $(MAKE) clean && $(MAKE) lib
	@echo "*************** Compile ode_robots (debug) ******************"
	cd ode_robots && $(MAKE) clean && $(MAKE) lib

.PHONY: install
##!install	install utils and possibly libs (if installation type: user)
install: usage install_utils install_libs

.PHONY: uninstall
##!uninstall	removes all the installed files again
uninstall: uninstall_intern

##!clean	removed the object files (but keeps the libs)
clean: usage
	cd guilogger && $(MAKE) clean
	cd ode_robots && $(MAKE) clean
	cd ode_robots/simulations && $(MAKE) clean
	cd selforg && $(MAKE) clean

##!********* less common targets ***********


.PHONY: guilogger
##!guilogger	compile guilogger
guilogger:
	cd guilogger && qmake guilogger.pro && qmake src/src.pro && make

.PHONY: neuronviz
##!neuronviz	compile neuronviz
neuronviz:
	cd neuronviz/src && $(MAKE)

.PHONY: javactrl
##!javactrl	compile javacontroller (experimental)
javactrl:
	cd javacontroller/src && $(MAKE)

.PHONY: soundman
##!soundman	compile soundman (experimental)
soundman:
	cd ode_robots/utils && javac SoundMan.java SoundManipulation.java SoundManGUI.java

.PHONY: install_utils
install_utils:
	-mkdir -p $(PREFIX)bin $(PREFIX)lib/soundMan $(PREFIX)share/lpzrobots
	-cd neuronviz/src && $(MAKE) install
	-cd javacontroller/src && $(MAKE) install
	-@cp guilogger/bin/guilogger $(PREFIX)bin/ && echo "copied guilogger to $(PREFIX)/bin/" || echo "Could not copy guilogger binary to $(PREFIX)bin/! Please install it by hand."
	-cp ode_robots/utils/*.class $(PREFIX)lib/soundMan/
	-cp ode_robots/utils/soundMan $(PREFIX)bin/soundMan
	sed -i -e "s|PREFIX=.*|PREFIX=$(PREFIX)|" $(PREFIX)bin/soundMan
	-cp ode_robots/utils/feedfile.pl $(PREFIX)bin/
	-cp ode_robots/utils/encodevideo.sh $(PREFIX)bin/
	-cp ode_robots/utils/selectcolumns.pl $(PREFIX)bin/
	cp -r ode_robots/osg/data $(PREFIX)share/lpzrobots/
	-find $(PREFIX)share/lpzrobots/ -type d -name "CVS" | xargs rm -r


.PHONY: install_libs
install_libs:
ifeq ($(INSTALL_TYPE),user)
	@echo "*************** Install selforg *********************"
	cp selforg/libselforg.a $(PREFIX)lib
	cp selforg/libselforg_opt.a $(PREFIX)lib
	-mkdir -p $(PREFIX)include
	cp -rL selforg/include/selforg $(PREFIX)include/
	@echo "*************** Install ode_robots ******************"
	cp ode_robots/libode_robots.a $(PREFIX)lib
	cp ode_robots/libode_robots_opt.a $(PREFIX)lib
	cp -rL ode_robots/include/ode_robots $(PREFIX)include/	
	@echo "*************** Finished ******************"
	@echo "Be aware that you have to use the Makefile.user in the simulations"
	@echo " since you install the simulator libaries and header files into global paths"
endif

.PHONY: uninstall_intern
uninstall_intern:
	-cd neuronviz/src && $(MAKE) uninstall
	-cd javacontroller/src && $(MAKE) uninstall
	-rm -f $(PREFIX)bin/guilogger
	-rm -f $(PREFIX)lib/soundMan/SoundMan*.class
	-rm -f $(PREFIX)bin/soundMan
	-rm -f $(PREFIX)bin/feedfile.pl
	-rm -f $(PREFIX)bin/encodevideo.sh 
	-rm -f $(PREFIX)bin/selectcolumns.pl
ifeq ($(INSTALL_TYPE),user)
	-rm -f $(PREFIX)lib/libselforg.a $(PREFIX)lib/libselforg_opt.a
	-rm -rf $(PREFIX)include/selforg
	-rm -f $(PREFIX)lib/libode_robots.a $(PREFIX)lib/libode_robots_opt.a
	-rm -rf $(PREFIX)include/ode_robots
endif


Makefile.conf:	
	@bash createMakefile.conf.sh

.PHONY: tags
##!tags		create TAGS file for emacs
tags: 
	etags `find selforg -type f -regex ".*\.[h]p?p?"`
	etags -a `find ode_robots -type f -regex ".*\.[h]p?p?"`

.PHONY: doc
##!doc 		generate doxygen documentation in html folder
doc:
	doxygen Doxyfile	

.PHONY: docintern
docintern:
	date "+PROJECT_NUMBER=%2d-%B-%Y" > builddate
	cat builddate Doxyfile.intern | nice -19 doxygen -	
	find doc/html -type f | xargs chmod ug+rw
	find doc/html -type f | xargs chmod o+r

.PHONY: docwarn
##!docwarn	show the warnings from doxygen
docwarn:
	cat doxygen.warn

##!todo		show the marked todos in the sourcecode
todo:
	cd ode_robots && make todo
	cd selforg && make todo





usage:
# The logo was creates with >$ figlet "LPZRobots" > logo.txt 
#  plus some editing
	@cat logo.txt
	@echo $(USAGE)
