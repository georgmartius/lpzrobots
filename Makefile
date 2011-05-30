#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005
SHELL=/bin/bash

include Makefile.conf

USAGE   = "Try 'make help' for more..."
USAGE2  = "lpzrobots Makefile Targets:"
USAGE3  = "Usually you do:\nmake prepare\nmake ode\t\t\# if not installed\nsudo make install_ode \t\# if not installed\nmake libs \t\t\# get a cup of tea\nsudo make install\n"

##!help		show this help text (default)
help: 
	@cat logo.txt
	@echo $(USAGE2)
	@grep -E "^\#\#\!.*" Makefile | sed -e "s/##!/   /"
	@echo -e  $(USAGE3)

.PHONY: prepare
##!prepare	build tools and create dependency files (do that first)
prepare: usage 
	-$(MAKE) guilogger
	-$(MAKE) matrixviz
	-$(MAKE) soundman
	-$(MAKE) javacontroller
	cd selforg && $(MAKE) depend
	cd ode_robots && $(MAKE) depend
	cd ga_tools && $(MAKE) depend
	-$(MAKE) tags
	@echo "********************************************************************************"
	@echo "Don't worry if you have seen a lot of errors above."
	@echo "This is all optional stuff which is not stricly required."
	@echo "Probably you want to install the ODE (\"make ode\" and \"make install_ode\")"
	@echo " and Openscenegraph now (e.g. openscenegraph-dev from your linux distribution)"
	@echo " and then do \"make libs\" and \"(sudo) make install\""
	@echo "Usually you can use make -j 2 on multicore machines but not for the installation target."

.PHONY: ode
##!ode		compile open dynamics engine in double precession (custom version)
ode:
	cd opende; sh autogen.sh && ./configure --disable-asserts --enable-double-precision --prefix=$(PREFIX) --disable-demos && $(MAKE) && echo "you probably want to run \"make install_ode\" now (possibly as root)"


.PHONY: install_ode
##!install_ode	install the customized ode library (libode_dbl)
install_ode:
	@echo "*************** Install ode -double version**********"
	cd opende && $(MAKE) install

.PHONY: libs
##!libs		compile our libaries in optimized and debug version
libs: usage
	@echo "*************** install selforg *****************"
	cd selforg && $(MAKE) 
	@echo "*************** Compile ode_robots **************"
	cd ode_robots && $(MAKE)
	@echo "*************** Compile ga_tools **************"
	cd ga_tools && $(MAKE)

.PHONY: install
##!install	install utils and libs
install: usage install_utils install_libs

.PHONY: uninstall
##!uninstall	removes all the installed files again (except ode)
uninstall: uninstall_intern

##!clean	removed the object files and libs
clean: usage
	cd guilogger && $(MAKE) clean
	cd ode_robots && $(MAKE) clean
	cd selforg && $(MAKE) clean
	cd ga_tools && $(MAKE) clean


##!clean-all	like clean but also removes the libraries and clears simulations
clean-all: usage
	cd guilogger && $(MAKE) distclean
	cd ode_robots && $(MAKE) clean-all
	cd ode_robots/simulations && $(MAKE) clean
	cd selforg && $(MAKE) clean-all
	cd selforg/simulations && $(MAKE) clean
	cd selforg/examples && $(MAKE) clean
	cd ga_tools && $(MAKE) clean-all
	cd ga_tools/simulations && $(MAKE) clean
	rm -f Makefile.conf

##!distclean	see clean-all
distclean :  clean-all

##!********* less common targets ***********

.PHONY: conf
##!conf		configure the installation prefix and installation type (to redo it)
conf: usage
	-mv Makefile.conf Makefile.conf.bak
# automatically creates Makefile.conf since it is included 
	$(MAKE) Makefile.conf


.PHONY: guilogger
##!guilogger	compile guilogger
guilogger:
	cd guilogger && ./configure && $(MAKE)

.PHONY: matrixviz
##!matrixviz	compile matrixviz
matrixviz:
	cd matrixviz && ./configure && $(MAKE)

.PHONY: javactrl
##!javactrl	compile javacontroller (experimental)
javactrl:
	cd javacontroller/src && $(MAKE)

.PHONY: soundman
##!soundman	compile soundman (experimental)
soundman:
	cd soundman/src	&& javac -d ../class/ SoundMan.java SoundManipulation.java SoundManGUI.java


.PHONY: uninstall_ode
##!uninstall_ode  uninstall the customized ode library
uninstall_ode:
	@echo "*************** uninstall ode -double version********"
	cd opende && make uninstall


.PHONY: install_utils
install_utils:
	-mkdir -p $(PREFIX)bin $(PREFIX)/lib/soundMan $(PREFIX)share/lpzrobots
	-@if [ -d matrixviz/bin/matrixviz.app ]; then \
          cp matrixviz/bin/matrixviz.app/Contents/MacOS/matrixviz $(PREFIX)/bin/ && echo "===> copied matrixviz.app to $(PREFIX)/bin/"; \
         elif [ -e matrixviz/bin/matrixviz ]; then \
	   cp matrixviz/bin/matrixviz $(PREFIX)/bin/ && echo "===> copied matrixviz to $(PREFIX)/bin/"; \
	 fi
	-cd javacontroller/src && $(MAKE) PREFIX=$(PREFIX) install	
	-@if [ -d guilogger/bin/guilogger.app ]; then \
          cp guilogger/bin/guilogger.app/Contents/MacOS/guilogger $(PREFIX)/bin/ && echo "===> copied guilogger to $(PREFIX)/bin/"; \
         elif [ -e guilogger/src/bin/guilogger ]; then \
	   cp guilogger/src/bin/guilogger $(PREFIX)/bin/ && echo "===> copied guilogger to $(PREFIX)/bin/"; \
	 else cp guilogger/bin/guilogger $(PREFIX)/bin/  && echo "===> copied guilogger to $(PREFIX)/bin/"; \
	fi	
	-cp soundman/class/*.class $(PREFIX)/lib/soundMan/
	-cp soundman/bin/soundMan $(PREFIX)/bin/soundMan
	sed -i -e "s|PREFIX=.*|PREFIX=$(PREFIX)|" $(PREFIX)/bin/soundMan


.PHONY: install_libs
install_libs:
ifeq ($(TYPE),user)
	@echo "*************** Install selforg *********************"
	cd selforg/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) install 
#	     $(PREFIX)share/lpzrobots/ga_tools
	@echo "*************** Install ode_robots ******************"
	cd ode_robots/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) install 
	@echo "*************** Install ga_tools ******************"
	cp ga_tools/libga_tools.a $(PREFIX)lib
#	cp ga_tools/libga_tools_opt.a $(PREFIX)/lib
	cp -RL ga_tools/include/ga_tools $(PREFIX)/include/	
	@echo "*************** Install example simulations ******************"
	cp -RL ga_tools/simulations $(PREFIX)/share/lpzrobots/ga_tools/
	-chmod -R ugo+r $(PREFIX)/share/lpzrobots
	@echo "*************** Finished ******************"
	@echo "Make sure that the $(PREFIX)lib directory is in our lib search path"
	@echo " and $(PREFIX)include is searched for includes."
	@echo "You can find example simulations in $(PREFIX)share/lpzrobots/, but copy"
	@echo " them first to your home directory to work with them."
endif

.PHONY: uninstall_intern
uninstall_intern:
	@echo "*************** Uninstall selforg *********************"
	cd selforg/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) uninstall 
	@echo "*************** Uninstall ode_robots ******************"
	cd ode_robots/ && $(MAKE) TYPE=$(TYPE) PREFIX=$(PREFIX) uninstall 
	-rm -f $(PREFIX)/bin/guilogger
	-rm -f $(PREFIX)/bin/matrixviz
	-cd javacontroller/src && $(MAKE) PREFIX=$(PREFIX) uninstall
	-rm -f $(PREFIX)/lib/soundMan/SoundMan*.class
	-rm -f $(PREFIX)/bin/soundMan
ifeq ($(TYPE),user)
	-rm -f $(PREFIX)/lib/libga_tools.a 
# $(PREFIX)lib/libga_tools_opt.a
	-rm -rf $(PREFIX)/include/ga_tools
	-rm -rf $(PREFIX)/share/lpzrobots
endif



Makefile.conf:	
	@bash createMakefile.conf.sh


.PHONY: tags
##!tags		create TAGS file for emacs
tags: 
	rm TAGS
	cd selforg && $(MAKE) tags
	cd ode_robots && $(MAKE) tags
	cd ga_tools && $(MAKE) tags

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

.PHONY: todo
##!todo		show the marked todos in the sourcecode
todo:
	cd ode_robots && make todo
	cd selforg && make todo

usage:
# The logo was creates with >$ figlet "LPZRobots" > logo.txt 
#  plus some editing
	@cat logo.txt
	@echo $(USAGE)
