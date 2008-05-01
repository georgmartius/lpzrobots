#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>

include Makefile.conf

# subdirectories which contain libraries or binaries needed some of the other project in this tree 
all: 
	-make guilogger
	-make neuronviz
	-make soundman
	-make javacontroller
	cd selforg && make depend
	cd ode_robots && make depend
	-make tags
	@if test ! -e opende/Makefile && ld -lode -L$HOME/lib 2>/dev/null; then echo -e "You need to setup ODE from first!\nYou have 2 options: use a precompiled one from our webpage or compile the one in opende\nFor compiling please run:\ncd opende; sh autogen.sh\n./configure --enable-release --enable-double-precision\nmake\nsudo make install\n\nOn most SUSE linux computers it's necessary to run thereafter\n\nsudo ldconfig\n\nfor a correct linking of the libode.so!\n"; exit; fi
	@echo "Also consider to use make -j 2 or more if you have a multicore machine"

.PHONY: guilogger
guilogger:
	cd guilogger && qmake guilogger.pro && qmake src/src.pro && make

.PHONY: neuronviz
neuronviz:
	cd neuronviz/src && make

.PHONY: javacontroller
javacontroller:
	cd javacontroller/src && make

.PHONY: neuronviz
soundman:
	cd ode_robots/utils && javac SoundMan.java SoundManipulation.java SoundManGUI.java

.PHONY: install
install:
	-mkdir -p $(PREFIX)bin $(PREFIX)lib/soundMan
	-cd neuronviz/src && make install
	-cd javacontroller/src && make install
	-@cp guilogger/bin/guilogger $(PREFIX)bin/ && echo "copied guilogger to $(PREFIX)/bin/" || echo "Could not copy guilogger binary to $(PREFIX)bin/! Please install it by hand."
	-cp ode_robots/utils/*.class $(PREFIX)lib/soundMan/
	-cp ode_robots/utils/soundMan $(PREFIX)bin/soundMan
	sed -i -e "s|PREFIX=.*|PREFIX=$(PREFIX)|" $(PREFIX)bin/soundMan
	-cp ode_robots/utils/feedfile.pl $(PREFIX)bin/
	-cp ode_robots/utils/encodevideo.sh $(PREFIX)bin/
	-cp ode_robots/utils/selectcolumns.pl $(PREFIX)bin/
ifeq ($(INSTALL_ONLY_UTILS),no)
	make install_libs
endif



.PHONY: install_libs
install_libs:
	@echo "*************** Compile selforg (debug) *********************"
	cd selforg && make clean && make lib	
	cp selforg/libselforg.a $(PREFIX)lib
	@echo "*************** Compile selforg (optimized) *****************"
	cd selforg && make clean && make opt	
	cp selforg/libselforg_opt.a $(PREFIX)lib
	@echo "*************** Install selforg headers *********************"
	-mkdir -p $(PREFIX)include
	cp -rL selforg/include/selforg $(PREFIX)include/
	@echo "*************** Compile ode_robots (debug) ******************"
	cd ode_robots && make clean && make lib	
	cp ode_robots/libode_robots.a $(PREFIX)lib
	@echo "*************** Compile ode_robots (optimized) **************"
	cd ode_robots && make clean && make opt	
	cp ode_robots/libode_robots_opt.a $(PREFIX)lib
	@echo "*************** Install ode_robots headers ******************"
	cp -rL ode_robots/include/ode_robots $(PREFIX)include/	
	@echo "Be aware that you have to use the Makefile.systemwide in the simulations"
	@echo " if you install the simulator libaries and header files into the global paths" 
.PHONY: uninstall
uninstall:
	-cd neuronviz/src && make uninstall
	-cd javacontroller/src && make uninstall
	-rm -f $(PREFIX)bin/guilogger
	-rm -f $(PREFIX)lib/soundMan/SoundMan*.class
	-rm -f $(PREFIX)bin/soundMan
	-rm -f $(PREFIX)bin/feedfile.pl
	-rm -f $(PREFIX)bin/encodevideo.sh 
	-rm -f $(PREFIX)bin/selectcolumns.pl
ifeq ($(INSTALL_ONLY_UTILS),no)
	-rm -f $(PREFIX)lib/libselforg.a $(PREFIX)lib/libselforg_opt.a
	-rm -rf $(PREFIX)include/selforg
	-rm -f $(PREFIX)lib/libode_robots.a $(PREFIX)lib/libode_robots_opt.a
	-rm -rf $(PREFIX)include/ode_robots
endif



.PHONY: tags
tags: 
	etags `find selforg -type f -regex ".*\.[h]p?p?"`
	etags -a `find ode_robots -type f -regex ".*\.[h]p?p?"`

.PHONY: doc
doc:
	doxygen Doxyfile	

.PHONY: docintern
docintern:
	date "+PROJECT_NUMBER=%2d-%B-%Y" > builddate
	cat builddate Doxyfile.intern | nice -19 doxygen -	
	find doc/html -type f | xargs chmod ug+rw
	find doc/html -type f | xargs chmod o+r

.PHONY: docwarn
docwarn:
	cat doxygen.warn

todo:
	cd ode_robots && make todo
	cd selforg && make todo

clean:
	cd guilogger && make clean
	cd ode_robots && make clean
	cd ode_robots/simulations && make clean
	cd selforg && make clean

