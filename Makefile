#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

PREFIX=$HOME

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
	-mkdir -p $(HOME)/.lpzrobots
	-mkdir -p $(HOME)/bin $(HOME)/lib/soundMan
	-cd neuronviz/src && make install
	-cd javacontroller/src && make install
	-@cp guilogger/bin/guilogger $(HOME)/bin/ && echo "copied guilogger to $(HOME)/bin/" || echo "Could not copy guilogger binary to $(HOME)/bin/! Please install it by hand."
	-cp ode_robots/utils/Sound*.class $(HOME)/lib/
	-cp ode_robots/utils/*.class $(HOME)/lib/soundMan/
	-cp ode_robots/utils/soundMan $(HOME)/bin/
	-cp ode_robots/utils/feedfile.pl $(HOME)/bin/
	-cp ode_robots/utils/encodevideo.sh $(HOME)/bin/
	-cp ode_robots/utils/selectcolumns.pl $(HOME)/bin/

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

