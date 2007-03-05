#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

PREFIX=$HOME

# subdirectories which contain libraries or binaries needed some of the other project in this tree 
all: 
	-make guilogger
	-make neuronviz
	-make soundman
	cd selforg && make depend
	cd ode_robots && make depend
	@if test ! -e opende/Makefile; then echo -e "You need to setup ODE from opende folder first!\nPlease run:\ncd opende; sh autogen.sh\n./configure --enable-opcode --enable-double-precision\nmake\nmake install #(as root)\n\nOn most SUSE linux computers it's necessary to run thereafter\n\nldconfig #(as root)\n\nfor a correct linking of the libode.so!\n"; exit; fi

.PHONY: guilogger
guilogger:
	cd guilogger && qmake guilogger.pro && make

.PHONY: neuronviz
neuronviz:
	cd neuronviz/src && make

.PHONY: neuronviz
soundman:
	cd ode_robots/utils && javac SoundMan.java SoundManipulation.java

.PHONY: install
install:
	-cd neuronviz/src && make install
	-@cp guilogger/bin/guilogger $(HOME)/bin/ && echo "copied guilogger to $(HOME)/bin/" || echo "Could not copy guilogger binary to $(HOME)/bin/! Please install it by hand."
	-cp ode_robots/utils/SoundMan.class ode_robots/utils/SoundManipulation.class $(HOME)/lib/
	-cp ode_robots/utils/soundMan $(HOME)/bin/

.PHONY: tags
tags: 
	etags `find selforg -type f -regex ".*\.[hc]p?p?"`
	etags -a `find ode_robots -type f -regex ".*\.[hc]p?p?"`

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

