#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

PREFIX=$HOME

# subdirectories which contain libraries or binaries needed some of the other project in this tree 
all: matrixlib guilogger neuronviz
	cd selforg && make depend
	cd ode_robots && make depend

.PHONY: matrixlib
matrixlib:
	cd matrixlib && make -k

.PHONY: guilogger
guilogger:
#	cd guilogger && qmake guilogger.pro && make

.PHONY: neuronviz
neuronviz:
#	cd neuronviz/src && make

install:
#	@cp guilogger/bin/guilogger $(HOME)/bin/ && echo "copied guilogger to $(HOME)/bin/" || echo "Could not copy guilogger binary to $(HOME)/bin/! Please install it by hand."
#	cd neuronviz/src && make install
	
tags: 
	etags `find -name "*.[ch]*"` 

.PHONY: doc
doc:
	doxygen Doxyfile	

clean:
	cd matrixlib && make clean
	cd guilogger && make clean

