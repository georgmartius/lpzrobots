#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

# subdirectories which contain libraries or binaries needed some of the other project in this tree 
all: matrixlib guilogger

.PHONY: matrixlib
matrixlib:
	cd matrixlib && make -k

.PHONY: guilogger
guilogger:
	cd guilogger && qmake guilogger.pro && make

tags: 
	etags `find -name "*.[ch]*"` 

clean:
	cd matrixlib && make clean
	cd guilogger && make clean

