#File:     Makefile for Matrix libary test
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

# us -pg for profiling
TEST_DEBUG_CFLAGS = -Wall -I. -I../include/selforg -DUNITTEST -O1 -g
TEST_OPTIM_CFLAGS = -Wall -I. -I../include/selforg -O3 -DUNITTEST -DNDEBUG
#LIB_DEBUG_CFLAGS = -Wall -I. 
#LIB_OPTIM_CFLAGS = -Wall -I. -O3 -DNDEBUG 
#EXAMPLE_CFLAGS   = -Wall -I. 

LIBS   = -lm

CXX = g++
AR = ar

# ATMEL STUFF
#AVRCXX     = avr-gcc
#ATmega     = 32
#MCU_TARGET = -mmcu=atmega${ATmega}
#AVRUTILS   = ../avr/utils

.PHONY: all
all: unittests_debug unittests
#    libmatrix_avr_debug.a libmatrix_avr.a

unittests_debug: matrix.h matrix.cpp matrix.tests.hpp Makefile
	$(CXX) $(LIBS) $(TEST_DEBUG_CFLAGS) matrix.cpp -o unittests_debug

unittests: matrix.h matrix.cpp  matrix.tests.hpp Makefile
	$(CXX) $(LIBS) $(TEST_OPTIM_CFLAGS) matrix.cpp -o unittests

#libmatrix_debug.a: matrix.h matrix.cpp
#	$(CXX) $(LIB_DEBUG_CFLAGS) -c matrix.cpp -o libmatrix_debug.a	

#libmatrix.a: matrix.h matrix.cpp
#	$(CXX) $(LIB_OPTIM_CFLAGS) -c matrix.cpp -o libmatrix.a	

# example: example.cpp libmatrix_debug.a matrix.h matrix.cpp
# 	$(CXX) $(LIBS) $(EXAMPLE_CFLAGS) -L. -lmatrix_debug example.cpp -o example


#libmatrix_avr_debug.a: matrix.h matrix.cpp
#	$(AVRCXX) $(LIB_DEBUG_CFLAGS) $(MCU_TARGET) -I$(AVRUTILS) -c matrix.cpp -o  libmatrix_avr_debug.a	
#
#
#libmatrix_avr.a: matrix.h matrix.cpp
#	 $(AVRCXX) $(LIB_OPTIM_CFLAGS) $(MCU_TARGET) -I$(AVRUTILS) -c matrix.cpp -o libmatrix_avr.a	
#

.PHONY: tags
tags: 
	etags `find -name "*.[ch]*"` 

.PHONY: clean
clean:
	rm -f *.o unittests_debug unittests example
#	rm -f *.o unittests_debug unittests libmatrix_debug.a libmatrix.a example
