#File:     Makefile for Matrix libary test
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005
#
# $Log$
# Revision 1.8  2010-06-03 09:51:44  martius
# added gsl and eigenvector/values stuff
#
# Revision 1.7  2009/08/03 08:40:02  guettler
# added support for sparsematrix, but still disabled in target all
#
#

# us -pg for profiling
TEST_DEBUG_CFLAGS = -Wall -I. -I../include/selforg -DUNITTEST -g
TEST_OPTIM_CFLAGS = -Wall -I. -I../include/selforg -O3 -DUNITTEST -DNDEBUG
TEST_OPTIMSSE_CFLAGS = -Wall -I. -I../include/selforg -O3 -DUNITTEST -DNDEBUG -ftree-vectorize -msse2
#LIB_DEBUG_CFLAGS = -Wall -I. 
#LIB_OPTIM_CFLAGS = -Wall -I. -O3 -DNDEBUG 
#EXAMPLE_CFLAGS   = -Wall -I. 

LIBS   = -lm $(shell gsl-config --libs)

CXX = g++ $(shell gsl-config --cflags)
AR = ar

# ATMEL STUFF
#AVRCXX     = avr-gcc
#ATmega     = 32
#MCU_TARGET = -mmcu=atmega${ATmega}
#AVRUTILS   = ../avr/utils

.PHONY: all
#all: unittests_debug unittests unittests_sse sparsematrix_debug
all: unittests_debug unittests unittests_sse
#    libmatrix_avr_debug.a libmatrix_avr.a

unittests_debug: matrix.h matrix.cpp matrixutils.h matrixutils.cpp matrix.tests.hpp Makefile
	$(CXX) $(LIBS) $(TEST_DEBUG_CFLAGS) matrix.cpp matrixutils.cpp -o unittests_debug

unittests: matrix.h matrix.cpp matrixutils.h matrixutils.cpp matrix.tests.hpp Makefile
	$(CXX) $(LIBS) $(TEST_OPTIM_CFLAGS) matrix.cpp matrixutils.cpp -o unittests

unittests_sse: matrix.h matrix.cpp matrixutils.h matrixutils.cpp  matrix.tests.hpp Makefile
	$(CXX) $(LIBS) $(TEST_OPTIMSSE_CFLAGS) matrix.cpp matrixutils.cpp -o unittests_sse

sparsematrix_debug: sparsematrix.h sparsearray.h sparsematrix.tests.hpp Makefile
	$(CXX) $(LIBS) $(TEST_DEBUG_CFLAGS) sparsematrix.h -o sparsematrix_test_debug

test:
	g++ -lm -Wall -I. -I../include/selforg -O1 -DNDEBUG -ftree-vectorize -msse2 -ftree-vectorizer-verbose=5 -funsafe-math-optimizations -c sse_test.cpp

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
