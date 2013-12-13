#File:     Makefile for Matrix libary test
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005
#
# $Log$
# Revision 1.9  2010-06-08 15:46:01  martius
# makefiles adapted
#
# Revision 1.8  2010/06/03 09:51:44  martius
# added gsl and eigenvector/values stuff
#
# Revision 1.7  2009/08/03 08:40:02  guettler
# added support for sparsematrix, but still disabled in target all
#
#

# us -pg for profiling
BASECFLAGS=-std=c++11 -Wall -I. -I../include/selforg $(shell gsl-config --cflags)
TEST_DEBUG_CFLAGS = $(BASECFLAGS) -DUNITTEST -g -O0
TEST_OPTIM_CFLAGS = $(BASECFLAGS) -O -DUNITTEST -DNDEBUG -mtune=native
TEST_OPTIMSSE_CFLAGS = $(BASECFLAGS) -O3 -DUNITTEST -DNDEBUG -ftree-vectorize -msse2 -mtune=native

LIBS   = -lm $(shell gsl-config --libs)

CXX = clang++
#CXX = g++
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
	$(CXX) $(TEST_DEBUG_CFLAGS) matrix.cpp matrixutils.cpp  $(LIBS) -o unittests_debug

unittests: matrix.h matrix.cpp matrixutils.h matrixutils.cpp matrix.tests.hpp Makefile
	$(CXX) $(TEST_OPTIM_CFLAGS) matrix.cpp matrixutils.cpp $(LIBS) -o unittests

unittests_sse: matrix.h matrix.cpp matrixutils.h matrixutils.cpp  matrix.tests.hpp Makefile
	$(CXX) $(TEST_OPTIMSSE_CFLAGS) matrix.cpp matrixutils.cpp $(LIBS) -o unittests_sse

sparsematrix_debug: sparsematrix.h sparsearray.h sparsematrix.tests.hpp Makefile
	$(CXX) $(TEST_DEBUG_CFLAGS) sparsematrix.h $(LIBS) -o sparsematrix_test_debug

test:
	$(CXX) $(BASECFLAGS) -O1 -DNDEBUG -ftree-vectorize -msse2 -ftree-vectorizer-verbose=5 -funsafe-math-optimizations -c sse_test.cpp

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
	rm -f *.o unittests_debug unittests unittests_sse example
#	rm -f *.o unittests_debug unittests libmatrix_debug.a libmatrix.a example
