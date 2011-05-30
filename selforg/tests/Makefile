#File:     Makefile for Matrix libary test
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005
#

TESTS = configurabletest

TEST_DEBUG_CFLAGS = -Wall -I. -I../include -DUNITTEST -g

LIBS   = -lm $(shell gsl-config --libs) -lselforg -L../ -lpthread

CXX = g++ $(shell gsl-config --cflags)
AR = ar

# ATMEL STUFF
#AVRCXX     = avr-gcc
#ATmega     = 32
#MCU_TARGET = -mmcu=atmega${ATmega}
#AVRUTILS   = ../avr/utils

.PHONY: all
#all: unittests_debug unittests unittests_sse sparsematrix_debug
all: $(TESTS)
#    libmatrix_avr_debug.a libmatrix_avr.a

configurabletest: configurabletest.cpp ../libselforg.a
	$(CXX) $(TEST_DEBUG_CFLAGS) configurabletest.cpp $(LIBS) -o configurabletest
	./configurabletest


.PHONY: tags
tags: 
	etags `find -name "*.[ch]*"` 

.PHONY: clean
clean:
	rm -f *.o unittests_debug unittests example
#	rm -f *.o unittests_debug unittests libmatrix_debug.a libmatrix.a example
