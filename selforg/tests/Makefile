#File:     Makefile for Matrix libary test
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005
#

TESTS = configurabletest

TEST_DEBUG_CFLAGS = -Wall -I. -I../include -DUNITTEST -g

LIBS   = -lm $(shell gsl-config --libs) -lselforg -L../ -lpthread

CXX = g++ $(shell gsl-config --cflags)
AR = ar

.PHONY: all
#all: unittests_debug unittests unittests_sse sparsematrix_debug
all: 
	for T in $(TESTS); do $(MAKE) TEST=$$T $$T; done
	$(MAKE) run

run:
	for T in $(TESTS); do ./$$T; done


$(TEST): $(TEST).cpp ../libselforg.a
	$(CXX) $(TEST_DEBUG_CFLAGS) $(TEST).cpp $(LIBS) -o $(TEST)


# configurabletest: configurabletest.cpp ../libselforg.a
# 	$(CXX) $(TEST_DEBUG_CFLAGS) configurabletest.cpp $(LIBS) -o configurabletest
# 	./configurabletest


.PHONY: tags
tags: 
	etags `find -name "*.[ch]*"` 

.PHONY: clean
clean:
	rm -f *.o unittests_debug unittests example
#	rm -f *.o unittests_debug unittests libmatrix_debug.a libmatrix.a example
