#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main ../simulation
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -I. -g -I/usr/local/include
## Optimisation
#CFLAGS = -Wall -I. -O3 -DNDEBUG

LIBHOME=../../
ODEHOME=../
MATRIX=matrix
#MATRIX=matrix_debug

LIBS   = -lm -L$(LIBHOME)/matrixlib/  -l$(MATRIX) -lode -ldrawstuff -lGL -lGLU -lglut
INC   = -I$(LIBHOME)/matrixlib/ -I$(LIBHOME)/controller -I$(LIBHOME)/gnuplot -I$(LIBHOME)/ode_robots -I$(ODEHOME)/obstacles -I$(ODEHOME)/robots 

cc = g++ $(CFLAGS) $(INC) -c
#cc = g++ $(CFLAGS) -c -O -g3 -pg
CXX = g++ $(CFLAGS) $(INC)  -c

CC = g++ $(CFLAGS) 
#CC = g++ -g3 -pg  

$(EXEC): Makefile.depend $(OFILES)
	$(CC) $(OFILES) $(LIBS) $(INC) -o $(EXEC)

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

include Makefile.depend
