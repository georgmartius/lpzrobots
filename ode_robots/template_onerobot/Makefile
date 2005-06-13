#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -I. -g
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

$(EXEC): $(OFILES)
	$(CC) $(OFILES) $(LIBS) $(INC) -o $(EXEC)

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

# DO NOT DELETE
