#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main_nchannelmatrixcontroller_test
CFILES = $(addsuffix .cc, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DUNITTEST -DNDEBUG

LIBHOME=/home/fhesse/projects/robots
LIBHOME2=/home/fhesse/c++/

LIBS   = -lm -L$(LIBHOME)/matrixlib/  -lmatrix_debug -lode -ldrawstuff -lGL -lGLU -lglut
INC   = -I$(LIBHOME)/matrixlib/ -I$(LIBHOME)/controller -I$(LIBHOME2)/gnuplot/

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

