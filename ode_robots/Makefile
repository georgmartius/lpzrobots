#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main
CFILES = $(addsuffix .cc, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -g
## Optimisation
#CFLAGS = -Wall -O3 -DNDEBUG

LPZROBOTS=../
LIBHOME2=/home/fhesse/c++/

LIBS   = -lm -L$(LPZROBOTS)/matrixlib/  -lmatrix_debug -lode -ldrawstuff -lGL -lGLU -lglut
INC   = -I. -I$(LPZROBOTS)/matrixlib/ -I$(LPZROBOTS)/controller -I$(LPZROBOTS)/gnuplot/

cc = g++ $(CFLAGS) $(INC) -c
#cc = g++ $(CFLAGS) -c -O -g3 -pg
CXX = g++ $(CFLAGS) $(INC)  -c

CC = g++ $(CFLAGS) 
#CC = g++ -g3 -pg  

$(EXEC): Makefile.depend $(OFILES)
	$(CC) $(OFILES) $(LIBS) $(INC) -o $(EXEC)

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

Makefile.depend:
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend


include Makefile.depend