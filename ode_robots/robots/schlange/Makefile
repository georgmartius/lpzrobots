#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005


FILES = schlange
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -I. -g -I/usr/local/include
## Optimisation
#CFLAGS = -Wall -I. -O3 -DNDEBUG

LIBHOME=../..
ODEHOME=$(LIBHOME)/ode_robots
MATRIX=matrix
#MATRIX=matrix_debug
SIMPATHS = . obstacles robots agents


LIBS   = -lm -L$(LIBHOME)/matrixlib/  -l$(MATRIX) -lode -ldrawstuff -lGL -lGLU -lglut
INC   = -I$(LIBHOME)/matrixlib/ -I$(LIBHOME)/controller -I$(LIBHOME)/gnuplot \
	$(addprefix -I$(ODEHOME)/, $(SIMPATHS)) 

cc = g++ $(CFLAGS) $(INC) -c
#cc = g++ $(CFLAGS) -c -O -g3 -pg
# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)

# used for binary compilation (linking)
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
