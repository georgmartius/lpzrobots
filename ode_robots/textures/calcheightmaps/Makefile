#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = calcheightmaps
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = calcheightmaps

# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DNDEBUG

LPZROBOTS  = ../../..
ODEROBOTS  = $(LPZROBOTS)/ode_robots
CONTROLLER = $(LPZROBOTS)/controller
MATRIXLIB  = $(LPZROBOTS)/matrixlib
#MATRIX     = matrix
MATRIX     = matrix_debug

include $(ODEROBOTS)/Makefile.conf

LIBS   = -lm -L$(ODEROBOTS) -lode_robots\
	-lode -ldrawstuff -lGL -lGLU -lglut \
	-L$(MATRIXLIB)  -l$(MATRIX)

# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)

# used for binary compilation (linking)
CC = g++ $(CFLAGS)
#CC = g++ -g3 -pg  

#$(EXEC): libode_robots $(ODEROBOTS)/libode_robots.a Makefile Makefile.depend $(OFILES)
#	cd $(ODEROBOTS) && make libode_robots.a
#	$(CC) $(OFILES) $(LIBS) -o $(EXEC)
$(EXEC): Makefile Makefile.depend $(OFILES)
	$(CC) $(OFILES) $(LIBS) -o $(EXEC)

libode_robots:	
	cd $(ODEROBOTS) && make libode_robots.a		

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

include Makefile.depend
