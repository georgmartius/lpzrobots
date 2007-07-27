#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main serial_unix
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = commtest

# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DNDEBUG

LIBS   = -lm -pthread 

# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)

# used for binary compilation (linking)
CC = g++ $(CFLAGS)
#CC = g++ -g3 -pg  

all:  $(EXEC)



$(EXEC): Makefile Makefile.depend $(OFILES)
	$(CC) $(OFILES) $(LIBS) -o $(EXEC)

depend: Makefile.depend

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

include Makefile.depend
