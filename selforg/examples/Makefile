#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = controllertest
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = controllertest

# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DUNITTEST -DNDEBUG

LIBS   = -lm -L../../matrixlib/ -lmatrix_debug -L../ -lselforg 
INC   = -I../../matrixlib/ -I../include

cc = g++ $(CFLAGS) $(INC) -c
#cc = g++ $(CFLAGS) -c -O -g3 -pg
CXX = g++ $(CFLAGS) $(INC)  -c

CC = g++ $(CFLAGS) 
#CC = g++ -g3 -pg  

$(EXEC): Makefile.depend $(OFILES)
	$(CC) $(OFILES) $(LIBS)  -o $(EXEC)

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

search:
	find -name "*.[ch]*" -exec grep -Hn "something" {} \;

todo:
	find -name "*.[ch]*" -exec grep -Hni "TODO" {} \;

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

include Makefile.depend