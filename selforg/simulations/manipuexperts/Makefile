#File:     Makefile for integration example
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2006

# add files to compile here
#  multiexpertpair
FILES = main cmdline console  multiexpertsubopt
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DUNITTEST -DNDEBUG 

SELFORG = $(HOME)/src/lpzrobots/selforg
LIBS   = -lm -L$(SELFORG) -lselforg -lreadline

INC   = -I$(SELFORG)/include

cc = g++ $(CFLAGS) $(INC)
#cc = g++ $(CFLAGS) -c -O -g3 -pg
CXX = g++ $(CFLAGS) $(INC)

CC = g++ $(CFLAGS) 
#CC = g++ -g3 -pg  

$(EXEC): Makefile.depend $(OFILES) libselforg
	$(CC) $(OFILES) $(LIBS)  -o $(EXEC)

libselforg: 
	cd ../../ && make


depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

todo:
	find -name "*.[ch]*" -exec grep -Hni "TODO" {} \;

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

include Makefile.depend
