#File:     Makefile for lpzrobots simulation
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main walkcontroller
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start
EXEC_OPT = start_opt

LPZROBOTS  = ../../..
ODEROBOTS  = $(LPZROBOTS)/ode_robots
SELFORG    = $(LPZROBOTS)/selforg

ODEROBOTSLIB := ode_robots
ODEROBOTSLIB_OPT := ode_robots_opt
LIBODEROBOTS = lib$(ODEROBOTSLIB).a
LIBODEROBOTS_OPT = lib$(ODEROBOTSLIB_OPT).a

SELFORGLIB = selforg
SELFORGLIB_OPT = selforg_opt
LIBSELFORG = lib$(SELFORGLIB).a

LIBS   = -lm \
	-L$(ODEROBOTS) -l$(ODEROBOTSLIB) \
	-L$(SELFORG)   -l$(SELFORGLIB) \
	-lode -losgProducer -lProducer -losgText -losgUtil -losg -lGL -lGLU -lglut \
	-lreadline

INC   = -I. -I$(ODEROBOTS)/include/ode_robots -I$(SELFORG)/include

CXX = g++
# -pg for profiling
CPPFLAGS = -Wall -I. -g $(INC)
## Optimisation
CPPFLAGS_OPT = -Wall -I. -O3 -DNDEBUG $(INC)

normal: libode_robots $(EXEC)

$(EXEC): $(ODEROBOTS)/$(LIBODEROBOTS)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC)

opt: libode_robots_opt $(EXEC_OPT)

$(EXEC_OPT): ODEROBOTSLIB = $(ODEROBOTSLIB_OPT)
$(EXEC_OPT): SELFORGLIB = $(SELFORGLIB_OPT)
$(EXEC_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(EXEC_OPT): $(ODEROBOTS)/$(LIBODEROBOTS_OPT)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_OPT)

libode_robots:	
	cd $(ODEROBOTS) && make

libode_robots_opt:	
	cd $(ODEROBOTS) && make opt

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

cleanall: clean
	cd $(ODEROBOTS) && make clean
	cd $(SELFORG) && make clean

clean:
	rm -f $(EXEC) $(EXEC_OPT) *.o Makefile.depend

include Makefile.depend
