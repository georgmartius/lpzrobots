#File:     Makefile for lpzrobots simulation
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start
EXEC_OPT = start_opt

LPZROBOTS  = ../../..
ECBROBOTS  = $(LPZROBOTS)/ecbrobots
SELFORG    = $(LPZROBOTS)/selforg

ECBROBOTSLIB := ecbrobots
ECBROBOTSLIB_OPT := ecbrobots_opt
LIBECBROBOTS = lib$(ECBROBOTSLIB).a
LIBECBROBOTS_OPT = lib$(ECBROBOTSLIB_OPT).a

SELFORGLIB = selforg
SELFORGLIB_OPT = selforg_opt
LIBSELFORG = lib$(SELFORGLIB).a

LIBS   = -lm \
	-L$(ECBROBOTS) -l$(ECBROBOTSLIB) \
	-L$(SELFORG)   -l$(SELFORGLIB) \
	-lGL -lGLU -lglut \
	-lreadline \
	-lpthread

INC   = -I. -I$(ECBROBOTS)/include/ecbrobots -I$(SELFORG)/include

CXX = g++
# -pg for profiling
CPPFLAGS = -Wall -I. -g $(INC)
## Optimisation
CPPFLAGS_OPT = -Wall -I. -O3 -DNDEBUG $(INC)

normal: libecbrobots $(EXEC)

$(EXEC): $(ECBROBOTS)/$(LIBECBROBOTS)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC)

opt: libecbrobots_opt $(EXEC_OPT)

$(EXEC_OPT): ECBROBOTSLIB = $(ECBROBOTSLIB_OPT)
$(EXEC_OPT): SELFORGLIB = $(SELFORGLIB_OPT)
$(EXEC_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(EXEC_OPT): $(ECBROBOTS)/$(LIBECBROBOTS_OPT)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_OPT)

libecbrobots:
	cd $(ECBROBOTS) && make

libecbrobots_opt:
	cd $(ECBROBOTS) && make opt

Makefile.depend:
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

depend:
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags:
	etags `find -name "*.[ch]"`

cleanall: clean
	cd $(ECBROBOTS) && make clean
	cd $(SELFORG) && make clean

clean:
	rm -f $(EXEC) $(EXEC_OPT) *.o Makefile.depend

todo:
	grep -ni "Todo" $(CFILES)


include Makefile.depend
