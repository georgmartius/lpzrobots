# File:       Makefile for lpzrobots simulations
# Author:     Georg Martius  <georg.martius at web.de>
# Date:       Oct 2009
# modified:   Frank Güttler <guettler@informatik.uni-leipzig.de>
# Date:       2011


EXEC = start
# add files to compile in the conf file
include Makefile.conf

EXEC_OPT  = $(EXEC)_opt
EXEC_DBG  = $(EXEC)_dbg

CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

CONFIGURATORLIB := configurator
CONFIGURATORLIB_DBG := configurator
CONFIGURATORLIB_OPT := configurator
LIBCONFIGURATOR = $(CONFIGURATOR)/lib$(CONFIGURATORLIB).so

SELFORGLIB = selforg
SELFORGLIB_DBG = selforg_dbg
SELFORGLIB_OPT = selforg_opt
LIBSELFORG      = $(SELFORG)/lib$(SELFORGLIB).a

GSLLIBS = $(shell gsl-config --libs)

LIBS  += -lm \
	-L$(CONFIGURATOR) -l$(CONFIGURATORLIB) \
	-L$(SELFORG)   -l$(SELFORGLIB) \
	 $(GSLLIBS) \
  -lQtCore -lQtGui -lQtXml \
	-lreadline -lncurses -lpthread $(ADDITIONAL_LIBS)

INC   += -I. -I$(CONFIGURATOR)/include -I$(SELFORG)/include 

CXX = g++
ODEFLAGS = $(shell ode-dbl-config --cflags)
CPPBASEFLAGS = $(ODEFLAGS) -pthread $(INC) -Wno-deprecated -Wall
# -pg for profiling
CPPFLAGS = $(CPPBASEFLAGS) -g -O1
## Optimisation
CPPFLAGS_DBG = $(CPPBASEFLAGS) -g
## Optimisation
CPPFLAGS_OPT = $(CPPBASEFLAGS) -O3 -DNDEBUG

normal: libode_robots 
	$(MAKE) $(EXEC)
opt:    libode_robots_opt 
	$(MAKE) ODEROBOTSLIB="$(ODEROBOTSLIB_OPT)" SELFORGLIB="$(SELFORGLIB_OPT)" CPPFLAGS="$(CPPFLAGS_OPT)" $(EXEC_OPT)
dbg:    libode_robots_dbg 
	$(MAKE) ODEROBOTSLIB="$(ODEROBOTSLIB_DBG)" SELFORGLIB="$(SELFORGLIB_DBG)" CPPFLAGS="$(CPPFLAGS_DBG)" $(EXEC_DBG)

$(EXEC): Makefile Makefile.depend $(OFILES) $(LIBODEROBOTS) $(LIBSELFORG)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC)

$(EXEC_OPT): Makefile Makefile.depend $(OFILES) $(LIBODEROBOTS) $(LIBSELFORG)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_OPT)

$(EXEC_DBG): Makefile Makefile.depend $(OFILES) $(LIBODEROBOTS) $(LIBSELFORG)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_DBG)


libode_robots:	
	cd $(ODEROBOTS) && $(MAKE) lib

libode_robots_dbg:	
	cd $(ODEROBOTS) && $(MAKE) dbg

libode_robots_opt:	
	cd $(ODEROBOTS) && $(MAKE) opt



Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags $(find -name "*.[ch]")

cleandist: clean-all
clean-all: clean
	cd $(ODEROBOTS) && make clean-all
	cd $(SELFORG) && make clean-all


clean:
	rm -f $(EXEC) $(EXEC_DBG) $(EXEC_OPT) *.o Makefile.depend

-include Makefile.depend
