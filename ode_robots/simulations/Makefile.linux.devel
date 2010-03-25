#File:     Makefile for lpzrobots simulation
#Author:   Georg Martius  <georg.martius at web.de>
#Date:     Sept 2008

#Do not edit, this is copied to the simulations while installation! 
#Make your changes in Makefile.conf

include Makefile.conf

CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start
EXEC_DBG = start_dbg
EXEC_OPT = start_opt
EXEC_PROF = start_prof

ODEROBOTSLIB := ode_robots
ODEROBOTSLIB_DBG := ode_robots_dbg
ODEROBOTSLIB_OPT := ode_robots_opt
ODEROBOTSLIB_PROF := ode_robots_prof
LIBODEROBOTS = lib$(ODEROBOTSLIB).a
LIBODEROBOTS_DBG = lib$(ODEROBOTSLIB_DBG).a
LIBODEROBOTS_OPT = lib$(ODEROBOTSLIB_OPT).a
LIBODEROBOTS_PROF = lib$(ODEROBOTSLIB_PROF).a

SELFORGLIB = selforg
SELFORGLIB_DBG = selforg_dbg
SELFORGLIB_OPT = selforg_opt
LIBSELFORG = lib$(SELFORGLIB).a
ODELIBS = $(shell ode-dbl-config --libs)

LIBS   = -lm \
	-L$(ODEROBOTS) -l$(ODEROBOTSLIB) \
	-L$(SELFORG)   -l$(SELFORGLIB) \
	 $(ODELIBS) \
	-losgShadow -losgText -losgUtil -losgViewer -losgGA -lOpenThreads -losg -lGL -lGLU -lglut \
	-lreadline -lncurses -lpthread $(ADDITIONAL_LIBS)

INC   = -I. -I$(ODEROBOTS)/include -I$(SELFORG)/include

CXX = g++
ODEFLAGS = $(shell ode-dbl-config --cflags)
CPPBASEFLAGS = $(ODEFLAGS) -pthread $(INC) -Wno-deprecated -Wall
# -pg for profiling
CPPFLAGS = $(CPPBASEFLAGS) -g -O1
## Optimisation
CPPFLAGS_DBG = $(CPPBASEFLAGS) -g
## Optimisation
CPPFLAGS_OPT = $(CPPBASEFLAGS) -O3 -DNDEBUG

normal: libode_robots $(EXEC)

$(EXEC): $(ODEROBOTS)/$(LIBODEROBOTS)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC)

opt: libode_robots_opt $(EXEC_OPT)

$(EXEC_OPT): ODEROBOTSLIB = $(ODEROBOTSLIB_OPT)
$(EXEC_OPT): SELFORGLIB = $(SELFORGLIB_OPT)
$(EXEC_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(EXEC_OPT): $(ODEROBOTS)/$(LIBODEROBOTS_OPT)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_OPT)


dbg: libode_robots_dbg $(EXEC_DBG)

$(EXEC_DBG): ODEROBOTSLIB = $(ODEROBOTSLIB_DBG)
$(EXEC_DBG): SELFORGLIB = $(SELFORGLIB_DBG)
$(EXEC_DBG): CPPFLAGS = $(CPPFLAGS_DBG)
$(EXEC_DBG): $(ODEROBOTS)/$(LIBODEROBOTS_DBG)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_DBG)

prof: libode_robots_prof $(EXEC_PROF)

$(EXEC_PROF): ODEROBOTSLIB = $(ODEROBOTSLIB_PROF)
$(EXEC_PROF): $(ODEROBOTS)/$(LIBODEROBOTS_PROF)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_PROF)

libode_robots:	
	+cd $(ODEROBOTS) && make

libode_robots_dbg:	
	+cd $(ODEROBOTS) && make dbg

libode_robots_opt:	
	+cd $(ODEROBOTS) && make opt

libode_robots_prof:	
	+cd $(ODEROBOTS) && make prof

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

cleanall: clean
	cd $(ODEROBOTS) && make clean-all
	cd $(SELFORG) && make clean-all

clean:
	rm -f $(EXEC) $(EXEC_DBG) $(EXEC_OPT) *.o Makefile.depend

-include Makefile.depend
