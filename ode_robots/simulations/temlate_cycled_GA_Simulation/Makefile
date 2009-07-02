#File:     Makefile for lpzrobots simulation
#Author:   Georg Martius  <georg.martius at web.de>
#Date:     Sept 2008

include Makefile.conf

CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start
EXEC_OPT = start_opt
EXEC_PROF = start_prof

ODEROBOTSLIB := ode_robots
ODEROBOTSLIB_OPT := ode_robots_opt
ODEROBOTSLIB_PROF := ode_robots_prof
LIBODEROBOTS = lib$(ODEROBOTSLIB).a
LIBODEROBOTS_OPT = lib$(ODEROBOTSLIB_OPT).a
LIBODEROBOTS_PROF = lib$(ODEROBOTSLIB_PROF).a

SELFORGLIB = selforg
SELFORGLIB_OPT = selforg_opt
LIBSELFORG = lib$(SELFORGLIB).a

GA_TOOLSLIB = ga_tools
GA_TOOLSLIB_OPT = ga_tools_opt
LIBGA_TOOLS = lib$(GA_TOOLSLIB).a

LIBS   = -lm \
	-L$(ODEROBOTS) -l$(ODEROBOTSLIB) \
	-L$(SELFORG)   -l$(SELFORGLIB) \
	-L$(GA_TOOLS)  -l$(GA_TOOLSLIB) \
	-lode -losgShadow -losgText -losgUtil -losgViewer -losgGA -lOpenThreads -losg -lGL -lGLU -lglut \
	-lreadline -lpthread

INC   = -I. -I$(ODEROBOTS)/include -I$(SELFORG)/include -I$(GA_TOOLS)/include

CXX = g++
# -pg for profiling
CPPFLAGS = -Wall  -g -pthread $(INC)
## Optimisation
CPPFLAGS_OPT = -Wall -pthread  -O3 -DNDEBUG $(INC)

normal: libode_robots $(EXEC)

$(EXEC): $(ODEROBOTS)/$(LIBODEROBOTS)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC)

opt: libode_robots_opt $(EXEC_OPT)

$(EXEC_OPT): ODEROBOTSLIB = $(ODEROBOTSLIB_OPT)
$(EXEC_OPT): SELFORGLIB = $(SELFORGLIB_OPT)
$(EXEC_OPT): GA_TOOLSLIB = $(GA_TOOLSLIB_OPT)
$(EXEC_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(EXEC_OPT): $(ODEROBOTS)/$(LIBODEROBOTS_OPT)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_OPT)

prof: libode_robots_prof $(EXEC_PROF)

$(EXEC_PROF): ODEROBOTSLIB = $(ODEROBOTSLIB_PROF)
$(EXEC_PROF): $(ODEROBOTS)/$(LIBODEROBOTS_PROF)  Makefile Makefile.depend $(OFILES)
	$(CXX) $(CPPFLAGS) $(OFILES) $(LIBS) -o $(EXEC_PROF)

libode_robots:	
	+cd $(ODEROBOTS) && make

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
	cd $(ODEROBOTS) && make clean
	cd $(SELFORG) && make clean
	cd $(GA_TOOLS) && make clean

clean:
	rm -f $(EXEC) $(EXEC_OPT) *.o Makefile.depend

-include Makefile.depend
