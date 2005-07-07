#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

LPZROBOTS=..
ODEROBOTS=.

include Makefile.conf

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))

LIB=libode_robots.a

# -pg for profiling
# CFLAGS = -Wall -I. -g
## Optimisation #DNDEBUG
CFLAGS = -Wall -g -I. -O3 
# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)


# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)

# used for lib-packing 
AR = ar -rs

$(LIB): Makefile.depend $(OFILES)
	@echo "$(OFILES)";
	$(AR) $(LIB) $(OFILES)

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CPPFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CPPFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(LIB) *.o Makefile.depend

include Makefile.depend
