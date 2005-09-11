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

UTILS = utils/splitvideo

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

$(LIB): Makefile.depend $(OFILES) $(UTILS)
	@echo "$(OFILES)";
	$(AR) $(LIB) $(OFILES)

utils/splitvideo: utils/splitvideo.c
	$(CXX) -o utils/splitvideo utils/splitvideo.c


Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CPPFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	makedepend $(CFLAGS) $(INC) $(CPPFILES)  -f- > Makefile.depend 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(LIB) *.o Makefile.depend
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done

include Makefile.depend
