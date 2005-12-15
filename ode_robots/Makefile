#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

LPZROBOTS=..
ODEROBOTS=.
SELFORG=../selforg
MATRIXLIB=../matrixlib
INCLUDEDIR=include/ode_robots
REVINCLUDEDIR=../..
LIB=libode_robots.a

include Makefile.conf

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))


UTILS = 
# utils/splitvideo

# -pg for profiling
# CFLAGS = -Wall -I. -g
## Optimisation #DNDEBUG
#CFLAGS = -Wall -g -I. -O3 
CFLAGS = -Wall -g -I. 
# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)


# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)

# used for lib-packing 
AR = ar -rs

$(LIB): libselforg $(SELFORG)/libselforg.a Makefile.depend $(OFILES) $(UTILS)
	@echo "$(OFILES)";
	$(AR) $(LIB) $(OFILES)

libselforg:	
	cd $(SELFORG) && make libselforg.a		


utils/splitvideo: utils/splitvideo.c
	$(CXX) -o utils/splitvideo utils/splitvideo.c


Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CPPFILES) -f- > Makefile.depend 2>/dev/null
	cd $(INCLUDEDIR)
	for file in $(HFILES); do \
		ln -sf $(REVINCLUDEDIR)/$$file $(INCLUDEDIR)/; \
	done

depend: 
	rm Makefile.depend
	make Makefile.depend

tags: 
	etags `find -name "*.[ch]*"` 

clean:
	rm -f $(LIB) *.o Makefile.depend
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

include Makefile.depend
