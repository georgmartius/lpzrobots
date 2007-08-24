#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

LPZROBOTS=..
ODEROBOTS=.
SELFORG=../selforg
INCLUDEDIR=include/ode_robots
REVINCLUDEDIR=../..
LIB=libode_robots.a
LIB_OPT=libode_robots_opt.a

include Makefile.conf

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))

UTILS = 
# utils/splitvideo

# -pg for profiling
CPPFLAGS = -Wall -g -I. $(INC)

## Optimisation
CPPFLAGS_OPT = -Wall -I. $(INC) -O -DNDEBUG

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rs

.PHONY: lib opt clean todo depend tags

lib: libselforg $(LIB) $(UTILS)

opt: libselforg_opt $(LIB_OPT)

$(LIB): Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB) $(OFILES)

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(LIB_OPT): Makefile.depend $(SELFORG)/libselforg_opt.a $(OFILES)
	$(AR) $(LIB_OPT) $(OFILES)

libselforg:	
	cd $(SELFORG) && make libselforg.a		

libselforg_opt:	
	cd $(SELFORG) && make libselforg_opt.a		

utils/splitvideo: utils/splitvideo.c
	$(CXX) -o utils/splitvideo utils/splitvideo.c

#%.o: %.cpp
#	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $(OBJDIR)/$(@F)  $< 

Makefile.depend: 	
	for file in $(HFILES); do \
		ln -sf $(REVINCLUDEDIR)/$$file $(INCLUDEDIR)/; \
	done
	makedepend $(CFLAGS) $(INC) $(CPPFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	rm Makefile.depend
	make Makefile.depend

tags: 
	etags `find -type f -regex ".*\.[hc]p?p?"` 

clean:
	rm -f $(LIB) Makefile.depend
	find $(INCLUDEDIR) -type l -exec rm \{\} \;
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done

todo:
	grep -ni "Todo" $(CPPFILES) $(HFILES)
	grep -ni "Fixme" $(CPPFILES) $(HFILES)

include Makefile.depend
