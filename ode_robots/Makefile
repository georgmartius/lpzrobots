#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

#
# $Log$
# Revision 1.27  2009-12-01 17:32:10  martius
# adapted Makefiles to ignore backward compat. errors
#
# Revision 1.26  2009/08/10 07:45:31  guettler
# added -Wno-deprecated to avoid warnings about using old includes
# (because we use some where are no new ones are provided yet)
#
#

LPZROBOTS=..
ODEROBOTS=.
SELFORG=../selforg
INCLUDEDIR=include/ode_robots
REVINCLUDEDIR=../..
LIB=libode_robots.a
LIB_OPT=libode_robots_opt.a
LIB_PROF=libode_robots_prof.a

include Makefile.conf

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))

UTILS = install_prefix.conf
# utils/splitvideo

# -pg for profiling
CPPFLAGS = -Wall -g -Wno-deprecated -pipe -I. -pthread $(INC) 

## Optimisation
CPPFLAGS_OPT = -Wall -I. -Wno-deprecated -pipe -pthread $(INC) -O -DNDEBUG

## Optimisation
CPPFLAGS_PROF = -Wall -I. -Wno-deprecated -pipe -pthread $(INC) -O -DQPROF


# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rs

.PHONY: lib opt clean todo depend tags

lib: $(UTILS) libselforg $(LIB)

opt: $(UTILS) libselforg_opt $(LIB_OPT)

prof: $(UTILS) libselforg $(LIB_PROF)

$(LIB): Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB) $(OFILES)

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(LIB_OPT): Makefile.depend $(SELFORG)/libselforg_opt.a $(OFILES)
	$(AR) $(LIB_OPT) $(OFILES)

$(LIB_PROF): CPPFLAGS = $(CPPFLAGS_PROF)
$(LIB_PROF): Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB_PROF) $(OFILES)

libselforg:	
	+cd $(SELFORG) && make libselforg.a		

libselforg_opt:	
	+cd $(SELFORG) && make libselforg_opt.a		

utils/splitvideo: utils/splitvideo.c
	$(CXX) -o utils/splitvideo utils/splitvideo.c

#%.o: %.cpp
#	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $(OBJDIR)/$(@F)  $< 

Makefile.depend: 	
	for file in $(HFILES); do \
		ln -sf $(REVINCLUDEDIR)/$$file $(INCLUDEDIR)/; \
	done
	makedepend $(INC) $(CPPFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	rm Makefile.depend
	make Makefile.depend

tags: 
	etags `find -type f -regex ".*\.[hc]p?p?"` 

##!'search'           		ask for a search string and scans all tex files
search:
	echo -n "search rexp: "; read F && grep -in "$$F" ${CPPFILES} ${HFILES}


clean-all: clean
	rm -f $(LIB) $(LIB_OPT) $(LIB_PROF)

clean:
	rm -f Makefile.depend
	find $(INCLUDEDIR) -type l -exec rm \{\} \;
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done

install_prefix.conf:
	@echo "create install_prefix.conf"
	@echo '// for system installations (installation type:user) the macro PREFIX should be defined:' > install_prefix.conf
	@echo '// e.g. #define PREFIX "/usr/local"'  >> install_prefix.conf

todo:
	grep -ni "Todo" $(CPPFILES) $(HFILES)
	grep -ni "Fixme" $(CPPFILES) $(HFILES)

FIND=`cat tofind`
find:
	@[ -n "$(FIND)" ] &&  grep -ni "$(FIND)"  $(CPPFILES) $(HFILES)


-include Makefile.depend
