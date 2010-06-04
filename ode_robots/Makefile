#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

#
# $Log$
# Revision 1.34  2010-06-04 13:11:58  martius
# added distclean
#
# Revision 1.33  2010/05/19 11:45:44  martius
# parallel make works now
#
# Revision 1.32  2010/05/17 15:37:54  martius
# typo for debug version
#
# Revision 1.31  2010/03/25 16:39:51  martius
# primitive has addForce/addTorque function
#
# Revision 1.30  2010/03/24 16:51:38  martius
# QuickMP uses now the number of processors in the system
# optical flow improved
# video recording works with offscreen rendering
# Make system: Optimization -O1 is switched on by default (add a debug version without optimization)
#
# Revision 1.29  2010/03/09 11:53:41  martius
# renamed globally ode to ode-dbl
#
# Revision 1.28  2010/03/05 14:32:55  martius
# camera sensor added
# for that the scenegraph structure was changed into root, world, scene
# camera does not work with shadows
# works with newest version of ode (0.11)
#
# Revision 1.27  2009/12/01 17:32:10  martius
# adapted Makefiles to ignore backward compat. errors
#
# Revision 1.26  2009/08/10 07:45:31  guettler
# added -Wno-deprecated to avoid warnings about using old includes
# (because we use some where are no new ones are provided yet)
#
#


PACKAGE_NAME=ode_robots
LPZROBOTS=..
ODEROBOTS=.
SELFORG=../selforg
INCLUDEDIR=include/ode_robots
REVINCLUDEDIR=../..
LIB=lib$(PACKAGE_NAME).a
LIB_DBG=lib$(PACKAGE_NAME)_dbg.a
LIB_OPT=lib$(PACKAGE_NAME)_opt.a
LIB_PROF=lib$(PACKAGE_NAME)_prof.a

include Makefile.conf

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))

UTILS = install_prefix.conf
# utils/splitvideo
ODEFLAGS = $(shell ode-dbl-config --cflags)
CPPBASEFLAG = -Wall -Wno-deprecated -pipe -I. $(ODEFLAGS) -pthread $(INC) 

CPPFLAGS = -g -O1 $(CPPBASEFLAG)

## Debug (no optimization
CPPFLAGS_DBG = $(CPPBASEFLAG) -g

## Optimisation
CPPFLAGS_OPT = $(CPPBASEFLAG) -O3 -DNDEBUG

## Optimisation
CPPFLAGS_PROF = $(CPPBASEFLAG) -pg -O -DQPROF

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rs

.PHONY: lib opt clean clean-all distclean todo depend tags

lib: $(UTILS) 
	$(MAKE) libselforg 
	$(MAKE) $(LIB)

dbg: $(UTILS)
	$(MAKE) libselforg_dbg
	$(MAKE) $(LIB_DBG)

opt: $(UTILS)
	$(MAKE) libselforg_opt
	$(MAKE) $(LIB_OPT)

prof: $(UTILS)
	$(MAKE) libselforg
	$(MAKE) $(LIB_PROF)

$(LIB): $(UTILS) Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB) $(OFILES)

$(LIB_DBG): CPPFLAGS = $(CPPFLAGS_DBG)
$(LIB_DBG): Makefile.depend $(SELFORG)/libselforg_dbg.a $(OFILES)
	$(AR) $(LIB_DBG) $(OFILES)

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(LIB_OPT): Makefile.depend $(SELFORG)/libselforg_opt.a $(OFILES)
	$(AR) $(LIB_OPT) $(OFILES)

$(LIB_PROF): CPPFLAGS = $(CPPFLAGS_PROF)
$(LIB_PROF): Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB_PROF) $(OFILES)

libselforg:
	cd $(SELFORG) && $(MAKE) libselforg.a		

libselforg_dbg:	
	cd $(SELFORG) && $(MAKE) libselforg_dbg.a		

libselforg_opt:	
	cd $(SELFORG) && $(MAKE) libselforg_opt.a		

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


distclean: clean-all
clean-all: clean
	rm -f $(LIB) $(LIB_DBG) $(LIB_OPT) $(LIB_PROF)

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
	@grep -ni "Todo" $(CPPFILES) $(HFILES)
	@grep -ni "Fixme" $(CPPFILES) $(HFILES)

FIND=`cat tofind`
find:
	@[ -n "$(FIND)" ] &&  grep -ni "$(FIND)"  $(CPPFILES) $(HFILES)


-include Makefile.depend
