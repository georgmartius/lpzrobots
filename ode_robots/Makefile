#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

PACKAGE_NAME=ode_robots
LPZROBOTS=..
ODEROBOTS=.
SELFORG=$(LPZROBOTS)/selforg
INCLUDEDIR=include/ode_robots
REVINCLUDEDIR=../..
LIBBASE=lib$(PACKAGE_NAME)
LIB=$(LIBBASE).a
LIB_DBG= $(LIBBASE)_dbg.a
LIB_OPT= $(LIBBASE)_opt.a
STRIP=no
# this can is overwritten from commandline
PREFIX=/usr/local
# for debian auto-package installation
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif

include Makefile.conf

BUILD_DIR=build

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))

UTILS = install_prefix.conf
# utils/splitvideo
ODEFLAGS = $(shell ode-dbl-config --cflags)
CPPBASEFLAGS = -Wall -Wno-deprecated -pipe -I. $(ODEFLAGS) -pthread $(INC) 

# see debug and optimization versions below
CPPFLAGS = -g -O1 $(CPPBASEFLAGS)

## profiling
#CPPFLAGS_PROF = $(CPPBASEFLAGS) -pg -O -DQPROF

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rcs

.PHONY: lib opt clean clean-all distclean todo depend tags

libs: lib opt dbg

lib: $(UTILS) 
	cd $(SELFORG) && $(MAKE) lib
	$(MAKE) library

dbg: $(UTILS)
	cd $(SELFORG) && $(MAKE) dbg
	$(MAKE) BUILD_DIR=build_dbg LIB=$(LIB_DBG) CPPFLAGS="$(CPPBASEFLAGS) -g" library

opt: $(UTILS)
	cd $(SELFORG) && $(MAKE) opt
	$(MAKE) BUILD_DIR=build_opt LIB=$(LIB_OPT) CPPFLAGS="$(CPPBASEFLAGS) -O3 -DNDEBUG" STRIP="yes" library

library: $(LIB)


$(LIB): $(UTILS) Makefile.depend $(OFILES)	
	$(AR) $(LIB) $(OFILES)


${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(CPPFLAGS) $(CFLAGS) -o "$@" "$<"


utils/splitvideo: utils/splitvideo.c
	$(CXX) -o utils/splitvideo utils/splitvideo.c

#%.o: %.cpp
#	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $(OBJDIR)/$(@F)  $< 

Makefile.depend: 	
	for file in $(HFILES); do \
		ln -sf $(REVINCLUDEDIR)/$$file $(INCLUDEDIR)/; \
	done
	makedepend $(INC) $(CPPFILES) -p'$$(BUILD_DIR)/' -f- > Makefile.depend 2>/dev/null

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
	rm -rf build build_dbg build_opt
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

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
