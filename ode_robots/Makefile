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
TYPE=DEVEL
STRIP=no
# this can is overwritten from commandline
PREFIX=/usr/local
# for debian auto-package installation
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif
MAKEDEPEND=makedepend

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
CPPBASEFLAGS = -Wall -pipe -I. $(ODEFLAGS) -pthread $(INC) 

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
ifeq ($(TYPE),DEVEL)
	cd $(SELFORG) && $(MAKE) lib
endif
	$(MAKE) library

dbg: $(UTILS)
ifeq ($(TYPE),DEVEL)
	cd $(SELFORG) && $(MAKE) dbg
endif
	$(MAKE) BUILD_DIR=build_dbg LIB=$(LIB_DBG) CPPFLAGS="$(CPPBASEFLAGS) -g" library

opt: $(UTILS)
ifeq ($(TYPE),DEVEL)
	cd $(SELFORG) && $(MAKE) opt
endif
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
	$(MAKEDEPEND) $(INC) $(CPPFILES) -p'$$(BUILD_DIR)/' -f- > Makefile.depend 2>/dev/null

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

install: Makefile.depend lib opt dbg
	install -d  $(PREFIX)/bin $(PREFIX)/share/lpzrobots $(PREFIX)/share/lpzrobots/ode_robots
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d  $(PREFIX)/lib $(PREFIX)/include/ode_robots 
	install --mode 644 $(LIB) $(LIB_OPT) $(LIB_DBG) $(PREFIX)/lib/
	install --mode 644 -t $(PREFIX)/include/ode_robots include/ode_robots/*.h
endif
	install --mode 755 utils/feedfile.pl $(PREFIX)/bin/
	install --mode 755 utils/encodevideo.sh $(PREFIX)/bin/
	install --mode 755 utils/writeTextToFrames.sh $(PREFIX)/bin/
	install --mode 755 utils/selectcolumns.pl $(PREFIX)/bin/
	cp -R osg/data $(PREFIX)/share/lpzrobots/
ifeq ($(TYPE),DEVEL)
	-find $(PREFIX)/share/lpzrobots/ -type d -name "CVS" | xargs rm -r
else
	@echo "*************** Install example simulations ******************"
	cp -RL simulations $(PREFIX)/share/lpzrobots/ode_robots/
	chmod -R ugo+r $(PREFIX)/share/lpzrobots
endif

uninstall:
ifneq ($(TYPE),DEVEL)
	-rm -f $(PREFIX)/lib/$(LIB) $(PREFIX)/lib/$(LIB_OPT) $(PREFIX)/lib/$(LIB_DBG)
	-rm -rf $(PREFIX)/include/ode_robots
endif
	-rm -rf $(PREFIX)/share/lpzrobots/ode_robots/
	-rm -f $(PREFIX)/bin/feedfile.pl
	-rm -f $(PREFIX)/bin/encodevideo.sh 
	-rm -f $(PREFIX)/bin/selectcolumns.pl
	-rm -rf $(PREFIX)/share/lpzrobots/data


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
