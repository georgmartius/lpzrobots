#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005


SELFORGCFG=`if type selforg-config >/dev/null 2>&1; then echo "selforg-config"; elif [ -x ../selforg/selforg-config ]; then echo "../selforg/selforg-config"; else echo "Cannot find selforg-config" 1>&2; fi`


INCLUDEDIR=include/ode_robots
# this is the command to come from the include dir back to the base
REVINCLUDEDIR=../..

# the CFGOPTS are set by the opt and dbg target
CFGOPTS=
LIB := $(shell ./ode_robots-config $(CFGOPTS) --srcprefix="." --libfile)
SHAREDLIB=$(shell ./ode_robots-config $(CFGOPTS) --srcprefix="." --solibfile)

STRIP=no
TYPE=DEVEL
# this can is overwritten from commandline
PREFIX=$(shell ./ode_robots-config $(CFGOPTS) --prefix)
# for debian auto-package installation
# this may not work....?
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif
MAKEDEPEND=makedepend

ifeq ($(TYPE),DEVEL)
SELFORG=$(shell $(SELFORGCFG) $(CFGOPTS) --srcprefix)
endif

include Makefile.conf

BUILD_DIR=build

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))

UTILS = install_prefix.conf

# additional inc paths
INC +=  -I. -Iinclude -Iinclude/ode_robots -I$(shell $(SELFORGCFG) $(CFGOPTS) --srcprefix)/include

# use -pg for profiling
CPPFLAGS = -Wall -pipe -fpic $(INC) $(shell $(SELFORGCFG) $(CFGOPTS) --cflags) \
 $(shell ./ode_robots-config --srcprefix="." $(CFGOPTS) --intern --cflags)  

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rcs

.PHONY: lib opt clean clean-all distclean todo depend tags search find findi install install_scripts

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
	$(MAKE) BUILD_DIR=build_dbg CFGOPTS=--dbg library

opt: $(UTILS)
ifeq ($(TYPE),DEVEL)
	cd $(SELFORG) && $(MAKE) opt
endif
	$(MAKE) BUILD_DIR=build_opt CFGOPTS=--opt STRIP="yes" library

shared:
ifeq ($(TYPE),DEVEL)
	cd $(SELFORG) && $(MAKE) shared
endif
	$(MAKE) $(SHAREDLIB)

library: $(LIB)


$(LIB): $(UTILS) Makefile.depend $(OFILES)	
	$(AR) $(LIB) $(OFILES)
	@if [ "$(STRIP)" = "yes" ]; then strip --strip-debug $(LIB); fi

$(SHAREDLIB): Makefile.depend $(OFILES)
	$(CXX) -shared -Wl,-soname,$(SHAREDLIB) -o $(SHAREDLIB) $(OFILES) 

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
	$(MAKEDEPEND) -- $(CPPFLAGS) -- $(CPPFILES) -p'$$(BUILD_DIR)/' -f- > Makefile.depend 2>/dev/null

depend: 
	rm Makefile.depend
	make Makefile.depend


tags:
	etags ${CPPFILES} ${HFILES}
	etags -a -o ../TAGS ${CPPFILES} ${HFILES} 

##!'search'           		ask for a search string and scans all tex files
search:
	echo -n "search rexp: "; read F && grep -in "$$F" ${CPPFILES} ${HFILES}


##!'find'           		scans all tex files for the pattern stored in the file "tofind"
find:
	@[ -e "tofind" ] &&  grep -f tofind -n  ${CPPFILES} ${HFILES}


##!'findi'           		case insensitive version of target `find'
findi:
	@[ -e "tofind" ] &&  grep -f tofind -in  ${CPPFILES} ${HFILES}


distclean: clean-all
clean-all: clean
	rm -f $(LIB) $(LIB_DBG) $(LIB_OPT) $(LIB_PROF)

clean:
	rm -f Makefile.depend
	rm -rf build build_dbg build_opt
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

install_scripts:
	@echo "*************** Install binaries and scripts to $(PREFIX)/bin **********"
	install -d $(PREFIX)/bin
	install --mode 755 ode_robots-config $(PREFIX)/bin/
	install --mode 755 utils/feedfile.pl $(PREFIX)/bin/
	install --mode 755 utils/encodevideo.sh $(PREFIX)/bin/
	install --mode 755 utils/writeTextToFrames.sh $(PREFIX)/bin/
	install --mode 755 utils/selectcolumns.pl $(PREFIX)/bin/


install: Makefile.depend install_scripts lib opt dbg
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d  $(PREFIX)/lib $(PREFIX)/include/ode_robots 
	install --mode 644 $(LIB) $(LIB_OPT) $(LIB_DBG) $(PREFIX)/lib/
	install --mode 644 -t $(PREFIX)/include/ode_robots include/ode_robots/*.h
endif
	install -d $(PREFIX)/share/lpzrobots $(PREFIX)/share/lpzrobots/ode_robots
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
	-rm -f $(PREFIX)/bin/ode_robots-config
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


-include Makefile.depend
