#File:     Makefile for ode_robot simulation framwwork
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005


SELFORGCFG=`if type selforg-config >/dev/null 2>&1; then echo "selforg-config"; elif [ -x ../selforg/selforg-config ]; then echo "../selforg/selforg-config"; else echo "Cannot find selforg-config" 1>&2; fi`


PACKAGE=ga_tools
INCLUDEDIR=include/$(PACKAGE)
# this is the command to come from the include dir back to the base
REVINCLUDEDIR=../..

# the CFGOPTS are set by the opt and dbg target
CFGOPTS=
LIB := $(shell ./$(PACKAGE)-config $(CFGOPTS) --srcprefix="." --libfile)
SHAREDLIB=$(shell ./$(PACKAGE)-config $(CFGOPTS) --srcprefix="." --solibfile)
TYPE=$(shell ./$(PACKAGE)-config --type)

STRIP=no
# this can is overwritten from commandline
PREFIX=$(shell ./$(PACKAGE)-config $(CFGOPTS) --prefix)
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
INC +=  -I. -Iinclude -Iinclude/$(PACKAGE) -I$(shell $(SELFORGCFG) $(CFGOPTS) --srcprefix)/include

# use -pg for profiling
CPPFLAGS = -Wall -pipe -fpic $(INC) $(shell $(SELFORGCFG) $(CFGOPTS) --cflags) \
 $(shell ./$(PACKAGE)-config --srcprefix="." $(CFGOPTS) --intern --cflags)  

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rcs

.PHONY: lib opt clean clean-all distclean todo depend tags search find findi install install_scripts install_lib uninstall uninstall_lib

libs: lib opt dbg
	$(MAKE) shared
# the shared lib cannot be done as a dependency because with -j 2 it collides with the lib target

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
	$(CXX) -shared -Wl,-soname,`basename $(SHAREDLIB)` -o $(SHAREDLIB) $(OFILES) 

${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(CPPFLAGS) $(CFLAGS) -o "$@" "$<"

${BUILD_DIR}/%.gch: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(CPPFLAGS) $(CFLAGS) -o "$@" "$<"

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


# Workaroud for bug in etags: filenames should not have leading ./
CPPFILESNoDot := $(subst ./,,$(CPPFILES))
HFILESNoDot   := $(subst ./,,$(HFILES))
##!'tags'           		creates tags file for emacs
tags:
	@etags -l c++ $(HFILESNoDot) $(CPPFILESNoDot)
	@etags -l c++ -a -o ../TAGS ${HFILESNoDot} ${CPPFILESNoDot}

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

clean:
	rm -f Makefile.depend
	rm -rf build build_dbg build_opt
	rm -f $(SHAREDLIB)	
	rm -f $(shell ./$(PACKAGE)-config --srcprefix="." --libfile)
	rm -f $(shell ./$(PACKAGE)-config --opt --srcprefix="." --libfile)
	rm -f $(shell ./$(PACKAGE)-config --dbg --srcprefix="." --libfile)
	find $(INCLUDEDIR) -type l -exec rm \{\} \;


install_scripts:
	@echo "*************** Install binaries and scripts to $(PREFIX)/bin **********"
	install -d $(PREFIX)/bin
	install -m 755 $(PACKAGE)-config $(PREFIX)/bin/

install_lib:
	install -m 644 $(LIB) $(PREFIX)/lib/

install: Makefile.depend install_scripts
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d  $(PREFIX)/lib $(PREFIX)/include/$(PACKAGE) 
	$(MAKE) lib install_lib
	$(MAKE) CFGOPTS=--dbg dbg install_lib
	$(MAKE) CFGOPTS=--opt opt install_lib
	install -m 644 include/$(PACKAGE)/*.h $(PREFIX)/include/$(PACKAGE)
endif
	install -d $(PREFIX)/share/lpzrobots $(PREFIX)/share/lpzrobots/$(PACKAGE)
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install example simulations ******************"
	cp -RL simulations $(PREFIX)/share/lpzrobots/$(PACKAGE)/
	chmod -R ugo+r $(PREFIX)/share/lpzrobots/$(PACKAGE)
endif

uninstall_lib:
	-rm -f $(PREFIX)/lib/$(LIB)

uninstall:
ifneq ($(TYPE),DEVEL)
	$(MAKE) uninstall_lib
	$(MAKE) CFGOPTS=--dbg uninstall_lib
	$(MAKE) CFGOPTS=--opt uninstall_lib
	-rm -rf $(PREFIX)/include/$(PACKAGE)
endif
	-rm -rf $(PREFIX)/share/lpzrobots/$(PACKAGE)/
	-rm -f $(PREFIX)/bin/$(PACKAGE)-config


install_prefix.conf:
	@echo "create install_prefix.conf"
	@echo '// for system installations (installation type:user) the macro PREFIX should be defined:' > install_prefix.conf
	@echo '// e.g. #define PREFIX "/usr/local"'  >> install_prefix.conf

todo:
	@grep -ni "Todo" $(CPPFILES) $(HFILES)
	@grep -ni "Fixme" $(CPPFILES) $(HFILES)


-include Makefile.depend
