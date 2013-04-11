#File:     Makefile for selforg, the controller for self-organized behavior
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     November 2005

INCLUDEDIR=include/selforg
# this is the command to come from the include dir back to the base
REVINCLUDEDIR=../..

# the CFGOPTS are set by the opt and dbg target
CFGOPTS=
LIB := $(shell ./selforg-config $(CFGOPTS) --srcprefix="." --libfile)
SHAREDLIB=$(shell ./selforg-config $(CFGOPTS) --srcprefix="." --solibfile)
TYPE=$(shell ./selforg-config --type)

STRIP=no

SELFORG=.
PREFIX=$(shell ./selforg-config $(CFGOPTS) --prefix)
# for debian auto-package installation
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif
include Makefile.conf

# was -o, see what it does in Msys
ifeq ($(shell uname -s),Msys)
REVINCLUDEDIR=.
endif

BUILD_DIR=build

find_cpp_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_cpp_files))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))
OFILES     := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(CPPFILES))

# additional inc paths
INC +=  -I. -Iinclude -Iinclude/selforg

# use -pg for profiling
CPPFLAGS = -Wall -pipe -fpic $(INC) $(shell ./selforg-config --srcprefix="." $(CFGOPTS) --intern --cflags)

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rcs

.PHONY: lib opt dbg clean clean-all distclean todo depend tags install install_lib uninstall uninstall_lib

libs: lib opt dbg
	$(MAKE) shared
# the shared lib cannot be done as a dependency because with -j 2 it collides with the lib target

lib: $(LIB)

opt: 
	$(MAKE) BUILD_DIR=build_opt CFGOPTS=--opt STRIP="yes" lib

dbg: 
	$(MAKE) BUILD_DIR=build_dbg CFGOPTS=--dbg lib

shared:
	$(MAKE) $(SHAREDLIB)


$(LIB): Makefile.depend $(OFILES)
	$(AR) $(LIB) $(OFILES)
	@if [ "$(STRIP)" = "yes" ]; then strip -S $(LIB); fi


${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(CPPFLAGS) $(CFLAGS) -o "$@" "$<"

$(SHAREDLIB): Makefile.depend $(OFILES)
	$(CXX) -shared -Wl,-soname,`basename $(SHAREDLIB)` -o $(SHAREDLIB) $(OFILES) 


install_scripts:
	@echo "*************** Install config script $(PREFIX)/bin *********************"
	install -d $(PREFIX)/bin
	install -m 755 selforg-config $(PREFIX)/bin/	

install_lib:
	install -m 644 $(LIB) $(PREFIX)/lib/


install: Makefile.depend install_scripts
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d $(PREFIX)/lib/ $(PREFIX)/include/selforg $(PREFIX)/share/lpzrobots/selforg
	$(MAKE) install_lib
	$(MAKE) CFGOPTS=--dbg install_lib
	$(MAKE) CFGOPTS=--opt install_lib
	$(MAKE) LIB=$(SHAREDLIB) install_lib
	install -m 644 include/selforg/*.h $(PREFIX)/include/selforg
	@echo "*************** Install example simulations ******************"
	cp -RL simulations $(PREFIX)/share/lpzrobots/selforg/
endif


uninstall_lib:
	-rm -f $(PREFIX)/lib/$(LIB)

uninstall:
	-rm -f $(PREFIX)/bin/selforg-config
ifneq ($(TYPE),DEVEL)
	$(MAKE) uninstall_lib
	$(MAKE) CFGOPTS=--dbg uninstall_lib
	$(MAKE) CFGOPTS=--opt uninstall_lib
	$(MAKE) LIB=$(SHAREDLIB) uninstall_lib
	-rm -rf $(PREFIX)/include/selforg
	-rm -rf $(PREFIX)/share/lpzrobots/selforg/
endif


Makefile.depend: 
	for file in $(HFILES); do \
		ln -sf $(REVINCLUDEDIR)/$$file $(INCLUDEDIR)/; \
	done
	makedepend -- $(CPPFLAGS) -- $(CPPFILES) -p'$$(BUILD_DIR)/' -f- > Makefile.depend 2>/dev/null

depend: 
	rm Makefile.depend
	make Makefile.depend


.PHONY: check-syntax
check-syntax:
	$(CXX) $(CPPFLAGS) -Wextra -S -fsyntax-only $(CHK_SOURCES)

# Workaroud for bug in etags: filenames should not have leading ./
CPPFILESNoDot := $(subst ./,,$(CPPFILES))
HFILESNoDot   := $(subst ./,,$(HFILES))
##!'tags'           		creates tags file for emacs
tags:
	@etags --language-force="C++" $(HFILESNoDot) $(CPPFILESNoDot)
	@etags --language-force="C++" -a -o ../TAGS ${HFILESNoDot} ${CPPFILESNoDot}



.PHONY: search
##!'search'           		ask for a search string and scans all tex files
search:
	echo -n "search rexp: "; read F && grep -in "$$F" ${CPPFILES} ${HFILES}

##!'find'           		scans all tex files for the pattern stored in the file "tofind"
.PHONY: find
find:
	@[ -e "tofind" ] &&  grep -f tofind -n  ${CPPFILES} ${HFILES}


##!'findi'           		case insensitive version of target `find'
.PHONY: findi
findi:
	@[ -e "tofind" ] &&  grep -f tofind -in  ${CPPFILES} ${HFILES}

distclean: clean-all
clean-all: clean

clean:
	rm -f Makefile.depend
	rm -rf build build_dbg build_opt
	rm -f $(SHAREDLIB)
	rm -f $(shell ./selforg-config --srcprefix="." --libfile)
	rm -f $(shell ./selforg-config --opt --srcprefix="." --libfile)
	rm -f $(shell ./selforg-config --dbg --srcprefix="." --libfile)
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

todo:
	@grep -ni "Todo"  $(CPPFILES) $(HFILES)
	@grep -ni "Fixme"  $(CPPFILES) $(HFILES)


Makefile.conf.generated: 
	@echo "run ./configure first!"
	@exit 1;

include Makefile.depend
