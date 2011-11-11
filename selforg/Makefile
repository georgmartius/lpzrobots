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

STRIP=no
TYPE=DEVEL

SELFORG=.
PREFIX=$(shell ./selforg-config $(CFGOPTS) --prefix)
# for debian auto-package installation
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif
include Makefile.conf

ifeq ($(shell uname -o),Msys)
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

.PHONY: lib opt dbg clean clean-all distclean todo depend tags install install_scripts

libs: lib opt dbg

lib: $(LIB)

opt: 
	$(MAKE) BUILD_DIR=build_opt CFGOPTS=--opt STRIP="yes" lib

dbg: 
	$(MAKE) BUILD_DIR=build_dbg CFGOPTS=--dbg lib

shared:
	$(MAKE) $(SHAREDLIB)


$(LIB): Makefile.depend $(OFILES)
	$(AR) $(LIB) $(OFILES)
	@if [ "$(STRIP)" = "yes" ]; then strip --strip-debug $(LIB); fi


${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(CPPFLAGS) $(CFLAGS) -o "$@" "$<"

$(SHAREDLIB): Makefile.depend $(OFILES)
	$(CXX) -shared -Wl,-soname,$(SHAREDLIB) -o $(SHAREDLIB) $(OFILES) 


install_scripts:
	@echo "*************** Install config script $(PREFIX)/bin *********************"
	install -d $(PREFIX)/bin
	install --mode 755 selforg-config $(PREFIX)/bin/	

install: Makefile.depend install_scripts lib opt dbg
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d $(PREFIX)/lib/ $(PREFIX)/include/selforg $(PREFIX)/share/lpzrobots/selforg
	install --mode 644 $(LIB) $(LIB_OPT) $(LIB_DBG) $(PREFIX)/lib/
	install --mode 644 -t $(PREFIX)/include/selforg include/selforg/*.h
	@echo "*************** Install example simulations ******************"
	cp -RL simulations $(PREFIX)/share/lpzrobots/selforg/
endif

uninstall:
	-rm -f $(PREFIX)/bin/selforg-config
ifneq ($(TYPE),DEVEL)
	-rm -f $(PREFIX)/lib/$(LIB) $(PREFIX)/lib/$(LIB_OPT) $(PREFIX)/lib/$(LIB_DBG)
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

tags:
	etags ${CPPFILES} ${HFILES}
	etags -a -o ../TAGS ${CPPFILES} ${HFILES} 



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
	rm -f $(SHAREDLIB)
	rm -f $(shell ./selforg-config --opt --srcprefix="." --libfile)
	rm -f $(shell ./selforg-config --dbg --srcprefix="." --libfile)

clean:
	rm -f Makefile.depend
	rm -rf build build_dbg build_opt
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

todo:
	@grep -ni "Todo"  $(CPPFILES) $(HFILES)
	@grep -ni "Fixme"  $(CPPFILES) $(HFILES)

include Makefile.depend
