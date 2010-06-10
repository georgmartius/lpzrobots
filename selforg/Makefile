#File:     Makefile for selforg, the controller for self-organized behavior
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     November 2005

PACKAGE_NAME=selforg
SELFORG=.
INCLUDEDIR=include/selforg
REVINCLUDEDIR=../..
# this is the command to come form the include dir back to the base
LIBBASE=lib$(PACKAGE_NAME)
LIB=$(LIBBASE).a
LIB_OPT=$(LIBBASE)_opt.a
LIB_DBG=$(LIBBASE)_dbg.a
STRIP=no
TYPE=DEVEL
ifeq ($(PREFIX),)
PREFIX=/usr/local
endif
# for debian auto-package installation
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif
include Makefile.conf

BUILD_DIR=build

find_cpp_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_cpp_files))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))
OFILES     := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(CPPFILES))

# additional inc paths
INC +=  -I. -Iinclude $(shell gsl-config --cflags)
# one can possibly disable gsl using -DNO_GSL

# use -pg for profiling
CBASEFLAGS = -Wall -pipe -Wno-deprecated 
CPPFLAGS = $(CBASEFLAGS) -g -O $(INC)
# Optimisation see at opt target
# DEBUG  see at dbg target


# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rcs

.PHONY: lib opt dbg clean clean-all distclean todo depend tags

libs: lib opt dbg

lib: $(LIB)

opt: 
	$(MAKE) BUILD_DIR=build_opt LIB=$(LIB_OPT) \
 CPPFLAGS="$(CBASEFLAGS) -O3 -DNDEBUG  $(INC)" STRIP="yes" lib

dbg: 
	$(MAKE) BUILD_DIR=build_dbg LIB=$(LIB_DBG) \
 CPPFLAGS="$(CBASEFLAGS) -g  $(INC)" lib


$(LIB): Makefile.depend $(OFILES)
	$(AR) $(LIB) $(OFILES)
	@if [ "$(STRIP)" == "yes" ]; then strip --strip-debug $(LIB); fi


${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $(CPPFLAGS) $(CFLAGS) -o "$@" "$<"


install: Makefile.depend lib opt dbg
ifneq ($(TYPE),DEVEL)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d $(PREFIX)/lib/ $(PREFIX)/include/selforg $(PREFIX)/share/lpzrobots/selforg
	install --mode 644 $(LIB) $(LIB_OPT) $(LIB_DBG) $(PREFIX)/lib/
	install --mode 644 -t $(PREFIX)/include/selforg include/selforg/*.h
	@echo "*************** Install example simulations ******************"
	cp -RL simulations $(PREFIX)/share/lpzrobots/selforg/
endif

uninstall:
ifneq ($(TYPE),DEVEL)
	-rm -f $(PREFIX)/lib/$(LIB) $(PREFIX)/lib/$(LIB_OPT) $(PREFIX)/lib/$(LIB_DBG)
	-rm -rf $(PREFIX)/include/selforg
	-rm -rf $(PREFIX)/share/lpzrobots/selforg/
endif


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


distclean: clean-all
clean-all: clean
	rm -f $(LIB) $(LIB_OPT) $(LIB_DBG)

clean:
	rm -f Makefile.depend
	rm -rf build build_dbg build_opt
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

todo:
	@grep -ni "Todo"  $(CPPFILES) $(HFILES)
	@grep -ni "Fixme"  $(CPPFILES) $(HFILES)

FIND=`cat tofind`
find:
	@[ -n "$(FIND)" ] &&  grep -ni "$(FIND)"  $(CPPFILES) $(HFILES)

include Makefile.depend
