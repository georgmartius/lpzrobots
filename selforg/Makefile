#File:     Makefile for selforg, the controller for self-organized behavior
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     November 2005

PACKAGE_NAME=selforg
SELFORG=.
INCLUDEDIR=include/selforg
REVINCLUDEDIR=../..
# this is the command to come form the include dir back to the base
LIB=lib$(PACKAGE_NAME).a
LIB_DBG=lib$(PACKAGE_NAME)_dbg.a
LIB_OPT=lib$(PACKAGE_NAME)_opt.a
ifeq ($(PREFIX),)
PREFIX=/usr/local
endif
# for debian auto-package installation
ifneq ($(DESTDIR),) 
PREFIX=$(DESTDIR)
endif
include Makefile.conf

find_cpp_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_cpp_files))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))

# additional inc paths
INC +=  -I. -Iinclude $(shell gsl-config --cflags) $(shell gsl-config --cflags)
# one can possibly disable gsl using -DNO_GSL

# use -pg for profiling
CBASEFLAGS = -Wall -pipe -Wno-deprecated 
CPPFLAGS = $(CBASEFLAGS) -g -O $(INC)
# Optimisation 
CPPFLAGS_OPT = $(CBASEFLAGS) -O3 -DNDEBUG  $(INC)
# DEBUG 
CPPFLAGS_DBG = $(CBASEFLAGS) -g  $(INC)

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rs

.PHONY: lib opt dbg clean todo depend tags

libs:
	@echo "*************** Compile selforg (debug) *****************"
	$(MAKE) clean-all && $(MAKE) dbg
	@echo "*************** Compile selforg (optimized) *****************"
	$(MAKE) clean && $(MAKE) opt
	@echo "*************** Compile selforg (normal) *********************"
	$(MAKE) clean && $(MAKE) lib

lib: $(LIB)

opt: $(LIB_OPT)

dbg: $(LIB_DBG)


$(LIB): Makefile.depend $(OFILES)
	$(AR) $(LIB) $(OFILES)
	strip --only-keep-debug $(LIB)

# a test to make objectfiles go to another folder, but does not work
#$(LIB_OPT): OUTPUT_OPTION = -o .obj/`basename $@`

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(LIB_OPT): Makefile.depend $(OFILES)
#	@echo "$(OFILES)";
	$(AR) $(LIB_OPT) $(OFILES)
	strip $(LIB_OPT)

$(LIB_DBG): CPPFLAGS = $(CPPFLAGS_DBG)
$(LIB_DBG): Makefile.depend $(OFILES)
#	@echo "$(OFILES)";
	$(AR) $(LIB_DBG) $(OFILES)

install: Makefile.depend $(LIB_OPT) $(LIB_DBG) $(LIB)
	@echo "*************** Install libs and includes $(PREFIX)*********************"
	install -d $(PREFIX)/lib/ $(PREFIX)/include/selforg $(PREFIX)/share/lpzrobots/selforg
	install --mode 644 $(LIB_OPT) $(LIB_DBG) $(LIB) $(PREFIX)/lib/
	install --mode 644 -t $(PREFIX)/include/selforg include/selforg/*.h
	@echo "*************** Install example simulations ******************"
	cp -RL simulations $(PREFIX)/share/lpzrobots/selforg/

uninstall:
	-rm -f $(PREFIX)/lib/$(LIB_OPT) $(PREFIX)/lib/$(LIB_DBG) $(PREFIX)/lib/$(LIB)
	-rm -rf $(PREFIX)/include/selforg
	-rm -rf $(PREFIX)/share/lpzrobots/selforg/


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


clean-all: clean
	rm -f $(LIB) $(LIB_OPT) $(LIB_DBG)

clean:
	rm -f Makefile.depend
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

todo:
	@grep -ni "Todo"  $(CPPFILES) $(HFILES)
	@grep -ni "Fixme"  $(CPPFILES) $(HFILES)

FIND=`cat tofind`
find:
	@[ -n "$(FIND)" ] &&  grep -ni "$(FIND)"  $(CPPFILES) $(HFILES)

include Makefile.depend
