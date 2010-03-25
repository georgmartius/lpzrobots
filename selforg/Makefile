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

include Makefile.conf

find_cpp_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_cpp_files))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))

# additional inc paths
INC +=  -I. -Iinclude

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

lib: $(LIB)

$(LIB): Makefile.depend $(OFILES)
	$(AR) $(LIB) $(OFILES)

opt: $(LIB_OPT)

dbg: $(LIB_DBG)


# a test to make objectfiles go to another folder
#$(LIB_OPT): OUTPUT_OPTION = -o .obj/`basename $@`

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(LIB_OPT): Makefile.depend $(OFILES)
#	@echo "$(OFILES)";
	$(AR) $(LIB_OPT) $(OFILES)

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_DBG)
$(LIB_DBG): Makefile.depend $(OFILES)
#	@echo "$(OFILES)";
	$(AR) $(LIB_DBG) $(OFILES)

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
