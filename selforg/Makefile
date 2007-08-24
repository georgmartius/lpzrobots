#File:     Makefile for selforg, the controller for self-organized behavior
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     November 2005

SELFORG=.
INCLUDEDIR=include/selforg
REVINCLUDEDIR=../..
# this is the command to come form the include dir back to the base
LIB=libselforg.a
LIB_OPT=libselforg_opt.a

include Makefile.conf

find_cpp_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_cpp_files))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))


# use -pg for profiling
CFLAGS = -Wall -g -I. -Iinclude
# Optimisation 
CFLAGS_OPT = -Wall -I. -Iinclude -O -DNDEBUG

# used for single file compilation
CXX = g++ $(CFLAGS) $(INC)
CXX_OPT = g++ $(CFLAGS_OPT) $(INC)

# used for lib-packing 
AR = ar -rs

.PHONY: lib opt clean todo depend tags

lib: $(LIB)

$(LIB): Makefile.depend $(OFILES)
	$(AR) $(LIB) $(OFILES)

opt: $(LIB_OPT)

# a test to make objectfiles go to another folder
#$(LIB_OPT): OUTPUT_OPTION = -o .obj/`basename $@`

$(LIB_OPT): CXX = $(CXX_OPT)
$(LIB_OPT): Makefile.depend $(OFILES)
	@echo "$(OFILES)";
	$(AR) $(LIB_OPT) $(OFILES)

Makefile.depend: 
	for file in $(HFILES); do \
		ln -sf $(REVINCLUDEDIR)/$$file $(INCLUDEDIR)/; \
	done
	makedepend $(CFLAGS) $(INC) $(CPPFILES) -f- > Makefile.depend 2>/dev/null

depend: 
	rm Makefile.depend
	make Makefile.depend

tags:
	etags `find -type f -regex ".*\.[hc]p?p?"` 

clean:
	rm -f $(LIB) $(LIB_OPT) *.o Makefile.depend
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done
	find $(INCLUDEDIR) -type l -exec rm \{\} \;

todo:
	grep -ni "Todo"  $(CPPFILES) $(HFILES)
	grep -ni "Fixme"  $(CPPFILES) $(HFILES)


include Makefile.depend
