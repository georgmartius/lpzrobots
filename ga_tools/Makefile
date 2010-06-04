# TODO: update this makefile! it will not work properly with -j 4
#File:     Makefile for ga_tools package
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

PACKAGE_NAME=ga_tools
LPZROBOTS=..
PACKAGE_PATH=.
SELFORG=../selforg
INCLUDEDIR=include/$(PACKAGE_NAME)
REVINCLUDEDIR=../..
LIB=lib$(PACKAGE_NAME).a
LIB_DBG=lib$(PACKAGE_NAME)_dbg.a
LIB_OPT=lib$(PACKAGE_NAME)_opt.a
LIB_PROF=lib$(PACKAGE_NAME)_prof.a

include Makefile.conf

find_files = $(wildcard $(dir)/*.cpp)
CPPFILES   := $(foreach dir,$(dirs),$(find_files))
OFILES     := $(patsubst %.cpp,%.o, $(CPPFILES))
find_h_files = $(wildcard $(dir)/*.h)
HFILES   := $(foreach dir,$(dirs),$(find_h_files))

# -pg for profiling
CPPBASEFLAG = -Wall -Wno-deprecated -pipe -I. -pthread $(INC) 
CPPFLAGS = $(CPPBASEFLAG) -g -O

## Debug (no optimization)
CPPFLAGS_DBG = $(CPPBASEFLAG) -g

## Optimisation
CPPFLAGS_OPT = $(CPPBASEFLAG) -O3 -DNDEBUG

## Optimisation
CPPFLAGS_PROF = $(CPPBASEFLAG)-O -DQPROF

# used for single file compilation
CXX = g++

# used for lib-packing 
AR = ar -rs

.PHONY: lib opt clean distclean clean-all todo depend tags

lib: $(UTILS) libselforg $(LIB)

opt: $(UTILS) libselforg_opt $(LIB_OPT)

prof: $(UTILS) libselforg $(LIB_PROF)

$(LIB): Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB) $(OFILES)

$(LIB_DBG): CPPFLAGS = $(CPPFLAGS_DBG)
$(LIB_DBG): Makefile.depend $(SELFORG)/libselforg_dbg.a $(OFILES)
	$(AR) $(LIB_DBG) $(OFILES)

$(LIB_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(LIB_OPT): Makefile.depend $(SELFORG)/libselforg_opt.a $(OFILES)
	$(AR) $(LIB_OPT) $(OFILES)

$(LIB_PROF): CPPFLAGS = $(CPPFLAGS_PROF)
$(LIB_PROF): Makefile.depend $(SELFORG)/libselforg.a $(OFILES)
	$(AR) $(LIB_PROF) $(OFILES)

libselforg:	
	+cd $(SELFORG) && make libselforg.a		

libselforg_opt:	
	+cd $(SELFORG) && make libselforg_opt.a		


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

##!'search'           		ask for a search string and scans all tex files
search:
	echo -n "search rexp: "; read F && grep -in "$$F" ${CPPFILES} ${HFILES}


distclean: clean-all
clean-all: clean
	rm -f $(LIB) $(LIB_DBG) $(LIB_OPT) $(LIB_PROF)

clean:
	rm -f Makefile.depend
	find $(INCLUDEDIR) -type l -exec rm \{\} \;
	for dir in $(dirs); do \
		rm -f $$dir/*.o; \
	done

install_prefix.conf:
	@echo "create install_prefix.conf"
	@echo '// for system installations (installation type:user) the macro PREFIX should be defined:' > install_prefix.conf
	@echo '// e.g. #define PREFIX "/usr/local"'  >> install_prefix.conf

todo:
	grep -ni "Todo" $(CPPFILES) $(HFILES)
	grep -ni "Fixme" $(CPPFILES) $(HFILES)

FIND=`cat tofind`
find:
	@[ -n "$(FIND)" ] &&  grep -ni "$(FIND)"  $(CPPFILES) $(HFILES)


-include Makefile.depend
