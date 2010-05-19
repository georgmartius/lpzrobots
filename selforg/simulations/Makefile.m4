# m4 file
ifdef(`DEVEL', 
`define(`DEVORUSER', $1) define(`DEV', $1)'
,
`define(`DEVORUSER', $2) define(`DEV',)'
)
ifdef(`MAC', 
`define(`LINUXORMAC', $1)'
,
`define(`LINUXORMAC', $2)'
)
## File:     Makefile for selforg simulations
## Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
## Date:     Oct 2009

## add files to compile in the conf file
include Makefile.conf

EXEC_OPT = $(EXEC)_opt
EXEC_DBG = $(EXEC)_dbg

CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

SELFORGLIB = selforg
SELFORGLIB_OPT = selforg_opt
SELFORGLIB_DBG = selforg_dbg

LIBS   += -lm DEV(-L$(SELFORG)/) -l$(SELFORGLIB) -lpthread
INC    +=  DEVORUSER(-I$(SELFORG)/include, ) LINUXORMAC( ,-I/opt/local/include)

## -pg for profiling
CBASEFLAGS = -Wall -pipe -Wno-deprecated -pthread -I. $(INC)
CPPFLAGS = $(CBASEFLAGS) -g -O
## DEBUG
CPPFLAGS_DBG = $(CBASEFLAGS) -g
## Optimisation
CPPFLAGS_OPT = $(CBASEFLAGS) -O3 -DUNITTEST -DNDEBUG

CXX = g++

$(EXEC): Makefile.depend $(OFILES) DEV(libselforg)
	$(CXX) $(OFILES) $(LIBS)  -o $(EXEC)

$(EXEC_DBG): CPPFLAGS = $(CPPFLAGS_DBG)
$(EXEC_DBG): SELFORGLIB = $(SELFORGLIB_DBG)
$(EXEC_DBG): Makefile.depend $(OFILES) DEV(libselforg_dbg)
	$(CXX) $(OFILES) $(LIBS)  -o $(EXEC_DBG)


$(EXEC_OPT): CPPFLAGS = $(CPPFLAGS_OPT)
$(EXEC_OPT): SELFORGLIB = $(SELFORGLIB_OPT)
$(EXEC_OPT): Makefile.depend $(OFILES) DEV(libselforg_opt)
	$(CXX) $(OFILES) $(LIBS)  -o $(EXEC_OPT)

DEV(
libselforg: 
	cd $(SELFORG) && $(MAKE) lib

libselforg_dbg: 
	cd $(SELFORG) && $(MAKE) dbg

libselforg_opt: 
	cd $(SELFORG) && $(MAKE) opt
)

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) -f- > Makefile.depend 2>/dev/null

todo:
	find -name "*.[ch]*" -exec grep -Hni "TODO" {} \;

tags: 
	etags $(find -name "*.[ch]")

clean:
	rm -f $(EXEC) $(EXEC_OPT) *.o Makefile.depend

include Makefile.depend
