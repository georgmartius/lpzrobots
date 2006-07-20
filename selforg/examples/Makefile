#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES_CT = controllertest
CFILES_CT = $(addsuffix .cpp, $(FILES_CT))
OFILES_CT = $(addsuffix .o, $(FILES_CT))

FILES_FT = feedforwardtest
CFILES_FT = $(addsuffix .cpp, $(FILES_FT))
OFILES_FT = $(addsuffix .o, $(FILES_FT))


# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DUNITTEST -DNDEBUG 

LIBS   = -lm -L../ -lselforg 
INC   = -I../include

cc = g++ $(CFLAGS) $(INC) -c
#cc = g++ $(CFLAGS) -c -O -g3 -pg
CXX = g++ $(CFLAGS) $(INC)  -c

CC = g++ $(CFLAGS) 
#CC = g++ -g3 -pg  

controllertest: Makefile.depend $(OFILES_CT) libselforg
	$(CC) $(OFILES_CT) $(LIBS)  -o controllertest

feedforwardtest: Makefile.depend $(OFILES_FT) libselforg
	$(CC) $(OFILES_FT) $(LIBS)  -o feedforwardtest

libselforg: 
	cd ../ && make


depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES_FT) $(CFILES_CT) -f- > Makefile.depend 2>/dev/null

Makefile.depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES_FT) $(CFILES_CT) -f- > Makefile.depend 2>/dev/null

search:
	find -name "*.[ch]*" -exec grep -Hn "something" {} \;

todo:
	find -name "*.[ch]*" -exec grep -Hni "TODO" {} \;

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f controllertest feedforwardtest *.o Makefile.depend

include Makefile.depend