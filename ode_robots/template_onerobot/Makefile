#File:     Makefile for Matrix libary
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2005

FILES = main_template
CFILES = $(addsuffix .cpp, $(FILES))
OFILES = $(addsuffix .o, $(FILES))

EXEC = start

# -pg for profiling
CFLAGS = -Wall -I. -g
## Optimisation
#CFLAGS = -Wall -I. -O3 -DNDEBUG

LIBHOME=/home/fhesse/projects/lpzrobots
#LIBHOME2=/home/fhesse/c++/
MATRIX=matrix
#MATRIX=matrix_debug

LIBS   = -lm -L$(LIBHOME)/matrixlib/  -l$(MATRIX) -lode -ldrawstuff -lGL -lGLU -lglut
INC   = -I$(LIBHOME)/matrixlib/ -I$(LIBHOME)/controller -I.. #-I$(LIBHOME2)/gnuplot/

cc = g++ $(CFLAGS) $(INC) -c
#cc = g++ $(CFLAGS) -c -O -g3 -pg
CXX = g++ $(CFLAGS) $(INC)  -c

CC = g++ $(CFLAGS) 
#CC = g++ -g3 -pg  

$(EXEC): $(OFILES)
	$(CC) $(OFILES) $(LIBS) $(INC) -o $(EXEC)

depend: 
	makedepend $(CFLAGS) $(INC) $(CFILES) 2>/dev/null

tags: 
	etags `find -name "*.[ch]"` 

clean:
	rm -f $(EXEC) *.o Makefile.depend

# DO NOT DELETE

main_template.o: /usr/include/stdio.h /usr/include/features.h
main_template.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
main_template.o: /usr/lib/gcc-lib/i486-suse-linux/3.2.2/include/stddef.h
main_template.o: /usr/include/bits/types.h /usr/include/libio.h
main_template.o: /usr/include/_G_config.h /usr/include/wchar.h
main_template.o: /usr/include/bits/wchar.h /usr/include/gconv.h
main_template.o: /usr/lib/gcc-lib/i486-suse-linux/3.2.2/include/stdarg.h
main_template.o: /usr/include/bits/stdio_lim.h
main_template.o: /usr/include/bits/sys_errlist.h /usr/include/math.h
main_template.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
main_template.o: /usr/include/bits/mathcalls.h /usr/include/signal.h
main_template.o: /usr/include/bits/sigset.h /usr/include/bits/signum.h
main_template.o: config.h
main_template.o: /home/fhesse/projects/lpzrobots/controller/configurable.h
main_template.o: /home/fhesse/projects/lpzrobots/controller/invertnchannelcontroller.h
main_template.o: /home/fhesse/projects/lpzrobots/controller/invertcontroller.h
main_template.o: /home/fhesse/projects/lpzrobots/controller/abstractcontroller.h
main_template.o: /usr/include/stdlib.h /usr/include/string.h
main_template.o: /home/fhesse/projects/lpzrobots/matrixlib/matrix.h
main_template.o: /usr/include/assert.h
main_template.o: /home/fhesse/projects/lpzrobots/controller/invertnchannelcontroller.hpp
main_template.o: oplayground.h abstractobstacle.h ../vehicle_2wheels.h
