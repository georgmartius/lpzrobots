CC          = gcc
O_FILES     = component.o world.o

INCLUDES    = -I ../../utils


all: component.o world.o
       

component.o: component.cpp component.h
	$(CC) $(INCLUDES) -c component.cpp

world.o: world.cpp world.h
	$(CC) $(INCLUDES) -c world.cpp

clean:
	rm -f $(O_FILES)
