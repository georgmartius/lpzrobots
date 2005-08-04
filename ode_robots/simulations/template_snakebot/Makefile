CC          = g++
O_FILES     = world.o snakebot.o main.o component.o matrix.o one2onewiring.o agent.o

EXEC        = start


LIBS        = -ldrawstuff -lode -lopengl32 -lglut32 -lglu32 /lib/resources.RES

INCLUDES    = -I ../../../utils -I ../../agents -I ../../../controller -I ../../robots -I ../../components -I ../.. -I ../../../matrixlib


start: $(O_FILES)
	$(CC) $(O_FILES) $(LIBS) -o $(EXEC)

agent.o: ../../agents/agent.cpp ../../agents/agent.h
	$(CC) $(INCLUDES) -c ../../agents/agent.cpp

matrix.o: ../../../matrixlib/matrix.cpp ../../../matrixlib/matrix.h
	$(CC) $(INCLUDES) -c ../../../matrixlib/matrix.cpp

world.o: ../../components/world.cpp ../../components/world.h
	$(CC) $(INCLUDES) -c ../../components/world.cpp

component.o: ../../components/component.cpp  ../../components/component.h 
	$(CC) $(INCLUDES) -c ../../components/component.cpp

snakebot.o: ../../robots/snakebot.cpp ../../robots/snakebot.h
	$(CC) $(INCLUDES) -c ../../robots/snakebot.cpp

main.o: main.cpp
	$(CC) $(INCLUDES) -c main.cpp

one2onewiring.o: ../../agents/one2onewiring.cpp ../../agents/one2onewiring.h
	$(CC) $(INCLUDES) -c ../../agents/one2onewiring.cpp

clean:
	rm -f $(O_FILES) $(EXEC)
