CXXFILES = osgscene.cpp
LIBS     = -losgProducer -lProducer -losgText -losgUtil -losg -lGL -lGLU
EXEC = osgscene
CXX = g++ -Wall $(INC)
# INC = -I$(OPENSCENEGRAPH_INC_DIR) -I$(PRODUCER_INC_DIR) -I$(OPENTHREADS_INC_DIR)
# LIB = -L$(OPENSCENEGRAPH_LIB_DIR) -L$(PRODUCER_LIB_DIR) -L$(OPENTHREADS_LIB_DIR)
CPP = g++

OBJ = osgscene.o osgprimitive.o

all: $(OBJ)
	$(CPP) $(OBJ) -o $(EXEC) $(LIB) $(LIBS)



