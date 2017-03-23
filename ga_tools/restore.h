/*
 * restore.h
 *
 *  Created on: 21.10.2009
 *      Author: robot12
 */

#ifndef RESTORE_H_
#define RESTORE_H_

#include <string>
#include <vector>

class Prototype;
class IValue;

struct RESTORE_GA_HEAD {
  union {
    struct{
      int generationNumber;
      bool cleanStrategies;
      int numIndividuals;
      int numGeneration;
      int numGenes;
    };
    char* buffer;
  };
};

struct RESTORE_GA_GENERATION {
  union {
    struct {
      int number;
      int numberIndividuals;
      int size;
      int children;
      /*double q1;
      double q3;
      double w1;
      double w3;
      double min;
      double max;
      double avg;
      double med;
      double best;*/
    };

    char* buffer;
  };

  //std::vector<int> idsOfIndividual;
};

struct RESTORE_GA_INDIVIDUAL {
  //std::string name;

  union {
    struct {
      int ID;
      int numberGenes;
      int parent1;
      int parent2;
      bool mutated;
      bool fitnessCalculated;
      double fitness;
    };

    char* buffer;
  };

  //std::vector<int> idsOfGenes;
};

struct RESTORE_GA_GENE {
  //std::string prototype;

  union {
    struct {
      int ID;
    };

    char* buffer;
  };
};

template<class Typ>
struct RESTORE_GA_TEMPLATE {
  union {
    Typ value;
    char* buffer;
  };
};


#endif /* RESTORE_H_ */
