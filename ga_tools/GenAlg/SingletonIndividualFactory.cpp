/*
 * SingletonIndividualFactory.cpp
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#include "SingletonIndividualFactory.h"

SingletonIndividualFactory::SingletonIndividualFactory() {
}

SingletonIndividualFactory::~SingletonIndividualFactory() {
}

Individual* SingletonIndividualFactory::createIndividual(Generation* generation)const {
	Individual ind = new Individual(generation);
	GenPrototyp* prototyp;
	std::vector<GenPrototyp*>* storage;
	Gen* gen;

	storage = SingletonGenEngine::getSetOfGenPrototyps(generation);
	int num = storage->size();
	for(int x=0;x<num;x++) {
		genFactory = storage[x];
		gen = genFactory->createGen(ind,prototyp);
	}
}

Individual* createIndividual(Generation* generation, Individual* oldIndividual)const;								// copy
Individual* createIndividual(Generation* generation, Individual* individual1, Individual* individual2)const;		// recombinate
