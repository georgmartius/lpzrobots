/*
 * SingletonIndividualFactory.cpp
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#include "SingletonIndividualFactory.h"

SingletonIndividualFactory::SingletonIndividualFactory() {
	// nothing
}

SingletonIndividualFactory::~SingletonIndividualFactory() {
	// nothing
}

Individual* SingletonIndividualFactory::createIndividual(Generation* generation)const {
	Individual ind = new Individual(generation);
	GenPrototyp* prototyp;
	std::vector<GenPrototyp*>* storage;

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps(generation);
	int num = storage->size();
	for(int x=0;x<num;x++) {
		prototyp = storage[x];
		SingletonGenFactory::getInstance()->createGen(ind,prototyp);
	}

	return ind;
}

Individual* createIndividual(Generation* generation, Individual* oldIndividual)const {
	Individual ind = new Individual(generation);
	GenPrototyp* prototyp;
	std::vector<GenPrototyp*>* storage;
	Gen* gen;

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps(generation);
	int num = storage->size();
	for(int x=0;x<num;x++) {
		prototyp = storage[x];

		int num2 = oldIndividual->getSize();
		for( int y=0;y<num2;y++) {
			gen = oldIndividual->getGen(y);
			if(gen->getID()==prototyp->getID()) {
				SingletonGenFactory::getInstance()->createGen(ind,prototyp,gen);
				break;
			}
		}
	}

	return ind;
}

Individual* createIndividual(Generation* generation, Individual* individual1, Individual* individual2,randGen* random)const {
	Individual ind = new Individual(generation);
	GenPrototyp* prototyp;
	std::vector<GenPrototyp*>* storage;
	Gen* gen;
	int r1,r2;
	Individual* ind;

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps(generation);
	int num = storage->size();
	for(int x=0;x<num;x++) {
		prototyp = storage[x];
		r1 = ((int)random->getRandomvalue())%2;
		r2 = ((int)random->getRandomvalue())%1000;
		ind = r1==0?individual1:individual2;

		int num2 = oldIndividual->getSize();
		for( int y=0;y<num2;y++) {
			gen = ind->getGen(y);
			if(gen->getID()==prototyp->getID()) {
				if(r2<prototyp->getKontext()->getMutationProbability()) {
					SingletonGenFactory::getInstance()->createGen(ind,prototyp,gen,true);
					break;
				}
				else {
					SingletonGenFactory::getInstance()->createGen(ind,prototyp,gen,false);
					break;
				}
			}
		}
	}

	return ind;
}
