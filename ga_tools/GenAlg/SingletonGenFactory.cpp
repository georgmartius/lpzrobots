/*
 * SingletonGenFactory.cpp
 *
 *  Created on: 23.04.2009
 *      Author: robot12
 */

#include "SingletonGenFactory.h"

SingletonGenFactory::SingletonGenFactory() {
}

SingletonGenFactory::~SingletonGenFactory() {
}

Gen* SingletonGenFactory::createGen(Individual* individual, GenPrototyp* prototyp)const {
	Gen* gen = new Gen(individual,prototyp->getKontext(),prototyp->getName());
	IValue* value = prototyp->getRandomValue();

	gen->setValue(value);

	return gen;
}

Gen* SingletonGenFactory::createGen(Individual* individual, GenPrototyp* prototyp, Gen* oldGen, bool mutate=false)const {
	Gen* gen = new Gen(individual,prototyp->getKontext(),prototyp->getName());

	if(mutate) {
		IValue* oldValue = oldGen->getValue();
		IValue* mut = oldGen->getKontext()->getMutationFactor();
		IValue* value = (*oldValue)*(*mut);

		gen->setValue(value);
	}
	else {
		IValue* value = oldGen->getValue();

		gen->setValue(value);
	}

	return gen;
}
