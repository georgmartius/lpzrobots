/*
 * GenFactory.cpp
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#include "GenFactory.h"

GenFactory::GenFactory(void) {
	// nothing
}

GenFactory::GenFactory(std::string name) {
	m_name = name;
}

GenFactory::~GenFactory(void) {
	// nothing
}

Gen* GenFactory::createGen(Individual* individual, GenKontext* kontext) {
	Gen* gen = new Gen(this,individual);

	IValue* value = kontext->getRandomStrategie()->getRandomValue();
	gen->setValue(value);

	kontext->addGen(gen);
	individual->addGen(gen);

	return gen;
}

Gen* GenFactory::createGen(Individual* individual, GenKontext* kontext, Gen* oldGen) {
	Gen* gen = new Gen(this,individual);

	gen->setValue(oldGen->getValue());

	kontext->addGen(gen);
	individual->addGen(gen);
}

Gen* GenFactory::createGen(Individual* individual, GenKontext* kontext, Gen* oldGen, IValue& mutationFactor) {
	Gen* gen = new Gen(this,individual);
	IValue* value = (*gen->getValue()) * mutationFactor;

	gen->setValue(value);

	kontext->addGen(gen);
	individual->addGen(gen);
}
