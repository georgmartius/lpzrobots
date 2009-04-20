/*
 * GenPrototyp.cpp
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#include "GenPrototyp.h"

GenPrototyp::GenPrototyp() {
	// TODO Auto-generated constructor stub
}

GenPrototyp::GenPrototyp(RandGen* randGen) {
	// TODO Auto-generated constructor stub

	m_randGen = randGen;
}

GenPrototyp::~GenPrototyp() {
	// TODO Auto-generated destructor stub
}

void GenPrototyp::update(void) {
	m_MutationFactor = 0.001;
}

Gen<Typ>* GenPrototyp::createGen(Individual* individual) {
	Gen<Typ>* result = NULL;
	result = new Gen(this,individual);
	result->setValue(getRandomValue());
	m_storage.push_back(result);

	return result;
}
