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

GenPrototyp::~GenPrototyp() {
	// TODO Auto-generated destructor stub
}

void GenPrototyp::update(void) {
	m_MutationFactor = 0.001;
}

Gen<Typ>* GenPrototyp::createGen(Individual* individual) {
	Gen<Typ>* result = NULL;
	result = new Gen<Typ>(this,individual,true);
	m_storage.push_back(result);

	return result;
}
