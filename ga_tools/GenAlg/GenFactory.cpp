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

GenFactory::GenFactory(std::string name, abstractRandomStrategie* random, abstractMutationFactorOptimizerStrategie* mfOptimizer) {
	m_random = random;
	m_name = name;
	m_mutationFactorOptimizer = mfOptimizer;
}

GenFactory::~GenFactory(void) {
	if(m_random!=NULL) {
		delete m_random;
		m_random = NULL;
	}

	if(m_mutationFactorOptimizer!=NULL) {
		delete m_mutationFactorOptimizer;
		m_mutationFactorOptimizer = NULL;
	}
}

void GenFactory::updateGen(abstractGen* gen) {
	m_changed = true;
}

void GenFactory::removeGen(abstractGen* gen) {
	m_changed = true;
	m_storage.erase(m_storage.find(gen));
}

abstractGen* GenFactory::createGen(abstractIndividual* individual) {
	abstractGen* gen = new abstractGen(this,individual);
	UNKNOWN_DATA_TYP value = m_random->getRandomValue();
	gen->setValue(value);
	m_storage.push_back(gen);

	return gen;
}

void GenFactory::update(void) {
	if(m_changed)
		m_mutationFactor = m_mutationFactorOptimizer->updateMutationFactor(m_storage);
}
