/*
 * Individual.cpp
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#include "Individual.h"

Individual::Individual() {
	// nothing
}

Individual::Individual(Generation* generation) {
	m_fitnessIsCalculated = false;
	m_fitness = 10000000000000.0;
	m_generation = generation;

	generation->addIndividual(this);
}

Individual::~Individual() {
	m_generation = NULL;
}
