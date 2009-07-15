/*
 * TemplateCycledGaSimulationFitnessStrategy.cpp
 *
 *  Created on: 07.07.2009
 *      Author: robot12
 */

#include "TemplateCycledGaSimulationFitnessStrategy.h"

#include <ga_tools/Individual.h>

TemplateCycledGaSimulationFitnessStrategy::TemplateCycledGaSimulationFitnessStrategy() {
	// nothing
}

TemplateCycledGaSimulationFitnessStrategy::~TemplateCycledGaSimulationFitnessStrategy() {
	// nothing
}

double TemplateCycledGaSimulationFitnessStrategy::getFitness(const Individual* individual) {
	double* temp = m_storage[individual->getID()];
	return *(m_storage[individual->getID()]);
}
