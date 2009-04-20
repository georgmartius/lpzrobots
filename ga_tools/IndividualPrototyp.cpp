/*
 * IndividualPrototyp.cpp
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#include "IndividualPrototyp.h"

IndividualPrototyp::IndividualPrototyp() {
	// TODO Auto-generated constructor stub

}

IndividualPrototyp::~IndividualPrototyp() {
	// TODO Auto-generated destructor stub
}

Individual* IndividualPrototyp::createRandIndividual(void) {
	Individual* resultIndividual = NULL;

	resultIndividual = new Individual(this);
}
