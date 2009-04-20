/*
 * Individual.cpp
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#include "Individual.h"

Individual::Individual() {
	// TODO Auto-generated constructor stub

}

Individual::Individual(IndividualPrototyp* prototyp,bool randCreate) {
	// TODO Auto-generated constructor stub

	m_prototyp = prototyp;

	if(randCreate) {
		int num = m_prototyp->getNumGenPrototyp();
		for(int x=0;x<num;x++) {
			GenPrototyp* pro = m_prototyp->getGenPrototyp(x);
			pro->createGen(this);
		}
	}
}

Individual::~Individual() {
	// TODO Auto-generated destructor stub
}

Individual* Individual::recombinateWith(Individual* individual) {
}
