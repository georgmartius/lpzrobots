/*
 * SingletonIndividualFactory.cpp
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#include "SingletonIndividualFactory.h"

SingletonIndividualFactory::SingletonIndividualFactory() {
	// TODO Auto-generated constructor stub

}

SingletonIndividualFactory::~SingletonIndividualFactory() {
	// TODO Auto-generated destructor stub
}

Individual* SingletonIndividualFactory::createIndividual(void)const {
	Individual ind = new Individual();
	GenFactory* gen;

	int num = getSize();
	for(int x=0;x<num;x++) {
		gen = getGen(x);
		gen->createGen(ind);
	}
}
