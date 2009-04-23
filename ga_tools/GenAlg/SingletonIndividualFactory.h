/*
 * SingletonIndividualFactory.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef SINGLETONINDIVIDUALFACTORY_H_
#define SINGLETONINDIVIDUALFACTORY_H_

#include "types.h"

#include <string>
#include <vector>

#include "Individual.h"
#include "SingletonGenFactory.h"
#include "SingletonGenEngine.h"
#include "Generation.h"
#include "Gen.h"

class SingletonIndividualFactory {
public:
	inline static SingletonIndividualFactory* getInstance(void) {
		if(m_factory==0)m_factory = new SingletonIndividualFactory;
		return m_factory;
	}

	inline static void destroyFactory(void) {delete m_factory; m_factory=NULL;}

	// 3 Methodes to create an Individual
	Individual* createIndividual(Generation* generation)const;															// random
	Individual* createIndividual(Generation* generation, Individual* oldIndividual)const;								// copy
	Individual* createIndividual(Generation* generation, Individual* individual1, Individual* individual2)const;		// recombinate

private:
	static SingletonIndividualFactory* m_factory = 0;

	SingletonIndividualFactory();
	virtual ~SingletonIndividualFactory();
};

#endif /* SINGLETONINDIVIDUALFACTORY_H_ */
