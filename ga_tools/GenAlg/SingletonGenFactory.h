/*
 * SingletonGenFactory.h
 *
 *  Created on: 23.04.2009
 *      Author: robot12
 */

#ifndef SINGLETONGENFACTORY_H_
#define SINGLETONGENFACTORY_H_

#include "types.h"

#include "Gen.h"
#include "GenKontext.h"
#include "GenPrototyp.h"
#include "Individual.h"

class SingletonGenFactory {
public:
	inline static SingletonGenFactory* getInstance(void) {if(m_factory==0)m_factory = new SingletonGenFactory();return m_factory;}
	inline static void destroyGenFactory(void) {delete m_factory;m_factory=0;}

	// 3 Methodes to create an Gen
	Gen* createGen(Individual* individual, GenPrototyp* prototyp)const;													// random
	Gen* createGen(Individual* individual, GenPrototyp* prototyp, Gen* oldGen, bool mutate=false)const;				// copy + mutation

private:
	static SingletonGenFactory* m_factory = 0;

	/**
	 * disable the default constructor
	 */
	SingletonGenFactory();

	/**
	 * disable destructor
	 */
	virtual ~SingletonGenFactory();
};

#endif /* SINGLETONGENFACTORY_H_ */
