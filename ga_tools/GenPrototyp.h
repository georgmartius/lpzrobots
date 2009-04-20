/*
 * GenPrototyp.h
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#ifndef GENPROTOTYP_H_
#define GENPROTOTYP_H_

#include <vector>

#include "Gen.h"
#include "Individual.h"

class GenPrototyp {
public:
	GenPrototyp(RandGen* randGen);
	virtual ~GenPrototyp();

	/**
	 *	This function is for changing the mutationFactor. Is no overloading for this,
	 *	so the mutationFactor is fixed by 0.001.
	 */
	virtual void update(void);

	Gen* createGen(Individual* individual);

	virtual void* getRandomValue() = 0;

protected:
	Typ m_MutationFactor;
	std::vector<Gen*> m_storage;

	RandGen* m_randGen;

private:
	GenPrototyp();
};

#endif /* GENPROTOTYP_H_ */
