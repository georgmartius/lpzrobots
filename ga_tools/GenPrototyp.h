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

template<class Typ>

class GenPrototyp {
public:
	GenPrototyp();
	virtual ~GenPrototyp();

	/**
	 *	This function is for changing the mutationFactor. Is no overloading for this,
	 *	so the mutationFactor is fixed by 0.001.
	 */
	virtual void update(void);

	Gen<Typ>* createGen(Individual* individual);

protected:
	Typ m_MutationFactor;
	std::vector<Gen<Typ>*> m_storage;
};

#endif /* GENPROTOTYP_H_ */
