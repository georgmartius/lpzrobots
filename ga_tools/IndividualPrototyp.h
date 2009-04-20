/*
 * IndividualPrototyp.h
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#ifndef INDIVIDUALPROTOTYP_H_
#define INDIVIDUALPROTOTYP_H_

#include <vector>

#include "Individual.h"
#include "GenPrototyp.h"

class IndividualPrototyp {
public:
	IndividualPrototyp();
	virtual ~IndividualPrototyp();

	Individual* createRandIndividual(void);

	inline GenPrototyp* getGenPrototyp(int x)const {if(x<m_structur.size())return m_structur[x];return NULL;}
	inline Individual* getIndividual(int x)const {if(x<m_storage.size())return m_storage[x];return NULL;}
	inline int getNumGenPrototyp(void)const {return m_structur.size();}
	inline int getNumIndividual(void)const {return m_storage.size();}

protected:
	std::vector<GenPrototyp*> m_structur;
	std::vector<Individual*> m_storage;
};

#endif /* INDIVIDUALPROTOTYP_H_ */
