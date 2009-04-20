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

#define PTR_TO_INDIVIDUAL void*
#define PTR_TO_GEN_PROTOTYP void*

class IndividualPrototyp {
public:
	IndividualPrototyp();
	virtual ~IndividualPrototyp();

	Individual* createRandIndividual(void);

	inline PTR_TO_GEN_PROTOTYP getGenPrototyp(int x)const {if(x<m_structur.size())return m_structur[x];return NULL;}
	inline PTR_TO_INDIVIDUAL getIndividual(int x)const {if(x<m_storage.size())return m_storage[x];return NULL;}
	inline int getNumGenPrototyp(void)const {return m_structur.size();}
	inline int getNumIndividual(void)const {return m_storage.size();}

protected:
	std::vector<PTR_TO_GEN_PROTOTYP> m_structur;
	std::vector<PTR_TO_INDIVIDUAL> m_storage;
};

#endif /* INDIVIDUALPROTOTYP_H_ */
