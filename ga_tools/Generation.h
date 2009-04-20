/*
 * Generation.h
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#ifndef GENERATION_H_
#define GENERATION_H_

#include <list>

#include "IndividualPrototyp.h"

class Generation {
public:
	Generation(Generation* oldGeneration);
	Generation(int size);
	virtual ~Generation();

	/**
	 * This function is for changing the KillRate and the FutureGenerationSize. Is no overloading for this,
	 * so the KillRate is the half of SizeOfGeneration and FutureGenerationSize is the same like SizeOfGeneration.
	 */
	virtual void update(void);

	/**
	 * This function deleted all Individual which are to bade.
	 */
	virtual void kill(void);

	/**
	 * This Function create new Individual from the living rest of the old Generation.
	 */
	virtual void recombinate(Generation* oldGeneration);

	inline int getSizeOfGeneration(void)const {return m_SizeOfGeneration;}
	inline int getKillRate(void)const {return m_KillRate;}
	inline int getFutureGenerationSize(void)const {return m_FutureGenerationSize;}
	inline int getSize(void)const {return m_member.size();}
	inline Individual* getIndividual(int x)const {if(x<m_member.size())return m_member[x];return NULL;}

protected:
	std::list<Individual*> m_member;
	int m_SizeOfGeneration;
	int m_KillRate;
	int m_FutureGenerationSize;

private:
	Generation();
};

#endif /* GENERATION_H_ */
