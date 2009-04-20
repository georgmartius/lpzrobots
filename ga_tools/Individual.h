/*
 * Individual.h
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include "Gen.h"
#include "IndividualPrototyp.h"

#define PTR_TO_GEN void*

class Individual {
public:
	Individual(IndividualPrototyp* prototyp,bool randCreate=false);
	virtual ~Individual();

	inline IndividualPrototyp* getPrototyp(void)const {return m_prototyp;}
	inline double getFitness(void)const {return m_fitness;}
	inline PTR_TO_GEN getGen(int x)const {if(x<m-gene.size())return m_gene[x];return NULL;}

	/**
	 * This function calculate the fitness value for this individual. Good values are near zero.
	 * Bad value are higher oder lower than zero.
	 */
	virtual void calculateFitness(void) = 0;

	Individual* recombinateWith(Individual* individual);

protected:
	std::vector<PTR_TO_GEN> m_gene;
	double m_fitness;
	IndividualPrototyp* m_prototyp;

private:
	Individual();
};

#endif /* INDIVIDUAL_H_ */
