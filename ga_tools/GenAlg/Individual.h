/*
 * Individual.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include "types.h"

#include <vector>

#include "Gen.h"
#include "IFitnessStrategie.h"
#include "Generation.h"

class Individual {
public:
	Individual(Generation* generation);
	virtual ~Individual();

	inline int getSize(void)const {return m_gene.size();}
	inline Gen* getGen(int x)const {if(x<getSize())return m_gene[x];return NULL;}
	inline void addGen(Gen* gen) {m_gene.push_back(gen);}
	inline void removeGen(Gen* gen) {m_gene.erase(m_gene.find(gen));}
	inline void removeGen(int x) {if(x<getSize())m_gene.erase(m_gene.begin()+x);}
	inline double getFitness(void) {if(!m_fitnessIsCalculated && m_fitnessStrategie!=0){m_fitness = m_fitnessStrategie->getFitness(m_gene);m_fitnessIsCalculated=true;}return m_fitness;}
	inline Generation* getGeneration(void)const {return m_generation;}

	inline static void setFitnessStrategie(IFitnessStrategie* strategie) {m_fitnessStrategie = strategie;}

protected:
	std::vector<Gen*> m_gene;
	static IFitnessStrategie* m_fitnessStrategie = 0;
	Generation* m_generation;
	bool m_fitnessIsCalculated;
	double m_fitness;

private:
	/**
	 * disable the default constructor
	 */
	Individual();
};

#endif /* INDIVIDUAL_H_ */
