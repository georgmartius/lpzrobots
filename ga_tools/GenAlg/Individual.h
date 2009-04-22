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

class Individual {
public:
	Individual();
	virtual ~Individual();

	inline int getSize(void)const {return m_gene.size();
	inline Gen* getGen(int x)const {if(x<getSize())return m_gene[x];return NULL;}
	inline void addGen(Gen* gen) {m_gene.push_back(gen);}
	inline void removeGen(Gen* gen) {m_gene.erase(m_gene.find(gen));}
	inline void removeGen(int x) {if(x<getSize())m_gene.erase(m_gene.begin()+x);}
	inline double getFitness(void) {return m_fitnessStrategie->getFitness(m_gene);}

protected:
	std::vector<Gen*> m_gene;
	IFitnessStrategie* m_fitnessStrategie;

private:
	/**
	 * disable the default constructor
	 */
	Individual();
};

#endif /* INDIVIDUAL_H_ */
