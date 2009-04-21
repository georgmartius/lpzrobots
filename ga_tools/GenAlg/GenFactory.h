/*
 * GenFactory.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef GENFACTORY_H_
#define GENFACTORY_H_

#include "types.h"

#include <string>
#include <vector>

#include "abstractRandomStrategie.h"
#include "abstractMutationFactorOptimizerStrategie.h"
#include "Gen.h"
#include "abstractIndividual.h"

class GenFactory {
public:
	GenFactory(std::string name, abstractRandomStrategie* random, abstractMutationFactorOptimizerStrategie* mfOptimizer);
	~GenFactory(void);

	inline std::string getName(void)const {return m_name;}
	inline UNKNOWN_DATA_TYP getMutationFactor(void) {update();return m_mutationFactor;}

	void updateGen(abstractGen* gen);
	void removeGen(abstractGen* gen);
	abstractGen* createGen(abstractIndividual* individual);

protected:
	std::string m_name;
	abstractRandomStrategie* m_random;
	abstractMutationFactorOptimizerStrategie* m_mutationFactorOptimizer;
	UNKNOWN_DATA_TYP_PTR m_mutationFactor;
	std::vector<abstractGen*> m_storage;
	bool m_changed;

	void update(void);

private:
	/**
	 * disable default constructor
	 */
	GenFactory(void);
};

#endif /* GENFACTORY_H_ */
