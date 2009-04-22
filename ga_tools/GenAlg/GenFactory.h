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

#include "Gen.h"
#include "GenKontext.h"
#include "IValue.h"

class GenFactory {
public:
	GenFactory(std::string name);
	~GenFactory(void);

	inline std::string getName(void)const {return m_name;}

	Gen* createGen(Individual* individual, GenKontext* kontext);
	Gen* createGen(Individual* individual, GenKontext* kontext, Gen* oldGen);
	Gen* createGen(Individual* individual, GenKontext* kontext, Gen* oldGen, IValue& mutationFactor);


protected:
	std::string m_name;

private:
	/**
	 * disable default constructor
	 */
	GenFactory(void);
};

#endif /* GENFACTORY_H_ */
