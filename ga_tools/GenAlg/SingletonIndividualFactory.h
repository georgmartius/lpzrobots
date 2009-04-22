/*
 * SingletonIndividualFactory.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef SINGLETONINDIVIDUALFACTORY_H_
#define SINGLETONINDIVIDUALFACTORY_H_

#include "types.h"

#include <string>
#include <vector>

#include "Individual.h"
#include "GenFactory.h"

class SingletonIndividualFactory {
public:
	SingletonIndividualFactory(void);
	virtual ~SingletonIndividualFactory(void);

	inline static SingletonIndividualFactory* getInstance(void) {
		if(m_me==NULL)m_me=new SingletonIndividualFactory;
		return m_me;
	}

	inline int getSize(void)const {return m_genTypes.size();}
	inline GenFactory* getGen(int x)const {if(x<getSize())return m_genTypes[x];return NULL;}

	inline void addGen(GenFactory* gen) {m_genTypes.push_back(gen);}

	Individual* createIndividual(void)const;

protected:
	static SingletonIndividualFactory* m_me = NULL;
	std::vector<GenFactory*> m_genTypes;
};

#endif /* SINGLETONINDIVIDUALFACTORY_H_ */
