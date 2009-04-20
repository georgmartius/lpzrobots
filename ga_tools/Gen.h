/*
 * Gen.h
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#ifndef GEN_H_
#define GEN_H_

#include "GenPrototyp.h"
#include "Individual.h"

template<class Typ>

class Gen {
public:
	Gen(GenPrototyp<Typ>* prototyp,Individual* owner,bool randCreate=false);
	virtual ~Gen();

	inline Typ getValue(void)const {return m_value;}
	inline Typ setValue(Typ value) {m_value=value;}

	inline GenPrototyp<Typ> getPrototyp(void)const {return m_prototyp;}
	inline Individual* getIndividual(void)const {return m_owner;}

	virtual static Typ rand()=0;

protected:
	Typ m_value;
	GenPrototyp<Typ>* m_prototyp;
	Individual* m_owner;

private:
	Gen();
};

#endif /* GEN_H_ */
