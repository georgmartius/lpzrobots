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

class Gen {
public:
	Gen(GenPrototyp* prototyp, Individual* owner);
	virtual ~Gen();

	inline void* getValue(void)const {return m_value;}
	inline void* setValue(void* value) {m_value=value;}

	inline GenPrototyp getPrototyp(void)const {return m_prototyp;}
	inline Individual* getIndividual(void)const {return m_owner;}

protected:
	void* m_value;
	GenPrototyp* m_prototyp;
	Individual* m_owner;

private:
	Gen();
};

#endif /* GEN_H_ */
