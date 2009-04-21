/*
 * Gen.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef GEN_H_
#define GEN_H_

#include "types.h"

#include <string>

#include "GenFactory.h"
#include "abstractIndividual.h"

template<class Typ>
class Gen {
public:
	Gen(GenFactory* factory, abstractIndividual individual);
	virtual ~Gen(void);

	inline std::string getName(void)const {return m_name;}
	inline Typ* getValue(void)const {return m_value;}
	inline void setValue(Typ* value) {m_value=value;}
	inline void setValue(void* value) {m_valueReferenz=value;}
	inline abstractIndividual* getIndividual(void)const {return m_individual;}
	inline GenFactory getFactory(void)const {return m_creater;}

protected:
	union {
		Typ* m_value;
		void* m_valueReferenz;
	};

	GenFactory* m_creater;
	abstractIndividual* m_individual;
	std::string m_name;

private:
	/**
	 * disable default constructor
	 */
	Gen(void);
};

#endif /* GEN_H_ */
