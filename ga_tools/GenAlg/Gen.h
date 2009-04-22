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
#include "Individual.h"
#include "GenKontext.h"
#include "IValue.h"

class Gen {
public:
	Gen(GenFactory* factory, Individual individual, GenKontext* kontext);
	virtual ~Gen(void);

	inline std::string getName(void)const {return m_name;}
	inline IValue* getValue(void)const {return m_value;}
	inline void setValue(IValue* value) {m_value=value;}
	inline Individual* getIndividual(void)const {return m_individual;}
	inline GenFactory getFactory(void)const {return m_creater;}
	inline GenKontext* getKontext(void)const {return m_kontext;}

protected:
	IValue* m_value;

	GenFactory* m_creater;
	Individual* m_individual;
	std::string m_name;
	genKontext* m_kontext;

private:
	/**
	 * disable default constructor
	 */
	Gen(void);
};

#endif /* GEN_H_ */
