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

#include "Individual.h"
#include "GenKontext.h"
#include "IValue.h"

class Gen {
public:
	Gen(Individual individual, GenKontext* kontext, std::string name, int id);
	virtual ~Gen(void);

	inline std::string getName(void)const {return m_name;}
	inline IValue* getValue(void)const {return m_value;}
	inline void setValue(IValue* value) {m_value=value;}
	inline Individual* getIndividual(void)const {return m_individual;}
	inline GenKontext* getKontext(void)const {return m_kontext;}
	inline int getID(void)const {return m_ID;}

protected:
	IValue* m_value;
	Individual* m_individual;
	std::string m_name;
	GenKontext* m_kontext;
	int m_ID;

private:
	/**
	 * disable default constructor
	 */
	Gen(void);
};

#endif /* GEN_H_ */
