/*
 * Gen.cpp
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#include "Gen.h"

Gen::Gen(GenPrototyp* prototyp,Individual* owner,bool randCreate) {
	// TODO Auto-generated constructor stub

	if(randCreate)
		m_value = rand();

	m_prototyp = prototyp;
	m_owner = owner;
}

Gen::Gen() {
	// TODO Auto-generated constructor stub

	m_value=rand();
}

Gen::~Gen() {
	// TODO Auto-generated destructor stub
}
