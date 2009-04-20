/*
 * Gen.cpp
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#include "Gen.h"

Gen::Gen(GenPrototyp* prototyp,Individual* owner) {
	// TODO Auto-generated constructor stub

	m_prototyp = prototyp;
	m_owner = owner;
	m_value = NULL;
}

Gen::Gen() {
	// TODO Auto-generated constructor stub
}

Gen::~Gen() {
	// TODO Auto-generated destructor stub
}
