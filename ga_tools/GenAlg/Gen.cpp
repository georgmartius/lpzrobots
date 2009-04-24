/*
 * Gen.cpp
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#include "Gen.h"

Gen::Gen(void) {
	// nothing
}

Gen::Gen(abstractIndividual individual, GenKontext* kontext, std::string name, int id) {
	m_name = name;
	m_individual = individual;
	m_value = NULL;
	m_kontext = kontext;
	m_ID = id;

	m_individual->addGen(this);
	m_kontext->addGen(this);
}

Gen::~Gen(void) {
	m_individual = NULL;	// is the deleter

	delete m_value;
	m_value = NULL;

	m_kontext->removeGen(this);
	m_individual->removeGen(this);
}
