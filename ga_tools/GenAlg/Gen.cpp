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

Gen::Gen(GenFactory* factory, abstractIndividual individual, GenKontext* kontext) {
	m_name = factory->getName();
	m_creater = factory;
	m_individual = individual;
	m_value = NULL;
	m_kontext = kontext;
}

Gen::~Gen(void) {
	m_individual = NULL;	// is the deleter

	delete m_value;
	m_value = NULL;

	m_kontext->removeGen(this);
	m_individual->removeGen(this);
}
