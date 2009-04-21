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

Gen::Gen(GenFactory* factory, abstractIndividual individual) {
	m_name = factory->getName();
	m_creater = factory;
	m_individual = individual;
	m_valueReferenz = NULL;
}

Gen::~Gen(void) {
	m_creater->removeGen(this);
	m_creater = NULL;
	m_individual = NULL;	// is the deleter
}
