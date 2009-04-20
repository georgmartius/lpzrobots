/*
 * Generation.cpp
 *
 *  Created on: 20.04.2009
 *      Author: robot12
 */

#include "Generation.h"

Generation::Generation() {
	// TODO Auto-generated constructor stub
}

Generation::Generation(Generation* oldGeneration) {
	// TODO Auto-generated constructor stub

	oldGeneration->update();
	oldGeneration->kill();
	m_SizeOfGeneration = oldGeneration->getFutureGenerationSize();
	recombinate(oldGeneration);
}

Generation::Generation(int size) {
	// TODO Auto-generated constructor stub
}

Generation::~Generation() {
	// TODO Auto-generated destructor stub
}

void Generation::update() {
	m_KillRate = m_SizeOfGeneration/2;
	m_FutureGenerationSize = m_SizeOfGeneration;
}

void Generation::kill() {
	m_member.sort();
	for(int x=1;x<=m_KillRate;x++) {
		delete (m_member.end()-x);
		m_member.erase(m_member.end()-x);
	}
}

void Generation::recombinate(Generation* oldGeneration) {
	int num;

	// new size must be greater or equal the livingpart of the old generation
	if(m_SizeOfGeneration<oldGeneration.getSize())
		return;	//ERROR

	// old living copy
	num = oldGeneration.getSize();
	for(int x=0;x<num;x++) {
		m_member.push_back(oldGeneration.getIndividual(x));
	}

	// generate new from old
	num = m_SizeOfGeneration - oldGeneration.getSize();
	for(int x=0;x<num;x++) {
		// TODO erster Zufallswert für eines der Individual
		int rand1 = 0;
		// TODO zweiter Zufallswert für eines der Individual
		// rand2 must be different from rand1
		int rand2 = 1;

		m_member.push_back((m_member.begin()+rand1)->recombinateWith(m_member.begin()+rand2));
	}
}
