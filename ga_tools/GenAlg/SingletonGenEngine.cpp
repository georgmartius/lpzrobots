/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 *                                                                         *
 *   Informative Beschreibung der Klasse                                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-04-30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.3  2009/04/29 14:32:29  robot12
 *   some implements... Part4
 *
 *   Revision 1.2  2009/04/29 11:36:41  robot12
 *   some implements... Part3
 *
 *   Revision 1.1  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "SingletonGenEngine.h"

SingletonGenEngine::SingletonGenEngine() {
	m_actualGeneration = 0;
}

SingletonGenEngine::~SingletonGenEngine() {
	std::vector<GenPrototyp*>::iterator iterPro;
	std::vector<Generation*>::iterator iterGener;
	std::vector<Individual*>::iterator iterInd;
	std::vector<Gen*>::iterator iterGen;

	while(m_prototyp.size()>0) {
		iterPro = m_prototyp.begin();
		delete (*iterPro);
		m_prototyp.erase(iterPro);
	}
	m_prototyp.clear();

	while(m_generation.size()>0) {
		iterGener = m_generation.begin();
		delete (*iterGener);
		m_generation.erase(iterGener);
	}
	m_generation.clear();

	while(m_individual.size()>0) {
		iterInd = m_individual.begin();
		delete (*iterInd);
		m_individual.erase(iterInd);
	}
	m_individual.clear();

	while(m_gen.size()>0) {
		iterGen = m_gen.begin();
		delete (*iterGen);
		m_gen.erase(iterGen);
	}
	m_gen.clear();
}

void SingletonGenEngine::generateFirstGeneration(int startSize, int startKillRate) {
	// clean the generations
	std::vector<Generation*>::iterator iterGener;
	while(m_generation.size()>0) {
		iterGener = m_generation.begin();
		delete (*iterGener);
		m_generation.erase(iterGener);
	}
	m_generation.clear();

	// clean the individuals
	std::vector<Individual*>::iterator iterInd;
	while(m_individual.size()>0) {
		iterInd = m_individual.begin();
		delete (*iterInd);
		m_individual.erase(iterInd);
	}
	m_individual.clear();

	// clean the gens
	while(m_gen.size()>0) {
		iterGen = m_gen.begin();
		delete (*iterGen);
		m_gen.erase(iterGen);
	}
	m_gen.clear();

	// generate the first generation
	Generation* first = new Generation(0,startSize,startKillRate);
	addGeneration(first);
	m_actualGeneration=0;

	// generate the first contexts
	GenContext* context;
	GenPrototyp* prototyp;
	for(int a=0;a<m_prototyp.size();a++) {
		prototyp = m_prototyp[x];
		context = new GenContext(prototyp);
		prototyp->insertContext(first,context);
	}

	// generate the random individuals
	Individual* ind;
	for(int x=0;x<startSize;x++) {
		ind = SingletonIndividualFactory::getInstance()->createIndividual("Ind " + new std::string(x));
		first->addIndividual(ind);
	}
}

void SingletonGenEngine::prepareNextGeneration(int size, int killRate) {
	// generate the next generation
	Generation* next = new Generation(m_actualGeneration+1,size,killRate);
	addGeneration(next);
	m_actualGeneration++;

	// generate the next GenContext
	int num = m_prototyp.size();
	GenContext* context;
	GenPrototyp* prototyp;
	for(int x=0;x<num;x++) {
		prototyp = m_prototyp[x];
		context = new GenContext(prototyp);
		prototyp->insertContext(next,context);
	}
}

void SingletonGenEngine::runGenAlg(int startSize, int startKillRate, int numGeneration, RandGen* random) {
	// create first generation
	generateFirstGeneration(startSize,startKillRate);

	// generate the other generations
	for(int x=0;x<numGeneration;x++) {
		select();
		crosover(random);

		/*prepareNextGeneration();
		m_generation[m_actualGeneration-1]->select(m_generation[m_actualGeneration]);
		m_generation[m_actualGeneration]->crosover(random);*/

		// TODO Abbruchkriterium fehlt noch!!!
	}
}
