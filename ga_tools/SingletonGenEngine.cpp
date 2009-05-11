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
 *   Revision 1.3  2009-05-11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/07 14:47:47  robot12
 *   some comments
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.4  2009/04/30 11:35:53  robot12
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

#include "GenPrototype.h"
#include "Generation.h"
#include "Individual.h"
#include "ISelectStrategy.h"
#include "IGenerationSizeStrategy.h"
#include "Gen.h"
#include "GenContext.h"
#include "IFitnessStrategy.h"
#include "SingletonIndividualFactory.h"

SingletonGenEngine* SingletonGenEngine::m_engine = 0;

SingletonGenEngine::SingletonGenEngine() {
	m_actualGeneration = 0;
}

SingletonGenEngine::~SingletonGenEngine() {
	std::vector<GenPrototype*>::iterator iterPro;
	std::vector<Generation*>::iterator iterGener;
	std::vector<Individual*>::iterator iterInd;
	std::vector<Gen*>::iterator iterGen;

	while(m_prototype.size()>0) {
		iterPro = m_prototype.begin();
		delete (*iterPro);
		m_prototype.erase(iterPro);
	}
	m_prototype.clear();

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
	std::vector<Gen*>::iterator iterGen;
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
	GenPrototype* prototype;
	for(unsigned int a=0;a<m_prototype.size();a++) {
		prototype = m_prototype[a];
		context = new GenContext(prototype);
		prototype->insertContext(first,context);
	}

	// generate the random individuals
	Individual* ind;
	for(int x=0;x<startSize;x++) {
		ind = SingletonIndividualFactory::getInstance()->createIndividual();
		first->addIndividual(ind);
	}
}

void SingletonGenEngine::prepareNextGeneration(int size, int killRate) {
	// generate the next generation
	Generation* next = new Generation(m_actualGeneration+1,size,killRate);
	addGeneration(next);
	m_actualGeneration++;

	// generate the next GenContext
	int num = m_prototype.size();
	GenContext* context;
	GenPrototype* prototype;
	for(int x=0;x<num;x++) {
		prototype = m_prototype[x];
		context = new GenContext(prototype);
		prototype->insertContext(next,context);
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
		m_generation[m_actualGeneration]->crossOver(random);*/

		// Abbruchkriterium fehlt noch!!!
		// TODO
	}
}

void SingletonGenEngine::select(bool createNextGeneration) {
	if(createNextGeneration)
		prepareNextGeneration(m_generationSizeStrategy->calcGenerationSize(getActualGeneration()),getActualGeneration()->getKillRate());

	m_selectStrategy->select(m_generation[m_actualGeneration-1],m_generation[m_actualGeneration]);
}

void SingletonGenEngine::crosover(RandGen* random) {
	m_generation[m_actualGeneration]->crosover(random);
}

double SingletonGenEngine::getFitness(const Individual* individual) {
	return m_fitnessStrategy->getFitness(individual);
}

Individual* SingletonGenEngine::getBestIndividual(void) {
	const std::vector<Individual*>& storage = getActualGeneration()->getAllIndividual();
	Individual* result = storage[0];
	double value = result->getFitness();
	double test;
	int num = storage.size();

	for(int x=1;x<num;x++) {
		test = storage[x]->getFitness();
		if(test<value) {
			value = test;
			result = storage[x];
		}
	}

	return result;
}
