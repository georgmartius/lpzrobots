/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   This class is the engine of the gen. alg. It control all elements     *
 *   inside, prepare the next steps and hold the alg. on running.          *
 *                                                                         *
 *   $Log$
 *   Revision 1.10  2009-07-28 13:19:55  robot12
 *   add some clean ups
 *
 *   Revision 1.9  2009/07/21 08:39:01  robot12
 *   rename "crosover" to crossover
 *
 *   Revision 1.8  2009/07/15 12:53:36  robot12
 *   some bugfix's and new functions
 *
 *   Revision 1.7  2009/07/06 15:06:35  robot12
 *   bugfix
 *
 *   Revision 1.6  2009/07/02 15:24:53  robot12
 *   update and add new class InvertedFitnessStrategy
 *
 *   Revision 1.5  2009/06/30 10:30:04  robot12
 *   GenEngine finish and some comments added
 *
 *   Revision 1.4  2009/05/12 13:29:25  robot12
 *   some new function
 *   -> toString methodes
 *
 *   Revision 1.3  2009/05/11 14:08:51  robot12
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

#include <selforg/plotoptionengine.h>
#include <list>
#include <selforg/inspectableproxy.h>

#include "GenPrototype.h"
#include "Generation.h"
#include "Individual.h"
#include "ISelectStrategy.h"
#include "IGenerationSizeStrategy.h"
#include "Gen.h"
#include "GenContext.h"
#include "IFitnessStrategy.h"
#include "SingletonIndividualFactory.h"
#include "SingletonGenFactory.h"

SingletonGenEngine* SingletonGenEngine::m_engine = 0;

SingletonGenEngine::SingletonGenEngine() {
	m_actualGeneration = 0;
	m_cleanStrategies = false;
	m_selectStrategy = 0;
	m_generationSizeStrategy = 0;
	m_fitnessStrategy = 0;
}

SingletonGenEngine::~SingletonGenEngine() {
	std::vector<GenPrototype*>::iterator iterPro;
	std::vector<Generation*>::iterator iterGener;
	std::vector<Individual*>::iterator iterInd;
	std::vector<Gen*>::iterator iterGen;

	//delete all Prototypes
	while(m_prototype.size()>0) {
		iterPro = m_prototype.begin();
		delete (*iterPro);
		m_prototype.erase(iterPro);
	}
	m_prototype.clear();

	//delete all generation
	while(m_generation.size()>0) {
		iterGener = m_generation.begin();
		delete (*iterGener);
		m_generation.erase(iterGener);
	}
	m_generation.clear();

	//delete all individual
	while(m_individual.size()>0) {
		iterInd = m_individual.begin();
		delete (*iterInd);
		m_individual.erase(iterInd);
	}
	m_individual.clear();

	//delete all gens
	while(m_gen.size()>0) {
		iterGen = m_gen.begin();
		delete (*iterGen);
		m_gen.erase(iterGen);
	}
	m_gen.clear();

	SingletonGenFactory::destroyGenFactory();
	SingletonIndividualFactory::destroyFactory();

	// should we clean the strategies?
	if(m_cleanStrategies) {
		if(m_selectStrategy!=0) {
			delete m_selectStrategy;
			m_selectStrategy = 0;
		}

		if(m_generationSizeStrategy!=0) {
			delete m_generationSizeStrategy;
			m_generationSizeStrategy = 0;
		}

		if(m_fitnessStrategy!=0) {
			delete m_fitnessStrategy;
			m_fitnessStrategy = 0;
		}
	}
}

void SingletonGenEngine::generateFirstGeneration(int startSize, int startKillRate, bool withUpdate) {
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

	// update generation
	if(withUpdate)
		first->update();
}

void SingletonGenEngine::prepareNextGeneration(int size, int killRate) {
	// correct the killRate		<-- bad development of the generation size
	if(killRate>size)
		killRate = size - 2;

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

void SingletonGenEngine::prepare(int startSize, int startKillRate, Generation*& generation, InspectableProxy*& proxy, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext, bool withUpdate) {
	// create first generation
	generateFirstGeneration(startSize,startKillRate, withUpdate);

	// Control values
	generation = getActualGeneration();
	if(plotEngine!=0) {
		plotEngine->addInspectable(generation);
		plotEngine->step(1.0);
	}
	std::list<Inspectable*> actualContextList;
	//InspectableProxy* actualContext;
	if(plotEngineGenContext!=0) {
		actualContextList.clear();
		for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
			actualContextList.push_back((*iter)->getContext(getActualGeneration()));
			(*iter)->getContext(getActualGeneration())->update();
		}
		proxy = new InspectableProxy(actualContextList);
		plotEngineGenContext->addInspectable(&(*proxy));
		plotEngineGenContext->step(1.0);
	}
}

void SingletonGenEngine::measureStep(double time, Generation*& generation, InspectableProxy*& proxy, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext) {
	std::list<Inspectable*> actualContextList;

	if(plotEngine!=0) {
		generation = getActualGeneration();
		plotEngine->step(time);
	}
	if(plotEngineGenContext!=0) {
		actualContextList.clear();
		for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
			actualContextList.push_back((*iter)->getContext(getActualGeneration()));
		}
		proxy->replaceList(actualContextList);
		plotEngineGenContext->step(time);
	}
}

void SingletonGenEngine::runGenAlg(int startSize, int startKillRate, int numGeneration, RandGen* random, PlotOptionEngine* plotEngine, PlotOptionEngine* plotEngineGenContext) {
	//std::list<Inspectable*> actualContextList;
	InspectableProxy* actualContext;
	Generation* actual;

	// create first generation
	prepare(startSize,startKillRate,actual,actualContext,plotEngine,plotEngineGenContext);
	/*generateFirstGeneration(startSize,startKillRate);

	// Control values
	Generation& actual = *getActualGeneration();
	if(plotEngine!=0) {
		plotEngine->addInspectable(&actual);
		plotEngine->step(1.0);
	}
	std::list<Inspectable*> actualContextList;
	InspectableProxy* actualContext;
	if(plotEngineGenContext!=0) {
		actualContextList.clear();
		for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
			actualContextList.push_back((*iter)->getContext(getActualGeneration()));
			(*iter)->getContext(getActualGeneration())->update();
		}
		actualContext = new InspectableProxy(actualContextList);
		plotEngineGenContext->addInspectable(&(*actualContext));
		plotEngineGenContext->step(1.0);
	}*/

	// generate the other generations
	for(int x=0;x<numGeneration;x++) {
		select();
		crossover(random);
		update();

		printf("Generaion %i:\tabgeschlossen.\n",x);

		/*if(plotEngine!=0) {
			actual = getActualGeneration();
			plotEngine->step((double)(x+2));
		}
		if(plotEngineGenContext!=0) {
			actualContextList.clear();
			for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
				actualContextList.push_back((*iter)->getContext(getActualGeneration()));
			}
			actualContext->replaceList(actualContextList);
			plotEngineGenContext->step((double)(x+2));
		}*/
		measureStep((double)(x+2),actual,actualContext,plotEngine,plotEngineGenContext);

		/*prepareNextGeneration();
		m_generation[m_actualGeneration-1]->select(m_generation[m_actualGeneration]);
		m_generation[m_actualGeneration]->crossOver(random);*/

		// Abbruchkriterium fehlt noch!!!
		// TODO
	}
}

void SingletonGenEngine::select(bool createNextGeneration) {
	//std::cout<<"createNextGeneration:"<<(createNextGeneration?" yes\n":" no\n");
	if(createNextGeneration)
		getInstance()->prepareNextGeneration(m_generationSizeStrategy->calcGenerationSize(getActualGeneration()),getActualGeneration()->getKillRate());

	//std::cout<<"begin select\n";
	getInstance()->m_selectStrategy->select(getInstance()->m_generation[getInstance()->m_actualGeneration-1],getInstance()->m_generation[getInstance()->m_actualGeneration]);
	//std::cout<<"select OK\n";

	//std::cout<<"copy Gens\n";
	// insert the old gens in the new GenContext.
	const std::vector<Individual*>& old = getInstance()->m_generation[getInstance()->m_actualGeneration]->getAllIndividual();
	std::vector<Individual*>::const_iterator iter;
	int num;
	Gen* gen;
	GenPrototype* prototype;
	GenContext* newContext;

	for(iter=old.begin();iter!=old.end();iter++) {
		num = (*iter)->getSize();
		for(int x=0; x<num; x++) {
			gen = (*iter)->getGen(x);
			prototype = gen->getPrototype();
			newContext = prototype->getContext(getInstance()->m_generation[getInstance()->m_actualGeneration]);
			newContext->addGen(gen);
		}
	}

	//std::cout<<"alles OK\n";
}

void SingletonGenEngine::crossover(RandGen* random) {
	m_generation[m_actualGeneration]->crossover(random);
}

void SingletonGenEngine::update(double factor) {
	m_generation[m_actualGeneration]->update(factor);

	for(std::vector<GenPrototype*>::const_iterator iter = m_prototype.begin(); iter!=m_prototype.end(); iter++) {
		(*iter)->getContext(m_generation[m_actualGeneration])->update(factor);
	}
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

std::string SingletonGenEngine::getIndividualRoot(bool withMutation)const {
	std::string result="";

	for(std::vector<Individual*>::const_iterator iter=m_individual.begin();iter!=m_individual.end();iter++) {
		result += (*iter)->RootToString(withMutation) + "\n";
	}

	return result;
}

std::string SingletonGenEngine::getAllIndividualAsString(void)const {
	std::string result = "";

	for(std::vector<Individual*>::const_iterator iter=m_individual.begin();iter!=m_individual.end();iter++) {
		result += (*iter)->IndividualToString() + "\n";
	}

	return result;
}