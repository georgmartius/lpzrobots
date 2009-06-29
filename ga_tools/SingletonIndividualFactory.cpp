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
 *   This class is a factory for the class individual. It use the          *
 *   GenFactory to create a new individual, because the individual is a    *
 *   combination of Gens.                                                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-06-29 15:20:25  robot12
 *   finishing Individual Factory and add some comments
 *
 *   Revision 1.3  2009/05/11 14:08:51  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/06 13:28:22  robot12
 *   some implements... Finish
 *
 *   Revision 1.1  2009/05/04 15:27:55  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.6  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *   Revision 1.5  2009/04/28 13:23:55  robot12
 *   some implements... Part2
 *
 *   Revision 1.4  2009/04/27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "SingletonIndividualFactory.h"

//ga_tools includes
#include "SingletonGenFactory.h"
#include "SingletonGenEngine.h"
#include "Generation.h"
#include "Gen.h"
#include "GenContext.h"

SingletonIndividualFactory* SingletonIndividualFactory::m_factory = 0;
int SingletonIndividualFactory::m_number = 0;

SingletonIndividualFactory::SingletonIndividualFactory() {
	// nothing
}

SingletonIndividualFactory::~SingletonIndividualFactory() {
	// nothing
}

Individual* SingletonIndividualFactory::createIndividual(std::string name)const {
	Individual* ind = new Individual(name,m_number++);
	GenPrototype* prototype;
	std::vector<GenPrototype*> storage;

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps();	//become all GenPrototypes
	int num = storage.size();
	for(int x=0;x<num;x++) {
		prototype = storage[x];												//create a random Gen for every Prototype
		SingletonGenFactory::getInstance()->createGen(prototype->getContext(SingletonGenEngine::getInstance()->getActualGeneration()),ind,prototype);
	}

	SingletonGenEngine::getInstance()->addIndividual(ind);

	return ind;
}

Individual* SingletonIndividualFactory::createIndividual(Individual* individual1, Individual* individual2, RandGen* random, std::string name)const {
	Individual* newInd = new Individual(name,m_number++,individual1,individual2);
	GenPrototype* prototype;
	std::vector<GenPrototype*> storage;
	Gen* gen;
	int r1,r2;
	Individual* ind;
	Generation* generation = SingletonGenEngine::getInstance()->getActualGeneration();
	Generation* oldGeneration = SingletonGenEngine::getInstance()->getGeneration(SingletonGenEngine::getInstance()->getActualGenerationNumber()-1);

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps();
	int num = storage.size();
	for(int x=0;x<num;x++) {							//take randomized the gens from ind 1 or 2.
		prototype = storage[x];
		r1 = ((int)(random->rand()*10000.0))%2;
		r2 = ((int)(random->rand()*10000.0))%1000;
		ind = r1==0?individual1:individual2;

		gen = ind->getGen(x);
		if(r2<prototype->getMutationProbability()) {		//with a mutation probability it is possible that the gen mutate
			SingletonGenFactory::getInstance()->createGen(prototype->getContext(generation),newInd,prototype,prototype->getContext(oldGeneration),ind,gen,true);
			newInd->setMutated();
		}
		else {
			//SingletonGenFactory::getInstance()->createGen(prototype->getContext(generation),newInd,prototype,prototype->getContext(oldGeneration),ind,gen,false);
			prototype->getContext(generation)->addGen(gen);
			newInd->addGen(gen);
		}
	}

	SingletonGenEngine::getInstance()->addIndividual(newInd);

	return newInd;
}
