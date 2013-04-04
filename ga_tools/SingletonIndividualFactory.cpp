/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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
 *                                                                         *
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

        storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps();        //become all GenPrototypes
        int num = storage.size();
        for(int x=0;x<num;x++) {
                prototype = storage[x];                                                                                                //create a random Gen for every Prototype
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
        for(int x=0;x<num;x++) {                                                        //take randomized the gens from ind 1 or 2.
                prototype = storage[x];
                r1 = ((int)(random->rand()*10000.0))%2;
                r2 = ((int)(random->rand()*10000.0))%1000;
                ind = r1==0?individual1:individual2;

                gen = ind->getGen(x);
                if(r2<prototype->getMutationProbability()) {                //with a mutation probability it is possible that the gen mutate
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
