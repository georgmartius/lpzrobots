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
 *   Revision 1.4  2009-04-27 10:59:33  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#include "SingletonIndividualFactory.h"

SingletonIndividualFactory::SingletonIndividualFactory() {
	// nothing
}

SingletonIndividualFactory::~SingletonIndividualFactory() {
	// nothing
}

Individual* SingletonIndividualFactory::createIndividual(std::string name)const {
	Individual ind = new Individual(name,m_number++);
	GenPrototyp* prototyp;
	std::vector<GenPrototyp*>* storage;

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps();
	int num = storage->size();
	for(int x=0;x<num;x++) {
		prototyp = storage[x];
		SingletonGenFactory::getInstance()->createGen(prototyp);
	}

	return ind;
}

Individual* createIndividual(Individual* individual1, Individual* individual2,randGen* random)const {
	Individual ind = new Individual(individual1->getName()+"##"+individual2->getName());
	GenPrototyp* prototyp;
	std::vector<GenPrototyp*>* storage;
	Gen* gen;
	int r1,r2;
	Individual* ind;
	Generation* generation = SingletonGenEngine::getInstance()->getGeneration(SingletonGenEngine::getInstance()->getActiveGenerationNumber()-1);

	storage = SingletonGenEngine::getInstance()->getSetOfGenPrototyps(generation);
	int num = storage->size();
	for(int x=0;x<num;x++) {
		prototyp = storage[x];
		r1 = ((int)random->rand())%2;
		r2 = ((int)random->rand())%1000;
		ind = r1==0?individual1:individual2;

		int num2 = ind->getSize();
		for(int y=0;y<num2;y++) {
			gen = ind->getGen(y);
			if(gen->getID()==prototyp->getID()) {
				if(r2<prototyp->getKontext(generation)->getMutationProbability()) {
					SingletonGenFactory::getInstance()->createGen(prototyp,prototyp->getKontext(generation),gen,true);
					break;
				}
				else {
					SingletonGenFactory::getInstance()->createGen(prototyp,prototyp->getKontext(generation),gen,false);
					break;
				}
			}
		}
	}

	return ind;
}
