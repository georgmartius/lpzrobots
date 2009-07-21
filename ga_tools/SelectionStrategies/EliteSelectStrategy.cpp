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
 *   This class is a implementation for the ISelectStrategy. It make an    *
 *   elite select. This mean only the best individual comes in the next    *
 *   generation.                                                           *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2009-07-21 08:37:59  robot12
 *   add some comments
 *
 *   Revision 1.5  2009/06/25 13:34:17  robot12
 *   finish the select strategy and add some comments.
 *
 *   Revision 1.4  2009/05/14 15:29:56  robot12
 *   bugfix: mutation change the oldGen, not the new!!! now fixed
 *
 *   Revision 1.3  2009/05/11 14:08:53  robot12
 *   patch some bugfix....
 *
 *   Revision 1.2  2009/05/06 13:28:23  robot12
 *   some implements... Finish
 *
 *   Revision 1.1  2009/05/04 15:27:56  robot12
 *   rename of some files and moving files to other positions
 *    - SingletonGenAlgAPI has one error!!! --> is not ready now
 *
 *   Revision 1.2  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.1  2009/04/30 11:51:26  robot12
 *   some implements... new classes
 *
 *
 *
 ***************************************************************************/

#include "EliteSelectStrategy.h"

#include "Individual.h"
#include "Generation.h"
#include <list>

/**
 * help structur to sort the individual by the fitness values.
 */
class SfitnessEliteStrategyStruct {
public:
	double fitness;
	Individual* ind;

	inline bool operator<(SfitnessEliteStrategyStruct other) {
		return (fitness*fitness)<(other.fitness*other.fitness);
	}

	~SfitnessEliteStrategyStruct() {
		fitness=0.0;
		ind=0;
	}
};

EliteSelectStrategy::EliteSelectStrategy() {
	// nothing
}

EliteSelectStrategy::~EliteSelectStrategy() {
	// nothing
}

/*void mache(SfitnessEliteStrategyStruct i){
	printf("%lf\n",i.fitness);
}*/

void EliteSelectStrategy::select(Generation* oldGeneration, Generation* newGeneration) {
	std::list<SfitnessEliteStrategyStruct> list;						//a list with all individual of the old generation and there fitness values
	SfitnessEliteStrategyStruct* storage;								//one element from the list
	std::list<SfitnessEliteStrategyStruct>::iterator iter;				//iterator for the list.
	int num,x,kill;														//help variables

	// prepare the list
	num = oldGeneration->getCurrentSize();								//take all individual with there fitness values in the list.
	for(x=0;x<num;x++) {
		storage = new SfitnessEliteStrategyStruct();					//create new element for the list
		storage->ind = oldGeneration->getIndividual(x);
		storage->fitness = storage->ind->getFitness();
		list.push_back(*storage);										//add element to the list
		storage = 0;
	}

	//std::for_each(list.begin(),list.end(),mache);
	//printf("Test\n");

	// sort the list
	list.sort();														//sort the list

	//std::for_each(list.begin(),list.end(),mache);
	//printf("Test\n");

	// kill the badest
	kill = oldGeneration->getKillRate();
	iter = list.begin();
	std::advance(iter,num-kill);										//delete all elements which are not selected from the list
	while(iter!=list.end()) {
		//storage = &(*iter);
		//delete storage;
		iter=list.erase(iter);
	}
	/*for(x=num-1;x>=num-kill;x--) {
		iter = list.begin();
		for(int y=0;y<x;y++)
			iter++;
		delete (*iter);
		list.erase(iter);
	}*/

	//std::for_each(list.begin(),list.end(),mache);
	//printf("Test\n");

	// take the best in the new generation
	x = 0;																		//copy the best individual in the new generation
	for(iter = list.begin();iter != list.end() && x<newGeneration->getSize();iter++) {
		newGeneration->addIndividual(iter->ind);
		x++;
	}

	// clean
	iter=list.begin();
	while(iter!=list.end()) {
		//storage = &(*iter);
		//delete storage;
		iter=list.erase(iter);
	}
	list.clear();
}
